import cv2
import time
import json
import shutil
import numpy as np
import open3d as o3d
from copy import deepcopy
from enum import IntEnum
from loguru import logger
import pyrealsense2 as rs
from os import getcwd, makedirs
from os.path import join, exists
from utils import get_rgbd_file_lists


class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


class RealsenseRecorder(object):

    def __init__(self, end, frame_count=0, output_folder=None,
                 max_depth_in_meters=1.0, result_type='pcd'):
        super(RealsenseRecorder, self).__init__()
        self.end = end
        self.prev_cloud = None
        self.cloud_base = None
        self.pose_graph = None
        self.camera_intrinsic = None
        self.result_type = result_type

        self.output_folder = output_folder if output_folder else join(getcwd(), "dataset")
        self.color_folder = join(self.output_folder, "color")
        self.depth_folder = join(self.output_folder, "depth")
      
        self.world_trans = np.identity(4)
        self.frame_count = frame_count
        self.max_depth_in_meters = max_depth_in_meters

        self._init_camera()

    def create_rgbd_image(self, color_image, depth_image, convert_rgb_to_intensity):
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_image),
            o3d.geometry.Image(depth_image),
            depth_trunc=self.max_depth_in_meters,
            convert_rgb_to_intensity=convert_rgb_to_intensity)
        return rgbd_image

    def read_rgbd_image(self, color_file, depth_file):
        color_image = o3d.io.read_image(color_file)
        depth_image = o3d.io.read_image(depth_file)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_image, depth_image, 
            depth_trunc=self.max_depth_in_meters, 
            convert_rgb_to_intensity=False
        )
        return rgbd_image

    def register_one_rgbd_pair(source_rgbd_image, target_rgbd_image):
        option = o3d.odometry.OdometryOption()
        option.max_depth_diff = 0.07
        odo_init = np.identity(4)
        [success, trans, info] = o3d.odometry.compute_rgbd_odometry(
            source_rgbd_image, target_rgbd_image, self.camera_intrinsic, odo_init,
            o3d.odometry.RGBDOdometryJacobianFromHybridTerm(), option)
        return [success, trans, info]

    def register_color_icp(self, source, target):
        # voxel_radius = [0.04, 0.02, 0.01]
        voxel_radius = [0.02, 0.01, 0.005]
        max_iter = [50, 30, 14]
        current_transformation = np.identity(4)
        for scale in range(3):
            iteration = max_iter[scale]
            radius = voxel_radius[scale]

            source_down = source.voxel_down_sample(radius)
            target_down = target.voxel_down_sample(radius)

            source_down.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
            target_down.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

            result_icp = o3d.registration.registration_colored_icp(
                source_down, target_down, radius, current_transformation,
                o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                        relative_rmse=1e-6,
                                                        max_iteration=iteration))
            current_transformation = result_icp.transformation   
        return current_transformation   

    def integrate_and_save(self, mesh_or_pcd='pcd'):
        color_image_paths, depth_image_paths = get_rgbd_file_lists(self.output_folder)
        volume = o3d.integration.ScalableTSDFVolume(
            voxel_length=0.002,
            sdf_trunc=0.04,
            color_type=o3d.integration.TSDFVolumeColorType.RGB8
        )
        for i in range(len(self.pose_graph.nodes)):
            logger.info("Integrate rgbd frame %d." % i)
            rgbd_image = self.read_rgbd_image(color_image_paths[i], depth_image_paths[i])
            pose = self.pose_graph.nodes[i].pose
            volume.integrate(rgbd_image, self.camera_intrinsic, np.linalg.inv(pose))
        
        if mesh_or_pcd == 'mesh':
            mesh = volume.extract_triangle_mesh()
            mesh_path = join(getcwd(), "result_mesh.ply")
            mesh.compute_vertex_normals()
            o3d.io.write_triangle_mesh(mesh_path, mesh, False, True)
            logger.info(f"Color mesh has been saved in {mesh_path}")
        else:
            pcd = volume.extract_point_cloud()
            pcd_path = join(getcwd(), "result_colored_pcd.ply")
            o3d.io.write_point_cloud(pcd_path, pcd, False, True)
            logger.info(f"Color point cloud has been saved in {pcd_path}")

    def _init_camera(self):
        logger.debug("Init realsense stream pipeline")
        self.pipeline = rs.pipeline()

        logger.debug("Get pipeline's config")
        config = self._get_realsense_pipeline_config()

        logger.debug("Start streaming")
        self.profile = self.pipeline.start(config)       

    def _init_camera_intrinsic(self, color_frame):
        logger.debug("Saving camera intrinsic info")
        config_path = join(getcwd(), "camera_intrinsic.json")
        self._save_intrinsic_as_json(config_path, color_frame)
        self.camera_intrinsic = o3d.io.read_pinhole_camera_intrinsic(config_path)
        logger.debug("Saved success")

    def _init_pose_graph(self):
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
        self.pose_graph = o3d.registration.PoseGraph()

    @staticmethod
    def make_clean_folder(path_folder):
        if not exists(path_folder):
            makedirs(path_folder)
        else:
            user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
            if user_input.lower() == 'y':
                shutil.rmtree(path_folder)
                makedirs(path_folder)
            else:
                exit()

    @staticmethod
    def _get_realsense_pipeline_config() -> rs.config:
        #Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()

        # note: using 640 x 480 depth resolution produces smooth depth boundaries
        #       using rs.format.bgr8 for color image format for OpenCV based image visualization
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        return config

    @staticmethod
    def _save_intrinsic_as_json(filename, frame):
        intrinsics = frame.profile.as_video_stream_profile().intrinsics
        intrinsic_dict = {}
        with open(filename, 'w') as outfile:
            intrinsic_dict = {
                    'width':
                        intrinsics.width,
                    'height':
                        intrinsics.height,
                    'intrinsic_matrix': [
                        intrinsics.fx, 0, 0, 0, intrinsics.fy, 0, intrinsics.ppx,
                        intrinsics.ppy, 1
                    ]
            }
            json.dump(intrinsic_dict, outfile, indent=4)

    @staticmethod
    def _get_align_object() -> rs.align:
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)
        return align

    @staticmethod
    def _get_visualizer():
        return o3d.visualization.Visualizer()

    def _get_depth_scale(self):
        logger.debug("Init depth sensor")
        depth_sensor = self.profile.get_device().first_depth_sensor()
        
        logger.debug("Config depth sensor")
        # Using preset HighAccuracy for recording
        depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_scale = depth_sensor.get_depth_scale()
        return depth_scale

    def _skip_auto_exposure_time(self):
        logger.debug("Skip 5 first frames to give the Auto-Exposure time to adjust")
        for x in range(5):
            self.pipeline.wait_for_frames()
            logger.debug(f"Skip {x+1} frame") 

    def run(self):
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_scale = self._get_depth_scale()

        # Create an align object
        # The "align_to" is the stream type to which we plan to align depth frames.
        align = self._get_align_object()

        # Will not display the background of objects more than
        #  max_depth meters away
        clipping_distance = self.max_depth_in_meters / depth_scale
        logger.debug(f"Clipping distance is {self.max_depth_in_meters} meter")

        # Prepare clean folder to store color and depth images
        logger.debug("Make clean folder to store color and depth images")
        self.make_clean_folder(self.output_folder)
        self.make_clean_folder(self.color_folder)
        self.make_clean_folder(self.depth_folder)
        logger.debug("Success")

        self._skip_auto_exposure_time()
        # Streaming loop
        try:
            flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
            while True:
                # Get frameset of color and depth
                frames = self.pipeline.wait_for_frames()

                # Align the depth frame to color frame
                aligned_frames = align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                # Init camera intrinsic and pose graph
                if self.frame_count == 0:
                    self._init_camera_intrinsic(color_frame)
                    self._init_pose_graph()

                    logger.debug("----------Start streaming----------")
                    input("Please press any key to start.")

                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    continue

                # Extract color and depth image from aligned frame
                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Create grey rgbd image
                rgbd_image = self.create_rgbd_image(color_image, depth_image, True)

                # Create current point cloud
                current_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                    rgbd_image, self.camera_intrinsic
                )
                current_pcd.transform(flip_transform)

                if self.frame_count == 0:
                    self.cloud_base = current_pcd
                    self.prev_cloud = current_pcd
                    visaulizer = self._get_visualizer()
                    visaulizer.create_window()
                    visaulizer.add_geometry(self.cloud_base)
                else:
                    current_transformation = self.register_color_icp(
                        source=current_pcd, target=self.prev_cloud
                    )
                    logger.info(f"Register frame {self.frame_count} success")  

                trans_odometry_inv = np.linalg.inv(self.world_trans)
                self.pose_graph.nodes.append(o3d.registration.PoseGraphNode(trans_odometry_inv))
                logger.info("Update pose graph success")

                cv2.imwrite(join(self.color_folder, "%06d.jpg" % self.frame_count), color_image)
                cv2.imwrite(join(self.depth_folder, "%06d.png" % self.frame_count), depth_image)
                logger.info(f"Saved color+depth image {self.frame_count}")
                self.frame_count += 1

                # Update visualizer
                if self.frame_count % 10 == 0:
                    visaulizer.update_geometry(self.cloud_base)
                    visaulizer.poll_events()
                    visaulizer.update_renderer()

                if self.frame_count == self.end:
                    self.integrate_and_save(mesh_or_pcd=self.result_type)
                    o3d.io.write_point_cloud(join(getcwd(), "result_raw.ply"), self.cloud_base, False, True)
                    logger.debug("----------Tear down----------")
                    break
        finally:
            self.pipeline.stop()
            visaulizer.destroy_window()


if __name__ == "__main__":
    recorder = RealsenseRecorder(end=20, result_type='pcd')
    recorder.run()
