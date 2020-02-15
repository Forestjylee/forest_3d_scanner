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

    def __init__(self, end, frame_count=0, output_folder=None, voxel_size=0.002,
                 max_depth_in_meters=1.0, result_type='pcd'):
        super(RealsenseRecorder, self).__init__()
        self.end = end
        self.voxel_size = voxel_size
        self.result_type = result_type

        self.prev_cloud = None
        self.cloud_base = None
        self.camera_intrinsic = None

        self.output_folder = output_folder if output_folder else join(getcwd(), "dataset")
        self.color_folder = join(self.output_folder, "color")
        self.depth_folder = join(self.output_folder, "depth")
      
        self.world_trans = np.identity(4)
        self.frame_count = frame_count
        self.max_depth_in_meters = max_depth_in_meters

        self._init_camera()

    def create_pcd_from_color_and_depth(self, color_image, depth_image):
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_image, depth_image,
            depth_trunc=self.max_depth_in_meters,
            convert_rgb_to_intensity=False
        )
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, self.camera_intrinsic
        )
        return pcd

    def read_rgbd_image(self, color_file, depth_file):
        color_image = o3d.io.read_image(color_file)
        depth_image = o3d.io.read_image(depth_file)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_image, depth_image, 
            depth_trunc=self.max_depth_in_meters, 
            convert_rgb_to_intensity=False
        )
        return rgbd_image

    def register_color_icp(self, source, target):
        """
        Use colored ICP method to compute transformation of two point cloud
        """
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
            INIT_FLAG = False
            flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
            while True:
                # Get frameset of color and depth
                frames = self.pipeline.wait_for_frames()

                # Align the depth frame to color frame
                aligned_frames = align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    continue

                # Init camera intrinsic and pose graph
                if not INIT_FLAG:
                    self._init_camera_intrinsic(color_frame)
                    logger.debug("----------Start streaming----------")
                    input("Please press any key to start.")
                    INIT_FLAG = True
                    time.sleep(3)
                    continue

                # Extract color and depth image from aligned frame
                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Get color and depth path
                color_path = join(self.color_folder, "%06d.jpg" % self.frame_count)
                depth_path = join(self.depth_folder, "%06d.png" % self.frame_count)
                cv2.imwrite(color_path, color_image)
                cv2.imwrite(depth_path, depth_image)
                logger.info(f"Saved color+depth image {self.frame_count}")

                # Create grey rgbd image
                # rgbd_image = self.create_rgbd_image(color_image, depth_image, True)

                # Create current point cloud
                current_pcd = self.create_pcd_from_color_and_depth(
                    o3d.io.read_image(color_path),
                    o3d.io.read_image(depth_path)
                )
                current_pcd.transform(flip_transform)

                if self.frame_count == 0:
                    self.cloud_base = current_pcd
                    self.prev_cloud = current_pcd
                    # visaulizer = self._get_visualizer()
                    # visaulizer.create_window()
                    # visaulizer.add_geometry(self.cloud_base)
                else:
                    current_transformation = self.register_color_icp(
                        source=current_pcd, target=self.prev_cloud
                    )
                    logger.info(f"Register frame {self.frame_count} success") 

                    self.world_trans = np.dot(self.world_trans, current_transformation)
                    # self.world_trans = np.dot(current_transformation, self.world_trans)
                    self.prev_cloud = deepcopy(current_pcd)
                    current_pcd.transform(self.world_trans)
                    self.cloud_base = self.cloud_base + current_pcd  
                    # self.cloud_base = self.cloud_base.voxel_down_sample(self.voxel_size)  

                self.frame_count += 1

                # Update visualizer
                # if self.frame_count % 5 == 0:
                #     visaulizer.update_geometry(self.cloud_base)
                #     visaulizer.poll_events()
                #     visaulizer.update_renderer()

                if self.frame_count == self.end:
                    logger.info("Removing noise points in point cloud")
                    self.cloud_base.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                    self.cloud_base = self.cloud_base.voxel_down_sample(self.voxel_size)
                    logger.info(self.cloud_base)
                    result_path = join(getcwd(), "colored_result.ply")
                    if self.result_type == "pcd":
                        o3d.io.write_point_cloud(result_path, self.cloud_base, False, True)
                    else:
                        self.cloud_base.estimate_normals()
                        distances = self.cloud_base.compute_nearest_neighbor_distance()
                        avg_dist = np.mean(distances)
                        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
                            self.cloud_base, o3d.utility.DoubleVector([avg_dist, avg_dist * 2])
                        )
                        o3d.io.write_triangle_mesh(result_path, mesh, False, True)
                        o3d.io.write_point_cloud(join(getcwd(), "colored_pcd.ply"), self.cloud_base, False, True)
                    logger.info(f"Result has been saved in {result_path}")
                    break
        finally:
            self.pipeline.stop()
            # visaulizer.destroy_window()
            logger.debug("----------Tear down----------")


if __name__ == "__main__":
    recorder = RealsenseRecorder(end=30, result_type='mesh',
                                 max_depth_in_meters=1.0, voxel_size=0.0025)
    recorder.run()
