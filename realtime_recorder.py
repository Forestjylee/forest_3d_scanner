import json
import numpy as np
import open3d as o3d
from os import getcwd
from copy import deepcopy
from os.path import join
from enum import IntEnum
from loguru import logger
import pyrealsense2 as rs


class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


class RealsenseRecorder(object):

    def __init__(self, frame_count=0, max_depth_in_meters=1.0):
        super(RealsenseRecorder, self).__init__()
        self.prev_cloud = None
        self.cloud_base = None
        self.camera_intrinsic = None
        self.pose_graph = None
        self.prev_rgbd_image = None
        self.next_rgbd_image = None  
      
        self.trans_odometry = np.identity(4)
        self.world_trans = np.identity(4)
        self.frame_count = frame_count
        self.max_depth_in_meters = max_depth_in_meters

        logger.debug("Init realsense stream pipeline")
        self.pipeline = rs.pipeline()

        logger.debug("Get pipeline's config")
        config = self._get_realsense_pipeline_config()

        logger.debug("Start streaming")
        self.profile = self.pipeline.start(config)

    def create_rgbd_image(self, color_image, depth_image, convert_rgb_to_intensity):
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_image),
            o3d.geometry.Image(depth_image),
            depth_trunc=self.max_depth_in_meters,
            convert_rgb_to_intensity=convert_rgb_to_intensity)
        return rgbd_image

    def register_one_rgbd_pair(source_rgbd_image, target_rgbd_image):
        option = o3d.odometry.OdometryOption()
        option.max_depth_diff = 0.07
        odo_init = np.identity(4)
        [success, trans, info] = o3d.odometry.compute_rgbd_odometry(
            source_rgbd_image, target_rgbd_image, self.camera_intrinsic, odo_init,
            o3d.odometry.RGBDOdometryJacobianFromHybridTerm(), option)
        return [success, trans, info]

    def init_camera_intrinsic(self, color_frame):
        logger.debug("Saving camera intrinsic info")
        config_path = join(getcwd(), "camera_intrinsic.json")
        self._save_intrinsic_as_json(config_path, color_frame)
        self.camera_intrinsic = o3d.io.read_pinhole_camera_intrinsic(config_path)
        logger.debug("Saved success")

    def init_pose_graph(self):
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
        self.pose_graph = o3d.registration.PoseGraph()
        trans_odometry = np.identity(4)
        self.pose_graph.nodes.append(o3d.registration.PoseGraphNode(trans_odometry))

    def update_pose_graph(self):
        [success, trans, info] = self.register_one_rgbd_pair(
            self.prev_rgbd_image, self.next_rgbd_image
        )
        self.trans_odometry = np.dot(trans, self.trans_odometry)
        trans_odometry_inv = np.linalg.inv(self.trans_odometry)
        self.pose_graph.nodes.append(o3d.registration.PoseGraphNode(trans_odometry_inv))
        o3d.io.write_pose_graph(join(getcwd(), "pose_graph.json"), self.pose_graph)

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

        self._skip_auto_exposure_time()
        # Streaming loop
        try:
            visaulizer = self._get_visualizer()
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

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                rgbd_image = self.create_rgbd_image(color_image, depth_image, False)

                if self.frame_count == 0:
                    self.init_camera_intrinsic(color_frame)
                    self.prev_rgbd_image = rgbd_image
                else:
                    self.next_rgbd_image = rgbd_image
                    

                current_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                    rgbd_image, self.camera_intrinsic)
                current_pcd.transform(flip_transform)

                if self.frame_count == 0:
                    self.cloud_base = current_pcd
                    self.prev_cloud = current_pcd
                    # visaulizer.create_window()
                    # visaulizer.add_geometry(self.cloud_base)
                else:
                    voxel_radius = [0.04, 0.02, 0.01]
                    max_iter = [50, 30, 14]
                    current_transformation = np.identity(4)
                    for scale in range(3):
                        iter = max_iter[scale]
                        radius = voxel_radius[scale]

                        source_down = current_pcd.voxel_down_sample(radius)
                        target_down = self.prev_cloud.voxel_down_sample(radius)

                        source_down.estimate_normals(
                            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
                        target_down.estimate_normals(
                            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

                        result_icp = o3d.registration.registration_colored_icp(
                            source_down, target_down, radius, current_transformation,
                            o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                                    relative_rmse=1e-6,
                                                                    max_iteration=iter))
                        current_transformation = result_icp.transformation
                    self.world_trans = np.dot(self.world_trans, current_transformation)
                    self.prev_cloud = deepcopy(current_pcd)
                    current_pcd.transform(self.world_trans)
                    self.cloud_base = self.cloud_base + current_pcd  
                    self.cloud_base = self.cloud_base.voxel_down_sample(0.002)  
                logger.info(f"Process frame {self.frame_count}")              
                self.frame_count += 1

                # visaulizer.update_geometry(self.cloud_base)
                # visaulizer.poll_events()
                # visaulizer.update_renderer()
                if self.frame_count ==100:
                    o3d.io.write_point_cloud(join(getcwd(), "result.ply"), self.cloud_base, False, True)
                    break

        finally:
            self.pipeline.stop()
            # visaulizer.destroy_window()


if __name__ == "__main__":
    recorder = RealsenseRecorder()
    recorder.run()
