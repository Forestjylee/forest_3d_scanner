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
from os.path import join, exists, dirname

from utils import get_rgbd_file_lists
from caffe_object_detection import caffe_get_detector, caffe_detect_body


class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


class RealsenseRecorder(object):

    def __init__(self, end, output_folder=None, voxel_size=0.0025,
                 max_depth_in_meters=1.0, icp_type='point_to_plane', only_body=False):
        super(RealsenseRecorder, self).__init__()
        self.icp_type = icp_type
        self.only_body = only_body
        self.voxel_size = voxel_size
        self.max_correspondence_distance_coarse = voxel_size * 15
        self.max_correspondence_distance_fine = voxel_size * 1.5      

        self.output_folder = output_folder if output_folder else join(getcwd(), "dataset")
        self.color_folder = join(self.output_folder, "color")
        self.depth_folder = join(self.output_folder, "depth")
        
        self.end = end
        self.frame_count = 0     
        self.prev_cloud = None
        self.cloud_base = None
        self.pose_graph = None
        self.rgbd_images = []
        self.camera_intrinsic = None
        self.world_trans = np.identity(4)
        self.max_depth_in_meters = max_depth_in_meters

        self.__init_camera()

        self.__init_object_detector()


    def create_pcd_from_rgbd_image(self, rgbd_image):
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, self.camera_intrinsic
        )
        pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        pcd_down.estimate_normals()
        return pcd_down

    def create_rgbd_image(self, color_file, depth_file):
        color_image = o3d.io.read_image(color_file)
        depth_image = o3d.io.read_image(depth_file)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_image, depth_image, 
            depth_trunc=self.max_depth_in_meters, 
            convert_rgb_to_intensity=False
        )
        return rgbd_image

    def register_point_to_plane_icp(self, source, target):
        icp_coarse = o3d.registration.registration_icp(source, target,
                self.max_correspondence_distance_coarse, np.identity(4),
                o3d.registration.TransformationEstimationPointToPlane())
        icp_fine = o3d.registration.registration_icp(source, target,
                self.max_correspondence_distance_fine, icp_coarse.transformation,
                o3d.registration.TransformationEstimationPointToPlane())
        transformation_icp = icp_fine.transformation
        information_icp = o3d.registration.get_information_matrix_from_point_clouds(
                source, target, self.max_correspondence_distance_fine,
                icp_fine.transformation)
        return transformation_icp, information_icp

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

    def update_pose_graph(self, transformation_icp, information_icp):
        self.pose_graph.nodes.append(o3d.registration.PoseGraphNode(np.linalg.inv(self.world_trans)))
        self.pose_graph.edges.append(
            o3d.registration.PoseGraphEdge(
                self.frame_count-1, self.frame_count,
                transformation_icp, information_icp, uncertain=False
            )
        )
        logger.debug("Update pose graph success")

    def optimize_pose_graph(self) -> None:
        option = o3d.registration.GlobalOptimizationOption(
            max_correspondence_distance=self.max_correspondence_distance_fine,
            edge_prune_threshold=0.25,
            reference_node=0
        )
        o3d.registration.global_optimization(
            self.pose_graph, o3d.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.registration.GlobalOptimizationConvergenceCriteria(), option
        )

    def integrate_rgbd_images(self) -> o3d.geometry.TriangleMesh:
        volume = o3d.integration.ScalableTSDFVolume(
            voxel_length = self.voxel_size,
            sdf_trunc = 0.01, 
            color_type = o3d.integration.TSDFVolumeColorType.RGB8
        )

        for index, rgbd_image in enumerate(self.rgbd_images):
            volume.integrate(rgbd_image, self.camera_intrinsic, np.linalg.inv(self.pose_graph.nodes[index].pose))
        
        mesh = volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        return mesh

    def __init_camera(self):
        logger.debug("Init realsense stream pipeline")
        self.pipeline = rs.pipeline()

        logger.debug("Get pipeline's config")
        config = self._get_realsense_pipeline_config()

        logger.debug("Start streaming")
        self.profile = self.pipeline.start(config)       

    def __init_object_detector(self):
        self.detector = caffe_get_detector(
            join(getcwd(), 'static', 'models', 'MobileNetSSD', 'MobileNetSSD_deploy.prototxt'),
            join(getcwd(), 'static', 'models', 'MobileNetSSD', 'MobileNetSSD_deploy.caffemodel')
        )
        logger.debug("Load object detection model successfully")

    def __init_camera_intrinsic(self, color_frame):
        logger.debug("Saving camera intrinsic info")
        config_path = join(self.output_folder, "camera_intrinsic.json")
        self._save_intrinsic_as_json(config_path, color_frame)
        self.camera_intrinsic = o3d.io.read_pinhole_camera_intrinsic(config_path)
        logger.debug("Saved success")

    def __init_pose_graph(self):
        self.pose_graph = o3d.registration.PoseGraph()
        odometry = np.identity(4)
        self.pose_graph.nodes.append(o3d.registration.PoseGraphNode(odometry))

    def __get_depth_scale(self):
        logger.debug("Init depth sensor")
        depth_sensor = self.profile.get_device().first_depth_sensor()
        
        logger.debug("Config depth sensor")
        # Using preset HighAccuracy for recording
        depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_scale = depth_sensor.get_depth_scale()
        return depth_scale

    def __skip_auto_exposure_time(self):
        logger.debug("Skip 5 first frames to give the Auto-Exposure time to adjust")
        for x in range(5):
            self.pipeline.wait_for_frames()
            logger.debug(f"Skip {x+1} frame") 

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

    def run(self):
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_scale = self.__get_depth_scale()

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

        self.__skip_auto_exposure_time()
        # Streaming loop
        try:
            INIT_FLAG = False
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
                    self.__init_camera_intrinsic(color_frame)
                    if self.icp_type == 'point_to_plane':
                        self.__init_pose_graph()
                        logger.debug("Using point to plane ICP registration")
                    else:
                        logger.debug("Using color ICP registration")
                    logger.debug("----------Start streaming----------")
                    input("Please press any key to start.")
                    INIT_FLAG = True
                    logger.info("Start processing after 3 secs.")
                    time.sleep(3)
                    continue

                # Extract color and depth image from aligned frame
                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Object detection
                if self.only_body:
                    detect_result = caffe_detect_body(detector=self.detector, image=color_image)
                    if detect_result is None:
                        logger.info("There is no body in the scene")
                        continue
                    else:
                        col_min, col_max, row_min, row_max = detect_result

                # Get color and depth path
                color_path = join(self.color_folder, "%06d.jpg" % self.frame_count)
                depth_path = join(self.depth_folder, "%06d.png" % self.frame_count)
                cv2.imwrite(color_path, color_image)
                cv2.imwrite(depth_path, depth_image)
                logger.info(f"Saved color+depth image {self.frame_count}")

                # Create current rgbd image
                rgbd_image = self.create_rgbd_image(
                    color_path, depth_path
                )

                # Create current point cloud
                current_pcd = self.create_pcd_from_rgbd_image(rgbd_image)

                if self.frame_count == 0:
                    self.cloud_base = current_pcd
                    self.prev_cloud = current_pcd
                else:
                    if self.icp_type == 'color':
                        current_transformation = self.register_color_icp(
                            source=current_pcd, target=self.prev_cloud
                        )
                        self.world_trans = np.dot(self.world_trans, current_transformation)
                        self.prev_cloud = deepcopy(current_pcd)
                        current_pcd.transform(self.world_trans)
                        self.cloud_base = self.cloud_base + current_pcd 
                    elif self.icp_type == 'point_to_plane':
                        transformation_icp, information_icp = self.register_point_to_plane_icp(
                            source=self.prev_cloud, target=current_pcd
                        )
                        self.world_trans = np.dot(transformation_icp, self.world_trans)
                        self.update_pose_graph(transformation_icp, information_icp)
                        self.prev_cloud = deepcopy(current_pcd)
                        self.rgbd_images.append(rgbd_image)

                    logger.info(f"Register frame {self.frame_count} success") 
                self.frame_count += 1

                # Display background removed color image currently
                # Remove background - Set pixels further than clipping_distance to grey
                grey_color = 153
                #depth image is 1 channel, color is 3 channels
                depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
                bg_removed = np.where((depth_image_3d > self.max_depth_in_meters / depth_scale) | \
                        (depth_image_3d <= 0), grey_color, color_image)

                bg_removed = cv2.rectangle(bg_removed, (col_min, row_min), (col_max, row_max), (255, 0, 0), 1)
                cv2.namedWindow('Recorder Realsense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('Recorder Realsense', bg_removed)
                cv2.waitKey(1)

                if self.frame_count == self.end:
                    result_path = join(self.output_folder, "online_raw_mesh.ply")
                    if self.icp_type == "point_to_plane":
                        pose_graph_path = join(self.output_folder, "pose_graph.json")
                        o3d.io.write_pose_graph(pose_graph_path, self.pose_graph)
                        logger.info(f"Pose graph has been saved in {pose_graph_path}")
                        # self.optimize_pose_graph()
                        mesh = self.integrate_rgbd_images()
                    elif self.icp_type == "color":
                        logger.info("Removing noise points in point cloud")
                        self.cloud_base.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                        self.cloud_base = self.cloud_base.voxel_down_sample(self.voxel_size)
                        
                        self.cloud_base.estimate_normals()
                        distances = self.cloud_base.compute_nearest_neighbor_distance()
                        avg_dist = np.mean(distances)
                        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
                            self.cloud_base, o3d.utility.DoubleVector([avg_dist, avg_dist * 2])
                        )
                    logger.info(mesh)
                    o3d.io.write_triangle_mesh(result_path, mesh, False)
                    logger.info(f"Result has been saved in {result_path}")
                    break
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()
            logger.debug("----------Tear down----------")


if __name__ == "__main__":
    recorder = RealsenseRecorder(end=60, icp_type='point_to_plane',
                                 max_depth_in_meters=1.0, voxel_size=0.0025, only_body=True)
    recorder.run()
