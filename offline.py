import numpy as np
import open3d as o3d
from os import getcwd
from os.path import join, exists
from utils import get_rgbd_file_lists
from global_config import OfflineConfig

offline_config  = OfflineConfig()
voxel_size = offline_config.voxel_size
max_correspondence_distance_coarse = voxel_size * offline_config.max_correspondence_distance_coarse
max_correspondence_distance_fine = voxel_size * offline_config.max_correspondence_distance_fine


def load_point_clouds_and_rgbd_images(dataset_folder, voxel_size, camera_intrinsic):
    print(f"Loading images from {dataset_folder}...")
    color_files, depth_files = get_rgbd_file_lists(dataset_folder)
    pcds = []
    rgbd_images = []
    for i in range(len(depth_files)):
        color = o3d.io.read_image(color_files[i])
        depth = o3d.io.read_image(depth_files[i])
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, 
            depth_trunc=1.0,
            convert_rgb_to_intensity=False
        )
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, camera_intrinsic)
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcd_down.estimate_normals()
        pcds.append(pcd_down)
        rgbd_images.append(rgbd_image)
        print(f"Load color+depth image {i} successfully.")
    return pcds, rgbd_images


def pairwise_registration(source, target):
    icp_coarse = o3d.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp


def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.registration.PoseGraphNode(odometry))
    for i in range(len(pcds)-1):
        print(f"Calculating transformation matrix between image {i} and image {i+1}.")
        transformation_icp, information_icp = pairwise_registration(
            pcds[i], pcds[i+1]
        )
        odometry = np.dot(transformation_icp, odometry)
        pose_graph.nodes.append(o3d.registration.PoseGraphNode(np.linalg.inv(odometry)))
        pose_graph.edges.append(
            o3d.registration.PoseGraphEdge(
                i, i+1, transformation_icp,
                information_icp, uncertain=False
            )
        )
    return pose_graph


def generate_mesh(pose_graph, rgbd_images, camera_intrinsic, voxel_size, with_point_cloud=False):
    volume = o3d.integration.ScalableTSDFVolume(
        voxel_length=voxel_size,
        sdf_trunc=0.01,
        color_type=o3d.integration.TSDFVolumeColorType.RGB8
    )
    for i in range(len(pose_graph.nodes)):
        print(f"Integrating pose graph node {i}.")
        pose = pose_graph.nodes[i].pose
        volume.integrate(rgbd_images[i], camera_intrinsic, np.linalg.inv(pose))
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    if with_point_cloud:
        pcd = volume.extract_point_cloud()
        return mesh, pcd
    else:
        return mesh, None


def color_map_optimization(mesh, rgbd_images, camera_trajectory, maximum_iteration=200):
    print(f"Optimizing mesh, this may cost a bit time, please wait...")
    option = o3d.color_map.ColorMapOptimizationOption()
    option.maximum_iteration = maximum_iteration
    option.non_rigid_camera_coordinate = False
    o3d.color_map.color_map_optimization(mesh, rgbd_images, camera_trajectory, option)
    return mesh


def offline_main():
    if(offline_config.debug == True):
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    dataset_folder = offline_config.dataset
    camera_intrinsic = join(dataset_folder, offline_config.camera_intrinsic_filename)
    if not exists(dataset_folder) or not  exists(camera_intrinsic):
        print("Dataset or camera intrinsic is not exist.")
        exit(-1)
    camera_intrinsic = o3d.io.read_pinhole_camera_intrinsic(camera_intrinsic)
    pcds, rgbd_images = load_point_clouds_and_rgbd_images(dataset_folder, voxel_size, camera_intrinsic)

    print("Full registration ...")
    pose_graph = full_registration(pcds,
                                   max_correspondence_distance_coarse,
                                   max_correspondence_distance_fine)

    # Optimizing PoseGraph
    option = o3d.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.25,
        reference_node=0)
    o3d.registration.global_optimization(
        pose_graph, o3d.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.registration.GlobalOptimizationConvergenceCriteria(), option)

    # save pose graph
    pose_graph_path = join(dataset_folder, offline_config.pose_graph_filename)
    o3d.io.write_pose_graph(pose_graph_path, pose_graph)
    print(f"Pose graph has been saved in {pose_graph_path}")

    mesh, pcd = generate_mesh(pose_graph, rgbd_images, camera_intrinsic, 
                              voxel_size, with_point_cloud=offline_config.is_output_point_cloud)
    mesh_path = join(dataset_folder, offline_config.raw_mesh_filename)
    o3d.io.write_triangle_mesh(mesh_path, mesh, False)
    print(f"Raw mesh has been saved in {mesh_path}")

    # write point cloud
    if pcd is not None:
        pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pcd_path = join(dataset_folder, offline_config.point_cloud_filename)
        o3d.io.write_point_cloud(pcd_path, pcd, False)
        print(f"Point cloud has been saved in {pcd_path}")

    # Optimizing color map
    if offline_config.is_optimize_mesh:
        camera_trajectory = o3d.camera.PinholeCameraTrajectory()
        params_list = []
        for node in pose_graph.nodes:
            camera_params = o3d.camera.PinholeCameraParameters()
            camera_params.intrinsic = camera_intrinsic
            camera_params.extrinsic = np.linalg.inv(node.pose)
            params_list.append(camera_params)
        camera_trajectory.parameters = params_list

        mesh = color_map_optimization(mesh, rgbd_images, camera_trajectory, offline_config.optimization_iteration_count)
        mesh_path = join(dataset_folder, offline_config.optimized_mesh_filename)
        o3d.io.write_triangle_mesh(mesh_path, mesh, False)
        print(f"Optimized mesh has been saved in {mesh_path}")



if __name__ == "__main__":
    offline_main()
