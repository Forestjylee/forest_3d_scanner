import argparse
import numpy as np
import open3d as o3d
from os import getcwd
from os.path import join, exists
from utils import get_rgbd_file_lists


voxel_size = 0.0025
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5


def load_point_clouds_and_rgbd_images(dataset_folder, voxel_size, camera_intrinsic):
    color_files, depth_files = get_rgbd_file_lists(dataset_folder)
    pcds = []
    rgbd_images = []
    for i in range(len(depth_files)):
        depth = o3d.io.read_image(depth_files[i])
        color = o3d.io.read_image(color_files[i])
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, 
            depth_trunc=1.0,
            convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, camera_intrinsic)
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcd_down.estimate_normals()
        pcds.append(pcd_down)
        rgbd_images.append(rgbd_image)
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


def generate_mesh(pose_graph, rgbd_images, camera_intrinsic, voxel_size):
    volume = o3d.integration.ScalableTSDFVolume(
        voxel_length=voxel_size,
        sdf_trunc=0.01,
        color_type=o3d.integration.TSDFVolumeColorType.RGB8
    )
    for i in range(len(pose_graph.nodes)):
        pose = pose_graph.nodes[i].pose
        volume.integrate(rgbd_images[i], camera_intrinsic, np.linalg.inv(pose))
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    return mesh


def color_map_optimization(mesh, rgbd_images, camera_trajectory, maximum_iteration=200):
    option = o3d.color_map.ColorMapOptimizationOption()
    option.maximum_iteration = maximum_iteration
    option.non_rigid_camera_coordinate = False
    o3d.color_map.color_map_optimization(mesh, rgbd_images, camera_trajectory, option)
    return mesh


if __name__ == "__main__":
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    DEFAULT_DATASET = join(getcwd(), "dataset")
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--dataset", type=str, help="Dataset folder path, which saves color images and depth images", default=DEFAULT_DATASET)
    parser.add_argument("-c", "--camera-intrinsic", type=str, help="Camera intrinsic matrix", default=join(DEFAULT_DATASET, "camera_intrinsic.json"))
    args = parser.parse_args()
    dataset_folder = args.dataset
    camera_intrinsic = args.camera_intrinsic
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
    # option = o3d.registration.GlobalOptimizationOption(
    #     max_correspondence_distance=max_correspondence_distance_fine,
    #     edge_prune_threshold=0.25,
    #     reference_node=0)
    # o3d.registration.global_optimization(
    #     pose_graph, o3d.registration.GlobalOptimizationLevenbergMarquardt(),
    #     o3d.registration.GlobalOptimizationConvergenceCriteria(), option)

    mesh = generate_mesh(pose_graph, rgbd_images, camera_intrinsic, voxel_size)
    mesh_path = join(dataset_folder, "offline_raw_mesh.ply")
    o3d.io.write_triangle_mesh(mesh_path, mesh, False)

    # Optimizing color map
    camera_trajectory = o3d.camera.PinholeCameraTrajectory()
    params_list = []
    for node in pose_graph.nodes:
        camera_params = o3d.camera.PinholeCameraParameters()
        camera_params.intrinsic = camera_intrinsic
        camera_params.extrinsic = np.linalg.inv(node.pose)
        params_list.append(camera_params)
    camera_trajectory.parameters = params_list

    mesh = color_map_optimization(mesh, rgbd_images, camera_trajectory, 200)
    o3d.io.write_triangle_mesh(join(dataset_folder, "offline_optimized_mesh.ply"), mesh, False)
