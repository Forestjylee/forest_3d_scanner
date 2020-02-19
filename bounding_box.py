import argparse
import open3d as o3d
from os.path import exists


if __name__ == "__main__":
    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
    parser = argparse.ArgumentParser()
    parser.add_argument("-volume", action="store_true", help="Print calculate bounding box's volume.")
    parser.add_argument("-m", "--mesh-file", type=str, help="Mesh file path.")
    args = parser.parse_args()

    mesh_path = args.mesh_file
    if not exists(mesh_path):
        print("Mesh file is not found.")
        exit(-1)
    
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    mesh.transform(flip_transform)
    bbox = mesh.get_axis_aligned_bounding_box()
    bbox_xyz_lengths = bbox.get_extent()
    x, y, z = bbox_xyz_lengths
    # center_coodinates = bbox.get_center()
    if args.volume:
        print(f"Bounding box's volume:  {bbox.volume()}")
        print(f"Length of the bounding box in x, y, and z dimension:  x=%.5f, y=%.5f, z=%.5f"%(x, y, z))
    o3d.visualization.draw_geometries([bbox, mesh], window_name=mesh_path, width=1280, height=768)
