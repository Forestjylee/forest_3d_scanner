import argparse
import numpy as np
import open3d as o3d
from os.path import exists


def show_mesh_with_rotation(mesh, set_background_black=True, window_name="mesh"):

    def rotate_view(vis):
        if set_background_black:
            opt = vis.get_render_option()
            opt.background_color = np.asarray([0, 0, 0])
        ctr = vis.get_view_control()
        ctr.rotate(5.0, 0.0)
        return False

    o3d.visualization.draw_geometries_with_animation_callback(
        [mesh], rotate_view,
        window_name=window_name,
        width=1280, height=768
    )


def show_mesh(mesh, window_name="mesh"):
    o3d.visualization.draw_geometries(
        [mesh], window_name=window_name,
         width=1280, height=768
    )


if __name__ == "__main__":
    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]

    parser = argparse.ArgumentParser()
    parser.add_argument("-display", action="store_true", help="Show animation visualization.")
    parser.add_argument("-m", "--mesh-file", type=str, help="Mesh file path.", default="None")
    args = parser.parse_args()
    mesh_filepath = args.mesh_file

    if not exists(mesh_filepath):
        print("Mesh file is not exist.")
        exit(-1)

    mesh = o3d.io.read_triangle_mesh(mesh_filepath)
    mesh.transform(flip_transform)
    if args.display:
        show_mesh_with_rotation(mesh, window_name=mesh_filepath)
    else:
        show_mesh(mesh, window_name=mesh_filepath)
