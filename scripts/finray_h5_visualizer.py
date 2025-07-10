from typing import Optional
import json
import trimesh
import os
import os.path as osp
from argparse import ArgumentParser
import numpy as np
from numpy import typing as nptype
import open3d as o3d
import time
from icecream import ic
import imageio
from typing import Optional
import pyvista as pv
import re
import matplotlib.pyplot as plt
import h5py
from yourdfpy import URDF
from tqdm import trange

parser = ArgumentParser()
parser.add_argument("--h5_path", type=str, default="")
parser.add_argument("--pose_id", type=int, default=0)
parser.add_argument("--ext", type=str, default="ply")
parser.add_argument("--fps", type=int, default=60)
parser.add_argument("--record_fps", type=int, default=24)
parser.add_argument("--video_folder", type=str, default="")
parser.add_argument("--width", type=int, default=1920)
parser.add_argument("--height", type=int, default=1080)
parser.add_argument("--camera", type=str, default="")
parser.add_argument("--object_type", type=str, default="")

NDArr = nptype.NDArray


def double_facing(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)

    # Create the back side of each triangle
    flipped_triangles = triangles[:, ::-1]

    # Combine the original and flipped triangles
    combined_triangles = np.vstack((triangles, flipped_triangles))

    # Create the new mesh
    stacked_mesh = o3d.geometry.TriangleMesh()
    stacked_mesh.vertices = o3d.utility.Vector3dVector(vertices)
    stacked_mesh.triangles = o3d.utility.Vector3iVector(combined_triangles)

    return stacked_mesh


def load_camera_params(vis, camera_params_path):
    if os.path.exists(camera_params_path):
        with open(camera_params_path, "r") as f:
            camera_params = json.load(f)
            ctr = vis.get_view_control()
            parameters = o3d.camera.PinholeCameraParameters()
            parameters.extrinsic = camera_params["extrinsic"]
            parameters.intrinsic = o3d.camera.PinholeCameraIntrinsic(
                width=camera_params["intrinsic"]["width"],
                height=camera_params["intrinsic"]["height"],
                fx=camera_params["intrinsic"]["fx"],
                fy=camera_params["intrinsic"]["fy"],
                cx=camera_params["intrinsic"]["cx"],
                cy=camera_params["intrinsic"]["cy"],
            )
            ctr.convert_from_pinhole_camera_parameters(parameters, True)
        print("Camera parameters loaded.")
        return camera_params

def check_normals(mesh):
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()

def add_vis_both_side(vis, mesh, surface_colors):
    obj1 = o3d.geometry.TriangleMesh()
    obj1.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.points))
    obj1.triangles = o3d.utility.Vector3iVector(np.asarray(mesh.faces, dtype=np.int32).reshape(-1, 4)[:, 1:][:, ::-1])
    obj1.vertex_colors = o3d.utility.Vector3dVector(surface_colors)
    check_normals(obj1)
    
    vis.add_geometry(obj1)

    obj2 = o3d.geometry.TriangleMesh()
    obj2.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.points))
    obj2.triangles = o3d.utility.Vector3iVector(np.asarray(mesh.faces, dtype=np.int32).reshape(-1, 4)[:, 1:])
    obj2.vertex_colors = o3d.utility.Vector3dVector(surface_colors)
    check_normals(obj2)
    
    vis.add_geometry(obj2)

def apply_force_color(mesh, contact_forces):
    if isinstance(mesh, o3d.geometry.PointCloud):
        if len(mesh.points) != len(contact_forces):
            print(f"Warning: Mismatch between mesh points ({len(mesh.points)}) and contact forces ({len(contact_forces)})")
            return
        
        force_magnitudes = np.linalg.norm(contact_forces, axis=1)
        max_force = np.max(force_magnitudes)

        if max_force == 0:  
            colors = np.zeros((len(mesh.points), 3))  # default should be purple
        else:
            colors = plt.cm.viridis(force_magnitudes / max_force)[:, :3]

        mesh.colors = o3d.utility.Vector3dVector(colors)

def vis_vtk_force_distribution(vis, points, cells, contact_forces, cmap=plt.cm.viridis):
    cells = cells.astype(np.int32)
    if np.max(cells) >= points.shape[0]:
        raise ValueError(f"[ERROR] cells index {np.max(cells)} >= points count {points.shape[0]}")
    if np.isnan(points).any() or np.isinf(points).any():
        raise ValueError("[ERROR] NaN/Inf found in points")
    if np.isnan(contact_forces).any() or np.isinf(contact_forces).any():
        raise ValueError("[ERROR] NaN/Inf found in contact forces")

    force_magnitudes = np.linalg.norm(contact_forces, axis=1)
    colors = cmap(force_magnitudes)[:, :3]
    num_cells = cells.shape[0]  
    cells_vtk = np.hstack([np.full((num_cells, 1), 4), cells]).astype(np.int32)
    cells_vtk = cells_vtk.flatten()
    cell_types = np.full(num_cells, pv.CellType.TETRA, dtype=np.uint8)
    tet_mesh = pv.UnstructuredGrid(cells_vtk, cell_types, points)
    surface = tet_mesh.extract_surface()
    surface_point_indices = surface.point_data["vtkOriginalPointIds"]

    surface_colors = np.zeros((surface.n_points, colors.shape[1]), dtype=float)
    for i, orig_index in enumerate(surface_point_indices):
        surface_colors[i] = colors[orig_index]

    add_vis_both_side(vis, surface, surface_colors)


def vis_obj_force_distribution(vis, vertices, faces, contacts):
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(faces)
    apply_force_color(mesh, contacts)
    vis.add_geometry(mesh)

def visualize_mesh_sequence_interactive(
    h5_path: str,
    pose_id: int,
    object_type: str,
    start_index: int = 1,
    end_index: int = 10,
    frame_delay: float = 0.5,
    interaction_delay: float = 0.01,
    default_color: np.ndarray = np.random.rand(1, 3),
    video_folder: str = "",
    record_fps: int = 24,
    width: int = 1920,
    height: int = 1080,
    camera_params_path: Optional[str] = None,
):
    os.makedirs(video_folder, exist_ok=True)
    with h5py.File(h5_path, "r") as h5_file:       
        finray_joint_states = h5_file["trajectories/finray_joint_states"][pose_id][:]
        finray_tf_states = h5_file["trajectories/finray_tf_states"][pose_id][:]

        leftfinger_cells = h5_file["trajectories/leftfinger_cells"][pose_id][:]
        rightfinger_cells = h5_file["trajectories/rightfinger_cells"][pose_id][:]
        leftfinger_points = h5_file["trajectories/leftfinger_points"][pose_id][:]
        rightfinger_points = h5_file["trajectories/rightfinger_points"][pose_id][:]

        if h5_file["object_config/object_type"][()] == b"rigid":
            object_vertices = h5_file["trajectories/object_vertices"][pose_id][:]
            object_faces = h5_file["trajectories/object_faces"][pose_id][:]
        else:
            object_cells = h5_file["trajectories/object_cells"][:]
            object_points = h5_file["rajectories/object_points"][pose_id][:]
        frame_count = h5_file["trajectories/left_f_collisions"][pose_id][:].shape[0]  

        end_index = min(end_index, frame_count - 1)

        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(width=width, height=height)
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        opt.mesh_show_wireframe = True

        writer = None

        h5_filename = os.path.splitext(os.path.basename(h5_path))[0]
        video_filename = f"{h5_filename}_{pose_id}.mp4"
        video_path = os.path.join(video_folder, video_filename)
        writer = imageio.get_writer(video_path, fps=record_fps)

        if camera_params_path:
            load_camera_params(vis, camera_params_path)

        finray_urdf = URDF.load(
            "./models/franka_description/panda_finray_with_mount.urdf",
            load_meshes=True,
            force_mesh=True,
            mesh_dir="./models/franka_description/meshes"
        )

        for i in trange(start_index, end_index + 1):
            vis.clear_geometries()
            joint_state = finray_joint_states[i]  # shape: (3,)
            joint_dict = {
                "panda_grasptarget_hand": joint_state[0],
                "panda_finger_joint1": joint_state[1],  # left
                "panda_finger_joint2": joint_state[2],  # right
            }
            finray_urdf.update_cfg(joint_dict)

            for link in ['hand.obj', 'finray_finger_L_mount_low.obj', 'finray_finger_R_mount_low.obj']:
                trimesh_obj = finray_urdf.scene.geometry.get(link).copy()
                transform = finray_urdf.scene.graph.get(link)[0]
                trimesh_obj.apply_transform(transform)

                tf_matrix = finray_tf_states[i] 
                vertices_homo = np.hstack([trimesh_obj.vertices, np.ones((trimesh_obj.vertices.shape[0], 1))])
                transformed_vertices = (vertices_homo @ tf_matrix.T)[:, :3]

                mesh = o3d.geometry.TriangleMesh()
                mesh.vertices = o3d.utility.Vector3dVector(transformed_vertices)
                mesh.triangles = o3d.utility.Vector3iVector(trimesh_obj.faces)
                mesh.compute_vertex_normals()
                vis.add_geometry(mesh)

            left_f_contacts = h5_file["trajectories/left_f_collisions"][pose_id, i] + h5_file["trajectories/left_f_frictions"][pose_id, i]
            right_f_contacts = h5_file["trajectories/right_f_collisions"][pose_id, i] + h5_file["trajectories/right_f_frictions"][pose_id, i]
            object_contacts = h5_file["trajectories/object_f_collisions"][pose_id, i] + h5_file["trajectories/object_f_frictions"][pose_id, i]


            vis_vtk_force_distribution(vis, leftfinger_points[i], leftfinger_cells, left_f_contacts)
            vis_vtk_force_distribution(vis, rightfinger_points[i], rightfinger_cells, right_f_contacts)

            if "rigid" in h5_path:
                vis_obj_force_distribution(vis, object_vertices[i], object_faces[i], object_contacts)
            else:
                vis_vtk_force_distribution(vis, object_points[i], object_cells, object_contacts)

            vis.poll_events()
            vis.update_renderer()

            image = vis.capture_screen_float_buffer(True)
            image = np.asarray(image)
            writer.append_data((image * 255).astype(np.uint8))

            time.sleep(frame_delay)

        writer.close()

        vis.destroy_window()



if __name__ == "__main__":
    args = parser.parse_args()
    start_index = 0
    end_index = 100

    visualize_mesh_sequence_interactive(
        h5_path=args.h5_path,
        pose_id=args.pose_id,
        start_index=start_index,
        end_index=end_index,
        frame_delay=1.0 / args.fps if args.fps > 0 else 0,
        interaction_delay=0.001, 
        video_folder=args.video_folder,
        record_fps=args.record_fps,
        camera_params_path=args.camera,
        width=args.width,
        height=args.height,
        default_color=None,
        object_type=args.object_type
    )