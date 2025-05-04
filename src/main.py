import cv2
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
from Camera import Camera, CameraParameters
from Scene import Scene, SceneMetaData
from copy import deepcopy
import json


if __name__=="__main__":

    print("-------Start box pose estimation------")
    dataset_folder = "/home/data/original"
    results_folder = "/home/data/results"
    if not os.path.exists(results_folder):
        os.makedirs(results_folder)

    color_src = os.path.join(dataset_folder, "one-box.color.npdata.npy")
    depth_src = os.path.join(dataset_folder, "one-box.depth.npdata.npy")

    intrinsic_src = os.path.join(dataset_folder, "intrinsics.npy")
    extrinsic_src = os.path.join(dataset_folder, "extrinsics.npy")

    print("--------Read camera parameters----------")
    camera_parameters = CameraParameters.read_parameters_from_src(intrinsic_src, extrinsic_src, color_src)
    camera = Camera(camera_parameters)
    scene_meta = SceneMetaData(pallet_height=0.12)
    scene = Scene(scene_meta)
    
    print("--------Create a point cloud from RGB and depth--------")
    pcd = camera.get_pcd(color_src, depth_src)
    point_cloud_path = os.path.join(results_folder, "pcd.ply")
    o3d.io.write_point_cloud(point_cloud_path, pcd)   
    print(f"Save the point cloud in {point_cloud_path}")

    print("-------Start Detection---------")
    print("Detect the Floor")
    floor = scene.get_floor(deepcopy(pcd))
    floor_path = os.path.join(results_folder, "floor.ply")
    floor_bbox = o3d.geometry.LineSet.create_from_oriented_bounding_box(floor)
    o3d.io.write_line_set(floor_path, floor_bbox)
    print(f"Save the Floor in {floor_path}")

    print("Detect the surface of the box then get the whole box")
    box_surface, box = scene.get_box(deepcopy(pcd))

    box_surface_path = os.path.join(results_folder, "box_surface.ply")
    box_surface_bbox = o3d.geometry.LineSet.create_from_oriented_bounding_box(box_surface)
    o3d.io.write_line_set(box_surface_path, box_surface_bbox)
    print(f"save the box surface in {box_surface_path}")

    box_mesh_path = os.path.join(results_folder, "box.ply")
    box_mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(box)
    box_mesh.paint_uniform_color([0.5, 0.5, 0.5])
    o3d.io.write_triangle_mesh(box_mesh_path, box_mesh)
    print(f"Save the box in {box_mesh_path}")

    # box pose relative to the world
    T_B_W = np.zeros((4, 4))
    T_B_W[:3, :3] = box.R
    T_B_W[:3, 3] = box.center
    T_B_W[3, 3] = 1
    # world relative to the camera. inverse the extrinisc
    T_W_C = np.linalg.inv(camera_parameters.extrinisc)

    # the pose of the object relative to the camera frame
    pose = T_W_C @ T_B_W
    R = pose[:3, :3]
    print("--------The pose of the object relative to the camera frame----------")
    print(pose)
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-6)

    pose_path = os.path.join(results_folder, "pose.json")
    print(f"Save the pose in{pose_path}")
    with open(pose_path, "w") as f:
        json.dump(pose.tolist(), f)

