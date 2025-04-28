import cv2
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
from Camera import Camera, CameraParameters
from Scene import Scene, SceneMetaData
from copy import deepcopy

if __name__=="__main__":

    dataset_folder = "/home/data/original"
    results_folder = "/home/data/test"

    color_src = os.path.join(dataset_folder, "one-box.color.npdata.npy")
    depth_src = os.path.join(dataset_folder, "one-box.depth.npdata.npy")

    intrinsic_src = os.path.join(dataset_folder, "intrinsics.npy")
    extrinsic_src = os.path.join(dataset_folder, "extrinsics.npy")

    camera_parameters = CameraParameters.read_parameters_from_src(intrinsic_src, extrinsic_src, color_src)
    camera = Camera(camera_parameters)
    scene_meta = SceneMetaData(pallet_height=0.1)
    scene = Scene(scene_meta)
    pcd = camera.get_pcd(color_src, depth_src)
    box = scene.get_box(deepcopy(pcd))
    mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(box)
    floor = scene.get_floor(deepcopy(pcd))
    mesh += o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(floor)
    o3d.io.write_triangle_mesh(os.path.join(results_folder, "all.ply"), mesh)