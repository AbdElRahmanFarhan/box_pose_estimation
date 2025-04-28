from __future__ import annotations
import open3d as o3d
from dataclasses import dataclass
import numpy as np
from typing import Tuple
import os

@dataclass
class CameraParameters:
    intrinsic: o3d.camera.PinholeCameraIntrinsic
    extrinisc: np.ndarray[np.float64]
    W: int
    H: int

    @classmethod
    def read_parameters_from_src(cls, intrinsic_src: os.PathLike, extrinsic_src: os.PathLike, color_src: os.PathLike) -> CameraParameters:
        intrinsic_mat = np.load(intrinsic_src)
        H, W = np.load(color_src).shape
        intrinsic = o3d.camera.PinholeCameraIntrinsic(width=W, height=H, intrinsic_matrix=intrinsic_mat)
        extrinisc = np.load(extrinsic_src)
        extrinisc[:3, 3] *= 1e-3 # convert to meters
        return cls(intrinsic=intrinsic, extrinisc=extrinisc, W=W, H=H)

class Camera:
    def __init__(self, camera_parameters: CameraParameters) -> None:
        self.camera_parameters = camera_parameters

    
    def get_rgbd(self, color_src: os.PathLike, depth_src: os.PathLike) -> o3d.geometry.RGBDImage:
        color_np = np.load(color_src).astype(np.uint8)
        color_full = np.repeat(color_np[:, :, np.newaxis], 3, axis=2)
        color_o3d = o3d.geometry.Image(color_full)

        depth_np = np.load(depth_src).astype(np.float32)
        depth_o3d = o3d.geometry.Image(depth_np)

        rgbd_o3d = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d, depth_scale=1)
        return rgbd_o3d

    def get_pcd(self, color_src: os.PathLike, depth_src: os.PathLike) -> o3d.geometry.PointCloud:
        rgbd_o3d = self.get_rgbd(color_src, depth_src)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_o3d, self.camera_parameters.intrinsic, self.camera_parameters.extrinisc)
        return pcd
