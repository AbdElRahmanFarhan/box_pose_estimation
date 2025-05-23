import numpy as np
import open3d as o3d
from dataclasses import dataclass
from typing import List, Tuple
from copy import deepcopy

@dataclass
class SceneMetaData:
    pallet_height: float
    # TODO: if I do not have many assumptions I will drop this class

class Scene:
    def __init__(self, scene_meta: SceneMetaData) -> None:
        self.scene_meta = scene_meta

    def detect_planes(self, pcd: o3d.geometry.PointCloud) -> List[o3d.geometry.OrientedBoundingBox]:
        pcd.estimate_normals()
        oboxes = pcd.detect_planar_patches(
        normal_variance_threshold_deg=60,
        coplanarity_deg=75,
        outlier_ratio=0.75,
        min_plane_edge_length=0,
        min_num_points=0,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
        # TODO: the parameters are hardcoded for now
        return oboxes

    def get_floor(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.OrientedBoundingBox:
        planes = self.detect_planes(pcd)
        floor = max(planes, key=lambda x: x.volume())
        return floor
    
    def is_parallel(self, plane_1: o3d.geometry.OrientedBoundingBox, plane_2: o3d.geometry.OrientedBoundingBox):
        z_axis_1 = plane_1.R[:, -1]
        z_axis_2 = plane_2.R[:, -1]
        dot = np.dot(z_axis_1, z_axis_2)
        if 1 - abs(dot) <= 1e-1:
            return True
        else:
            return False

    def get_dist_between_planes(self, ref: o3d.geometry.OrientedBoundingBox, var: o3d.geometry.OrientedBoundingBox) -> float:
        diff = var.center - ref.center
        normal = ref.R[:, 2]
        dist = np.dot(diff, normal) - ref.extent[-1]/2. - var.extent[-1]/2. 
        return dist

    def calculate_box(self, box_surface: o3d.geometry.OrientedBoundingBox, dist: float) -> o3d.geometry.OrientedBoundingBox:
        dist -= self.scene_meta.pallet_height
        box_extent = np.zeros((3,))
        box_extent[:-1] = box_surface.extent[:-1]
        box_extent[-1] = dist
        # define the box center relative to the surface center
        box_center = np.zeros((3,))
        box_center[-1] = -dist/2.
        # Transform the center
        box_center = box_surface.R @ box_center + box_surface.center
        box = o3d.geometry.OrientedBoundingBox(center=box_center, R=box_surface.R, extent=box_extent)        
        return box


    def get_box(self, pcd: o3d.geometry.PointCloud) -> Tuple[o3d.geometry.OrientedBoundingBox, o3d.geometry.OrientedBoundingBox]:
        planes = self.detect_planes(pcd)
        floor = self.get_floor(pcd)
        horizontal_planes = [deepcopy(plane) for plane in planes if self.is_parallel(floor, plane)]
        dists = [self.get_dist_between_planes(floor, plane) for plane in horizontal_planes]
        box_idx = np.argmax(dists)
        box_surface = horizontal_planes[box_idx]
        dist = dists[box_idx] 
        box = self.calculate_box(box_surface, dist)
        return box_surface, box

        



