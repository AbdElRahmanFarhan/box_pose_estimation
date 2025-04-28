import cv2
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os


    

data_folder = "/home/data"

color_np = np.load(os.path.join(data_folder, "original/one-box.color.npdata.npy"))
depth_np = np.load(os.path.join(data_folder, "original/one-box.depth.npdata.npy"))

color_np = np.uint8(color_np)
depth_np = np.float32(depth_np)


cv2.imwrite(os.path.join(data_folder, "test/color.png"), color_np)
plt.imsave(os.path.join(data_folder, "test/depth.png"), depth_np, vmax=3.0, vmin=0, cmap='grey')

h = depth_np.shape[0]
w = depth_np.shape[1]

intrinsic_mat = np.load(os.path.join(data_folder, "original/intrinsics.npy"))
extrinsic_mat = np.load(os.path.join(data_folder, "original/extrinsics.npy"))
print(extrinsic_mat)

color_full = np.repeat(color_np[:, :, np.newaxis], 3, axis=2)
color_o3d = o3d.geometry.Image(color_full)

depth_o3d = o3d.geometry.Image(depth_np)

rgbd_o3d = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d, depth_scale=1)

intrisic_o3d = o3d.camera.PinholeCameraIntrinsic(width=w, height=h, intrinsic_matrix=intrinsic_mat)

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_o3d, intrisic_o3d, extrinsic_mat)
pcd.estimate_normals()
# o3d.io.write_point_cloud(os.path.join(data_folder, "test/pointcloud.pcd"), pcd)
print(np.max(np.array(pcd.points)))
print(len(pcd.points))

oboxes = pcd.detect_planar_patches(
    normal_variance_threshold_deg=60,
    coplanarity_deg=75,
    outlier_ratio=0.75,
    min_plane_edge_length=0,
    min_num_points=0,
    search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

print("Detected {} patches".format(len(oboxes)))
floor = max(oboxes, key=lambda obox: obox.volume())
mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(floor)
o3d.io.write_triangle_mesh(os.path.join(data_folder, "test/floor.ply"), mesh)

horizontal_planes = []

# floor_z = floor.R[:, 2]
# for obox in oboxes:
#     z_axis = obox.R[:, 2]
#     similarity = np.dot(z_axis, floor_z)
#     if 1 - abs(similarity) <= 1e-3:
#         mesh += o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox)
# o3d.io.write_triangle_mesh(os.path.join(data_folder, "test/all.ply"), mesh)



horizontal_planes = []

for obox in oboxes:
    T = obox.R @ np.linalg.inv(floor.R)
    print(T[:, 2])
    if 1 - np.abs(np.dot(T[:, 2], np.array([0., 0., 1.]))) <= 1e-1:
        mesh += o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox)
# o3d.io.write_triangle_mesh(os.path.join(data_folder, "test/all.ply"), mesh)