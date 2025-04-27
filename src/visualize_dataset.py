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

intrinsic_mat = np.load(os.path.join(data_folder, "original/intrinsics.npy")).astype(np.float32)
print(intrinsic_mat)
extrinsic_mat = np.load(os.path.join(data_folder, "original/intrinsics.npy")).astype(np.float32)
print(extrinsic_mat)


color_o3d = o3d.geometry.Image(np.repeat(color_np[:, :, np.newaxis], 3, axis=2))

depth_o3d = o3d.geometry.Image(depth_np*1000)

rgbd_o3d = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d)

intrisic_o3d = o3d.camera.PinholeCameraIntrinsic(width=w, height=h, intrinsic_matrix=intrinsic_mat)

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_o3d, intrisic_o3d, extrinsic_mat)

o3d.io.write_point_cloud(os.path.join(data_folder, "test/pointcloud.pcd"), pcd)