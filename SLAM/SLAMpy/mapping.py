import cv2
import numpy as np
import pcl

# Dummy depth image (replace this with actual depth data from your RGB-D camera)
depth_image = cv2.imread('depth_image.png', cv2.IMREAD_GRAYSCALE).astype(np.float32)

# Dummy camera intrinsic parameters (replace these with your camera intrinsics)
fx, fy = 500, 500  # focal length in pixels
cx, cy = 320, 240  # principal point in pixels

# Create 3D point cloud from depth image
height, width = depth_image.shape
v, u = np.indices((height, width))
z = depth_image / 1000.0  # converting depth from millimeters to meters
x = (u - cx) * z / fx
y = (v - cy) * z / fy

# Mask out invalid depth values (typically represented as 0 in the depth image)
invalid_mask = (depth_image == 0)
x[invalid_mask] = y[invalid_mask] = z[invalid_mask] = np.nan

# Stack the X, Y, and Z coordinates to form the point cloud
point_cloud = np.dstack((x, y, z)).reshape(-1, 3)

# Convert the point cloud to a PCL PointCloud object
pcl_cloud = pcl.PointCloud()
pcl_cloud.from_array(point_cloud.astype(np.float32))

# Visualization (optional)
visual = pcl.pcl_visualization.CloudViewing()
visual.ShowMonochromeCloud(pcl_cloud)
visual.Spin()
