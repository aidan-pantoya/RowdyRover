import laspy
import numpy as np
import matplotlib.pyplot as plt

# Read the LAS file using laspy.read
las = laspy.read("test.las")

# Access point cloud data
x = las.x
y = las.y
z = las.z

# Plot point cloud data with adjusted scale
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z, s=0.1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Point Cloud Visualization')
ax.set_xlim(np.min(x), np.max(x))
ax.set_ylim(np.min(y), np.max(y))
ax.set_zlim(np.min(z), np.max(z))
plt.show()
