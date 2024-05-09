import laspy
import numpy as np

# Read the LAS file using laspy.read
las = laspy.read("test.las")

# Access spatial coordinates (x, y, z) and color information (red, green, blue)
x = las.x
y = las.y
z = las.z
r = las.red
g = las.green
b = las.blue

# Load the original PCD data
pcd_data = np.loadtxt("test.pcd", skiprows=11)  # Skip the header

# Extract spatial coordinates (x, y, z) and color information (rgb) from the PCD data
pcd_x = pcd_data[:, 0]
pcd_y = pcd_data[:, 1]
pcd_z = pcd_data[:, 2]
pcd_rgb = pcd_data[:, 3].astype(np.uint32)

# Compare spatial coordinates
print("Comparison of spatial coordinates:")
print("x coordinates match:", np.allclose(x, pcd_x))
print("y coordinates match:", np.allclose(y, pcd_y))
print("z coordinates match:", np.allclose(z, pcd_z))

# Convert RGB integer to separate channels
pcd_r = (pcd_rgb >> 16) & 255
pcd_g = (pcd_rgb >> 8) & 255
pcd_b = pcd_rgb & 255

# Compare color information (r, g, b)
print("Comparison of color information:")
print("red channel values match:", np.array_equal(r, pcd_r))
print("green channel values match:", np.array_equal(g, pcd_g))
print("blue channel values match:", np.array_equal(b, pcd_b))
