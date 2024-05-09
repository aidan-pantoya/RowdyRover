import laspy
import numpy as np
import os
import boto3
import subprocess
from botocore.exceptions import ClientError

# It's assumed that AWS credentials are set in the environment or AWS credentials file
session = boto3.Session(
    aws_access_key_id='',
    aws_secret_access_key='',
    region_name=''
)
s3 = session.resource('s3')

def create_empty_file(file_path):
    with open(file_path, 'w') as f:
        pass

def upload_directory(directory_path, bucket_name):
    print("Uploading to bucket...")
    s3_client = session.client('s3')

    # Create an empty file to represent the root folder
    root_file_path = os.path.join(directory_path, "__root__")
    create_empty_file(root_file_path)

    # Upload the empty file to represent the root folder
    s3_key = directory_path.strip('/') + '/'
    s3_client.upload_file(root_file_path, bucket_name, s3_key)

    # Remove the temporary empty file
    os.remove(root_file_path)

    # Iterate through the directory structure and upload files
    for root, dirs, files in os.walk(directory_path):
        for file in files:
            file_path = os.path.join(root, file)
            # The key is the file path relative to the directory_path
            s3_key = os.path.relpath(file_path, directory_path)
            # Replace backslashes with forward slashes for cross-platform compatibility
            s3_key = s3_key.replace('\\', '/')
            # Upload file to S3 preserving directory structure
            s3_key = s3_key.replace(os.path.sep, '/')
            s3_key = s3_key if not s3_key.startswith('/') else s3_key[1:]
            s3_client.upload_file(file_path, bucket_name, f"{directory_path.strip('/')}/{s3_key}")
    print("Uploaded to bucket")

file = input("Enter filename (excluding extension): ")
#file = "ecs_ent_right"
# Load PCD file
with open(f"{file}.pcd", "r") as f:
    lines = f.readlines()
    point_data = lines[11:]

# Initialize lists for point data
x_values, y_values, z_values, rgb_values = [], [], [], []

# Extract values from point data
for line in point_data:
    values = line.split()
    x_values.append(float(values[0]))
    y_values.append(float(values[1]))
    z_values.append(float(values[2]))
    rgb = int(values[3])
    r, g, b = (rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255
    rgb_values.append([r, g, b])

# Convert lists to NumPy arrays
x_array, y_array, z_array, rgb_array = map(np.array, (x_values, y_values, z_values, rgb_values))

# Calculate bounding box and scale
min_coords = np.minimum.reduce([x_array, y_array, z_array])
max_coords = np.maximum.reduce([x_array, y_array, z_array])
bbox_size = np.maximum(max_coords - min_coords, 1e-4)
scale = bbox_size / 65535

# Check for zero-size axes
if np.any(bbox_size <= 0):
    raise ValueError("Invalid bounding box: at least one axis has a size of zero")

# Create a new LAS object with the updated header
las = laspy.create(point_format=7, file_version="1.4")
las.header.offsets = min_coords
las.header.scales = scale

# Assign the point data to the LAS object
las.x, las.y, las.z = x_array, y_array, z_array
las.red, las.green, las.blue = rgb_array.T

# Write data to LAS file
las.write(f"{file}.las")

# Output sample of extracted point values
print("Sample of extracted x, y, and z values:")
for i in range(5):
    print(f"Point {i+1}: x={x_values[i]}, y={y_values[i]}, z={z_values[i]}")

# Corrected subprocess call with f-string formatting
process = subprocess.Popen([f'PotreeConverter.exe', f'{file}.las', '-o', 'outputFolder', '--generate-page', 'PointCloud'], stdout=subprocess.PIPE, text=True)

# Read the output
output, errors = process.communicate()

# Print the output
print(output)

directory_to_upload = 'outputFolder'
bucket_name = 'mypointclouds'

upload_directory(directory_to_upload, bucket_name)