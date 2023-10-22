import sys
import os
import bpy
import numpy as np
import mathutils
import math
import cv2
from mathutils import Vector

blend_dir = os.path.dirname(bpy.data.filepath)
if blend_dir not in sys.path:
   sys.path.append(blend_dir)
   
# Define your desired directory manually
output_directory = "C:/Users/apant/OneDrive/Desktop/Capstone/SimImages/"
   
# import PathFind
import ImageStitch
import CreateField

def create_rover_body(location=(0,0,0)):
    bpy.ops.mesh.primitive_cube_add(size=2, location=location)
    rover_body = bpy.context.active_object
    rover_body.name = "Rover_Body"
    return rover_body


def resize_image_to_screen(image, screen_width=600, screen_height = 300):
    img_height, img_width = image.shape[:2]
    scale = min(screen_width / img_width, screen_height / img_height)
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

def find_path_center(image_path):
    width = 600

    # Read the image
    img = cv2.imread(image_path)
    img = resize_image_to_screen(img, screen_width=width)
    
    ks = int(width * 0.02)
    if ks % 2 == 0:
        ks += 1
    img = cv2.GaussianBlur(img, (ks, ks), 0)

       # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Convert the RGB color to HSV
    green_rgb = np.uint8([[[33, 58, 33]]])
    green_hsv = cv2.cvtColor(green_rgb, cv2.COLOR_RGB2HSV)[0,0,:]

    # Define range for green color
    h_range = 40  
    s_range = 50
    v_range = 40  

    lower_green = np.array([green_hsv[0] - h_range, green_hsv[1] - int(s_range/2), green_hsv[2] - v_range])
    upper_green = np.array([green_hsv[0] + h_range, green_hsv[1] + s_range, green_hsv[2] + v_range])

    mask = cv2.inRange(hsv, lower_green, upper_green)


    # Use the mask to extract the green regions from the original image
    green_img = cv2.bitwise_and(img, img, mask=mask)
    
    # Convert the green regions to grayscale for contour detection
    gray = cv2.cvtColor(green_img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    _, thresh = cv2.threshold(blurred, 47, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        contour_area = cv2.contourArea(contour)
        if 1000 < contour_area < int(width * 300 * 0.1):
            nwimg = cv2.drawContours(img.copy(), [contour], -1, (0, 255, 0), 2)
            
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                continue

            nwimg = cv2.circle(nwimg, (cX, cY), 5, (0, 0, 255), -1)
            cv2.imshow("Image with path and center", nwimg)
            cv2.waitKey(500)
            cv2.destroyAllWindows()

    try:
        r_o_l = int(width / 2) - cX
        r_o_l = r_o_l * 90 / width
        if r_o_l > 0:
            print("Go Left degs:", r_o_l)
        elif r_o_l < 0:
            print('Go Right degs:', r_o_l)
        else:
            print("Go Straight")
    except:
        print("Turning around")
        r_o_l = 1000
    return r_o_l


def create_rover_with_cameras(rover_location=(0,0,0)):
        
    def create_camera(name, location, rotation):
        bpy.ops.object.camera_add(location=location, rotation=rotation)
        camera = bpy.context.active_object
        camera.name = name
        camera.data.lens = 24  # You can adjust the focal length if needed
        camera.data.sensor_width = 32  # You can adjust the sensor width if needed
        camera_data = bpy.data.objects[str(name)].data

        camera_data.clip_start = 0.1  
        camera_data.clip_end = 1000.0 
           
        return camera

    rover = create_rover_body(rover_location)
    
    separation_distance = 0 
    
    cam1_location = (rover_location[0] - separation_distance/2, rover_location[1], rover_location[2] + 1.5)
    cam2_location = (rover_location[0] + separation_distance/2, rover_location[1], rover_location[2] + 1.5)
    
    # Adjusting rotations to point the cameras outward for a total of 180 degrees.
    cam1_rotation = (1.5708, 0, 0.59)   # 45 degrees in z-axis 
    cam2_rotation = (1.5708, 0, -0.59)  # -45 degrees in z-axis
    
    camera1 = create_camera("Camera_POV1", cam1_location, cam1_rotation)
    camera2 = create_camera("Camera_POV2", cam2_location, cam2_rotation)
    
    camera1.parent = rover
    camera2.parent = rover

def render_camera_image(camera_name, output_filepath):
    bpy.context.scene.camera = bpy.data.objects[camera_name]
    
    bpy.context.scene.render.image_settings.file_format = 'PNG'
    bpy.context.scene.render.filepath = output_filepath
    
    bpy.ops.render.render(write_still=True)

def update_rover_location(rover_name, movement):
    rover = bpy.data.objects[rover_name]
    rover.location.x += movement[0]
    rover.location.y += movement[1]
    rover.location.z += movement[2]
    
    
def rotate_and_move_object(obj_name, x_degrees=0, y_degrees=0, z_degrees=0, distance=1.0):
    # Ensure the object exists
    if obj_name not in bpy.data.objects:
        print(f"No object named {obj_name} found.")
        return

    obj = bpy.data.objects[obj_name]
    
    # Convert degrees to radians, as Blender's rotation function uses radians
    x_radians = math.radians(x_degrees)
    y_radians = math.radians(y_degrees)
    z_radians = math.radians(z_degrees)

    # Apply rotation
    obj.rotation_euler[0] += x_radians
    obj.rotation_euler[1] += y_radians
    obj.rotation_euler[2] += z_radians

    # Calculate the movement vector based on the object's local coordinate system
    move_vector = obj.matrix_world.to_3x3() @ mathutils.Vector((0, distance, 0))

    # Apply movement
    obj.location += move_vector


def turn_obj_around(obj_name, turn_side, dist=5.0):
    obj = bpy.data.objects.get(obj_name)
    if not obj:
        print(f"Object {obj_name} not found.")
        return
    
    # Rotate the object by 180 degrees around its Z-axis
    obj.rotation_euler.z += math.radians(180)
    
    # Update the object's transformation to ensure the rotation is applied
    bpy.context.view_layer.update()

    # Calculate the move direction based on the turn_side
    if turn_side == 0:  # 0 is move right
        move_direction = obj.matrix_world @ Vector((dist, 2*dist, 0.0)) - obj.matrix_world @ Vector((0.0, 0.0, 0.0))
    else:  # 1 is move left
        move_direction = obj.matrix_world @ Vector((-dist, 2*dist, 0.0)) - obj.matrix_world @ Vector((0.0, 0.0, 0.0))
    
    # Update the object's location
    obj.location += move_direction

CreateField.create_field()

Rover_Locations = []
create_rover_with_cameras(rover_location=(0,0,0))
rotate_and_move_object("Rover_Body", x_degrees=0, y_degrees=0, z_degrees=-45, distance=0.0)

ImageRange = range(1,500)
turnSide = 0 # 0 is turn left, 1 is turn right.

for cntn in ImageRange:
    cnt = f"{cntn:07}"
    rover = bpy.data.objects.get("Rover_Body")
    rx,ry = round(rover.location.x,4), round(rover.location.y,4)
    camera1_img = os.path.join(output_directory, f"{cnt}_camera1_{rx}_{ry}.png")
    camera2_img = os.path.join(output_directory, f"{cnt}_camera2_{rx}_{ry}.png")

    concatenated_img_path = os.path.join(output_directory, f"{cnt}_concatenated_{rx}_{ry}.png")

    render_camera_image("Camera_POV1", camera1_img)
    render_camera_image("Camera_POV2", camera2_img)
    
    ImageStitch.concatenate_images(camera1_img, camera2_img, concatenated_img_path)
    
    # movement = PathFind.handle_input(concatenated_img_path)
    movement = find_path_center(concatenated_img_path)

    if movement != 1000:
        rotate_and_move_object("Rover_Body", x_degrees=0, y_degrees=0, z_degrees=movement, distance=1.0)
    else:
        turn_obj_around("Rover_Body",turnSide)
        if turnSide == 0:
            turnSide = 1
        else: 
            turnSide = 0
    Rover_Locations.append((rx,ry))

print("All Rover Locations:")
for xy in Rover_Locations:
    print(f"({xy[0]},{xy[1]})")