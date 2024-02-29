import sys
import os
import bpy
import numpy as np
import mathutils
import math
import cv2
from mathutils import Vector
import tensorflow as tf
from tensorflow.keras.preprocessing.image import load_img, img_to_array, array_to_img

blend_dir = os.path.dirname(bpy.data.filepath)
if blend_dir not in sys.path:
   sys.path.append(blend_dir)
   
# Define your desired directory manually
output_directory = "E:/BlenderML/Images/SimImages/"
output_directory2 = "E:/BlenderML/Images/MLMaskImages/"
output_directory3 = "E:/BlenderML/Images/ovRawImages/"
output_directory4 = "E:/BlenderML/Images/OverviewImages/"

# import PathFind
import ImageStitch
import CreateField

def create_rover_body(location=(0,0,0)):
    bpy.ops.mesh.primitive_cube_add(size=2, location=location)
    rover_body = bpy.context.active_object
    rover_body.name = "Rover_Body"
    return rover_body

def load_model_and_predict(image):
    model = tf.keras.models.load_model("E:/BlenderDataset/BLENDER_PDCT.h5")

#    img = cv2.imread(image_path)
#    img = cv2.resize(img, (224, 224))  # Resize the image to the size your model expects
#    img_array = img / 255.0  # Normalize the image
#    img_array = img_array[..., ::-1]  # Convert BGR to RGB
#    img_array = np.expand_dims(img_array, axis=0)  # Add a batch dimension

    prediction = model.predict(image)#img_array)

    return prediction


def predict_mask(raw_image_path, model, target_size):
    img = load_img(raw_image_path, target_size=target_size, color_mode='grayscale')
    img_array = img_to_array(img) / 255.0
    img_array_expanded = np.expand_dims(img_array, axis=0)

    predicted_mask = model.predict(img_array_expanded)

    return img_array, predicted_mask[0]

def resize_image_to_screen(image, screen_width=800, screen_height = 304):
    img_height, img_width = image.shape[:2]
    scale = min(screen_width / img_width, screen_height / img_height)
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

def find_path_center(image_path,output_directory,imgName):
    width = 800

    # Read the image
#    img = cv2.imread(image_path)
    
#    img = resize_image_to_screen(img, screen_width=width)
    
#    ks = int(width * 0.02)
#    if ks % 2 == 0:
#        ks += 1
#    img = cv2.GaussianBlur(img, (ks, ks), 0)

#       # Convert the image to HSV color space
#    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#    # Convert the RGB color to HSV
#    green_rgb = np.uint8([[[33, 58, 33]]])
#    green_hsv = cv2.cvtColor(green_rgb, cv2.COLOR_RGB2HSV)[0,0,:]

#    # Define range for green color
#    h_range = 40  
#    s_range = 60
#    v_range = 60  

#    lower_green = np.array([green_hsv[0] - h_range, green_hsv[1] - int(s_range/2), green_hsv[2] - v_range])
#    upper_green = np.array([green_hsv[0] + h_range, green_hsv[1] + s_range, green_hsv[2] + v_range])

#    mask = cv2.inRange(hsv, lower_green, upper_green)


#    # Use the mask to extract the green regions from the original image
#    green_img = cv2.bitwise_and(img, img, mask=mask)
#    
#    # Convert the green regions to grayscale for contour detection
#    gray = cv2.cvtColor(green_img, cv2.COLOR_BGR2GRAY)
#    cv2.imshow("Gray", gray)
#    cv2.waitKey(1000)
#    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
#    _, thresh = cv2.threshold(blurred, 47, 255, cv2.THRESH_BINARY_INV)

#    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    global blender_model
    model = blender_model
    
    raw_image, predicted_mask = predict_mask(image_path, model, (304, 800))
    

    if predicted_mask.dtype != np.uint8:
        # Convert the image to 8-bit format
        predicted_mask = (255 * predicted_mask).astype(np.uint8)


    _, thresh = cv2.threshold(predicted_mask, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    the_chosen_one = 0
    
    for contour in contours:
       contour_area = cv2.contourArea(contour)
       if contour_area > the_chosen_one:
           the_chosen_one = contour_area

    img = raw_image.copy()
    if the_chosen_one > 4000:
        for contour in contours:
           contour_area = cv2.contourArea(contour)
           if contour_area == the_chosen_one:
               mask = np.zeros_like(img)

               cv2.drawContours(mask, [contour], -1, (255, 255, 255), thickness=cv2.FILLED)

               if not os.path.exists(output_directory):
                   os.makedirs(output_directory)

               # Define the filename for the mask
               mask_filename = imgName

               # Save the mask as a .png image in the specified directory
               mask_path = os.path.join(output_directory, mask_filename)
               cv2.imwrite(mask_path, mask)

               # Find the centroid of the contour
               M = cv2.moments(contour)
               if M["m00"] != 0:
                   cX = int(M["m10"] / M["m00"])
                   cY = int(M["m01"] / M["m00"])
               else:
                   cX, cY = 0, 0

               # Draw a circle at the centroid on the original image
               img_with_circle = cv2.circle(img.copy(), (cX, cY), 5, (0, 0, 255), -1)

               # Show the image with the contour and centroid
               cv2.imshow("Image with path and center", img_with_circle)

               # Display the mask with the contour
               cv2.imshow("Mask", mask)

               cv2.waitKey(1)
#               cv2.destroyAllWindows()

               print(f"Mask saved to {mask_path}")
    else:
        print("NO PATH FOUND")
        return 1000


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


def rotate_the_object(obj_name, x_degrees=0, y_degrees=0, z_degrees=0):
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


def turn_obj_around(obj_name, turn_side, dist=1.0):
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
        move_direction = obj.matrix_world @ Vector((dist, dist, 0.0)) - obj.matrix_world @ Vector((0.0, 0.0, 0.0))
    else:  # 1 is move left
        move_direction = obj.matrix_world @ Vector((-dist, dist, 0.0)) - obj.matrix_world @ Vector((0.0, 0.0, 0.0))
    
    # Update the object's location
    obj.location += move_direction

def move_til_we_find_row(obj_name,turn_side,dist=1):
    obj = bpy.data.objects.get(obj_name)
    if not obj:
        print(f"Object {obj_name} not found.")
        return

    if turn_side == 0:  # 0 is move right
        move_direction = obj.matrix_world @ Vector((dist, dist, 0.0)) - obj.matrix_world @ Vector((0.0, 0.0, 0.0))
    else:  # 1 is move left
        move_direction = obj.matrix_world @ Vector((-dist, dist, 0.0)) - obj.matrix_world @ Vector((0.0, 0.0, 0.0))
    
    # Update the object's location
    obj.location += move_direction

def look_around_rover(RoverName,output_directory,output_directory2,output_directory3,cntn):
    cnt = f"{cntn:07}"
    rover = bpy.data.objects.get("Rover_Body")
    rx,ry = round(rover.location.x,4), round(rover.location.y,4)
    camera1_img = os.path.join(output_directory, f"{cnt}_camera1.png")
    camera2_img = os.path.join(output_directory, f"{cnt}_camera2.png")

    concatenated_img_path = os.path.join(output_directory3, f"{cnt}_concatenated_12_2023.png")

    render_camera_image("Camera_POV1", camera1_img)
    render_camera_image("Camera_POV2", camera2_img)
    
    ImageStitch.concatenate_images(camera1_img, camera2_img, concatenated_img_path)
    
    imgName = f"{cnt}_Mask_12_2023.png"
    movement = find_path_center(concatenated_img_path,output_directory2,imgName)
    return movement,rx,ry

def create_overview_camera(name="OverviewCam", location=(25,50,10), target_name='Rover_Body'):
    
    # Create a new camera
    bpy.ops.object.camera_add(location=location)
    camera = bpy.context.active_object
    camera.name = name
    camera.data.type = 'PERSP'

    # Ensure the target object exists
    target = bpy.data.objects.get(target_name)
    if not target:
        raise ValueError(f"No object found with the name {target_name}")

    # Create a track-to constraint to make the camera always look at the object
    constraint = camera.constraints.new(type='TRACK_TO')
    constraint.target = target
    constraint.track_axis = 'TRACK_NEGATIVE_Z'
    constraint.up_axis = 'UP_Y'

    return camera

def take_picture(directory,image_name,camera_name='OverviewCam'):
    camera = bpy.data.objects.get(camera_name)
    if not camera or camera.type != 'CAMERA':
        raise ValueError(f"No camera found with the name {camera_name}")

    # Set the active camera for the scene
    bpy.context.scene.camera = camera

    # Specify render settings
    bpy.context.scene.render.image_settings.file_format = 'PNG'
    filepath = os.path.join(directory, f"{image_name}.png")
    bpy.context.scene.render.filepath = filepath

    # Render the image
    bpy.ops.render.render(write_still=True)
    

blender_model = tf.keras.models.load_model("E:/BlenderDataset/BLENDER_PDCT.h5")

def runThroughField():
    CreateField.create_field()

    Rover_Locations = []
    create_rover_with_cameras(rover_location=(0,0,0))
    rotate_and_move_object("Rover_Body", x_degrees=0, y_degrees=0, z_degrees=-75, distance=0.0)

    cntn = 0
    turnSide = 0 # 0 is turn left, 1 is turn right.

    _ = create_overview_camera()
    runone = 0
    while cntn < 500:
        cntn +=1
        while runone == 0:
            movement,rx,ry= look_around_rover("RoverBody",output_directory,output_directory2,output_directory3,cntn)
            while movement == 1000:
                rotate_the_object("Rover_Body", x_degrees=0, y_degrees=0, z_degrees=15)
                movement,rx,ry= look_around_rover("RoverBody",output_directory,output_directory2,output_directory3,cntn)
            runone = 1
        movement,rx,ry= look_around_rover("RoverBody",output_directory,output_directory2,output_directory3,cntn)
        image_name = f"{cntn:07}_OverviewRover_12_2023.png"
        take_picture(output_directory4,image_name,camera_name='OverviewCam')
        if movement != 1000:
            rotate_and_move_object("Rover_Body", x_degrees=0, y_degrees=0, z_degrees=movement, distance=1.0)
        else:
            turn_obj_around("Rover_Body",turnSide)
            if turnSide == 0:
                while movement == 1000:
                    cntn += 1
                    move_til_we_find_row("Rover_Body",turnSide)
                    movement,rx,ry = look_around_rover("Rover_Body",output_directory,output_directory2,output_directory3,cntn)
                    image_name = f"{cntn:07}_OverviewRover_12_2023.png"
                    take_picture(output_directory4,image_name,camera_name='OverviewCam')
                turnSide = 1
            else: 
                while movement == 1000:
                    cntn += 1
                    move_til_we_find_row("Rover_Body",turnSide)
                    movement,rx,ry = look_around_rover("Rover_Body",output_directory,output_directory2,output_directory3,cntn)
                    image_name = f"{cntn:07}_OverviewRover_12_2023.png"
                    take_picture(output_directory4,image_name,camera_name='OverviewCam')
                turnSide = 0
        Rover_Locations.append((rx,ry))

    print("All Rover Locations:")
    for xy in Rover_Locations:
        print(f"({xy[0]},{xy[1]})")
        

bpy.app.timers.register(runThroughField,first_interval = 1.0)