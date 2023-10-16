import sys
import os
import bpy

blend_dir = os.path.dirname(bpy.data.filepath)
if blend_dir not in sys.path:
   sys.path.append(blend_dir)
   
import PathFind
import ImageStitch

def create_rover_body(location=(0,0,0)):
    bpy.ops.mesh.primitive_cube_add(size=2, location=location)
    rover_body = bpy.context.active_object
    rover_body.name = "Rover_Body"
    return rover_body

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

def create_rover_with_cameras(rover_location=(0,0,0)):
    rover = create_rover_body(rover_location)
    
    cam1_location = (rover_location[0], rover_location[1] - 1.5, rover_location[2] + 1.5)
    cam2_location = (rover_location[0], rover_location[1] + 1.5, rover_location[2] + 1.5)
    
    cam_rotation = (1.5708, 0, 0)  
    
    camera1 = create_camera("Camera_POV1", cam1_location, cam_rotation)
    camera2 = create_camera("Camera_POV2", cam2_location, cam_rotation)
    
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

create_rover_with_cameras(rover_location=(0,0,0))

ImageRange = range(1,10)

for x in ImageRange:
    camera1_img = os.path.join(bpy.path.abspath("//"), "camera1_"+str(x)+".png")
    camera2_img = os.path.join(bpy.path.abspath("//"), "camera2_"+str(x)+".png")
    concatenated_img_path = os.path.join(bpy.path.abspath("//"), "concatenated_"+str(x)+".png")

    render_camera_image("Camera_POV1", camera1_img)
    render_camera_image("Camera_POV2", camera2_img)

    ImageStitch.concatenate_images(camera1_img, camera2_img, concatenated_img_path)

    movement = PathFind.handle_input(concatenated_img_path)

    update_rover_location("Rover_Body", (movement,1,0))
