import bpy

def clear_scene():
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete()
    bpy.ops.object.select_by_type(type='CURVE')
    bpy.ops.object.delete()

clear_scene()

def create_field():
    # Create brown material for the plane
    plane_material = bpy.data.materials.new(name="Plane Material")
    plane_material.diffuse_color = (0.545, 0.271, 0.075, 1)  # RGBA for brown color

    # Create green material for the rows
    row_material = bpy.data.materials.new(name="Row Material")
    row_material.diffuse_color = (0.133, 0.545, 0.133, 1)  # RGBA for green color

    # Create the main plane
    plane_size = 300
    bpy.ops.mesh.primitive_plane_add(size=plane_size, enter_editmode=False, align='WORLD', location=(int(plane_size/3), int(plane_size/3), 0))

    # Assign material to the plane
    bpy.context.object.data.materials.append(plane_material)

    # Parameters for the rows
    number_of_rows = 20
    row_width = 2
    distance_between_rows = 4
    curve_depth = 50  # Adjusting the curve depth for a more subtle bend

    # Create the curved rows on the plane
    for i in range(number_of_rows):
        bpy.ops.curve.primitive_bezier_curve_add(enter_editmode=True, align='WORLD', location=(0, i * distance_between_rows, 0))
        
        # Adjust handles to make a curve
        bezier_points = bpy.context.object.data.splines[0].bezier_points
        bezier_points[0].co = [0, i * distance_between_rows, 0]
        bezier_points[0].handle_right = [50, i * distance_between_rows + curve_depth, 0]
        bezier_points[0].handle_left = [0, i * distance_between_rows, 0]
        bezier_points[1].co = [100, i * distance_between_rows, 0]
        bezier_points[1].handle_left = [50, i * distance_between_rows + curve_depth, 0]
        bezier_points[1].handle_right = [100, i * distance_between_rows, 0]

        # Set the row width and ensure it doesn't close on itself
        bpy.context.object.data.bevel_depth = row_width / 2
        bpy.context.object.data.fill_mode = 'FULL'
        bpy.context.object.data.use_fill_caps = False
        
        # Assign material to the curve
        bpy.context.object.data.materials.append(row_material)
        
        bpy.ops.object.mode_set(mode='OBJECT')
