import xml.etree.ElementTree as ET
import argparse
import random
import os
try:
    from PIL import Image, ImageDraw
except ImportError:
    print("Pillow library not found. Please install it using: pip install Pillow")
    Image = None
    ImageDraw = None

def update_wall_segment(model, link_name, new_pose_y, new_size_y):
    """Helper function to update the pose and size of a wall link."""
    link = model.find(f".//link[@name='{link_name}']")
    if link is None:
        print(f"Warning: Link '{link_name}' not found.")
        return

    # Update Pose
    pose = link.find('pose')
    pose_values = pose.text.split()
    pose_values[1] = f"{new_pose_y:.4f}"
    pose.text = ' '.join(pose_values)

    # Update Geometry (for both collision and visual)
    for geometry in link.findall('.//geometry'):
        box = geometry.find('box')
        size = box.find('size')
        size_values = size.text.split()
        size_values[1] = f"{new_size_y:.4f}"
        size.text = ' '.join(size_values)

def randomize_room_sdf(file_path):
    """
    Reads an SDF file, randomizes window and door Y-positions, resizes
    adjacent walls to maintain integrity, and overwrites the file.
    """
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        model = root.find('model')

        if model is None:
            print("Error: <model> tag not found in the SDF file.")
            return

        # --- Room Constants ---
        ROOM_Y_MAX = 2.5  # Y-coordinate of the left wall's center
        ROOM_Y_MIN = -2.5 # Y-coordinate of the right wall's center
        WINDOW_WIDTH = 0.33 # The size of the window opening in Y
        DOOR_OPENING_WIDTH = 1.0 # The size of the passage opening in Y

        # --- 1. Randomize Window Wall (wall1) ---
        # Define a safe range for the window's center to avoid the edges
        new_window_y_center = random.uniform(-1.5, 1.5)
        
        # Calculate the edges of the window opening
        window_top_edge = new_window_y_center + WINDOW_WIDTH / 2
        window_bottom_edge = new_window_y_center - WINDOW_WIDTH / 2

        # Update wall1_left (segment between window and the main left wall)
        new_size_y_left = ROOM_Y_MAX - window_top_edge
        new_pose_y_left = window_top_edge + (new_size_y_left / 2)
        update_wall_segment(model, "wall1_left", new_pose_y_left, new_size_y_left)

        # Update wall1_right (segment between window and the main right wall)
        new_size_y_right = window_bottom_edge - ROOM_Y_MIN
        new_pose_y_right = window_bottom_edge - (new_size_y_right / 2)
        update_wall_segment(model, "wall1_right", new_pose_y_right, new_size_y_right)

        # Update the horizontal window pieces to match the new center
        for link_name in ["wall1_bottom", "wall1_top"]:
            link = model.find(f".//link[@name='{link_name}']")
            if link is not None:
                pose = link.find('pose')
                pose_values = pose.text.split()
                pose_values[1] = f"{new_window_y_center:.4f}"
                pose.text = ' '.join(pose_values)

        # --- 2. Randomize Door and Passage Wall (wall2) ---
        # Define a safe range for the door/passage center
        new_door_y_center = random.uniform(-1.5, 1.5)

        # Calculate the edges of the passage opening
        passage_top_edge = new_door_y_center + DOOR_OPENING_WIDTH / 2
        passage_bottom_edge = new_door_y_center - DOOR_OPENING_WIDTH / 2

        # Update wall2_left (segment between passage and main left wall)
        new_size_y_left2 = ROOM_Y_MAX - passage_top_edge
        new_pose_y_left2 = passage_top_edge + (new_size_y_left2 / 2)
        update_wall_segment(model, "wall2_left", new_pose_y_left2, new_size_y_left2)

        # Update wall2_right (segment between passage and main right wall)
        new_size_y_right2 = passage_bottom_edge - ROOM_Y_MIN
        new_pose_y_right2 = passage_bottom_edge - (new_size_y_right2 / 2)
        update_wall_segment(model, "wall2_right", new_pose_y_right2, new_size_y_right2)

        # Update the separate door object to be placed in the opening
        door_link = model.find(".//link[@name='door']")
        if door_link is not None:
            pose = door_link.find('pose')
            pose_values = pose.text.split()
            pose_values[1] = f"{new_door_y_center:.4f}" # Center the door in the opening
            pose.text = ' '.join(pose_values)

        # --- 3. Write the changes back to the file ---
        tree.write(file_path, encoding='UTF-8', xml_declaration=True)
        
        print(f"Successfully randomized room and updated '{os.path.basename(file_path)}'.")
        print(f"  - New Window Y-Center: {new_window_y_center:.2f}")
        print(f"  - New Door/Passage Y-Center: {new_door_y_center:.2f}")

    except ET.ParseError as e:
        print(f"Error parsing XML file: {e}")
    except FileNotFoundError:
        print(f"Error: The file '{file_path}' was not found.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

def generate_2d_map(sdf_file, map_base_path, slice_height=1.2, resolution=0.05, map_size_m=(25, 25)):
    """
    Generates a 2D map (PGM and YAML) from an SDF file for a given height.

    :param sdf_file: Path to the input SDF file.
    :param map_base_path: Base path and name for the output map files (e.g., '/path/to/map').
    :param slice_height: The Z-height (in meters) at which to slice the world.
    :param resolution: Map resolution in meters per pixel.
    :param map_size_m: A tuple (width, height) of the map in meters.
    """
    if Image is None:
        print("Cannot generate map, Pillow library is not available.")
        return

    try:
        tree = ET.parse(sdf_file)
        root = tree.getroot()
        model = root.find('model')
    except (ET.ParseError, FileNotFoundError) as e:
        print(f"Error reading or parsing SDF file '{sdf_file}': {e}")
        return

    if model is None:
        print("Error: <model> tag not found in SDF file.")
        return

    # Map dimensions in pixels
    width_px = int(map_size_m[0] / resolution)
    height_px = int(map_size_m[1] / resolution)

    # Create a white image (255=free space)
    img = Image.new('L', (width_px, height_px), 255)
    draw = ImageDraw.Draw(img)

    # Map origin (center of the map in meters)
    origin_m_x = -map_size_m[0] / 2.0
    origin_m_y = -map_size_m[1] / 2.0

    def world_to_pixel(x, y):
        """Converts world coordinates (meters) to image pixel coordinates."""
        px = int((x - origin_m_x) / resolution)
        # In PGM, the origin is bottom-left, but Pillow's is top-left.
        # We flip the y-coordinate.
        py = height_px - 1 - int((y - origin_m_y) / resolution)
        return px, py

    obstacles_found = 0
    for link in model.findall('link'):
        pose_tag = link.find('pose')
        pose_text = pose_tag.text if pose_tag is not None and pose_tag.text else '0 0 0 0 0 0'
        pose = [float(p) for p in pose_text.split()]
        link_x, link_y, link_z = pose[0], pose[1], pose[2]

        collision = link.find('collision')
        if collision is None:
            continue

        box = collision.find('.//geometry/box/size')
        if box is None or not box.text:
            continue

        size = [float(s) for s in box.text.split()]
        size_x, size_y, size_z = size[0], size[1], size[2]

        # Check if the object intersects the slice height
        z_min = link_z - size_z / 2.0
        z_max = link_z + size_z / 2.0

        if z_min <= slice_height <= z_max:
            obstacles_found += 1
            # Calculate the 2D bounding box in world coordinates
            x_min_world = link_x - size_x / 2.0
            x_max_world = link_x + size_x / 2.0
            y_min_world = link_y - size_y / 2.0
            y_max_world = link_y + size_y / 2.0

            # Convert world coordinates to pixel coordinates
            p_min_x, p_max_y = world_to_pixel(x_min_world, y_min_world)
            p_max_x, p_min_y = world_to_pixel(x_max_world, y_max_world)

            # Draw the rectangle on the map (0=obstacle)
            draw.rectangle([p_min_x, p_min_y, p_max_x, p_max_y], fill=0)

    # --- Save PGM and YAML files ---
    pgm_file = f"{map_base_path}.pgm"
    yaml_file = f"{map_base_path}.yaml"

    img.save(pgm_file)

    yaml_content = {
        'image': os.path.basename(pgm_file),
        'resolution': resolution,
        'origin': [origin_m_x, origin_m_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25
    }

    with open(yaml_file, 'w') as f:
        import yaml
        yaml.dump(yaml_content, f, default_flow_style=False)

    print(f"Successfully generated 2D map for z={slice_height}m.")
    print(f"  - PGM saved to: '{pgm_file}'")
    print(f"  - YAML saved to: '{yaml_file}'")
    if obstacles_found == 0:
        print(f"  - Warning: No obstacles were found at the slice height of {slice_height}m.")

def convert_sdf_to_urdf(sdf_file, urdf_file):
    """
    Converts a Gazebo SDF file to a URDF file. put this file in the same directory as the SDF file of the obstacle to convert it to URDF to be used in the OMPL_based_planner

    This conversion is based on a specific structure where the SDF model
    is static and consists of multiple links, each representing a static obstacle.
    The resulting URDF will have a base_link and all SDF links will be
    attached to it via fixed joints.
    """
    try:
        sdf_tree = ET.parse(sdf_file)
        sdf_root = sdf_tree.getroot()
    except ET.ParseError as e:
        print(f"Error parsing SDF file: {e}")
        return
    except FileNotFoundError:
        print(f"Error: Input SDF file not found at '{sdf_file}'")
        return

    sdf_model = sdf_root.find('model')
    if sdf_model is None:
        print("Error: <model> tag not found in SDF file.")
        return

    robot_name = sdf_model.get('name', 'converted_robot')
    urdf_root = ET.Element('robot', name=robot_name)

    # --- Create a base_link ---
    # URDF requires a root link. We create a virtual one.
    base_link = ET.SubElement(urdf_root, 'link', name='base_link')
    
    # --- Process each link from SDF ---
    for i, sdf_link in enumerate(sdf_model.findall('link')):
        link_name = sdf_link.get('name')
        if not link_name:
            print(f"Warning: Skipping a link because it has no name.")
            continue

        urdf_link = ET.SubElement(urdf_root, 'link', name=link_name)

        # --- Pose -> Origin ---
        pose_str = "0 0 0 0 0 0"
        pose_tag = sdf_link.find('pose')
        if pose_tag is not None and pose_tag.text:
            pose_str = pose_tag.text
        
        pose_vals = pose_str.split()
        xyz_str = " ".join(pose_vals[0:3])
        rpy_str = " ".join(pose_vals[3:6])
        origin_attrs = {'xyz': xyz_str, 'rpy': rpy_str}

        # --- Inertial (optional, but good practice for static objects) ---
        inertial = ET.SubElement(urdf_link, 'inertial')
        ET.SubElement(inertial, 'origin', **origin_attrs)
        ET.SubElement(inertial, 'mass', value="999999999999999999999999999.0") # Mass is not critical for fixed links
        ET.SubElement(inertial, 'inertia', ixx="0.01", ixy="0", ixz="0", iyy="0.01", iyz="0", izz="0.01")

        # --- Visual ---
        sdf_visual = sdf_link.find('visual')
        if sdf_visual is not None:
            urdf_visual = ET.SubElement(urdf_link, 'visual')
            ET.SubElement(urdf_visual, 'origin', **origin_attrs)
            
            sdf_geom = sdf_visual.find('geometry')
            if sdf_geom is not None:
                geom_type = sdf_geom.find('*')
                if geom_type is not None:
                    urdf_geom = ET.SubElement(urdf_visual, 'geometry')
                    urdf_geom_type = ET.SubElement(urdf_geom, geom_type.tag)
                    size_tag = geom_type.find('size')
                    if size_tag is not None and size_tag.text:
                         urdf_geom_type.set('size', size_tag.text)
                    radius_tag = geom_type.find('radius')
                    if radius_tag is not None and radius_tag.text:
                         urdf_geom_type.set('radius', radius_tag.text)
                    length_tag = geom_type.find('length')
                    if length_tag is not None and length_tag.text:
                         urdf_geom_type.set('length', length_tag.text)

            sdf_material = sdf_visual.find('material')
            if sdf_material is not None:
                urdf_material = ET.SubElement(urdf_visual, 'material', name=f"{link_name}_color")
                color_tag = sdf_material.find('ambient') # or diffuse
                if color_tag is None:
                    color_tag = sdf_material.find('diffuse')
                
                if color_tag is not None and color_tag.text:
                    ET.SubElement(urdf_material, 'color', rgba=color_tag.text)

        # --- Collision ---
        sdf_collision = sdf_link.find('collision')
        if sdf_collision is not None:
            urdf_collision = ET.SubElement(urdf_link, 'collision')
            ET.SubElement(urdf_collision, 'origin', **origin_attrs)

            sdf_geom = sdf_collision.find('geometry')
            if sdf_geom is not None:
                geom_type = sdf_geom.find('*')
                if geom_type is not None:
                    urdf_geom = ET.SubElement(urdf_collision, 'geometry')
                    urdf_geom_type = ET.SubElement(urdf_geom, geom_type.tag)
                    size_tag = geom_type.find('size')
                    if size_tag is not None and size_tag.text:
                         urdf_geom_type.set('size', size_tag.text)
                    radius_tag = geom_type.find('radius')
                    if radius_tag is not None and radius_tag.text:
                         urdf_geom_type.set('radius', radius_tag.text)
                    length_tag = geom_type.find('length')
                    if length_tag is not None and length_tag.text:
                         urdf_geom_type.set('length', length_tag.text)

        # --- Joint ---
        # Create a fixed joint to connect this link to the base_link
        joint_name = f"joint_{i+1}_{link_name}"
        urdf_joint = ET.SubElement(urdf_root, 'joint', name=joint_name, type='fixed')
        ET.SubElement(urdf_joint, 'parent', link='base_link')
        ET.SubElement(urdf_joint, 'child', link=link_name)
        ET.SubElement(urdf_joint, 'origin', xyz="0 0 0", rpy="0 0 0")

    # --- Write to file ---
    tree = ET.ElementTree(urdf_root)
    ET.indent(tree, space="  ")  # For pretty printing
    tree.write(urdf_file, encoding='utf-8', xml_declaration=True)
    print(f"Successfully converted '{sdf_file}' to '{urdf_file}'")


if __name__ == '__main__':
    # Check for YAML dependency for map generation
    try:
        import yaml
    except ImportError:
        print("PyYAML not found, which is required for map generation.")
        print("Please install it using: pip install PyYAML")
        yaml = None


    ## Randomize SDF:
    sdf_file_path = os.path.join(
        os.path.expanduser('~'),
        'PX4-Autopilot/Tools/simulation/gz/models/realistic_room/model.sdf'
    )
    randomize_room_sdf(sdf_file_path)

    ## Generate URDF:
    urdf_file_path = os.path.join(
        os.path.expanduser('~'),
        'PX4-Autopilot/Tools/simulation/gz/models/realistic_room/model.urdf'
    )
    convert_sdf_to_urdf(sdf_file_path, urdf_file_path)

    ## Generate 2D Map for RViz:
    if yaml:
        map_file_base_path = os.path.join(
            os.path.expanduser('~'),
            'PX4-Autopilot/Tools/simulation/gz/models/realistic_room/map'
        )
        generate_2d_map(sdf_file_path, map_file_base_path, slice_height=1.2)
