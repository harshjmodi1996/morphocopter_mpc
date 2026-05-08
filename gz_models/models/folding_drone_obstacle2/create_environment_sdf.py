import csv
import os

import xml.etree.ElementTree as ET


def create_link(model, name, pose, size, albedo_map_path = None):
    """
    Helper function to create a single <link> element for the SDF model.

    Args:
        model (ET.Element): The root <model> element.
        name (str): The name of the link.
        pose (tuple): A tuple of (x, y, z, roll, pitch, yaw) for the link's pose.
        size (tuple): A tuple of (x, y, z) for the link's box geometry.
        albedo_map_path (str, optional): Path to the albedo map texture for PBR material.
    """
    link = ET.SubElement(model, 'link', name=name)

    # Set pose
    pose_str = ' '.join(map(str, pose))
    ET.SubElement(link, 'pose').text = pose_str

    # Set collision properties
    collision = ET.SubElement(link, 'collision', name=f'{name}_collision')
    geometry_coll = ET.SubElement(collision, 'geometry')
    box_coll = ET.SubElement(geometry_coll, 'box')
    ET.SubElement(box_coll, 'size').text = ' '.join(map(str, size))

    # Set visual properties
    visual = ET.SubElement(link, 'visual', name=f'{name}_visual')
    geometry_vis = ET.SubElement(visual, 'geometry')
    box_vis = ET.SubElement(geometry_vis, 'box')
    ET.SubElement(box_vis, 'size').text = ' '.join(map(str, size))

    # Add material for better appearance
    if albedo_map_path:
        material = ET.SubElement(visual, 'material')
        
        ET.SubElement(material, 'diffuse').text = '1 1 1 1'
        pbr = ET.SubElement(material, 'pbr')
        metal = ET.SubElement(pbr, 'metal')
        ET.SubElement(metal, 'albedo_map').text = albedo_map_path

    else:
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = '0.596 0.675 0.582 1'
        ET.SubElement(material, 'diffuse').text = '0.596 0.675 0.582 1'

    return link


def add_wall_to_model(model, wall_id, loc, size, min_primary, max_primary, min_z, max_z, wall_direction, has_window, wall_texture_path = None):
    """
    Adds a wall (either solid or with a window) to an existing SDF model element.

    Args:
        model (ET.Element): The root <model> element to add links to.
        wall_id (int): A unique identifier for the wall to ensure unique link names.
        loc (tuple): (x, y, z) location of the center of the window opening.
        size (tuple): (x_size, y_size, z_size) of the window opening.
                      x_size is the wall thickness.
        min_primary (float): The minimum extent of the wall in the primary direction.
        max_primary (float): The maximum extent of the wall in the primary direction.
        min_z (float): The minimum height of the wall.
        max_z (float): The maximum height of the wall.
        wall_direction (str): The primary direction of the wall ('x' or 'y').
        has_window (bool): If True, creates a wall with a window opening. If False, creates a solid wall.
        wall_texture_path (str, optional): Path to the wall texture.
    """
    loc_secondary, loc_primary, loc_z = loc
    size_secondary, size_primary, size_z = size

    if not has_window:
        # --- Create a single solid wall ---
        wall_size_primary = max_primary - min_primary
        wall_pose_primary = (max_primary + min_primary) / 2
        wall_size_z = max_z - min_z
        wall_pose_z = (max_z + min_z) / 2

        link_name = f'solid_wall_{wall_id}'
        if wall_direction == 'y':
            create_link(model, link_name, (loc_secondary, wall_pose_primary, wall_pose_z - 0.5, 0, 0, 0), (size_secondary, wall_size_primary, wall_size_z), wall_texture_path)
        elif wall_direction == 'x':
            create_link(model, link_name, (wall_pose_primary, loc_secondary, wall_pose_z - 0.5, 0, 0, 0), (wall_size_primary, size_secondary, wall_size_z), wall_texture_path)
        else:
            raise ValueError("wall_direction must be 'x' or 'y'")
    else:
        # --- Wall creation with window ---
        wall_primary_min, wall_primary_max = min_primary, max_primary
        wall_z_min, wall_z_max = min_z, max_z

        window_primary_min = loc_primary - size_primary / 2
        window_primary_max = loc_primary + size_primary / 2
        window_z_min = loc_z - size_z / 2
        window_z_max = loc_z + size_z / 2

        # 1. Top wall section
        top_wall_size_primary = wall_primary_max - wall_primary_min
        top_wall_pose_primary = (wall_primary_max + wall_primary_min) / 2
        top_wall_size_z = wall_z_max - window_z_max
        top_wall_pose_z = window_z_max + top_wall_size_z / 2

        if top_wall_size_z > 1e-6:
            link_name = f'top_wall_{wall_id}'
            if wall_direction == 'y':
                create_link(model, link_name, (loc_secondary, top_wall_pose_primary, top_wall_pose_z - 0.5, 0, 0, 0), (size_secondary, top_wall_size_primary, top_wall_size_z), wall_texture_path)
            elif wall_direction == 'x':
                create_link(model, link_name, (top_wall_pose_primary, loc_secondary, top_wall_pose_z - 0.5, 0, 0, 0), (top_wall_size_primary, size_secondary, top_wall_size_z), wall_texture_path)

        # 2. Bottom wall section
        bottom_wall_size_primary = wall_primary_max - wall_primary_min
        bottom_wall_size_z = window_z_min - wall_z_min
        bottom_wall_pose_primary = (wall_primary_max + wall_primary_min) / 2
        bottom_wall_pose_z = window_z_min - bottom_wall_size_z / 2

        if bottom_wall_size_z > 1e-6:
            link_name = f'bottom_wall_{wall_id}'
            if wall_direction == 'y':
                create_link(model, link_name, (loc_secondary, bottom_wall_pose_primary, bottom_wall_pose_z - 0.5, 0, 0, 0), (size_secondary, bottom_wall_size_primary, bottom_wall_size_z), wall_texture_path)
            elif wall_direction == 'x':
                create_link(model, link_name, (bottom_wall_pose_primary, loc_secondary, bottom_wall_pose_z - 0.5, 0, 0, 0), (bottom_wall_size_primary, size_secondary, bottom_wall_size_z), wall_texture_path)

        # 3. Left wall section (next to window)
        left_wall_size_primary = window_primary_min - wall_primary_min
        left_wall_pose_primary = wall_primary_min + left_wall_size_primary / 2
        left_wall_size_z = window_z_max - window_z_min
        left_wall_pose_z = (window_z_max + window_z_min) / 2

        if left_wall_size_primary > 1e-6:
            link_name = f'left_wall_{wall_id}'
            if wall_direction == 'y':
                create_link(model, link_name, (loc_secondary, left_wall_pose_primary, left_wall_pose_z - 0.5, 0, 0, 0), (size_secondary, left_wall_size_primary, left_wall_size_z), wall_texture_path)
            elif wall_direction == 'x':
                create_link(model, link_name, (left_wall_pose_primary, loc_secondary, left_wall_pose_z - 0.5, 0, 0, 0), (left_wall_size_primary, size_secondary, left_wall_size_z), wall_texture_path)

        # 4. Right wall section (next to window)
        right_wall_size_primary = wall_primary_max - window_primary_max
        right_wall_pose_primary = window_primary_max + right_wall_size_primary / 2
        right_wall_size_z = window_z_max - window_z_min
        right_wall_pose_z = (window_z_max + window_z_min) / 2

        if right_wall_size_primary > 1e-6:
            link_name = f'right_wall_{wall_id}'
            if wall_direction == 'y':
                create_link(model, link_name, (loc_secondary, right_wall_pose_primary, right_wall_pose_z - 0.5, 0, 0, 0), (size_secondary, right_wall_size_primary, right_wall_size_z), wall_texture_path)
            elif wall_direction == 'x':
                create_link(model, link_name, (right_wall_pose_primary, loc_secondary, right_wall_pose_z - 0.5, 0, 0, 0), (right_wall_size_primary, size_secondary, right_wall_size_z), wall_texture_path)


def create_sdf_from_csv(csv_path, output_path, wall_texture_path = None):
    """
    Generates a complete SDF file for an environment defined in a CSV file.

    Args:
        csv_path (str): Path to the input CSV file.
        output_path (str): The path to save the generated .sdf file.
        wall_texture_path (str, optional): Path to the wall texture.
    """
    # --- Create the root structure of the SDF file ---
    sdf = ET.Element('sdf', version='1.7')
    model = ET.SubElement(sdf, 'model', name='folding_drone_obstacle2')
    ET.SubElement(model, 'static').text = 'true'

    # --- Read CSV and add walls to the model ---
    try:
        with open(csv_path, mode='r', newline='') as infile:
            reader = csv.DictReader(infile)
            for i, row in enumerate(reader):
                # Sanitize and convert data types
                has_window = row['has_window'].strip().lower() in ('true', '1', 't', 'y', 'yes')
                loc = (float(row['loc_secondary']), float(row['loc_primary']), float(row['loc_z']))
                size = (float(row['window_size_secondary']), float(row['window_size_primary']), float(row['window_size_z']))
                min_primary = float(row['min_primary'])
                max_primary = float(row['max_primary'])
                min_z = float(row['min_z'])
                max_z = float(row['max_z'])
                wall_direction = row['wall_direction'].strip()

                add_wall_to_model(
                    model=model,
                    wall_id=i,
                    loc=loc,
                    size=size,
                    min_primary=min_primary,
                    max_primary=max_primary,
                    min_z=min_z,
                    max_z=max_z,
                    wall_direction=wall_direction,
                    has_window=has_window,
                    wall_texture_path=wall_texture_path
                )
    except FileNotFoundError:
        print(f"Error: CSV file not found at '{csv_path}'")
        return
    except (KeyError, ValueError) as e:
        print(f"Error processing CSV file: {e}. Please check column names and data types.")
        return

    # --- Write the XML tree to the file ---
    tree = ET.ElementTree(sdf)
    ET.indent(tree, space="  ")  # For pretty printing
    try:
        tree.write(output_path, encoding='UTF-8', xml_declaration=True)
        print(f"Successfully created SDF file at '{output_path}' from '{csv_path}'")
    except IOError as e:
        print(f"Error writing to file: {e}")


if __name__ == '__main__':
    # --- Configuration ---
    # Path to the CSV file defining the environment
    csv_file = 'environment_definition.csv'

    # Create a sample CSV if it doesn't exist
    if not os.path.exists(csv_file):
        print(f"'{csv_file}' not found. Creating a sample file.")
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            # Header
            writer.writerow(['has_window', 'loc_secondary', 'loc_primary', 'loc_z', 'size_secondary', 'size_primary', 'size_z', 'min_primary', 'max_primary', 'min_z', 'max_z', 'wall_direction'])
            # Row 1: Wall with a window
            writer.writerow([True, 3.0, 0.0, 1.5, 0.2, 1.5, 1.0, -2.5, 2.5, 0.0, 3.0, 'y'])
            # Row 2: Solid wall
            writer.writerow([False, -3.0, 0.0, 1.5, 0.2, 1.5, 1.0, -2.5, 2.5, 0.0, 3.0, 'y'])
            # Row 3: Another wall with a window, different orientation
            writer.writerow([True, 0.0, 0.0, 1.2, 0.2, 1.0, 1.0, -3.0, 3.0, 0.0, 2.5, 'x'])

    # Path for the output SDF file
    output_file = 'model.sdf'

    # --- Material Configuration ---
    # Set the path to your texture file. Use an absolute path for best results.
    wall_texture = '/home/harshmodi/PX4-Autopilot/Tools/simulation/gz/models/folding_drone_obstacle2/materials/textures/brickwall.png'

    # Generate the SDF file from the CSV definition
    create_sdf_from_csv(
        csv_path=csv_file,
        output_path=output_file,
        wall_texture_path=None
    )
