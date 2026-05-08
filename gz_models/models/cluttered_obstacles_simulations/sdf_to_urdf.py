import xml.etree.ElementTree as ET
import argparse
import os

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
    parser = argparse.ArgumentParser(
        description='Convert a Gazebo SDF file to a URDF file.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        'input_sdf', 
        help='Path to the input SDF file.'
    )
    parser.add_argument(
        '-o', '--output_urdf', 
        help='Path for the output URDF file. If not provided, it will be generated from the input name.'
    )
    args = parser.parse_args()

    if args.output_urdf:
        output_file = args.output_urdf
    else:
        base_name = os.path.splitext(args.input_sdf)[0]
        output_file = f"{base_name}.urdf"

    convert_sdf_to_urdf(args.input_sdf, output_file)

# --- How to run from the command line ---
#
# 1. Save this script as `sdf_to_urdf.py`.
# 2. Save your SDF file (e.g., `model.sdf`).
# 3. Run the script from your terminal:
#
#    python sdf_to_urdf.py model.sdf
#
# 4. This will create a `model.urdf` file in the same directory.
#
#    To specify a different output file name:
#
#    python sdf_to_urdf.py model.sdf -o my_robot.urdf
#