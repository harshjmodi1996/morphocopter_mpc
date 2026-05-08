import xml.etree.ElementTree as ET
import random
import os

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


if __name__ == '__main__':
    sdf_file_path = os.path.join(
        os.path.expanduser('~'),
        'PX4-Autopilot/Tools/simulation/gz/models/realistic_room/model.sdf'
    )
    randomize_room_sdf(sdf_file_path)