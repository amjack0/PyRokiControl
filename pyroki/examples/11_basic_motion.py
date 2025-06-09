"""Basic IK

Simplest Inverse Kinematics Example using PyRoki.
"""

import time

import numpy as np
import pyroki as pk
import viser
from robot_descriptions.loaders.yourdfpy import load_robot_description
from viser.extras import ViserUrdf

import pyroki_snippets as pks


def main():
    """Main function for basic IK."""

    urdf = load_robot_description("panda_description")
    target_link_name = "panda_hand"

    # Create robot.
    robot = pk.Robot.from_urdf(urdf)

    # Set up visualizer.
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")

    # Create interactive controller with initial position.
    # ik_target = server.scene.add_transform_controls(  # Removed interactive control
    #     "/ik_target", scale=0.2, position=(0.61, 0.0, 0.56), wxyz=(0, 0, 1, 0)
    # )
    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    # Define circle parameters
    center_x = 0.5
    center_y = 0.0
    radius = 0.1
    fixed_z = 0.4
    clr = (0.0, 1.0, 0.0)  # Green color for the target sphere

    # Add a sphere to visualize the target position
    target_sphere = server.scene.add_icosphere(
        "/ik_target_sphere", radius=0.02, color=clr, position=(center_x, center_y, fixed_z)
    )

    start_time_offset = time.time() # To make the motion start from a consistent point

    while True:
        # Calculate angle based on time for circular motion
        angle = (time.time() - start_time_offset) * 0.5 # Adjust 0.5 for speed

        # Calculate target position on the circle
        target_position_x = center_x + radius * np.cos(angle)
        target_position_y = center_y + radius * np.sin(angle)
        target_position_z = fixed_z
        target_position = np.array([target_position_x, target_position_y, target_position_z])

        # Solve IK.
        start_ik_time = time.time()
        solution = pks.solve_ik(
            robot=robot,
            target_link_name=target_link_name,
            target_position=target_position,
            target_wxyz=np.array([0, 0, 1, 0]), # Keep orientation fixed, or adjust as needed
        )

        # Update timing handle.
        elapsed_ik_time = time.time() - start_ik_time
        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_ik_time * 1000)

        # Update visualizer.
        urdf_vis.update_cfg(solution)
        target_sphere.position = target_position # Update the sphere's position

        # Small sleep to prevent busy-waiting and reduce CPU usage
        time.sleep(0.01)

if __name__ == "__main__":
    main()