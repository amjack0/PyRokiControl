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


class RobotMotionController:
    """
    Controls the motion of a robot's end-effector using Inverse Kinematics (IK).
    """

    def __init__(self, urdf_path: str, target_link_name: str = "panda_hand"):
        """
        Initialize the robot, PyRoki, and Viser visualizer.

        Args:
            urdf_path (str): Path to the URDF description of the robot.
            target_link_name (str): Name of the end-effector link to control.
        """
        self.robot = pk.Robot.from_urdf(load_robot_description(urdf_path))
        self.target_link_name = target_link_name

        # Set up visualizer.
        self.server = viser.ViserServer()
        self.server.scene.add_grid("/ground", width=2, height=2)
        self.urdf_vis = ViserUrdf(self.server, load_robot_description(urdf_path), root_node_name="/base")

        self.timing_handle = self.server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)
        self.target_sphere = self.server.scene.add_icosphere(
            "/ik_target_sphere", radius=0.02, color=(0.0, 1.0, 0.0) # Green color
        )

    def _solve_and_update(self, target_position: np.ndarray, target_wxyz: np.ndarray):
        """
        Solve IK for the given target and update the robot's visualization.

        Args:
            target_position (np.ndarray): Desired position of the end-effector.
            target_wxyz (np.ndarray): Desired orientation (wxyz quaternion) of the end-effector.
        """
        start_ik_time = time.time()
        solution = pks.solve_ik(
            robot=self.robot,
            target_link_name=self.target_link_name,
            target_position=target_position,
            target_wxyz=target_wxyz,
        )

        elapsed_ik_time = time.time() - start_ik_time
        self.timing_handle.value = 0.99 * self.timing_handle.value + 0.01 * (elapsed_ik_time * 1000)

        self.urdf_vis.update_cfg(solution)
        self.target_sphere.position = target_position

    def move_circular(
        self,
        center_x: float,
        center_y: float,
        radius: float,
        fixed_z: float,
        speed: float = 0.5,
        target_wxyz: np.ndarray = np.array([0, 0, 1, 0]),
    ):
        """
        Make the end-effector follow a circular path.

        Args:
            center_x (float): X-coordinate of the circle's center.
            center_y (float): Y-coordinate of the circle's center.
            radius (float): Radius of the circular path.
            fixed_z (float): Fixed Z-coordinate for the circular path.
            speed (float): Speed of rotation around the circle.
            target_wxyz (np.ndarray): Fixed orientation (wxyz quaternion) of the end-effector.
        """
        self.target_sphere.position = (center_x, center_y, fixed_z) # Set initial sphere position
        start_time_offset = time.time()

        while True:
            angle = (time.time() - start_time_offset) * speed
            target_position_x = center_x + radius * np.cos(angle)
            target_position_y = center_y + radius * np.sin(angle)
            target_position = np.array([target_position_x, target_position_y, fixed_z])

            self._solve_and_update(target_position, target_wxyz)
            time.sleep(0.01)

    def move_linear(
        self,
        start_position: np.ndarray,
        end_position: np.ndarray,
        duration: float = 5.0, # Duration in seconds for one full traverse
        target_wxyz: np.ndarray = np.array([0, 0, 1, 0]),
    ):
        """
        Makes the end-effector follow a linear path between two points,
        repeating the motion back and forth.

        Args:
            start_position (np.ndarray): The starting position of the end-effector (x, y, z).
            end_position (np.ndarray): The ending position of the end-effector (x, y, z).
            duration (float): The time in seconds for a one-way traverse from start to end.
            target_wxyz (np.ndarray): The fixed orientation (wxyz quaternion) of the end-effector.
        """
        start_position = np.array(start_position)
        end_position = np.array(end_position)
        
        # Set initial sphere position
        self.target_sphere.position = start_position 

        start_time_offset = time.time()

        while True:
            elapsed_time = time.time() - start_time_offset
            
            # Normalize time to a 0-1 range, then extend to a 0-2 range for back-and-forth motion
            normalized_time = (elapsed_time % (2 * duration)) / (2 * duration)

            if normalized_time <= 0.5:
                # Moving from start_position to end_position (0 to 0.5 in normalized_time)
                # progress will go from 0 to 1
                progress = normalized_time * 2
                target_position = start_position + progress * (end_position - start_position)
            else:
                # Moving from end_position back to start_position (0.5 to 1.0 in normalized_time)
                # progress will go from 1 to 0
                progress = (1.0 - normalized_time) * 2
                target_position = start_position + progress * (end_position - start_position)

            self._solve_and_update(target_position, target_wxyz)
            time.sleep(0.01)

def main():
    # Main function to run the robot motion controller
    controller = RobotMotionController(urdf_path="panda_description", target_link_name="panda_hand")
    """
    # Make the end-effector follow a circular path
    controller.move_circular(
        center_x=0.5,
        center_y=0.0,
        radius=0.1,
        fixed_z=0.4,
        speed=0.5,
    )
    """
    # Make the end-effector follow a linear path
    controller.move_linear(
        start_position=np.array([0.6, 0.1, 0.5]),
        end_position=np.array([0.6, -0.1, 0.5]),
        duration=3.0, # 3 seconds to go from start to end
    )


if __name__ == "__main__":
    main()