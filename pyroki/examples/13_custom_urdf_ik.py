r"""
Usage:
    python 13_custom_urdf_ik.py [--robot-type <robot_type>] [--urdf-path <path_to_custom_urdf>] [--target-link-name <link_name>]

Examples:
    python .\13_custom_urdf_ik.py
    python .\13_custom_urdf_ik.py --robot-type ur10_description
    spython .\13_custom_urdf_ik.py --urdf-path D:\Python_projects\PyRokiControl\custom_urdf\01_custom_arm.urdf
    python .\13_custom_urdf_ik.py --urdf-path D:\Python_projects\PyRokiControl\custom_urdf\01_custom_arm.urdf --target-link-name end_effector_tool
    

Minimal PyRoki + Viser + Custom URDF
This script demonstrates how to control a custom robot using inverse kinematics (IK)
Capsule shape is not a standard primitive geometry type directly supported by URDF. 
URDF primarily supports:
    <box>
    <cylinder>
    <sphere>
If the URDF is contains mesh, dae or stl files, the path should be specified. Continous joint is not supported yet.
<mesh filename="package://your_robot_package_name/visual/base_link.dae"/>
Something like should work: <mesh filename="package://kuka_kr3_support/meshes/kr3r540/visual/base_link.stl"/>
Somethibg like does not work: <mesh filename="package://visual/base_link.dae"/> as yourdfpy interprets "visual" as the package name.
more urdf examples at: https://github.com/Daniella1/urdf_files_dataset/tree/main/urdf_files/ros-industrial/xacro_generated
"""

import pyroki as pk
import pyroki_snippets as pks
import viser
from viser.extras import ViserUrdf
from robot_descriptions.loaders.yourdfpy import load_robot_description
import numpy as np
import time
import tyro
from typing import Literal, Optional, List, Tuple
import yourdfpy
import sys
import signal
import jaxlie
import jax.numpy as jnp

# Define supported robot types
SUPPORTED_ROBOT_TYPES = Literal[
    "panda_description", "ur10_description", "cassie_description",
    "allegro_hand_description", "barrett_hand_description", "robotiq_2f85_description",
    "atlas_drc_description", "iiwa7_description", "iiwa14_description", 
    "fanuc_m710ic_description", 'ur3_description', 'ur5_description',
    "g1_description", "h1_description", "anymal_c_description", 
    "go2_description", "anymal_d_description"
]

class UrdfIK:
    """
    The URDF IK application.
    """
    def __init__(
        self,
        server: viser.ViserServer,
        robot_type: Optional[SUPPORTED_ROBOT_TYPES],
        urdf_path: Optional[str] = None,
        target_link_name: Optional[str] = None,
    ) -> None:
        """
        Initializes the URDF IK application.
        """
        self.server = server
        self.robot_type = robot_type
        self.urdf_path = urdf_path
        self.user_target_link_name = target_link_name

        # Initialize instance attributes
        self.robot: pk.Robot
        self.viser_urdf: ViserUrdf
        self.target_link_name: str
        self.slider_handles: List[viser.GuiInputHandle[float]]
        self.initial_config: List[float]
        self.ik_target: viser.TransformControlsHandle

        self._setup_robot_and_gui()

        # Handle Ctrl+C gracefully
        signal.signal(signal.SIGINT, self._signal_handler)

    def _setup_robot_and_gui(self) -> None:
        """
        Handles the initial setup of the robot model, visualizer,
        IK target, and GUI sliders.
        """
        urdf_model = None
        try:
            if self.urdf_path:  # Load URDF from file path.
                urdf_model = yourdfpy.URDF.load(self.urdf_path)
            elif self.robot_type:  # Load built-in robot description.
                urdf_model = load_robot_description(self.robot_type)
            else:
                raise ValueError("[custom_urdf_ik] Either 'robot_type' or 'urdf_path' must be provided.")
            # ignore the world and base, and read only the physical links
            all_links = [l for l in urdf_model.link_map.keys() if l not in ("world", "base")]
            if not all_links:
                raise RuntimeError("[custom_urdf_ik] No valid links found in URDF.")

            base_link_name = all_links[0]
            # Use provided target_link_name else default to last link
            if self.user_target_link_name in all_links:
                self.target_link_name = self.user_target_link_name
            elif self.user_target_link_name is not None:
                raise ValueError(f"[custom_urdf_ik] Provided target_link_name '{self.user_target_link_name}' not found in URDF.")
            else:
                self.target_link_name = all_links[-1]

            self.viser_urdf = ViserUrdf(self.server, urdf_or_path=urdf_model, root_node_name=f"/{base_link_name}")
            print(f"[custom_urdf_ik] [base_link, target_link]: ['{base_link_name}', '{self.target_link_name}']")
            print(f"[custom_urdf_ik] [all_links]: {all_links}")
            self.robot = pk.Robot.from_urdf(urdf_model)
        except Exception as e:
            print(f"[custom_urdf_ik] Error loading URDF: {e}")
            sys.exit(1) # Exit if robot model cannot be loaded
        
        with self.server.gui.add_folder("Joint position control"):
            self.slider_handles, self.initial_config = self._create_robot_control_sliders()

        # Calculate initial position using forward kinematics
        target_link_idx = self.robot.links.names.index(self.target_link_name)
        # Perform forward kinematics with the initial configuration and extract position of the target link
        T_root_link_target = jaxlie.SE3(self.robot.forward_kinematics(cfg=jnp.array(self.initial_config)))
        initial_target_position = T_root_link_target.translation()[target_link_idx]
        pos = np.array(initial_target_position)
        # print(f"[custom_urdf_ik] Initial target position(FK): {pos}")

        self.ik_target = self.server.scene.add_transform_controls("/ik_target", scale=0.2, position=pos, wxyz=(0,  0, 1, 0))
        # print(f"[custom_urdf_ik] Initial configuration: {self.initial_config}")
        self.viser_urdf.update_cfg(np.array(self.initial_config))
        grid_z_position = 0.0
        if self.viser_urdf._urdf.scene.bounds is not None:
            # If bounds are available, set grid at the bottom of the robot's bounding box, else default to 0.0
            grid_z_position = self.viser_urdf._urdf.scene.bounds[0, 2]
        print(f"[custom_urdf_viz] [Warning]: Grid Z position set to: {grid_z_position}")
        self.server.scene.add_grid("/grid", width=2, height=2, position=(0.0, 0.0, grid_z_position))
        reset_button = self.server.gui.add_button("Reset")

        @reset_button.on_click
        def _(_):
            for s, init_q in zip(self.slider_handles, self.initial_config):
                s.value = init_q

    def _create_robot_control_sliders(
        self,
    ) -> Tuple[List[viser.GuiInputHandle[float]], List[float]]:
        """Create slider for each joint of the robot."""
        slider_handles: List[viser.GuiInputHandle[float]] = []
        initial_config: List[float] = []
        for joint_name, (
            lower,
            upper,
        ) in self.viser_urdf.get_actuated_joint_limits().items():
            lower = lower if lower is not None else -np.pi
            upper = upper if upper is not None else np.pi
            if lower == -np.pi and upper == np.pi:
                print(f"[custom_urdf_ik] Warning: Joint '{joint_name}' has no specified limits. Defaulting to [-pi, pi].")
            initial_pos = 0.0 if lower < 0 and upper > 0 else (lower + upper) / 2.0
            slider = self.server.gui.add_slider(
                label=joint_name,
                min=lower,
                max=upper,
                step=1e-3,
                initial_value=initial_pos,
            )
            # The lambda captures `slider_handles` from the outer scope.
            slider.on_update(
                lambda _: self.viser_urdf.update_cfg(
                    np.array([s.value for s in slider_handles])
                )
            )
            slider_handles.append(slider)
            initial_config.append(initial_pos)
        return slider_handles, initial_config


    def _signal_handler(self, sig, frame) -> None:
        """Handles Ctrl+C gracefully."""
        print("[custom_urdf_ik] Shutting down Viser server...")
        self.server.stop()
        sys.exit(0)

    def run(self) -> None:
        """
        Runs the main loop for the IK application.
        """
        while True:
            try:
                ik_solution = pks.solve_ik(
                    robot=self.robot,
                    target_link_name=self.target_link_name,
                    target_position=np.array(self.ik_target.position),
                    target_wxyz=np.array(self.ik_target.wxyz),
                )
                if ik_solution is not None:
                    for slider, value in zip(self.slider_handles, ik_solution):
                        slider.value = value
            except Exception as e:
                print(f"[custom_urdf_ik] Error solving IK: {e}")
            time.sleep(0.01)

def main(
    robot_type: Optional[SUPPORTED_ROBOT_TYPES] = "panda_description",
    urdf_path: Optional[str] = None,
    target_link_name: Optional[str] = None,
) -> None:
    """
    Main function to initialize and run the UrdfIK.
    """
    server = viser.ViserServer()
    app = UrdfIK(server, robot_type, urdf_path, target_link_name)
    app.run()

if __name__ == "__main__":
    tyro.cli(main)