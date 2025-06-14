"""
Minimal PyRoki + Viser + Custom URDF
- https://github.com/nerfstudio-project/viser/blob/main/examples/09_urdf_visualizer.py

"""
# TODO: doing IK -- demonstrating how users can actually use custom robot models with PyRoki.
"""
Capsule shape is not a standard primitive geometry type directly supported by URDF. 
URDF primarily supports:
    <box>
    <cylinder>
    <sphere>
    <mesh> (for loading external 3D model files like .stl, .dae, .obj)
"""

import pyroki as pk
import pyroki_snippets as pks
import viser
from viser.extras import ViserUrdf
from robot_descriptions.loaders.yourdfpy import load_robot_description
import numpy as np
import time
import tyro
from typing import Literal, Optional
import yourdfpy
import sys
import signal

# This script is for single target IK (solve_ik), but can be extended to multiple targets (solve_ik_with_multiple_targets). 
# This script works well with custom URDF serial-chain robots including Panda, iiwa7_description, and fanuc_m710ic_description.

def create_robot_control_sliders(
    server: viser.ViserServer, viser_urdf: ViserUrdf
) -> tuple[list[viser.GuiInputHandle[float]], list[float]]:
    """Create slider for each joint of the robot. We also update robot model
    when slider moves."""
    slider_handles: list[viser.GuiInputHandle[float]] = []
    initial_config: list[float] = []
    for joint_name, (
        lower,
        upper,
    ) in viser_urdf.get_actuated_joint_limits().items():
        lower = lower if lower is not None else -np.pi
        upper = upper if upper is not None else np.pi
        if lower == -np.pi and upper == np.pi:
            print(f"[custom_urdf_ik] Warning: Joint '{joint_name}' has no specified limits. Defaulting to [-pi, pi].")
        initial_pos = 0.0 if lower < 0 and upper > 0 else (lower + upper) / 2.0
        slider = server.gui.add_slider(
            label=joint_name,
            min=lower,
            max=upper,
            step=1e-3,
            initial_value=initial_pos,
        )
        slider.on_update(  # When sliders move, we update the URDF configuration.
            lambda _: viser_urdf.update_cfg(
                np.array([slider.value for slider in slider_handles])
            )
        )
        slider_handles.append(slider)
        initial_config.append(initial_pos)
    return slider_handles, initial_config


def main(
    robot_type: Optional[Literal[
        "panda_description", "ur10_description", "cassie_description", "allegro_hand_description", 
        "barrett_hand_description", "robotiq_2f85_description", "atlas_drc_description",
        "iiwa7_description", "iiwa14_description", "fanuc_m710ic_description", 'ur3_description', 'ur5_description',
        "g1_description", "h1_description", "anymal_c_description", "go2_description", "anymal_d_description"]]
        = "panda_description",
    urdf_path: Optional[str] = None, # Added urdf_path as an optional input
) -> None:
    # Start viser server.
    server = viser.ViserServer()

    # Load URDF
    urdf_model = None
    base_link_name = None
    target_link_name = None
    try:
        if urdf_path: # Load URDF from file path.
            urdf_model = yourdfpy.URDF.load(urdf_path)
        elif robot_type: # Load built-in robot description.
            urdf_model = load_robot_description(robot_type)
        else:
            raise ValueError("[custom_urdf_ik] Either 'robot_type' or 'urdf_path' must be provided.")
        # Filter out 'world' and 'base' links.
        all_links = [l for l in urdf_model.link_map.keys() if l not in ("world", "base")]
        if not all_links:
            raise RuntimeError("[custom_urdf_ik] No valid links found in URDF.")

        base_link_name = all_links[0]
        target_link_name = all_links[-1]
        viser_urdf = ViserUrdf(server, urdf_or_path=urdf_model, root_node_name=f"/{base_link_name}")
        print(f"[custom_urdf_ik] URDF base_link: '{base_link_name}'")
        print(f"[custom_urdf_ik] URDF last_link: '{target_link_name}'")
        print(f"[custom_urdf_ik] URDF all_links: {all_links}")
        robot = pk.Robot.from_urdf(urdf_model)
    except Exception as e:
        print(f"[custom_urdf_ik] Error loading URDF: {e}")
        return

    # target_link_name = "panda_hand" # TODO: make this a parameter by user.
    bounds = viser_urdf._urdf.scene.bounds
    center = (bounds[0] + bounds[1]) / 2
    pos = center + np.array([0, 0, 0.2])  # 20cm above center
    # Create interactive controller with initial position.
    ik_target = server.scene.add_transform_controls(
        "/ik_target", scale=0.2, position=pos, wxyz=(0, 0, 1, 0)) # TODO: position and wxyz as parameter
    
    # Create sliders in GUI that help us move the robot joints.
    with server.gui.add_folder("Joint position control"):
        (slider_handles, initial_config) = create_robot_control_sliders(server, viser_urdf)
    
    print(f"[custom_urdf_ik] Initial configuration: {initial_config}")

    # Set initial robot configuration.
    viser_urdf.update_cfg(np.array(initial_config))

    # Create grid with the minimum z value of the trimesh scene.
    server.scene.add_grid("/grid", width=2, height=2, position=(0.0, 0.0, viser_urdf._urdf.scene.bounds[0, 2],),)

    # Create joint reset button.
    reset_button = server.gui.add_button("Reset")

    @reset_button.on_click
    def _(_):
        for s, init_q in zip(slider_handles, initial_config):
            s.value = init_q

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("[custom_urdf_ik] Shutting down Viser server...")
        server.stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    while True:
        # Solve IK
        try:
            ik_solution = pks.solve_ik(
                robot=robot,
                target_link_name=target_link_name,
                target_position=np.array(ik_target.position),
                target_wxyz=np.array(ik_target.wxyz),
            )
            if ik_solution is not None:
                # Update sliders with the IK solution.
                for slider, value in zip(slider_handles, ik_solution):
                    slider.value = value
        except Exception as e:
            print(f"[custom_urdf_ik] Error solving IK: {e}")
        # time.sleep(0.01)


if __name__ == "__main__":
    tyro.cli(main)