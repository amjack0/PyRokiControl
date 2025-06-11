"""
Minimal PyRoki + Viser + Custom URDF
- https://github.com/nerfstudio-project/viser/blob/main/examples/09_urdf_visualizer.py

"""
import pyroki as pk
import viser
from viser.extras import ViserUrdf
from robot_descriptions.loaders.yourdfpy import load_robot_description
import numpy as np
import time
import tyro
from typing import Literal, Optional
import yourdfpy

# Basic Motion planning for legged robots

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
        "g1_description", "h1_description", "anymal_c_description", "go2_description", "anymal_d_description"]]
        = "panda_description",
    urdf_path: Optional[str] = None, # Added urdf_path as an optional input
) -> None:
    # Start viser server.
    server = viser.ViserServer()

    urdf_path = r"D:\Python_projects\PyRokiControl\custom_urdf\02_custom_quadruped.urdf" # TODO: as a parameter

    # Load URDF based on input
    if urdf_path:
        # Load the URDF file using yourdfpy.URDF.load() first
        try:
            urdf_model = yourdfpy.URDF.load(urdf_path)
            viser_urdf = ViserUrdf(server, urdf_or_path=urdf_model)
        except Exception as e:
            print(f"[custom_urdf_ik] Error loading custom URDF from '{urdf_path}': {e}")
            return
    elif robot_type:
        # Otherwise use available robot_type
        viser_urdf = ViserUrdf(server, urdf_or_path=load_robot_description(robot_type))
    else:
        raise ValueError("[custom_urdf_ik] Either 'robot_type' or 'urdf_path' must be provided.")
    
    # Create sliders in GUI that help us move the robot joints.
    with server.gui.add_folder("Joint position control"):
        (slider_handles, initial_config) = create_robot_control_sliders(server, viser_urdf)

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

    # Sleep forever.
    while True:
        time.sleep(10.0)


if __name__ == "__main__":
    tyro.cli(main)