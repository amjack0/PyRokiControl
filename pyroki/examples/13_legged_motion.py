# Minimal PyRoki + Viser humanoid visualization
import pyroki as pk
import viser
from viser.extras import ViserUrdf
from robot_descriptions.loaders.yourdfpy import load_robot_description
import numpy as np
import time

# Basic Motion planning for legged robots


def main():
    robot_name = "anymal_d_description"
    try:
        urdf = load_robot_description(robot_name)
        robot = pk.Robot.from_urdf(urdf)
    except Exception as e:
        print(f"Failed to load {robot_name}: {e}")

    server = viser.ViserServer()
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")
    server.scene.add_grid("/ground", width=2, height=2, position=(0, 0, -0.68))

    # Visualize the default configuration
    urdf_vis.update_cfg(np.zeros(len(robot.joints.names)))
    # Keep the server running
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()