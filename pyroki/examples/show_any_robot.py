# Minimal PyRoki + Viser humanoid visualization
import pyroki as pk
import viser
from viser.extras import ViserUrdf
from robot_descriptions.loaders.yourdfpy import load_robot_description
import numpy as np
import time

# Minimal PyRoki + Viser example. 

# Visualize any robot from the list of available robot descriptions
ROBOT_DESCRIPTIONS = [
    'a1_description', 'a1_mj_description', 'ability_hand_description', 'adam_lite_mj_description',
    'aliengo_description', 'aliengo_mj_description', 'allegro_hand_description',
    'allegro_hand_mj_description', 'aloha_mj_description',
    'anymal_b_description', 'anymal_b_mj_description', 'anymal_c_description',
    'anymal_c_mj_description', 'anymal_d_description', 'apollo_mj_description',
    'arx_l5_mj_description', 'atlas_drc_description', 'atlas_v4_description',
    'b1_description', 'b2_description', 'barrett_hand_description',
    'baxter_description', 'berkeley_humanoid_description', 'bolt_description',
    'cassie_description', 'cassie_mj_description', 'cf2_description',
    'cf2_mj_description', 'double_pendulum_description', 'draco3_description',
    'edo_description', 'elf2_description', 'elf2_mj_description',
    'ergocub_description', 'eve_r3_description', 'fanuc_m710ic_description',
    'fetch_description', 'finger_edu_description', 'fr3_mj_description',
    'g1_description', 'g1_mj_description', 'gen2_description',
    'gen3_description', 'gen3_lite_description', 'gen3_mj_description',
    'ginger_description', 'go1_description', 'go1_mj_description',
    'go2_description', 'go2_mj_description', 'h1_description',
    'h1_mj_description', 'hyq_description', 'icub_description',
    'iiwa14_description', 'iiwa14_mj_description', 'iiwa7_description',
    'jaxon_description', 'jvrc_description', 'jvrc_mj_description',
    'laikago_description', 'leap_hand_mj_description', 'minitaur_description',
    'mini_cheetah_description', 'nextage_description', 'op3_mj_description',
    'panda_description', 'panda_mj_description', 'pepper_description',
    'piper_description', 'piper_mj_description', 'poppy_ergo_jr_description',
    'poppy_torso_description', 'pr2_description', 'r2_description',
    'reachy_description', 'rhea_description', 'robotiq_2f85_description',
    'robotiq_2f85_mj_description', 'robotiq_2f85_v4_mj_description',
    'romeo_description', 'rsk_description', 'sawyer_mj_description',
    'shadow_dexee_mj_description', 'shadow_hand_mj_description',
    'sigmaban_description', 'simple_humanoid_description',
    'skydio_x2_description', 'skydio_x2_mj_description', 'solo_description',
    'so_arm100_mj_description', 'spot_mj_description', 'spryped_description',
    'stretch_3_mj_description', 'stretch_description', 'stretch_mj_description',
    'talos_description', 'talos_mj_description', 'tiago_description',
    'trifinger_edu_description', 'upkie_description', 'ur10e_mj_description',
    'ur10_description', 'ur3_description', 'ur5e_mj_description',
    'ur5_description', 'valkyrie_description', 'viper_mj_description',
    'widow_mj_description', 'xarm7_mj_description', 'yumi_description',
    'z1_description', 'z1_mj_description', '_empty_description'
]


def main():
    # Load the humanoid robot URDF
    urdf = load_robot_description("cassie_description")
    robot = pk.Robot.from_urdf(urdf)

    # Start the Viser server, add the robot and ground grid
    server = viser.ViserServer()
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")
    server.scene.add_grid("/ground", width=2, height=2) # position=(0, 0, -0.75)

    # Visualize the default configuration
    urdf_vis.update_cfg(np.zeros(len(robot.joints.names)))

    print("[show_any_robot] Viser server running. Open the link in your browser to view the robot...")

    # Keep the server running
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()

# TODO: every robot loades with ground grid at the origin, it should be at the robot's base and not at the origin. It shoould work for all robots.
# detect the robot's base position and set the ground grid there.
# load robots with default configuration, and NOT with zero configuration.
# test all robots in the list and visualize them.