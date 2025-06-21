# PyRokiControl - Basic Inverse Kinematics Example

This project demonstrates basic inverse kinematics (IK) control of a robot end-effector using [PyRoki](https://pyroki-toolkit.github.io/) and Viser for visualization.

## Features:

- Visualizes custom URDF files or use built-in robot descriptions.
- Inverse Kinematics (IK) to control the robot's end-effector.
- Moves the end-effector along circular or linear paths using IK.
- Interactive 3D visualization of the robot using Viser.

## Requirements:

- Python 3.8+
- [PyRoki](https://github.com/chungmin99/pyroki)
- Viser
- NumPy

Install dependencies with:

```sh
pip install pyroki viser numpy
```

## Usage:

1. Clone this repository.
2. Run the example:

```sh
python pyroki/examples/11_basic_motion.py
```

By default, the script moves the end-effector along a linear path. To use a circular path, uncomment the relevant section in `main()`.

```
For custom URDF and IK:
python 13_custom_urdf_ik.py [--robot-type <robot_type>] [--urdf-path <path_to_custom_urdf>] [--target-link-name <link_name>]
```

## Demo:
- Boston Dynamics - Spot

https://github.com/user-attachments/assets/ad603933-f311-4947-8684-9f9543d1c2d8

- Atlas, Universal Robot, Fanuc and Custom Arm

https://github.com/user-attachments/assets/f43c3819-eb76-4d6a-a9b6-70f2d79e5952

