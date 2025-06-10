# PyRokiControl - Basic Inverse Kinematics Example

This project demonstrates basic inverse kinematics (IK) control of a robot end-effector using [PyRoki](https://pyroki-toolkit.github.io/) and Viser for visualization.

## Features

- Visualizes a robot arm using a URDF description.
- Moves the end-effector along circular or linear paths using IK.
- Real-time visualization and timing feedback.

## Requirements

- Python 3.8+
- [PyRoki](https://github.com/chungmin99/pyroki)
- Viser
- NumPy

Install dependencies with:

```sh
pip install pyroki viser numpy
```

## Usage

1. Clone this repository.
2. Run the example:

```sh
python pyroki/examples/11_basic_motion.py
```

By default, the script moves the end-effector along a linear path. To use a circular path, uncomment the relevant section in `main()`.

## File Structure

- `pyroki/examples/11_basic_motion.py` — Main example script for IK motion control.