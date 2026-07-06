# MuJoCo Biped Robot Model

MuJoCo / MJCF で作成した小型2足ロボットモデルと、モデル確認用の Python スクリプトです。

`robo.xml` を MuJoCo Viewer で表示し、初期姿勢の確認、IMU 相当の姿勢表示、IK スライダーによる簡易操作を行えます。

PWMサーボによる2足歩行ロボットです。

Details:
https://homemadegarbage.com/servomodel02

## Files

```text
.
├── robo.xml
├── model_viewer.py
├── ik_mujoco_sliders.py
└── assets/

```

## Overview

This repository contains a MuJoCo model of a small biped robot.

The robot has 8 servo actuators:

```text
servo1_pos
servo2_pos
servo3_pos
servo4_pos
servo5_pos
servo6_pos
servo7_pos
servo8_pos
```

The model uses STL mesh files stored in the `assets` directory.

The simulation timestep is set to `0.001` seconds.

## Requirements

- Python 3
- MuJoCo
- mujoco Python package

Install the Python package:

```bash
pip install mujoco
```

## Usage

### 1. View the MuJoCo model

```bash
python model_viewer.py
```

This opens `robo.xml` in the MuJoCo Viewer.

At startup, the script sets the following initial hip roll angles:

```text
servo6_joint = -15 deg
servo8_joint = +15 deg
```

It also prints IMU-like information to the console:

```text
roll
pitch
roll_rate
pitch_rate
```

### Run without viewer

```bash
python model_viewer.py --no-viewer
```

This only loads the model, applies the initial posture, and prints the IMU-like values.

### Pause physics stepping

```bash
python model_viewer.py --pause
```

This is useful for static model inspection.

### Change IMU print rate

```bash
python model_viewer.py --imu-print-rate 20
```

## IK Slider Viewer

```bash
python ik_mujoco_sliders.py
```

This script opens the model with additional IK control sliders in the MuJoCo Viewer.

The script does not generate a new XML file.  
It reads `robo.xml`, adds virtual IK slider joints and actuators in memory, and loads the modified model directly with `mujoco.MjModel.from_xml_string()`.

Open the MuJoCo Viewer Control panel and move the following sliders:

```text
ik_left_y_ctrl
ik_right_y_ctrl
ik_height_ctrl
ik_roll_ctrl
ik_yaw_ctrl
```

## IK Controls

| Slider | Description |
|---|---|
| `ik_left_y_ctrl` | Left leg Y target |
| `ik_right_y_ctrl` | Right leg Y target |
| `ik_height_ctrl` | Leg height target |
| `ik_roll_ctrl` | Body roll target |
| `ik_yaw_ctrl` | Hip yaw target |

The IK solver uses a simple two-link leg model.

```text
LEG_LENGTH = 0.035 m
PITCH_LIMIT = ±68 deg
```

The roll control includes fixed base offsets for the left and right hip roll servos:

```text
servo6 = -15 deg - roll
servo8 = +15 deg - roll
```

## Model Notes

The robot model is defined in `robo.xml`.

Main settings:

```xml
<option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"/>
```

The servo model uses MuJoCo position actuators with torque limits and damping/armature parameters.

The leg mechanism contains multiple hinge joints and equality constraints.  
These constraints connect sites on the thigh and lower-leg linkages to approximate the mechanical linkage structure.

The ground is defined as a plane:

```xml
<geom type="plane" size="5 5 0.1" pos="0 0 0" rgba="0.6 0.8 1.0 1"/>
```

## Directory Notes

`robo.xml` uses:

```xml
<compiler angle="radian" meshdir="assets" autolimits="true"/>
```

Therefore, the STL files must be placed in the `assets` directory relative to `robo.xml`.

## License

MIT License
