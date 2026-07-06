# -*- coding: utf-8 -*-
import argparse
import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT / "robo.xml"


def quat_to_roll_pitch(quat):
    w, x, y, z = quat
    r02 = 2.0 * (x * z + w * y)
    r12 = 2.0 * (y * z - w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)

    roll_y = math.atan2(r02, r22)
    sin_pitch_x = -r12
    if abs(sin_pitch_x) >= 1.0:
        pitch_x = math.copysign(math.pi / 2.0, sin_pitch_x)
    else:
        pitch_x = math.asin(sin_pitch_x)
    return roll_y, pitch_x


def base_imu_observation(data):
    roll, pitch = quat_to_roll_pitch(data.qpos[3:7])
    roll_rate = float(data.qvel[4])
    pitch_rate = float(data.qvel[3])
    return roll, pitch, roll_rate, pitch_rate


def format_imu_line(data):
    roll, pitch, roll_rate, pitch_rate = base_imu_observation(data)
    return (
        f"roll={math.degrees(roll):8.3f} deg  "
        f"pitch={math.degrees(pitch):8.3f} deg  "
        f"roll_rate={math.degrees(roll_rate):8.3f} deg/s  "
        f"pitch_rate={math.degrees(pitch_rate):8.3f} deg/s"
    )


def set_joint_angle(model, data, joint_name, angle_rad):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError(f"Joint not found: {joint_name}")
    qpos_adr = model.jnt_qposadr[joint_id]
    data.qpos[qpos_adr] = angle_rad


def set_actuator_ctrl(model, data, actuator_name, angle_rad):
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
    if actuator_id < 0:
        raise ValueError(f"Actuator not found: {actuator_name}")
    data.ctrl[actuator_id] = angle_rad


def site_pos(model, data, site_name):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if site_id < 0:
        raise ValueError(f"Site not found: {site_name}")
    return data.site_xpos[site_id].copy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--pause", action="store_true", help="do not step physics while the viewer is open")
    parser.add_argument("--step", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--imu-print-rate", type=float, default=10.0, help="IMU console print rate in Hz")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    servo6 = math.radians(-15.0)
    servo8 = math.radians(15.0)
    set_joint_angle(model, data, "servo6_joint", servo6)
    set_joint_angle(model, data, "servo8_joint", servo8)
    set_actuator_ctrl(model, data, "servo6_pos", servo6)
    set_actuator_ctrl(model, data, "servo8_pos", servo8)
    mujoco.mj_forward(model, data)


    print("model:", MODEL_PATH.name)
    print("servo6_joint: -15.000 deg")
    print("servo8_joint:  15.000 deg")
    print("imu:", format_imu_line(data))

    if args.no_viewer:
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Press ESC to quit.")
        print("MuJoCo mouse perturb works while physics is stepping. Use --pause only for static inspection.")
        next_imu_print = time.monotonic()
        imu_period = 1.0 / args.imu_print_rate if args.imu_print_rate > 0.0 else None
        while viewer.is_running():
            if args.pause and not args.step:
                mujoco.mj_forward(model, data)
            else:
                mujoco.mj_step(model, data)
            if imu_period is not None and time.monotonic() >= next_imu_print:
                print("imu:", format_imu_line(data))
                next_imu_print += imu_period
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
