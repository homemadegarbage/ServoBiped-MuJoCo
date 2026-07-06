# -*- coding: utf-8 -*-
"""Use MuJoCo viewer actuator sliders as high-level IK controls for bipe2.xml.

This script is self-contained: it includes the IK helper functions and does not
write a generated MJCF file.  The modified model is built in memory and loaded
with ``mujoco.MjModel.from_xml_string``.

Hip roll has a fixed base shift:
  servo6 = -15deg - roll
  servo8 = +15deg - roll
"""

from __future__ import annotations

import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parent
SOURCE_XML = ROOT / "robo.xml"

LEG_LENGTH = 0.035  # Link length in meters from the four-bar site spacing.
PITCH_LIMIT = math.radians(68.0)

SERVO6_BASE = math.radians(-15.0)
SERVO8_BASE = math.radians(15.0)

IK_ACTUATORS = {
    "ik_left_y_ctrl": 0.0,
    "ik_right_y_ctrl": 0.0,
    "ik_height_ctrl": 0.065,
    "ik_roll_ctrl": 0.0,
    "ik_yaw_ctrl": 0.0,
}


@dataclass
class LegTarget:
    y: float = 0.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def leg_ik(y: float, z: float) -> tuple[float, float]:
    """Return (hip_pitch, knee_pitch) in radians."""
    reach = clamp(math.hypot(y, z), 1.0e-5, 2.0 * LEG_LENGTH - 1.0e-5)

    cos1 = clamp(reach / (2.0 * LEG_LENGTH), -1.0, 1.0)
    th1d = math.acos(cos1)

    cos2 = clamp(
        (2.0 * LEG_LENGTH * LEG_LENGTH - reach * reach)
        / (2.0 * LEG_LENGTH * LEG_LENGTH),
        -1.0,
        1.0,
    )
    th2d = math.acos(cos2)

    phi = math.atan2(y, z)
    hip_pitch = phi + th1d
    knee_pitch = (math.pi - th2d) - hip_pitch

    return (
        clamp(hip_pitch, -PITCH_LIMIT, PITCH_LIMIT),
        clamp(knee_pitch, -PITCH_LIMIT, PITCH_LIMIT),
    )


def actuator_id(model: mujoco.MjModel, name: str) -> int:
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if idx < 0:
        raise KeyError(f"Actuator not found: {name}")
    return idx


def set_ctrl(model: mujoco.MjModel, data: mujoco.MjData, act_id: int, value: float) -> None:
    low, high = model.actuator_ctrlrange[act_id]
    data.ctrl[act_id] = clamp(value, low, high)


def indent(element: ET.Element, level: int = 0) -> None:
    pad = "\n" + level * "  "
    child_pad = "\n" + (level + 1) * "  "

    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_pad
        for child in element:
            indent(child, level + 1)
        if not element[-1].tail or not element[-1].tail.strip():
            element[-1].tail = pad

    if level and (not element.tail or not element.tail.strip()):
        element.tail = pad


def require(parent: ET.Element | None, name: str) -> ET.Element:
    if parent is None:
        raise ValueError(f"Missing <{name}> in {SOURCE_XML}")
    return parent


def _is_relative_file(path_text: str) -> bool:
    return bool(path_text) and not Path(path_text).is_absolute()


def _set_compiler_asset_dirs(root: ET.Element) -> None:
    """Make mesh/texture paths stable when loading MJCF from an XML string."""
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)

    meshdir = compiler.get("meshdir")
    texturedir = compiler.get("texturedir")

    compiler.set("meshdir", str((ROOT / meshdir).resolve()) if meshdir else str(ROOT))
    compiler.set("texturedir", str((ROOT / texturedir).resolve()) if texturedir else str(ROOT))


def _make_include_paths_absolute(root: ET.Element) -> None:
    """Resolve MJCF include files because from_xml_string has no source path."""
    for element in root.iter("include"):
        file_name = element.get("file")
        if file_name and _is_relative_file(file_name):
            element.set("file", str((ROOT / file_name).resolve()))


def generate_ik_xml_string() -> str:
    tree = ET.parse(SOURCE_XML)
    root = tree.getroot()
    root.set("model", f"{root.get('model', 'bipe2')}_ik_sliders")

    _set_compiler_asset_dirs(root)
    _make_include_paths_absolute(root)

    worldbody = require(root.find("worldbody"), "worldbody")
    actuator = require(root.find("actuator"), "actuator")

    if worldbody.find(".//body[@name='ik_slider_body']") is None:
        slider_body = ET.Element("body", {"name": "ik_slider_body", "pos": "0 0 -1"})
        ET.SubElement(
            slider_body,
            "inertial",
            {"pos": "0 0 0", "mass": "0.001", "diaginertia": "1e-8 1e-8 1e-8"},
        )
        ET.SubElement(
            slider_body,
            "joint",
            {"name": "ik_left_y", "type": "slide", "axis": "1 0 0", "damping": "1000"},
        )
        ET.SubElement(
            slider_body,
            "joint",
            {"name": "ik_right_y", "type": "slide", "axis": "0 1 0", "damping": "1000"},
        )
        ET.SubElement(
            slider_body,
            "joint",
            {"name": "ik_height", "type": "slide", "axis": "0 0 1", "damping": "1000"},
        )
        ET.SubElement(
            slider_body,
            "joint",
            {"name": "ik_roll", "type": "hinge", "axis": "1 0 0", "damping": "1000"},
        )
        ET.SubElement(
            slider_body,
            "joint",
            {"name": "ik_yaw", "type": "hinge", "axis": "0 0 1", "damping": "1000"},
        )
        worldbody.append(slider_body)

    existing_actuators = {child.get("name") for child in actuator}
    slider_actuators = [
        ("ik_left_y_ctrl", "ik_left_y", "-0.035 0.035"),
        ("ik_right_y_ctrl", "ik_right_y", "-0.035 0.035"),
        ("ik_height_ctrl", "ik_height", "0.020 0.069"),
        ("ik_roll_ctrl", "ik_roll", f"{math.radians(-35.0)} {math.radians(35.0)}"),
        ("ik_yaw_ctrl", "ik_yaw", f"{math.radians(-60.0)} {math.radians(60.0)}"),
    ]

    insert_at = 0
    for name, joint, ctrlrange in slider_actuators:
        if name in existing_actuators:
            continue

        actuator.insert(
            insert_at,
            ET.Element(
                "motor",
                {
                    "name": name,
                    "joint": joint,
                    "ctrllimited": "true",
                    "ctrlrange": ctrlrange,
                    "gear": "1e-6",
                },
            ),
        )
        insert_at += 1

    indent(root)
    return ET.tostring(root, encoding="unicode")


def apply_ik_bipe2(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    ids: dict[str, int],
    left: LegTarget,
    right: LegTarget,
    z: float,
    roll: float,
    yaw: float,
) -> None:
    l_hip, l_knee = leg_ik(left.y, z)
    r_hip, r_knee = leg_ik(right.y, z)

    set_ctrl(model, data, ids["servo1_pos"], -l_knee)
    set_ctrl(model, data, ids["servo2_pos"], -l_hip)
    set_ctrl(model, data, ids["servo3_pos"], r_knee)
    set_ctrl(model, data, ids["servo4_pos"], r_hip)
    set_ctrl(model, data, ids["servo5_pos"], yaw)
    set_ctrl(model, data, ids["servo7_pos"], yaw)
    set_ctrl(model, data, ids["servo6_pos"], SERVO6_BASE - roll)
    set_ctrl(model, data, ids["servo8_pos"], SERVO8_BASE - roll)


def main() -> None:
    xml_string = generate_ik_xml_string()

    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)

    ik_ids = {name: actuator_id(model, name) for name in IK_ACTUATORS}
    servo_ids = {
        name: actuator_id(model, name)
        for name in (
            "servo1_pos",
            "servo2_pos",
            "servo3_pos",
            "servo4_pos",
            "servo5_pos",
            "servo6_pos",
            "servo7_pos",
            "servo8_pos",
        )
    }

    for name, value in IK_ACTUATORS.items():
        data.ctrl[ik_ids[name]] = value

    with mujoco.viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True) as viewer:
        print("Loaded IK slider model from memory.")
        print("Open the viewer Control panel and use ik_*_ctrl sliders.")
        print("servo6 has -15deg base shift, servo8 has +15deg base shift.")
        print("No site L-R distance constraint is applied.")

        while viewer.is_running():
            left = LegTarget(float(data.ctrl[ik_ids["ik_left_y_ctrl"]]))
            right = LegTarget(float(data.ctrl[ik_ids["ik_right_y_ctrl"]]))
            z = float(data.ctrl[ik_ids["ik_height_ctrl"]])
            roll = float(data.ctrl[ik_ids["ik_roll_ctrl"]])
            yaw = float(data.ctrl[ik_ids["ik_yaw_ctrl"]])

            apply_ik_bipe2(model, data, servo_ids, left, right, z, roll, yaw)
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
