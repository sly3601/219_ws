#!/usr/bin/env python3
"""Read-only xacro/URDF and coordinate checks; never builds or starts ROS nodes.

This checks deployed descriptions and the coordinate algebra, not the execution
of FloatingBaseModel's C++ code or the physical correctness of motor calibration.
"""

import argparse
import copy
import json
import math
from pathlib import Path
import random
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


# Frozen identification reference, used only for auditing, never by the controller.
FROZEN = {
    "hip": (9.833942875227822e-10, 1.0170625680080705, 0.0),
    "thigh": (0.09591841174893605, 1.2980958388469037, 0.0),
    "calf": (1.6499038712067307e-09, 1.0794450668988391, 0.055704868111569605),
}
LEGS = ("FR", "FL", "RR", "RL")
KINDS = ("hip", "thigh", "calf")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sign(value):
    return (value > 0.0) - (value < 0.0)


def read_parameters(joint):
    name = joint.get("name")
    dynamics = joint.findall("dynamics")
    identified = joint.findall("identified_dynamics")
    require(len(dynamics) == len(identified) == 1, f"{name}: missing/duplicate metadata")
    values = (float(dynamics[0].attrib["damping"]),
              float(dynamics[0].attrib["friction"]),
              float(identified[0].attrib["Ja"]))
    require(all(math.isfinite(v) and v >= 0.0 for v in values), f"{name}: invalid B/Fc/Ja")
    return values


def check_description(root, controller, motors, frozen_links, checker):
    joints = {j.attrib["name"]: j for j in root.findall("joint")}
    moving = [j for j in joints.values() if j.get("type") != "fixed"]
    require(len(moving) == 12, "Expected exactly 12 moving joints")
    paths = []
    parents = {j.find("child").attrib["link"]: j for j in joints.values()}
    for foot in controller["feet_names"]:
        link = foot
        path = []
        while link != controller["base_name"]:
            joint = parents[link]
            if joint.get("type") != "fixed":
                path.append(joint.attrib["name"])
            link = joint.find("parent").attrib["link"]
        paths.extend(reversed(path))
    expected_names = [f"{leg}_{kind}_joint" for leg in LEGS for kind in KINDS]
    require(paths == controller["joints"] == expected_names,
            "URDF foot paths, controller joint order and expected leg order disagree")
    require(set(motors) == set(paths), "Hardware and model joint names disagree")

    generator = random.Random(219)
    for index, name in enumerate(paths):
        joint = joints[name]
        kind = name.split("_")[1]
        B, Fc, Ja = read_parameters(joint)
        require((B, Fc, Ja) == FROZEN[kind], f"{name}: differs from frozen identification")
        expected_axis = [1.0, 0.0, 0.0] if kind == "hip" else [0.0, 1.0, 0.0]
        require(list(map(float, joint.find("axis").attrib["xyz"].split())) == expected_axis,
                f"{name}: unexpected URDF axis")
        direction = motors[name]["direction"]
        require(direction in (-1, 1), f"{name}: invalid hardware direction")
        require(math.isfinite(float(motors[name]["offset"])), f"{name}: invalid offset")

        # Verify coordinate covariance and passive dissipation with each joint's
        # actual hardware sign, including zero, tiny speeds and both directions.
        samples = [(v, a) for v in (-2.0, -0.01, -1e-9, 0.0, 1e-9, 0.01, 2.0)
                   for a in (-10.0, 0.0, 10.0)]
        samples.extend((generator.uniform(-20, 20), generator.uniform(-100, 100))
                       for _ in range(100))
        for velocity_raw, acceleration_raw in samples:
            velocity_joint = direction * velocity_raw
            acceleration_joint = direction * acceleration_raw
            demand_joint = B * velocity_joint + Fc * sign(velocity_joint) + Ja * acceleration_joint
            demand_raw = demand_joint / direction
            reference_raw = B * velocity_raw + Fc * sign(velocity_raw) + Ja * acceleration_raw
            require(math.isclose(demand_raw, reference_raw, rel_tol=1e-13, abs_tol=1e-13),
                    f"{name}: torque direction does not transform with velocity")
            passive_power = -(B * velocity_joint + Fc * sign(velocity_joint)) * velocity_joint
            require(passive_power <= 0.0, f"{name}: passive friction would inject energy")
        require(B * 0.0 + Fc * sign(0.0) == 0.0, f"{name}: unexpected zero-speed rule")
        require(Ja == 0.0 or 6 + index in (8, 11, 14, 17), f"{name}: wrong Ja index")

    for leg in LEGS:
        for suffix in ("hip_rotor", "thigh_rotor", "calf_rotor", "foot"):
            link = root.find(f"./link[@name='{leg}_{suffix}']")
            require(link is not None and link.find("inertial") is None,
                    f"{leg}_{suffix}: attachment missing or inertia counted twice")
    if frozen_links:
        for name, expected in frozen_links.items():
            block = root.find(f"./link[@name='{name}']/inertial")
            actual = {"mass": float(block.find("mass").attrib["value"])}
            actual.update(zip(("com_x", "com_y", "com_z"),
                              map(float, block.find("origin").attrib["xyz"].split())))
            actual.update({k: float(v) for k, v in block.find("inertia").attrib.items()})
            require(all(abs(actual[k] - v) <= 1e-14 for k, v in expected.items()),
                    f"{name}: frozen rigid-body inertia changed")

    # check_urdf is an installed XML/URDF parser, not a build or a simulator.
    with tempfile.NamedTemporaryFile(suffix=".urdf", mode="w", encoding="utf-8") as stream:
        stream.write(ET.tostring(root, encoding="unicode"))
        stream.flush()
        result = subprocess.run([checker, stream.name], capture_output=True, text=True, check=False)
    require(result.returncode == 0, "URDF parser rejected custom metadata:\n" + result.stderr)


def check_invalid_metadata(root):
    # These exercise the offline audit's rejection rules, not the C++ parser.
    original = root.find("./joint[@name='FL_calf_joint']")
    mutations = [lambda j: j.remove(j.find("identified_dynamics")),
                 lambda j: j.append(copy.deepcopy(j.find("identified_dynamics"))),
                 lambda j: j.find("identified_dynamics").set("Ja", "-1"),
                 lambda j: j.find("identified_dynamics").set("Ja", "nan"),
                 lambda j: j.find("dynamics").set("friction", "-1")]
    for mutate in mutations:
        joint = copy.deepcopy(original)
        mutate(joint)
        try:
            read_parameters(joint)
        except (KeyError, ValueError):
            continue
        raise ValueError("Offline audit accepted malformed dynamics")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--handoff", type=Path, help="Also compare all 12 rigid-body inertias to handoff JSON")
    args = parser.parse_args()
    share = Path(get_package_share_directory("sysu219_description"))
    controller = yaml.safe_load((share / "config/robot_control.yaml").read_text())["sysu219_guide_controller"]["ros__parameters"]
    hardware = yaml.safe_load((share / "config/hardware_config.yaml").read_text())["motors"]
    motors = {motor["name"]: motor for motor in hardware}
    require(len(motors) == len(hardware), "Duplicate hardware joint names")
    checker = shutil.which("check_urdf")
    require(checker is not None, "Source the installed ROS environment: check_urdf is unavailable")
    frozen_links = None
    if args.handoff:
        match = re.search(r'```json\s*\n(\{\s*\n\s*"model".*?\n\})\s*\n```',
                          args.handoff.read_text(), re.S)
        require(match is not None, "Handoff has no embedded frozen parameter JSON")
        frozen_links = json.loads(match.group(1))["links"]

    print(f"Description package: {share}")
    modes = (("hardware", {}),
             ("gazebo_classic", {"GAZEBO": "true", "CLASSIC": "true"}),
             ("gazebo", {"GAZEBO": "true", "CLASSIC": "false"}),
             ("fixed_base_classic", {"GAZEBO": "true", "CLASSIC": "true", "DEBUG": "true"}))
    for label, mapping in modes:
        root = ET.fromstring(xacro.process_file(str(share / "xacro/robot.xacro"), mappings=mapping).toxml())
        check_description(root, controller, motors, frozen_links, checker)
        check_invalid_metadata(root)
        print(f"PASS {label}: 12 joint mappings/parameters/axes, coordinate signs, attachments, URDF parser")
    for leg in LEGS:
        directions = [motors[f"{leg}_{kind}_joint"]["direction"] for kind in KINDS]
        print(f"{leg} hardware direction (hip/thigh/calf): {directions}")
    print("PASS static descriptions and coordinate algebra only; C++ not built/executed; no robot started.")


if __name__ == "__main__":
    main()
