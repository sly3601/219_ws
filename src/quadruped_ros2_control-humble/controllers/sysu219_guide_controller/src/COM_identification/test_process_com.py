"""Offline synthetic-data checks; no ROS, hardware, build tools, or model edits."""
import csv
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from process_com import (G, JOINTS, PREFIXES, StaticModel, analyze, estimate_forces,
                         check_static_hold, fit_solution, rpy, skew)


def fixture():
    root = ET.Element("robot", name="static_test")

    def link(name, mass=0., com=(0, 0, 0)):
        item = ET.SubElement(root, "link", name=name)
        if mass:
            inertial = ET.SubElement(item, "inertial")
            ET.SubElement(inertial, "mass", value=str(mass))
            ET.SubElement(inertial, "origin", xyz=" ".join(map(str, com)))

    def joint(name, parent, child, origin, axis=None):
        item = ET.SubElement(root, "joint", name=name, type="fixed" if axis is None else "revolute")
        ET.SubElement(item, "parent", link=parent); ET.SubElement(item, "child", link=child)
        ET.SubElement(item, "origin", xyz=" ".join(map(str, origin)))
        if axis is not None:
            ET.SubElement(item, "axis", xyz=" ".join(map(str, axis)))
            ET.SubElement(item, "dynamics", damping=".1", friction="0")

    link("base"); link("trunk", 23., (-.025, .006, -.02))
    joint("base_trunk", "base", "trunk", (0, 0, 0))
    link("imu", .01, (.01, 0, 0)); joint("imu_fixed", "trunk", "imu", (0, .02, 0))
    for leg, x, side in (("FR", .34, -1), ("FL", .34, 1), ("RR", -.34, -1), ("RL", -.34, 1)):
        link(leg+"_hip", 1.5, (.005, side*.06, 0))
        link(leg+"_thigh", 1.3, (.01, 0, -.13))
        link(leg+"_calf", .8, (0, 0, -.12)); link(leg+"_foot")
        joint(leg+"_hip_joint", "trunk", leg+"_hip", (x, side*.094, 0), (1, 0, 0))
        joint(leg+"_thigh_joint", leg+"_hip", leg+"_thigh", (0, side*.124, 0), (0, 1, 0))
        joint(leg+"_calf_joint", leg+"_thigh", leg+"_calf", (0, 0, -.2835), (0, 1, 0))
        joint(leg+"_foot_fixed", leg+"_calf", leg+"_foot", (0, 0, -.2835))
    return ET.tostring(root, encoding="unicode")


def balanced_force(model, state, com):
    h = 23*com+state["other_moment"]
    A = np.vstack((np.tile(np.eye(3), (1, 4)), np.hstack([skew(p) for p in state["positions"]])))
    rhs = np.r_[-model.nominal_mass*state["gravity"], skew(state["gravity"]) @ h]
    force = np.linalg.lstsq(A, rhs, rcond=None)[0].reshape(4, 3)
    tau = state["g_joint"]-np.concatenate([J.T @ f for J, f in zip(state["jacobians"], force)])
    return force, tau


class StaticChecks(unittest.TestCase):
    def setUp(self):
        self.xml = fixture()
        self.feet = [leg+"_foot" for leg in ("FR", "FL", "RR", "RL")]
        self.model = StaticModel(self.xml, "base", self.feet)
        self.q = np.tile([0., .7, -1.4], 4)
        self.com = np.array([-.025, .006, -.02])

    def test_gravity_and_jacobians_against_finite_difference(self):
        R = rpy([.04, -.03, .2]); state = self.model.evaluate(self.q, R); step = 1e-6
        for j in range(12):
            plus, minus = self.q.copy(), self.q.copy(); plus[j] += step; minus[j] -= step
            a, b = self.model.evaluate(plus, R), self.model.evaluate(minus, R)
            gradient = -state["gravity"] @ ((a["other_moment"]-b["other_moment"])/(2*step))
            self.assertAlmostEqual(gradient, state["g_joint"][j], places=7)
            np.testing.assert_allclose((a["positions"]-b["positions"])[j//3]/(2*step),
                                       state["jacobians"][j//3, :, j%3], atol=1e-9)

    def test_force_sign_mass_com_and_rank(self):
        states, forces = [], []
        for angles in ([0, 0, 0], [.035, 0, 0], [-.035, 0, 0], [0, .035, 0], [0, -.035, 0]):
            state = self.model.evaluate(self.q, rpy(angles))
            expected, tau = balanced_force(self.model, state, self.com)
            actual, _ = estimate_forces(state, tau, np.zeros(12), self.model.B)
            np.testing.assert_allclose(actual, expected, atol=1e-10)
            states.append(state); forces.append(actual)
        fit = fit_solution(states, forces, self.model.other_mass)
        self.assertEqual(fit["rank"], 3)
        self.assertAlmostEqual(fit["mass"], self.model.nominal_mass, places=9)
        np.testing.assert_allclose(fit["com"], self.com, atol=1e-10)
        unobservable = fit_solution([states[0]]*3, [forces[0]]*3, self.model.other_mass)
        self.assertEqual(unobservable["rank"], 2)
        self.assertIsNone(unobservable["com"])

    def test_singular_jacobian_refused(self):
        state = self.model.evaluate(np.zeros(12), np.eye(3))
        with self.assertRaisesRegex(ValueError, "singular"):
            estimate_forces(state, np.zeros(12))

    def test_static_hold_reports_drift_and_acceleration_but_rejects_clear_motion(self):
        time = np.arange(1001)*.01
        q = np.tile(self.q, (len(time), 1)); dq = np.zeros_like(q)
        quats = np.tile([1., 0, 0, 0], (len(time), 1))
        gyro = np.zeros((len(time), 3)); accel = np.tile([0., 0., G], (len(time), 1))
        self.assertFalse(check_static_hold(time, q, dq, quats, gyro, accel)["warnings"])
        drift = q.copy(); drift[:, 0] += .0004*time
        self.assertTrue(check_static_hold(time, drift, dq, quats, gyro, accel)["warnings"])
        moving = accel.copy(); moving[400:450, 0] = .5
        self.assertTrue(check_static_hold(time, q, dq, quats, gyro, moving)["warnings"])
        drifting_quat = quats.copy(); angle = np.deg2rad(.04)*time
        drifting_quat[:, 0] = np.cos(angle/2); drifting_quat[:, 3] = np.sin(angle/2)
        self.assertTrue(check_static_hold(time, q, dq, drifting_quat, gyro, accel)["warnings"])
        fast = dq.copy(); fast[500, 0] = .05
        with self.assertRaisesRegex(ValueError, "velocity"):
            check_static_hold(time, q, fast, quats, gyro, accel)

    def test_end_to_end_and_incomplete_acquisition(self):
        with tempfile.TemporaryDirectory(prefix="com_id_offline_") as directory:
            folder = Path(directory)
            meta = dict(base="base", trunk="trunk", feet=self.feet, stand_q=self.q.tolist(), sample_s=10.,
                        pose_names=[str(i) for i in range(13)])
            (folder/"metadata.json").write_text(json.dumps(meta), encoding="utf-8")
            (folder/"robot_description.urdf").write_text(self.xml, encoding="utf-8")
            baseline = self.model.evaluate(self.q, np.eye(3))["positions"]
            header = ["steady_ns", "ros_ns", "sequence", "period_s", "pose", "phase"]
            header += [prefix+'_'+suffix for prefix in PREFIXES for suffix in ("q", "dq", "tau", "q_cmd", "kp", "kd")]
            header += ["qw", "qx", "qy", "qz", "gx", "gy", "gz", "ax", "ay", "az"]
            sequence = 0
            with (folder/"samples.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream); writer.writerow(header)
                for pose in range(13):
                    roll = .035 if pose == 9 else -.035 if pose == 11 else 0.
                    pitch = .035 if pose == 5 else -.035 if pose == 7 else 0.
                    height = -.005 if pose == 1 else .005 if pose == 3 else 0.
                    R = rpy([roll, pitch, 0]); q = self.q.copy()
                    goal = (R.T @ (baseline-[0, 0, height]).T).T
                    for _ in range(20):
                        state = self.model.evaluate(q, R)
                        for leg in range(4):
                            q[3*leg:3*leg+3] += np.linalg.solve(state["jacobians"][leg],
                                goal[leg]-state["positions"][leg])
                    state = self.model.evaluate(q, R)
                    _, tau = balanced_force(self.model, state, self.com)
                    # Explicit quaternion for a single-axis roll/pitch, including alternate signs.
                    quat = np.array([np.cos((roll+pitch)/2), np.sin(roll/2), np.sin(pitch/2), 0.])
                    acceleration = R.T @ np.array([0., 0., G])
                    for sample in range(1001):
                        row = [int((1+pose*20+sample*.01)*1e9), 0, sequence, .004, pose, 3]
                        for j in range(12):
                            row += [q[j], 0., tau[j], q[j], 260., 3.8]
                        row += [*(quat if sample % 2 else -quat), 0., 0., 0., *acceleration]
                        writer.writerow(row); sequence += 1
            (folder/"acquisition.json").write_text(json.dumps(dict(completed=True, writer_failed=False,
                written_samples=sequence)), encoding="utf-8")
            result = analyze(folder)
            np.testing.assert_allclose(result["trunk_com_link_m"], self.com, atol=1e-9)
            self.assertTrue(result["xyz_quality_checks_passed"], result["warnings"])
            expected = (23*self.com+self.model.evaluate(self.q, np.eye(3))["other_moment"])/self.model.nominal_mass
            np.testing.assert_allclose(result["fixstand_measured_pose_whole_com_B_m"], expected, atol=1e-9)
            self.assertFalse(result["automatic_model_update"])
            (folder/"acquisition.json").write_text('{"completed":false,"writer_failed":false}', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "incomplete"):
                analyze(folder)


if __name__ == "__main__":
    unittest.main()
