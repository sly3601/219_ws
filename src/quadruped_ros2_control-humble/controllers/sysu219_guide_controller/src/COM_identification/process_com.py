#!/usr/bin/env python3
"""Offline static CoM identification. NumPy only; never starts ROS or modifies a model."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

JOINTS = [f"{leg}_{kind}_joint" for leg in ("FR", "FL", "RR", "RL")
          for kind in ("hip", "thigh", "calf")]
PREFIXES = [j.removesuffix("_joint") for j in JOINTS]
G = 9.81


def require(condition, message):
    if not condition:
        raise ValueError(message)


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation(axis, angle):
    axis = np.asarray(axis, float)
    axis /= np.linalg.norm(axis)
    S = skew(axis)
    return np.eye(3) + np.sin(angle) * S + (1 - np.cos(angle)) * (S @ S)


def rpy(values):
    r, p, y = values
    return rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)


def quaternion(values):
    q = np.asarray(values, float)
    require(np.all(np.isfinite(q)) and abs(np.linalg.norm(q) - 1) < .01, "Invalid quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def xyz(element, attribute="xyz"):
    return np.fromstring(element.get(attribute, "0 0 0"), sep=" ") if element is not None else np.zeros(3)


class StaticModel:
    """URDF tree FK + COM Jacobians. Static forces do not require inertia tensors or trunk COM."""
    def __init__(self, xml, base, feet, trunk="trunk"):
        root = ET.fromstring(xml)
        links = {link.get("name"): link for link in root.findall("link")}
        require(base in links and trunk in links, "Missing base/trunk link")
        children = {}
        for joint in root.findall("joint"):
            require(joint.find("mimic") is None, "Mimic joints not supported")
            children.setdefault(joint.find("parent").get("link"), []).append(joint)
        self.nodes = []
        seen_joints = []
        self.B, self.Fc = np.zeros(12), np.zeros(12)

        def append(name, parent=-1, joint=None):
            inertia = links[name].find("inertial")
            mass = float(inertia.find("mass").get("value")) if inertia is not None else 0.
            com = xyz(inertia.find("origin")) if inertia is not None else np.zeros(3)
            origin = joint.find("origin") if joint is not None else None
            j = -1
            axis = np.array([1., 0., 0.])
            if joint is not None and joint.get("type") != "fixed":
                require(joint.get("type") in ("revolute", "continuous"), "Unsupported joint")
                require(joint.get("name") in JOINTS, "Unexpected moving joint")
                j = JOINTS.index(joint.get("name")); seen_joints.append(j)
                axis = xyz(joint.find("axis")) if joint.find("axis") is not None else axis
                require(np.linalg.norm(axis) > 1e-8, "Zero joint axis")
                axis = axis / np.linalg.norm(axis)
                dynamics = joint.find("dynamics")
                require(dynamics is not None, "Missing identified joint friction")
                self.B[j] = float(dynamics.get("damping", "0"))
                self.Fc[j] = float(dynamics.get("friction", "0"))
            require(mass >= 0 and np.isfinite(mass) and np.all(np.isfinite(com)), "Invalid link inertia")
            index = len(self.nodes)
            self.nodes.append((name, parent, j, xyz(origin), rpy(xyz(origin, "rpy")), axis, mass, com))
            for child in children.get(name, []):
                append(child.find("child").get("link"), index, child)
        append(base)
        require(sorted(seen_joints) == list(range(12)), "Expected twelve independent joints")
        names = [node[0] for node in self.nodes]
        self.feet = [names.index(name) for name in feet]
        self.trunk = names.index(trunk)
        # Trunk must be rigidly attached to base: its first moment is constant in B.
        index = self.trunk
        while index >= 0:
            require(self.nodes[index][2] == -1, "Trunk cannot be below a moving joint")
            index = self.nodes[index][1]
        self.nominal_mass = sum(node[6] for node in self.nodes)
        self.other_mass = self.nominal_mass - self.nodes[self.trunk][6]
        require(self.other_mass > 0 and np.all(self.B >= 0) and np.all(self.Fc >= 0), "Invalid model parameters")

    def evaluate(self, q, R):
        q = np.asarray(q, float)
        require(q.shape == (12,) and np.all(np.isfinite(q)), "Invalid joint positions")
        gravity = R.T @ np.array([0., 0., -G])
        states, first_other, g_joint = [], np.zeros(3), np.zeros(12)
        trunk_p = trunk_R = None
        for i, (_, parent, joint, origin, origin_R, axis, mass, com) in enumerate(self.nodes):
            if parent < 0:
                p, rot, Jv, Jw = np.zeros(3), np.eye(3), np.zeros((3, 12)), np.zeros((3, 12))
            else:
                pp, pR, pv, pw = states[parent]
                displacement = pR @ origin
                p, rot = pp + displacement, pR @ origin_R
                Jv, Jw = pv - skew(displacement) @ pw, pw.copy()
                if joint >= 0:
                    Jw[:, joint] += rot @ axis
                    rot = rot @ rotation(axis, q[joint])
            c = p + rot @ com
            Jcom = Jv - skew(rot @ com) @ Jw
            g_joint -= Jcom.T @ (mass * gravity)
            if i != self.trunk:
                first_other += mass * c
            else:
                trunk_p, trunk_R = p, rot
            states.append((p, rot, Jv, Jw))
        return dict(gravity=gravity, g_joint=g_joint, other_moment=first_other,
                    positions=np.array([states[i][0] for i in self.feet]),
                    jacobians=np.array([states[i][2][:, 3*k:3*k+3] for k, i in enumerate(self.feet)]),
                    trunk_p=trunk_p, trunk_R=trunk_R)


def estimate_forces(state, tau, dq=None, B=None):
    # Physical static joint balance: g + viscous demand = tau_feedback + J^T f_ground.
    # Do NOT add/subtract motor PD again. Static stiction is unknown, not Fc*sign(noisy dq).
    rhs = state["g_joint"] - np.asarray(tau)
    if dq is not None:
        rhs += B * dq
    forces, sigma = [], []
    for leg, J in enumerate(state["jacobians"]):
        values = np.linalg.svd(J, compute_uv=False)
        require(values[-1] >= .015 and values[0] / values[-1] < 100, "Near-singular foot Jacobian")
        forces.append(np.linalg.solve(J.T, rhs[3*leg:3*leg+3]))
        sigma.append(values[-1])
    return np.asarray(forces), min(sigma)


def fit_first_moment(states, forces):
    A, b = [], []
    for state, force in zip(states, forces):
        mu = state.get("mean_mu", np.cross(state["positions"], force).sum(axis=0))
        S = skew(state["gravity"])
        known = state.get("mean_known_gravity_moment", S @ state["other_moment"])
        A.append(S); b.append(mu - known)
    A, b = np.vstack(A), np.concatenate(b)
    h, _, rank, singular = np.linalg.lstsq(A, b, rcond=1e-5)
    condition = float(singular[0] / singular[-1]) if rank == 3 else None
    return h, int(rank), condition, (A @ h - b).reshape(-1, 3)


def fit_solution(states, forces, other_mass, known_mass=None):
    masses = np.array([state.get("mean_mass", -np.dot(force.sum(axis=0), state["gravity"]) / G**2)
                       for state, force in zip(states, forces)])
    mass = float(np.median(masses)) if known_mass is None else known_mass
    require(mass > other_mass and np.isfinite(mass), "Estimated trunk mass is non-positive")
    h, rank, condition, residual = fit_first_moment(states, forces)
    return dict(mass=mass, trunk_mass=mass-other_mass, masses=masses, h=h,
                com=h / (mass-other_mass) if rank == 3 else None,
                rank=rank, condition=condition, residual=residual)


def columns(data, suffix):
    return np.column_stack([data[f"{prefix}_{suffix}"] for prefix in PREFIXES])


def check_static_hold(time, q, dq, quats, gyros, accel):
    """Reject clear motion; report slow drift/acceleration as result-quality diagnostics."""
    warnings = []
    require(np.max(np.abs(dq)) <= .015 and np.max(np.linalg.norm(gyros, axis=1)) < .015,
            "Formal hold contains excessive joint/angular velocity")
    joint_drift = float(np.max(np.abs(q-q[0])))
    if joint_drift > .003+1e-9:
        warnings.append("正式段存在关节角累计漂移，静态假设可能不成立。")
    unit = quats/np.linalg.norm(quats, axis=1)[:, None]
    angles = 2*np.arccos(np.clip(np.abs(unit @ unit[0]), 0, 1))
    orientation_drift = float(np.max(angles))
    if orientation_drift > np.deg2rad(.3)+1e-9:
        warnings.append("正式段存在姿态累计漂移，静态假设或IMU稳定性需要核对。")
    acceleration = np.array([quaternion(qi) @ ai + [0, 0, -G] for qi, ai in zip(quats, accel)])
    intervals = np.diff(time)
    require(len(intervals) > 0 and np.all(intervals > 0) and np.max(intervals) <= .05,
            "Formal hold has a sample timing gap")
    weights = np.r_[np.median(intervals), intervals]
    groups = np.floor((time-time[0])/.25).astype(int)
    window_peak = 0.
    for group in np.unique(groups):
        selected = groups == group
        if weights[selected].sum() < .20:
            continue
        value = np.linalg.norm(np.average(acceleration[selected], axis=0, weights=weights[selected]))
        window_peak = max(window_peak, float(value))
    if window_peak > .3+1e-9:
        warnings.append("正式段加速度窗口偏离静态重力，检查运动或IMU标定。")
    return dict(max_joint_drift_rad=joint_drift, max_orientation_drift_deg=float(np.rad2deg(orientation_drift)),
                max_acceleration_window_mean_m_s2=window_peak, warnings=warnings)


def analyze(folder, known_mass=None, reference_q=None):
    folder = Path(folder)
    meta = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
    acquisition = json.loads((folder / "acquisition.json").read_text(encoding="utf-8"))
    require(acquisition.get("completed") and not acquisition.get("writer_failed"),
            "Acquisition incomplete/aborted: no parameter estimate will be accepted")
    xml = (folder / "robot_description.urdf").read_text(encoding="utf-8")
    model = StaticModel(xml, meta["base"], meta["feet"], meta["trunk"])
    data = np.genfromtxt(folder / "samples.csv", delimiter=",", names=True, encoding="utf-8")
    require(data.ndim == 1 and len(data) > 100, "Insufficient CSV rows")
    require(len(data) == acquisition["written_samples"], "CSV/marker row count mismatch")
    require(np.all(np.diff(data["steady_ns"]) > 0) and np.all(np.diff(data["sequence"]) == 1),
            "Non-monotonic timestamps or lost samples")
    q, dq, tau = (columns(data, suffix) for suffix in ("q", "dq", "tau"))
    cmd = columns(data, "q_cmd")
    quats = np.column_stack([data[k] for k in ("qw", "qx", "qy", "qz")])
    gyros = np.column_stack([data[k] for k in ("gx", "gy", "gz")])
    accel = np.column_stack([data[k] for k in ("ax", "ay", "az")])
    require(np.all(np.isfinite(np.column_stack((q, dq, tau, quats, gyros, accel, cmd)))), "Nonfinite feedback")
    summaries, states, pose_forces, force_rows, warnings = [], [], [], [], []
    for pose, name in enumerate(meta["pose_names"]):
        raw = np.flatnonzero((data["phase"] == 3) & (data["pose"] == pose))
        require(len(raw) >= 100, f"Pose {name}: missing formal samples")
        duration = (data["steady_ns"][raw[-1]]-data["steady_ns"][raw[0]])*1e-9
        require(duration >= meta["sample_s"]-.2, f"Pose {name}: truncated formal hold")
        static_checks = check_static_hold(data["steady_ns"][raw]*1e-9, q[raw], dq[raw],
                                          quats[raw], gyros[raw], accel[raw])
        warnings.extend(f"{name}: {warning}" for warning in static_checks["warnings"])
        # Use actual timestamps, max 20 Hz offline evaluation; never assume a fixed loop interval.
        selected, previous = [], -np.inf
        for index in raw:
            t = data["steady_ns"][index]*1e-9
            if t-previous >= .049:
                selected.append(index); previous = t
        valid, per_forces, per_moments, per_g, per_other, per_mass, per_acc, per_height = [], [], [], [], [], [], [], []
        for index in selected:
            if np.max(abs(dq[index])) > .015 or np.linalg.norm(gyros[index]) > .015:
                continue
            R = quaternion(quats[index]); state = model.evaluate(q[index], R)
            try:
                force, sigma = estimate_forces(state, tau[index], dq[index], model.B)
            except ValueError:
                continue
            force_world = (R @ force.T).T
            mass = force_world[:, 2].sum()/G
            if (not .5*model.nominal_mass < mass < 1.5*model.nominal_mass or
                np.min(force_world[:, 2]) < .02*mass*G or
                np.max(np.linalg.norm(force_world[:, :2], axis=1)/force_world[:, 2]) > 1.0):
                continue
            foot_world = (R @ state["positions"].T).T
            valid.append(index); per_forces.append(force); per_mass.append(mass)
            per_moments.append(np.cross(state["positions"], force).sum(axis=0))
            per_g.append(state["gravity"]); per_other.append(state["other_moment"])
            per_acc.append(R @ accel[index] + np.array([0, 0, -G]))
            per_height.append(np.ptp(foot_world[:, 2]))
            force_rows.append([int(data["sequence"][index]), pose, data["steady_ns"][index]*1e-9,
                               *force_world.flatten(), mass, sigma])
        require(len(valid) >= .8*len(selected), f"Pose {name}: too many invalid/static/contact-force samples")
        # Average physical moments, not products of separately averaged foot positions and forces.
        # Encode the exact averaged moment in an equivalent state for the linear solve.
        qmean = q[valid].mean(axis=0)
        aligned = quats[valid].copy()
        aligned[aligned @ aligned[0] < 0] *= -1
        Rmean = quaternion(aligned.mean(axis=0)/np.linalg.norm(aligned.mean(axis=0)))
        state = model.evaluate(qmean, Rmean)
        state["gravity"] = np.mean(per_g, axis=0)
        state["other_moment"] = np.mean(per_other, axis=0)
        state["mean_mu"] = np.mean(per_moments, axis=0)
        # cross(g,h_other) must also be averaged before subtraction.
        state["mean_known_gravity_moment"] = np.mean(
            [skew(g) @ h for g, h in zip(per_g, per_other)], axis=0)
        state["mean_mass"] = float(np.mean(per_mass))
        states.append(state); pose_forces.append(np.mean(per_forces, axis=0))
        summary = dict(pose=pose, name=name, samples=len(valid), duration_s=duration,
            static_checks=static_checks,
            q_mean_rad=qmean.tolist(), q_std_rad=q[valid].std(axis=0).tolist(),
            q_cmd_mean_rad=cmd[valid].mean(axis=0).tolist(),
            quat_mean_wxyz=(aligned.mean(axis=0)/np.linalg.norm(aligned.mean(axis=0))).tolist(),
            estimated_mass_kg=float(np.mean(per_mass)), estimated_mass_std_kg=float(np.std(per_mass)),
            net_force_relative_residual=float(np.linalg.norm(np.mean(per_forces, axis=0).sum(axis=0)
                + np.mean(per_mass)*state["gravity"])/(np.mean(per_mass)*G)),
            mean_specific_acceleration_residual_m_s2=float(np.linalg.norm(np.mean(per_acc, axis=0))),
            foot_height_spread_mean_m=float(np.mean(per_height)))
        summaries.append(summary)
    # Exact per-pose averaged static regression, with equal weight per posture.
    A = np.vstack([skew(s["gravity"]) for s in states])
    b = np.concatenate([s["mean_mu"]-s["mean_known_gravity_moment"] for s in states])
    solution = fit_solution(states, pose_forces, model.other_mass, known_mass)
    h, rank, condition = solution["h"], solution["rank"], solution["condition"]
    mass, trunk_mass, com_B = solution["mass"], solution["trunk_mass"], solution["com"]
    residual = solution["residual"]
    if rank < 3:
        warnings.append("质心三维不可观测；未输出伪逆强行补零的躯干XYZ。需要实际姿态变化。")
    if condition is not None and condition > 150:
        warnings.append("实际倾角激励不足，三维辨识条件数过大，尤其Z不可靠。")
    if np.std([s["mean_mass"] for s in states])/mass > .05:
        warnings.append("不同站姿推算质量相差明显，须检查静摩擦、力矩标定、模型或接触。")
    if max(s["net_force_relative_residual"] for s in summaries) > .10:
        warnings.append("静态合力与重力不平衡超过10%，足力估计不可信。")
    if max(s["mean_specific_acceleration_residual_m_s2"] for s in summaries) > .6:
        warnings.append("平均IMU加速度与静态重力不符，检查真实运动、IMU标定和姿态。")
    if max(s["foot_height_spread_mean_m"] for s in summaries) > .02:
        warnings.append("正运动学推算四足高度差超过2cm，检查地面、关节零位、姿态与几何。")
    if max(max(s["q_std_rad"]) for s in summaries) > .003:
        warnings.append("静态段关节位置波动较大。")
    if np.sqrt(np.mean(residual**2)) > 2:
        warnings.append("跨姿态重力矩拟合残差超过2Nm。")
    ref = states[0]
    Rref = quaternion(summaries[0]["quat_mean_wxyz"])
    mu_ref = Rref @ ref["mean_mu"]
    projection = [-mu_ref[1]/(mass*G), mu_ref[0]/(mass*G)]
    repeat = []
    for i in (2, 4, 6, 8, 10, 12):
        R = quaternion(summaries[i]["quat_mean_wxyz"])
        mu = R @ states[i]["mean_mu"]
        measured = np.array([-mu[1], mu[0]])/(mass*G)
        # Bring back to the same nominal leg mass distribution and orientation before comparing.
        if com_B is not None:
            predicted = (R @ (trunk_mass*com_B + states[i]["other_moment"])/mass)[:2]
            expected_ref = (Rref @ (trunk_mass*com_B + ref["other_moment"])/mass)[:2]
            measured = measured - predicted + expected_ref
        repeat.append((measured-np.array(projection)).tolist())
    repeat_max = float(np.max(np.linalg.norm(repeat, axis=1)))
    if repeat_max > .015:
        warnings.append("回中重复性差异超过15mm：存在方向相关摩擦、漂移或反馈偏差。")
    holdout = []
    if rank == 3:
        for i in range(len(states)):
            mask = np.ones(len(states)*3, bool); mask[3*i:3*i+3] = False
            leave, _, leave_rank, _ = np.linalg.lstsq(A[mask], b[mask], rcond=1e-5)
            if leave_rank == 3:
                holdout.append((leave/trunk_mass-com_B).tolist())
        if holdout and (np.max(np.abs(np.asarray(holdout)[:, :2])) > .015 or
                        np.max(np.abs(np.asarray(holdout)[:, 2])) > .04):
            warnings.append("留一姿态重算的质心变化过大，跨姿态辨识不稳定。")
        if np.max(np.abs(com_B)) > .5:
            warnings.append("躯干质心估计远离机身，检查力矩、模型与可观测性。")
    sensitivity = None
    if rank == 3:
        rng = np.random.default_rng(219)
        perturbations = []
        for _ in range(128):
            # Constant per-joint offset across all poses. Fc is a sensitivity scale,
            # NOT a measured stiction bound or statistical confidence interval.
            bias = rng.uniform(-1, 1, 12)*model.Fc
            delta_b = []
            delta_m = []
            for state in states:
                delta_force = np.array([np.linalg.solve(J.T, -bias[3*k:3*k+3])
                                        for k, J in enumerate(state["jacobians"])])
                delta_b.extend(np.cross(state["positions"], delta_force).sum(axis=0))
                delta_m.append(-np.dot(delta_force.sum(axis=0), state["gravity"])/G**2)
            perturbed_mass = mass if known_mass is not None else float(np.median(
                [s["mean_mass"]+dm for s, dm in zip(states, delta_m)]))
            if perturbed_mass > model.other_mass:
                dh = np.linalg.lstsq(A, np.asarray(delta_b), rcond=1e-5)[0]
                perturbations.append((h+dh)/(perturbed_mass-model.other_mass)-com_B)
        sensitivity = np.quantile(np.abs(perturbations), .95, axis=0).tolist()
    valid_xyz = bool(rank == 3 and condition < 150 and not warnings)
    if sensitivity is not None and sensitivity[2] > .03:
        warnings.append("以已辨识Fc为力矩偏差尺度的敏感性分析显示Z易变；该尺度不是实测静摩擦上界。")
        valid_xyz = False
    result = dict(schema=1, source="estimated contact forces from motor torque feedback; not measured GRF",
        urdf_sha256=hashlib.sha256(xml.encode()).hexdigest(), acquisition_complete=True,
        coordinate_frame=meta["base"], mass_source="external supplied mass" if known_mass is not None else "torque-derived estimate",
        estimated_whole_mass_kg=mass, estimated_trunk_mass_kg=trunk_mass,
        rank=int(rank), condition=condition, moment_fit_rms_Nm=float(np.sqrt(np.mean(residual**2))),
        trunk_com_B_m=None if com_B is None else com_B.tolist(),
        trunk_com_link_m=None if com_B is None else (ref["trunk_R"].T @ (com_B-ref["trunk_p"])).tolist(),
        fixstand_measured_pose_whole_com_B_m=None if com_B is None else
            ((trunk_mass*com_B+ref["other_moment"])/mass).tolist(),
        fixstand_target_pose_whole_com_B_m=None if com_B is None else
            ((trunk_mass*com_B+model.evaluate(meta["stand_q"], np.eye(3))["other_moment"])/mass).tolist(),
        fixstand_projection_level_frame_xy_m=projection,
        center_repeat_corrected_delta_xy_m=repeat, center_repeat_max_m=repeat_max,
        leave_one_pose_out_com_delta_m=holdout,
        torque_offset_Fc_scale_sensitivity_abs95_m=sensitivity,
        xyz_quality_checks_passed=valid_xyz, automatic_model_update=False, warnings=warnings, poses=summaries,
        limitations=["No external ground truth; passing checks does not prove absolute accuracy.",
                    "Unmodeled stiction, torque bias/scale, kinematic and IMU errors remain possible.",
                    "No inertia tensor identification. No body XYZ/velocity estimate is used.",
                    "FixStand COM is not dynamic trotting COM unless joint posture matches."])
    if reference_q is not None:
        supplied = json.loads(Path(reference_q).read_text(encoding="utf-8"))
        supplied = supplied.get("q", supplied) if isinstance(supplied, dict) else supplied
        state = model.evaluate(supplied, np.eye(3))
        result["supplied_reference_pose_whole_com_B_m"] = None if com_B is None else (
            (trunk_mass*com_B+state["other_moment"])/mass).tolist()
    with (folder / "forces_estimated.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sequence", "pose", "steady_s"]+
                        [f"{leg}_f{axis}_estimated_N" for leg in ("FR", "FL", "RR", "RL") for axis in "xyz"]+
                        ["mass_estimated_kg", "min_jacobian_sigma_m"])
        writer.writerows(force_rows)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--mass-kg", type=float, help="Optional independently measured whole mass")
    parser.add_argument("--reference-q", type=Path, help="Optional JSON array of 12 actual nominal-trot joint angles")
    args = parser.parse_args()
    folder = args.directory
    try:
        require(args.mass_kg is None or np.isfinite(args.mass_kg) and args.mass_kg > 0, "Invalid supplied mass")
        result = analyze(folder, args.mass_kg, args.reference_q)
        lines = ["# COM identification", "", "由反馈力矩推算，未自动修改任何模型。", "",
                 f"三维质量检查通过：{result['xyz_quality_checks_passed']}（不等于物理真值已验证）",
                 f"整机质量估计 kg：{result['estimated_whole_mass_kg']:.6f}",
                 f"躯干质量估计 kg：{result['estimated_trunk_mass_kg']:.6f}",
                 f"躯干质心 / trunk坐标 m：{result['trunk_com_link_m']}",
                 f"FixStand实测腿姿整机质心 / B m：{result['fixstand_measured_pose_whole_com_B_m']}",
                 f"FixStand配置目标腿姿整机质心 / B m：{result['fixstand_target_pose_whole_com_B_m']}",
                 f"秩：{result['rank']}；条件数：{result['condition']}；力矩RMS Nm：{result['moment_fit_rms_Nm']:.6f}",
                 f"回中最大差异 m：{result['center_repeat_max_m']:.6f}",
                 f"Fc尺度力矩偏差敏感性 m（非置信区间）：{result['torque_offset_Fc_scale_sensitivity_abs95_m']}", "",
                 "FixStand腿姿不等于动态trot腿姿；只有姿态相同时两者整机质心才相同。", "", "## 检查提示", ""]
        lines += [f"- {warning}" for warning in result["warnings"]] or ["- 未触发检查阈值，仍需独立验证。"]
        (folder / "results.json").write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
        (folder / "report.md").write_text("\n\n".join(lines)+"\n", encoding="utf-8")
        print("\n".join(lines))
        return 0 if result["xyz_quality_checks_passed"] else 2
    except (ValueError, OSError, KeyError, ET.ParseError, np.linalg.LinAlgError) as error:
        print(f"COM identification refused: {error}", file=sys.stderr)
        if folder.is_dir():
            (folder / "results.json").write_text(json.dumps(dict(xyz_quality_checks_passed=False,
                automatic_model_update=False, error=str(error)), indent=2, ensure_ascii=False), encoding="utf-8")
            (folder / "report.md").write_text(f"# COM identification refused\n\n{error}\n\n未修改模型。\n", encoding="utf-8")
        return 2


if __name__ == "__main__":
    sys.exit(main())
