#!/usr/bin/env python3
"""Offline static trunk X/Y CoM identification (Z held at an explicit reference). NumPy only; never starts ROS or modifies a model."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import traceback
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


def fit_solution(states, forces, other_mass, known_mass=None, fixed_trunk_z=None):
    masses = np.array([state.get("mean_mass", -np.dot(force.sum(axis=0), state["gravity"]) / G**2)
                       for state, force in zip(states, forces)])
    mass = float(np.median(masses)) if known_mass is None else known_mass
    require(mass > other_mass and np.isfinite(mass), "Estimated trunk mass is non-positive")
    if fixed_trunk_z is None:
        h, rank, condition, residual = fit_first_moment(states, forces)
        extra = {}
    else:
        trunk_mass = mass-other_mass
        ref = states[0]
        basis = ref["trunk_R"][:, :2]
        fixed_h = trunk_mass*(ref["trunk_p"]+ref["trunk_R"][:, 2]*fixed_trunk_z)
        A = np.vstack([skew(s["gravity"])@basis for s in states])
        b = np.concatenate([s.get("mean_mu", np.cross(s["positions"], f).sum(axis=0))
            -s.get("mean_known_gravity_moment", skew(s["gravity"])@s["other_moment"])
            -skew(s["gravity"])@fixed_h for s, f in zip(states, forces)])
        xy, _, rank, singular = np.linalg.lstsq(A, b, rcond=1e-5)
        condition = float(singular[0]/singular[-1]) if rank == 2 else None
        h = fixed_h+basis@xy
        residual = (A@xy-b).reshape(-1, 3)
        extra = dict(A=A, b=b, fixed_h=fixed_h, basis=basis, xy_first_moment=xy)
    return dict(mass=mass, trunk_mass=mass-other_mass, masses=masses, h=h,
                com=h / (mass-other_mass) if rank == (3 if fixed_trunk_z is None else 2) else None,
                rank=rank, condition=condition, residual=residual, **extra)


def columns(data, suffix):
    return np.column_stack([data[f"{prefix}_{suffix}"] for prefix in PREFIXES])


def check_static_hold(time, q, dq, quats, gyros, accel):
    """Report motion quality without preventing raw-data collection or candidate calculation."""
    warnings = []
    speed_index = np.unravel_index(np.argmax(np.abs(dq)), dq.shape)
    speed_peak = float(np.abs(dq[speed_index]))
    gyro_norm = np.linalg.norm(gyros, axis=1)
    gyro_index = int(np.argmax(gyro_norm))
    if not (np.max(np.abs(dq)) <= .015 and np.max(np.linalg.norm(gyros, axis=1)) < .015):
        warnings.append("[MOTION_QUALITY] 正式段速度质量超限："
            f"joint={JOINTS[speed_index[1]]}, |dq|max={speed_peak:.9f} rad/s (limit<=0.015), "
            f"gyro_norm_max={gyro_norm[gyro_index]:.9f} rad/s (limit<0.015)；保留原始数据，继续离线计算。")
    joint_drift = float(np.max(np.abs(q-q[0])))
    drift_index = np.unravel_index(np.argmax(np.abs(q-q[0])), q.shape)
    drift_row, drift_joint = (int(i) for i in drift_index)
    drift_detail = dict(joint=JOINTS[drift_joint], row_offset=drift_row,
        elapsed_s=float(time[drift_row]-time[0]), observed_rad=joint_drift, limit_rad=.003,
        initial_q_rad=float(q[0, drift_joint]), peak_q_rad=float(q[drift_row, drift_joint]),
        final_q_rad=float(q[-1, drift_joint]), definition="max(abs(q(t)-q(first_formal_sample))); not q-q_cmd")
    if joint_drift > .003+1e-9:
        warnings.append(f"[JOINT_DRIFT] {drift_detail['joint']} 正式段反馈角累计变化 "
            f"{joint_drift:.9f} rad ({np.rad2deg(joint_drift):.6f}°) > 0.003000000 rad；"
            f"起点q={drift_detail['initial_q_rad']:.9f}，峰值q={drift_detail['peak_q_rad']:.9f}，"
            f"峰值在正式段+{drift_detail['elapsed_s']:.3f}s。此项检查实际角变化，不检查目标跟踪偏差。")
    unit = quats/np.linalg.norm(quats, axis=1)[:, None]
    angles = 2*np.arccos(np.clip(np.abs(unit @ unit[0]), 0, 1))
    orientation_drift = float(np.max(angles))
    if orientation_drift > np.deg2rad(.3)+1e-9:
        warnings.append(f"[ORIENTATION_DRIFT] 正式段IMU姿态累计变化 "
            f"{np.rad2deg(orientation_drift):.6f}° > 0.300000°；静态假设或IMU稳定性需要核对。")
    acceleration = np.array([quaternion(qi) @ ai + [0, 0, -G] for qi, ai in zip(quats, accel)])
    intervals = np.diff(time)
    require(len(intervals) > 0 and np.all(intervals > 0) and np.max(intervals) <= .05,
            f"Formal hold has a sample timing gap: intervals={len(intervals)}, "
            f"max_gap_s={float(np.max(intervals)) if len(intervals) else None}, limit<=0.05 s")
    weights = np.r_[np.median(intervals), intervals]
    groups = np.floor((time-time[0])/.25).astype(int)
    window_peak = 0.
    window_peak_start = None
    for group in np.unique(groups):
        selected = groups == group
        if weights[selected].sum() < .20:
            continue
        value = np.linalg.norm(np.average(acceleration[selected], axis=0, weights=weights[selected]))
        if value > window_peak:
            window_peak = float(value)
            window_peak_start = float(time[selected][0]-time[0])
    if window_peak > .3+1e-9:
        warnings.append(f"[ACCELERATION_WINDOW] 正式段加速度窗口均值模长 "
            f"{window_peak:.6f} m/s² > 0.300000 m/s²，窗口起点+{window_peak_start:.3f}s；检查运动或IMU标定。")
    return dict(max_joint_drift_rad=joint_drift, max_orientation_drift_deg=float(np.rad2deg(orientation_drift)),
                joint_drift_detail=drift_detail,
                max_joint_speed_rad_s=speed_peak, max_joint_speed_joint=JOINTS[speed_index[1]],
                max_gyro_norm_rad_s=float(gyro_norm[gyro_index]),
                acceleration_peak_window_start_s=window_peak_start,
                max_acceleration_window_mean_m_s2=window_peak, warnings=warnings)


def model_debug(model, xml):
    """Describe model assumptions only; never correct forces or change acceptance."""
    root = ET.fromstring(xml)
    feet = [model.nodes[i][0] for i in model.feet]
    radii = {}
    for link in root.findall("link"):
        if link.get("name") not in feet:
            continue
        radii[link.get("name")] = [float(s.get("radius")) for s in link.findall("collision/geometry/sphere")]
    notes = []
    if np.any(model.Fc > 0):
        notes.append("[UNMODELED_STATIC_FRICTION] URDF含非零Fc，单帧足力反算未计入未知静摩擦；"
                     "静止不会消除静摩擦。Fc敏感性分析不是已验证补偿，也不是静摩擦上界。")
    if any(radius > 0 for values in radii.values() for radius in values):
        notes.append("[CONTACT_POINT_ASSUMPTION] 反算使用foot链接原点的点力模型；URDF脚底是有半径的球，"
                     "实际接触点偏置和接触力矩未纳入该反算。")
    plugins = [p.text for p in root.findall("ros2_control/hardware/plugin")]
    if any(p in ("sysu219_guide_controller/GazeboEncoderSystem", "gazebo_ros2_control/GazeboSystem")
           for p in plugins):
        notes.append("[GAZEBO_EFFORT_SOURCE] 此Gazebo接口的effort沿用GetForce驱动力矩通道，"
                     "不是实测足底接触力，也不能当作已包含被动静摩擦的净关节力矩。")
    return dict(nominal_whole_mass_kg=float(model.nominal_mass),
        nominal_trunk_mass_kg=float(model.nodes[model.trunk][6]),
        nominal_trunk_com_link_m=model.nodes[model.trunk][7].tolist(),
        reference_is_urdf_not_independent_measurement=True,
        friction_Fc_Nm=dict(zip(JOINTS, model.Fc.tolist())),
        viscous_B_Nms_rad=dict(zip(JOINTS, model.B.tolist())),
        foot_collision_sphere_radii_m=radii, hardware_plugins=plugins,
        force_model="J_foot_origin.T*f = g_leg + B*dq - tau_feedback; unknown static friction omitted",
        physical_accuracy_verified=False, notes=notes)


def analyze(folder, known_mass=None, reference_q=None, output_directory=None, diagnostics=None, trunk_z_m=None):
    folder = Path(folder)
    output = folder if output_directory is None else Path(output_directory)
    trace = {} if diagnostics is None else diagnostics
    trace["stage"] = "read_metadata"
    meta = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
    trace["metadata"] = meta
    trace["stage"] = "read_acquisition"
    acquisition = json.loads((folder / "acquisition.json").read_text(encoding="utf-8"))
    trace["acquisition"] = acquisition
    require(acquisition.get("completed") and not acquisition.get("writer_failed"),
            "Acquisition incomplete/aborted: no parameter estimate will be accepted; "
            f"completed={acquisition.get('completed')}, writer_failed={acquisition.get('writer_failed')}, "
            f"written_samples={acquisition.get('written_samples')}, "
            f"reason={acquisition.get('reason', 'not recorded in acquisition.json; cannot infer abort cause')}")
    trace["stage"] = "read_model"
    xml = (folder / "robot_description.urdf").read_text(encoding="utf-8")
    model = StaticModel(xml, meta["base"], meta["feet"], meta["trunk"])
    trace["model"] = model_debug(model, xml)
    real_mass_default = known_mass is None and "hardware_sysu219/HardwareSysu219" in trace["model"]["hardware_plugins"]
    if real_mass_default:
        known_mass = 41.8  # User-supplied approximate real whole-robot mass; not torque-derived.
    trace["mass_policy"] = dict(whole_mass_kg=known_mass, real_default_applied=real_mass_default,
        CAN_torque_mapping="accepted as accurate by user; no rescaling or inferred offset correction")
    trace["stage"] = "read_samples"
    data = np.genfromtxt(folder / "samples.csv", delimiter=",", names=True, encoding="utf-8")
    require(data.ndim == 1 and len(data) > 100,
            f"Insufficient CSV rows: shape={data.shape}; expected one-dimensional array with >100 rows")
    trace["data"] = dict(rows=len(data),
        csv_sha256=hashlib.sha256((folder / "samples.csv").read_bytes()).hexdigest(),
        first_sequence=int(data["sequence"][0]), last_sequence=int(data["sequence"][-1]),
        recorded_span_s=float((data["steady_ns"][-1]-data["steady_ns"][0])*1e-9),
        last_pose=int(data["pose"][-1]), last_phase=int(data["phase"][-1]))
    require(len(data) == acquisition["written_samples"],
            f"CSV/marker row count mismatch: CSV={len(data)}, marker={acquisition['written_samples']}")
    require(np.all(np.diff(data["steady_ns"]) > 0) and np.all(np.diff(data["sequence"]) == 1),
            "Non-monotonic timestamps or lost samples")
    q, dq, tau = (columns(data, suffix) for suffix in ("q", "dq", "tau"))
    cmd = columns(data, "q_cmd")
    quats = np.column_stack([data[k] for k in ("qw", "qx", "qy", "qz")])
    gyros = np.column_stack([data[k] for k in ("gx", "gy", "gz")])
    accel = np.column_stack([data[k] for k in ("ax", "ay", "az")])
    require(np.all(np.isfinite(np.column_stack((q, dq, tau, quats, gyros, accel, cmd)))), "Nonfinite feedback")
    intervals = np.diff(data["steady_ns"])*1e-9
    trace["timing"] = dict(sample_interval_min_s=float(intervals.min()), sample_interval_max_s=float(intervals.max()),
        sample_interval_median_s=float(np.median(intervals)),
        reported_control_period_min_s=float(data["period_s"].min()),
        reported_control_period_max_s=float(data["period_s"].max()),
        note="Host sampling times only; no hardware packet timestamp or sample-age measurement")
    formal = data["phase"] == 3
    trace["joint_signal_summary_formal"] = {
        name: {field: dict(min=float(values[formal, j].min()), max=float(values[formal, j].max()),
            mean=float(values[formal, j].mean()), std=float(values[formal, j].std()),
            distinct_levels=int(len(np.unique(values[formal, j]))))
            for field, values in (("q_rad", q), ("dq_rad_s", dq), ("CAN_tau_Nm", tau), ("q_cmd_rad", cmd))}
        for j, name in enumerate(JOINTS)} if np.any(formal) else {}
    summaries, states, pose_forces, force_rows, warnings = [], [], [], [], []
    trace["poses"] = summaries
    for pose, name in enumerate(meta["pose_names"]):
        trace["stage"] = "formal_pose_check"
        trace["current_pose"] = dict(index=pose, name=name)
        raw = np.flatnonzero((data["phase"] == 3) & (data["pose"] == pose))
        require(len(raw) >= 100, f"Pose {name}: missing formal samples: actual={len(raw)}, required>=100")
        duration = (data["steady_ns"][raw[-1]]-data["steady_ns"][raw[0]])*1e-9
        require(duration >= meta["sample_s"]-.2,
                f"Pose {name}: truncated formal hold: actual={duration:.6f} s, required>={meta['sample_s']-.2:.6f} s")
        static_checks = check_static_hold(data["steady_ns"][raw]*1e-9, q[raw], dq[raw],
                                          quats[raw], gyros[raw], accel[raw])
        static_checks["joint_drift_detail"]["sequence"] = int(
            data["sequence"][raw[static_checks["joint_drift_detail"]["row_offset"]]])
        if "motion_quality_bad_cycles" in data.dtype.names:
            bad_cycles = int(np.max(data["motion_quality_bad_cycles"][raw]))
            static_checks["controller_motion_quality_bad_cycles"] = bad_cycles
            if bad_cycles and not any("[MOTION_QUALITY]" in w for w in static_checks["warnings"]):
                static_checks["warnings"].append(f"[MOTION_QUALITY_CYCLE] 控制器记录到{bad_cycles}个速度质量超限周期；"
                    "100Hz原始行未必包含峰值帧。原始数据已保留，不因此中止流程。")
        warnings.extend(f"{name}: {warning}" for warning in static_checks["warnings"])
        # Use actual timestamps, max 20 Hz offline evaluation; never assume a fixed loop interval.
        selected, previous = [], -np.inf
        for index in raw:
            t = data["steady_ns"][index]*1e-9
            if t-previous >= .049:
                selected.append(index); previous = t
        valid, per_forces, per_moments, per_g, per_other, per_mass, per_acc, per_height = [], [], [], [], [], [], [], []
        rejected = dict(motion=0, jacobian=0, mass_range=0, insufficient_vertical_support=0, friction_ratio=0)
        trace["current_pose"].update(raw_samples=len(raw), selected_samples=len(selected), rejected_counts=rejected)
        trace["stage"] = "force_estimation"
        for index in selected:
            if np.max(abs(dq[index])) > .015 or np.linalg.norm(gyros[index]) > .015:
                rejected["motion"] += 1
                continue
            R = quaternion(quats[index]); state = model.evaluate(q[index], R)
            try:
                force, sigma = estimate_forces(state, tau[index], dq[index], model.B)
            except ValueError:
                rejected["jacobian"] += 1
                continue
            force_world = (R @ force.T).T
            mass = force_world[:, 2].sum()/G
            if (not .5*model.nominal_mass < mass < 1.5*model.nominal_mass or
                np.min(force_world[:, 2]) < .02*mass*G or
                np.max(np.linalg.norm(force_world[:, :2], axis=1)/force_world[:, 2]) > 1.0):
                if not .5*model.nominal_mass < mass < 1.5*model.nominal_mass:
                    rejected["mass_range"] += 1
                if np.min(force_world[:, 2]) < .02*mass*G:
                    rejected["insufficient_vertical_support"] += 1
                with np.errstate(divide="ignore", invalid="ignore"):
                    if np.max(np.linalg.norm(force_world[:, :2], axis=1)/force_world[:, 2]) > 1.0:
                        rejected["friction_ratio"] += 1
                continue
            foot_world = (R @ state["positions"].T).T
            valid.append(index); per_forces.append(force); per_mass.append(mass)
            per_moments.append(np.cross(state["positions"], force).sum(axis=0))
            per_g.append(state["gravity"]); per_other.append(state["other_moment"])
            per_acc.append(R @ accel[index] + np.array([0, 0, -G]))
            per_height.append(np.ptp(foot_world[:, 2]))
            force_rows.append([int(data["sequence"][index]), pose, data["steady_ns"][index]*1e-9,
                               *force_world.flatten(), mass, sigma])
        trace["current_pose"]["valid_samples"] = len(valid)
        if len(valid) < .8*len(selected):
            warnings.append(f"{name}: [LOW_VALID_RATIO] 可用帧={len(valid)}/{len(selected)}，低于80%质量参考；"
                            f"rejected_counts={rejected}。仍使用可用帧计算候选，不宣称静态质量合格。")
        require(len(valid) > 0,
                f"Pose {name}: no numerically usable static/contact-force sample; "
                f"raw data preserved; rejected_counts={rejected}")
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
        summary = dict(pose=pose, name=name, samples=len(valid), raw_samples=len(raw),
            selected_samples=len(selected), rejected_counts=rejected, duration_s=duration,
            first_sequence=int(data["sequence"][raw[0]]), last_sequence=int(data["sequence"][raw[-1]]),
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
    trace["stage"] = "fit_and_quality_checks"
    # Two unknown trunk coordinates; fixed Z is an assumption, never an estimate.
    fixed_z = model.nodes[model.trunk][7][2] if trunk_z_m is None else trunk_z_m
    require(np.isfinite(fixed_z), "Invalid fixed trunk Z reference")
    solution = fit_solution(states, pose_forces, model.other_mass, known_mass, fixed_z)
    A, b = solution["A"], solution["b"]
    z_policy = dict(estimated=False, fixed_reference_link_m=float(fixed_z),
        source="URDF trunk inertial origin" if trunk_z_m is None else "explicit --trunk-z-m",
        note="Only trunk X/Y are fitted; X/Y depend on this fixed height when tilted.")
    h, rank, condition = solution["h"], solution["rank"], solution["condition"]
    mass, trunk_mass, com_B = solution["mass"], solution["trunk_mass"], solution["com"]
    residual = solution["residual"]
    if rank < 2:
        warnings.append(f"[RANK] 机身XY不可观测：秩={rank}，要求=2；XY不输出。")
    if condition is not None and condition > 150:
        warnings.append(f"[CONDITION] 条件数={condition:.6f} > 150；机身XY拟合病态。")
    if np.std([s["mean_mass"] for s in states])/mass > .05:
        warnings.append(f"[MASS_CONSISTENCY] 跨姿态质量标准差/整机质量="
            f"{np.std([s['mean_mass'] for s in states])/mass:.6f} > 0.05；CAN换算按用户确认准确，分析模型、接触及静力假设。")
    if max(s["net_force_relative_residual"] for s in summaries) > .10:
        worst = max(summaries, key=lambda s: s["net_force_relative_residual"])
        warnings.append(f"[FORCE_BALANCE] {worst['name']} 静态合力相对残差="
            f"{worst['net_force_relative_residual']:.6f} > 0.10；足力估计不可信。")
    if max(s["mean_specific_acceleration_residual_m_s2"] for s in summaries) > .6:
        worst = max(summaries, key=lambda s: s["mean_specific_acceleration_residual_m_s2"])
        warnings.append(f"[MEAN_ACCELERATION] {worst['name']} 平均加速度重力残差="
            f"{worst['mean_specific_acceleration_residual_m_s2']:.6f} m/s² > 0.6 m/s²。")
    if max(s["foot_height_spread_mean_m"] for s in summaries) > .02:
        worst = max(summaries, key=lambda s: s["foot_height_spread_mean_m"])
        warnings.append(f"[FOOT_HEIGHT] {worst['name']} 正运动学四足高度差="
            f"{worst['foot_height_spread_mean_m']:.6f} m > 0.02 m；检查地面、关节零位、姿态与几何。")
    if max(max(s["q_std_rad"]) for s in summaries) > .003:
        worst = max(summaries, key=lambda s: max(s["q_std_rad"]))
        joint = int(np.argmax(worst["q_std_rad"]))
        warnings.append(f"[JOINT_STD] {worst['name']} / {JOINTS[joint]} 关节角标准差="
            f"{worst['q_std_rad'][joint]:.9f} rad > 0.003 rad。")
    if np.sqrt(np.mean(residual**2)) > 2:
        warnings.append(f"[MOMENT_FIT] 跨姿态重力矩拟合RMS={np.sqrt(np.mean(residual**2)):.6f} N·m > 2 N·m。")
    ref = states[0]
    Rref = quaternion(summaries[0]["quat_mean_wxyz"])
    mu_ref = Rref @ ref["mean_mu"]
    projection = [-mu_ref[1]/(mass*G), mu_ref[0]/(mass*G)]
    repeat = []
    # Names identify repeat poses in both the compact six-pose and legacy thirteen-pose schedules.
    repeat_indices = [i for i, name in enumerate(meta["pose_names"])
                      if name == "reference_repeat" or name.startswith("center_after_")]
    for i in repeat_indices:
        R = quaternion(summaries[i]["quat_mean_wxyz"])
        mu = R @ states[i]["mean_mu"]
        measured = np.array([-mu[1], mu[0]])/(mass*G)
        # Bring back to the same nominal leg mass distribution and orientation before comparing.
        if com_B is not None:
            predicted = (R @ (trunk_mass*com_B + states[i]["other_moment"])/mass)[:2]
            expected_ref = (Rref @ (trunk_mass*com_B + ref["other_moment"])/mass)[:2]
            measured = measured - predicted + expected_ref
        repeat.append((measured-np.array(projection)).tolist())
    repeat_max = float(np.max(np.linalg.norm(repeat, axis=1))) if repeat else None
    if repeat_max is None:
        warnings.append("[MISSING_REPEAT] 回中复测段数量=0，要求>=1；无法检查重复性。")
    elif repeat_max > .015:
        warnings.append(f"[REPEATABILITY] 回中最大差异={repeat_max:.9f} m > 0.015 m；"
                        "存在方向相关摩擦、漂移或反馈偏差。")
    holdout = []
    if rank == 2:
        for i in range(len(states)):
            mask = np.ones(len(states)*3, bool); mask[3*i:3*i+3] = False
            leave, _, leave_rank, _ = np.linalg.lstsq(A[mask], b[mask], rcond=1e-5)
            if leave_rank == 2:
                holdout.append((ref["trunk_R"].T @ (solution["basis"]@leave/trunk_mass+solution["fixed_h"]/trunk_mass-com_B)).tolist())
        if holdout and np.max(np.abs(np.asarray(holdout)[:, :2])) > .015:
            warnings.append(f"[LEAVE_ONE_OUT] 留一姿态重算最大XY变化="
                f"{np.max(np.abs(np.asarray(holdout)[:, :2])):.9f} m > 0.015 m。")
        xy_link = (ref["trunk_R"].T@(com_B-ref["trunk_p"]))[:2]
        if np.max(np.abs(xy_link)) > .5:
            warnings.append(f"[COM_MAGNITUDE] 机身XY最大绝对坐标={np.max(np.abs(xy_link)):.6f} m > 0.5 m。")
    sensitivity = None
    if rank == 2:
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
                delta_fixed = solution["fixed_h"]*((perturbed_mass-model.other_mass)/trunk_mass-1)
                corrected = np.asarray(delta_b)-np.concatenate([skew(s["gravity"])@delta_fixed for s in states])
                dh = np.linalg.lstsq(A, corrected, rcond=1e-5)[0]
                changed_h = h+delta_fixed+solution["basis"]@dh
                perturbations.append((ref["trunk_R"].T@(changed_h/(perturbed_mass-model.other_mass)-com_B))[:2])
        sensitivity = np.quantile(np.abs(perturbations), .95, axis=0).tolist()
    valid_xy = bool(rank == 2 and condition is not None and condition < 150 and not warnings)
    com_link = None if com_B is None else ref["trunk_R"].T@(com_B-ref["trunk_p"])
    def xy_unknown_z(value):
        return None if value is None else [float(value[0]), float(value[1]), None]
    result = dict(schema=2, source="estimated contact forces from motor torque feedback; not measured GRF",
        urdf_sha256=hashlib.sha256(xml.encode()).hexdigest(), acquisition_complete=True,
        coordinate_frame=meta["base"], mass_source="external supplied mass" if known_mass is not None else "torque-derived estimate",
        estimated_whole_mass_kg=mass, estimated_trunk_mass_kg=trunk_mass,
        rank=int(rank), condition=condition, moment_fit_rms_Nm=float(np.sqrt(np.mean(residual**2))),
        trunk_com_B_m=xy_unknown_z(com_B),
        trunk_com_link_m=xy_unknown_z(com_link),
        trunk_com_xy_link_m=None if com_link is None else com_link[:2].tolist(), z_policy=z_policy,
        fixstand_measured_pose_whole_com_B_m=None if com_B is None else
            xy_unknown_z((trunk_mass*com_B+ref["other_moment"])/mass),
        fixstand_target_pose_whole_com_B_m=None if com_B is None else
            xy_unknown_z((trunk_mass*com_B+model.evaluate(meta["stand_q"], np.eye(3))["other_moment"])/mass),
        fixstand_projection_level_frame_xy_m=projection,
        center_repeat_pose_indices=repeat_indices,
        center_repeat_corrected_delta_xy_m=repeat, center_repeat_max_m=repeat_max,
        leave_one_pose_out_com_delta_m=[v[:2] for v in holdout],
        torque_offset_Fc_scale_sensitivity_abs95_m=sensitivity,
        xy_quality_checks_passed=valid_xy, xyz_quality_checks_passed=False, automatic_model_update=False, warnings=warnings, poses=summaries,
        limitations=["Only X/Y are estimated; fixed URDF/explicit Z is an assumption and affects tilted-pose X/Y.",
                    "No external ground truth; passing checks does not prove absolute accuracy.",
                    "Unmodeled stiction, torque bias/scale, kinematic and IMU errors remain possible.",
                    "No inertia tensor identification. No body XYZ/velocity estimate is used.",
                    "FixStand COM is not dynamic trotting COM unless joint posture matches."])
    if reference_q is not None:
        trace["stage"] = "read_reference_pose"
        supplied = json.loads(Path(reference_q).read_text(encoding="utf-8"))
        supplied = supplied.get("q", supplied) if isinstance(supplied, dict) else supplied
        state = model.evaluate(supplied, np.eye(3))
        result["supplied_reference_pose_whole_com_B_m"] = None if com_B is None else (
            xy_unknown_z((trunk_mass*com_B+state["other_moment"])/mass))
    nominal = trace["model"]
    trace["body_fit"] = dict(A=A.tolist(), b_Nm=b.reshape(-1, 3).tolist(), z_policy=z_policy,
        xy_first_moment_trunk_kg_m=solution["xy_first_moment"].tolist(),
        fixed_first_moment_B_kg_m=solution["fixed_h"].tolist(), singular_values=np.linalg.svd(A, compute_uv=False).tolist(),
        weighting="equal weight per pose, averaged valid frames; diagnostic raw frames do not change fit",
        poses=[dict(name=summaries[i]["name"],
            estimated_ground_force_B_N=pose_forces[i].sum(axis=0).tolist(),
            force_balance_residual_B_N=(pose_forces[i].sum(axis=0)+mass*s["gravity"]).tolist(),
            estimated_ground_moment_B_Nm=s["mean_mu"].tolist(),
            known_nontrunk_gravity_moment_B_Nm=s["mean_known_gravity_moment"].tolist(),
            required_body_moment_B_Nm=(s["mean_mu"]-s["mean_known_gravity_moment"]).tolist(),
            predicted_body_moment_B_Nm=(skew(s["gravity"])@h).tolist(),
            fit_residual_B_Nm=residual[i].tolist(),
            moment_parallel_gravity_Nm=float(np.dot(s["mean_mu"], s["gravity"])/np.linalg.norm(s["gravity"])))
            for i, s in enumerate(states)])
    trace["observability"] = dict(
        recorded=["joint q/dq/CAN tau", "q_cmd/kp/kd", "IMU quaternion/gyro/acceleration", "sequence/phase/period"],
        derived_not_measured=["contact forces", "contact moments", "force balance", "COM candidate", "PD proxy without feedforward"],
        not_recorded_or_independently_observed=["independent foot loads/contact state", "external support or cable forces",
            "ground-truth body translation", "motor/IMU hardware timestamps and sample age", "motor feedback temperature/error flags"],
        no_guarantee="Existing interfaces cannot establish absolute rest or identify every physical cause.")
    result.update(processing_status="computed", acquisition_status="complete",
        internal_quality_status="passed" if valid_xy else "failed",
        parameter_status="candidate_not_independently_validated",
        failed_quality_check_count=len(warnings), model_diagnostics=nominal,
        identification_target="trunk body COM X/Y in trunk link frame; legs excluded; Z not estimated",
        torque_derived_mass_diagnostic_kg=float(np.median(solution["masses"])),
        mass_policy=trace["mass_policy"],
        model_comparison=dict(
            reference="URDF snapshot parameters, not independent physical ground truth",
            whole_mass_delta_kg=mass-nominal["nominal_whole_mass_kg"],
            whole_mass_delta_percent=100*(mass/nominal["nominal_whole_mass_kg"]-1),
            trunk_com_delta_link_m=None if result["trunk_com_link_m"] is None else
                xy_unknown_z(com_link-nominal["nominal_trunk_com_link_m"])),
        diagnostics=trace)
    trace["computation_complete"] = True
    trace["stage"] = "write_estimated_forces"
    with (output / "forces_estimated.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sequence", "pose", "steady_s"]+
                        [f"{leg}_f{axis}_estimated_N" for leg in ("FR", "FL", "RR", "RL") for axis in "xyz"]+
                        ["mass_estimated_kg", "min_jacobian_sigma_m"])
        writer.writerows(force_rows)
    trace["stage"] = "completed"
    trace["stage"] = "write_body_debug"
    write_body_debug(output, data, model, mass, h, set(int(row[0]) for row in force_rows))
    trace["stage"] = "completed"
    return result


def write_body_debug(output, data, model, mass, h, fit_sequences):
    """All recorded frames, including motion/rejected frames; no feedback into the estimator."""
    legs = ("FR", "FL", "RR", "RL")
    q, dq, tau, cmd, kp, kd = (columns(data, k) for k in ("q", "dq", "tau", "q_cmd", "kp", "kd"))
    with (output/"body_wrench_debug.csv").open("w", newline="", encoding="utf-8") as wf, \
         (output/"body_joint_debug.csv").open("w", newline="", encoding="utf-8") as jf:
        w, j = csv.writer(wf), csv.writer(jf)
        vector_fields = ["gravity_B", "linear_acceleration_world", "sum_force_B_est", "force_residual_B",
                         "sum_moment_B_est", "known_nontrunk_moment_B", "required_body_moment_B",
                         "predicted_body_moment_B", "body_fit_residual_B"]
        header = ["sequence", "steady_s", "ros_s", "pose", "phase", "used_in_body_fit", "roll_deg", "pitch_deg", "yaw_deg"]
        header += [f"{name}_{axis}" for name in vector_fields for axis in "xyz"]
        header += ["moment_parallel_gravity_Nm", "torque_derived_mass_diagnostic_kg"]
        header += [f"{leg}_{name}_{axis}" for leg in legs for name in ("position_B_m", "force_B_est_N", "force_world_est_N", "moment_B_est_Nm") for axis in "xyz"]
        header += [f"{leg}_J_{r}{c}_m" for leg in legs for r in range(3) for c in range(3)]
        header += [f"{leg}_J_sigma_min_m" for leg in legs]+["diagnostic_error"]
        w.writerow(header)
        j.writerow(["sequence", "steady_s", "pose", "phase", "joint", "q_rad", "dq_rad_s", "tau_CAN_joint_Nm",
                    "q_cmd_rad", "kp", "kd", "P_term_Nm", "D_term_Nm", "PD_proxy_without_ff_Nm",
                    "tau_minus_PD_proxy_Nm", "known_leg_gravity_Nm", "B_dq_Nm", "force_solve_rhs_Nm"])
        for i, frame in enumerate(data):
            sequence = int(frame["sequence"])
            R = quaternion([frame[k] for k in ("qw", "qx", "qy", "qz")])
            roll = np.arctan2(R[2, 1], R[2, 2]); pitch = np.arcsin(np.clip(-R[2, 0], -1, 1)); yaw = np.arctan2(R[1, 0], R[0, 0])
            prefix = [sequence, frame["steady_ns"]*1e-9, frame["ros_ns"]*1e-9, int(frame["pose"]), int(frame["phase"]),
                      int(sequence in fit_sequences), *np.rad2deg([roll, pitch, yaw])]
            s = model.evaluate(q[i], R); bdq = model.B*dq[i]; rhs = s["g_joint"]+bdq-tau[i]
            for k, joint in enumerate(JOINTS):
                pt = kp[i, k]*(cmd[i, k]-q[i, k]); dt = -kd[i, k]*dq[i, k]
                j.writerow([sequence, prefix[1], prefix[3], prefix[4], joint, q[i, k], dq[i, k], tau[i, k],
                    cmd[i, k], kp[i, k], kd[i, k], pt, dt, pt+dt, tau[i, k]-pt-dt, s["g_joint"][k], bdq[k], rhs[k]])
            try:
                force, _ = estimate_forces(s, tau[i], dq[i], model.B)
                fw = (R@force.T).T; moments = np.cross(s["positions"], force); mu = moments.sum(axis=0)
                known = skew(s["gravity"])@s["other_moment"]; required = mu-known; predicted = skew(s["gravity"])@h
                acc = R@np.array([frame[k] for k in ("ax", "ay", "az")])+[0, 0, -G]
                vectors = [s["gravity"], acc, force.sum(axis=0), force.sum(axis=0)+mass*s["gravity"],
                           mu, known, required, predicted, predicted-required]
                row = prefix+np.concatenate(vectors).tolist()+[float(np.dot(mu, s["gravity"])/G),
                        -float(np.dot(force.sum(axis=0), s["gravity"]))/G**2]
                for leg in range(4):
                    row += [*s["positions"][leg], *force[leg], *fw[leg], *moments[leg]]
                row += s["jacobians"].ravel().tolist()
                row += [float(np.linalg.svd(J, compute_uv=False)[-1]) for J in s["jacobians"]]
                w.writerow(row+[""])
            except (ValueError, np.linalg.LinAlgError) as error:
                w.writerow(prefix+[""]*(len(header)-len(prefix)-1)+[str(error)])


def report_lines(result, folder, output):
    """Human-readable diagnostics, independent of the numerical acceptance rules."""
    model, comparison = result["model_diagnostics"], result["model_comparison"]
    checks = "通过" if result["xy_quality_checks_passed"] else "未通过"
    lines = ["# 机身质心XY辨识（trunk本体，排除四条腿；Z不辨识）", "",
        f"[COM_ID][COMPUTED] 采集完整，离线计算成功；内部质量检查{checks}。",
        f"失败检查数：{len(result['warnings'])}；参数状态：候选值，未独立验证。未自动修改模型。",
        f"机身质心XY候选 / trunk坐标 m：{result['trunk_com_xy_link_m']}",
        f"Z不辨识、不验收；仅固定参考={result['z_policy']['fixed_reference_link_m']:.9f} m，来源={result['z_policy']['source']}。倾斜时XY依赖该参考。",
        f"采用整机质量={result['estimated_whole_mass_kg']:.6f} kg；非机身模型质量="
        f"{model['nominal_whole_mass_kg']-model['nominal_trunk_mass_kg']:.6f} kg；"
        f"机身质量={result['estimated_trunk_mass_kg']:.6f} kg；来源={result['mass_source']}。",
        "CAN力矩换算按用户确认准确使用，不自动缩放、不猜测零偏、不重复叠加PD。",
        f"数据目录：{folder.resolve()}", f"输出目录：{output.resolve()}", "",
        "## 未通过的质量检查", ""]
    lines += [f"- {w}" for w in result["warnings"]] or ["- 未触发内部阈值；这不证明物理参数准确。"]
    lines += ["", "## 各姿态诊断", "",
        "raw=CSV正式段帧数；selected=离线选取帧数；valid=通过足力检查帧数，均不是独立测量次数。",
        "累计角变化=反馈q相对本段首帧的变化，不是q与目标q_cmd之差。",
        "| 姿态 | raw/selected/valid | 时长 s | 最大累计角变化 rad（限0.003） | 最大关节速度 rad/s（限0.015） | 最大IMU角速度 rad/s（限<0.015） |",
        "|---|---:|---:|---:|---:|---:|"]
    for pose in result["poses"]:
        c = pose["static_checks"]
        lines.append(f"| {pose['pose']}: {pose['name']} | {pose['raw_samples']}/{pose['selected_samples']}/{pose['samples']} "
            f"| {pose['duration_s']:.3f} | {c['max_joint_drift_rad']:.9f} "
            f"| {c['max_joint_speed_rad_s']:.9f} | {c['max_gyro_norm_rad_s']:.9f} |")
    for pose in result["poses"]:
        if pose["static_checks"]["warnings"]:
            detail = pose["static_checks"]["joint_drift_detail"]
            lines.append(f"定位：{pose['name']} / {detail['joint']}，CSV sequence={detail['sequence']}，"
                f"正式段+{detail['elapsed_s']:.6f}s；各项丢弃计数={pose['rejected_counts']}。")
    lines += ["", "## 逐姿态机身静力平衡（均为模型反算，不是实测接触力）", "",
        "静态无外部力矩时，合力矩沿重力的分量应为0；该分量无法通过改变质心坐标消除。",
        "| 姿态 | 合力残差模长 N（相对采用质量） | 沿重力的力矩 N·m | 机身拟合残差XYZ N·m |",
        "|---|---:|---:|---|"]
    for pose in result["diagnostics"]["body_fit"]["poses"]:
        lines.append(f"| {pose['name']} | {np.linalg.norm(pose['force_balance_residual_B_N']):.6f} "
            f"| {pose['moment_parallel_gravity_Nm']:.6f} | "
            f"{[round(v, 6) for v in pose['fit_residual_B_Nm']]} |")
    lines += ["", "## 模型假设与参考对照（独立于内部质量检查）", ""]
    lines += [f"- {note}" for note in model["notes"]]
    lines += ["- 未根据速度或驱动力矩的符号猜测静摩擦方向；本次输出修改没有补偿静摩擦。",
        "- 模型值来自本次URDF快照，不是外部实测真值；下述差值只报告，不新增验收门槛。",
        f"整机质量：采用={result['estimated_whole_mass_kg']:.6f} kg；"
        f"URDF={model['nominal_whole_mass_kg']:.6f} kg；"
        f"差值={comparison['whole_mass_delta_kg']:+.6f} kg ({comparison['whole_mass_delta_percent']:+.3f}%)。",
        f"躯干质量候选 kg：{result['estimated_trunk_mass_kg']:.6f}",
        f"躯干质心XY候选 / trunk m：{result['trunk_com_xy_link_m']}",
        f"URDF躯干质心 / trunk m：{model['nominal_trunk_com_link_m']}",
        f"躯干质心与URDF之差 / trunk m：{comparison['trunk_com_delta_link_m']}",
        f"力矩反算的等效总质量（仅诊断，不作为真机质量输入）：{result['torque_derived_mass_diagnostic_kg']:.6f} kg。",
        f"URDF关节Fc / N·m：{model['friction_Fc_Nm']}",
        f"URDF足底球碰撞半径 / m：{model['foot_collision_sphere_radii_m']}", "",
        "## 候选结果与拟合诊断", "",
        f"秩={result['rank']}（要求2，仅XY）；条件数={result['condition']}（限150）；"
        f"力矩RMS={result['moment_fit_rms_Nm']:.6f} N·m（限2）。",
        f"回中最大差异={result['center_repeat_max_m']} m（限0.015）；回中段索引={result['center_repeat_pose_indices']}。",
        f"XY的Fc尺度力矩偏差敏感性 m（非置信区间、非已验证补偿）：{result['torque_offset_Fc_scale_sensitivity_abs95_m']}",
        "FixStand腿姿不等于动态trot腿姿；传感器静止检查不能证明全机绝对静止。", "",
        "debug.json保存完整A/b矩阵、一阶矩、奇异值、逐姿态力/力矩分解、拟合贡献、数据校验值及可观测范围。",
        "body_wrench_debug.csv逐帧保留过渡/记录段，包括未入选帧：各腿足端、J、反算力、力矩、重力和拟合残差。",
        "body_joint_debug.csv逐关节逐帧输出CAN tau、q/dq、目标、P/D分量、腿部重力项、B*dq和求解右端；PD代理量不是实测力矩。",
        "不能从现有接口独立观测足底载荷、外部扶持/拉拽、机身绝对平动及传感器采样年龄；不保证一次采集能识别所有原因。"]
    return lines


def save_reports(output, result, lines, trace):
    for name, payload in (("results.json", result), ("debug.json", trace)):
        path = output / name
        temporary = output / (name+".tmp")
        temporary.write_text(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")
        temporary.replace(path)
    path = output / "report.md"
    temporary = output / "report.md.tmp"
    temporary.write_text("\n".join(lines)+"\n", encoding="utf-8")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--mass-kg", type=float, help="Optional independently measured whole mass")
    parser.add_argument("--trunk-z-m", type=float, help="Fixed trunk COM height reference; not estimated (default: URDF)")
    parser.add_argument("--reference-q", type=Path, help="Optional JSON array of 12 actual nominal-trot joint angles")
    parser.add_argument("--output-directory", type=Path,
                        help="Write reports/estimated-force output elsewhere without replacing the original run's reports")
    args = parser.parse_args()
    folder = args.directory
    output = args.output_directory if args.output_directory is not None else folder
    trace = dict(stage="startup", source_directory=str(folder.resolve()), output_directory=str(output.resolve()),
        processor=str(Path(__file__).resolve()),
        processor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        python_version=sys.version, numpy_version=np.__version__)
    try:
        require(folder.is_dir(), f"Input dataset directory does not exist: {folder.resolve()}")
        output.mkdir(parents=True, exist_ok=True)
        require(args.mass_kg is None or np.isfinite(args.mass_kg) and args.mass_kg > 0, "Invalid supplied mass")
        result = analyze(folder, args.mass_kg, args.reference_q, output, trace, args.trunk_z_m)
        trace.update(processing_status=result["processing_status"], internal_quality_status=result["internal_quality_status"],
                     failed_quality_checks=result["warnings"], model_comparison=result["model_comparison"])
        lines = report_lines(result, folder, output)
        save_reports(output, result, lines, trace)
        print("\n".join(lines))
        return 0 if result["xy_quality_checks_passed"] else 2
    except Exception as error:
        rejected = isinstance(error, (ValueError, KeyError, ET.ParseError, np.linalg.LinAlgError))
        status, code = ("DATA_REJECTED", 3) if rejected else ("PROCESSING_ERROR", 4)
        trace.update(processing_status=status.lower(), error_type=type(error).__name__, error=str(error))
        if not rejected:
            trace["traceback"] = traceback.format_exc()
        lines = [f"# COM identification: {status}", "",
            f"[COM_ID][{status}] {type(error).__name__}: {error}",
            f"处理阶段={trace['stage']}；当前姿态={trace.get('current_pose', '尚未进入姿态处理')}。",
            f"采集标记={trace.get('acquisition', '尚未读取或不存在')}。",
            f"数据目录={folder.resolve()}；输出目录={output.resolve()}。",
            "本次未生成可接受参数，未修改模型。"]
        print("\n".join(lines), file=sys.stderr)
        if not rejected:
            print(trace["traceback"], file=sys.stderr)
        if output.is_dir():
            try:
                save_reports(output, dict(xy_quality_checks_passed=False, xyz_quality_checks_passed=False, automatic_model_update=False,
                    processing_status=status.lower(), error=str(error), error_type=type(error).__name__,
                    diagnostics=trace), lines, trace)
            except OSError as write_error:
                print(f"[COM_ID][REPORT_WRITE_ERROR] 无法写入诊断文件：{write_error}", file=sys.stderr)
                return 4
        return code


if __name__ == "__main__":
    sys.exit(main())
