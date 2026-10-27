#!/usr/bin/env python3
"""Archive one hardware trot and screen it without ROS, builds, plots or pandas.

Raw files are frozen before calculation. Runtime parameter records are authoritative;
legacy manual snapshots are labelled unverified, never inferred from current source.
"""
import argparse
import csv
import datetime as dt
import hashlib
import io
import json
import math
from pathlib import Path
import re
import sys
import time

import numpy as np

VERSION = 2
PROJECT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = PROJECT / 'Troting_Dataset' / 'real_trials'
GAINS = ('kp_orientation', 'kd_orientation', 'kp_body_position',
         'kd_body_position', 'kp_swing', 'kd_swing')
# Screening thresholds, not contact detectors or stability guarantees.
LIMITS = dict(q_jump_rad=0.05, torque_jump_Nm=20.0, landing_z_m=0.03,
              lift_target_xy_m=0.05, hf_gyro_rad_s=0.2, hf_acc_m_s2=0.8)


def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [clean(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def write_json(path, value):
    path.write_text(json.dumps(clean(value), ensure_ascii=False, indent=2,
                               allow_nan=False) + '\n')


def frozen_bytes(path):
    # Ignore appended bytes after opening: later tuning must not change this archive.
    with path.open('rb') as stream:
        size = path.stat().st_size
        return stream.read(size)


def trials(log_root, pid=None, before=None):
    found = []
    pattern = f'ros2_control_node_{pid if pid is not None else "*"}_*.log'
    for path in log_root.glob(pattern):
        raw = frozen_bytes(path)
        text = raw.decode('utf-8', errors='replace')
        if '[sysu219_hardware]' not in text:
            continue  # Never select Gazebo by file modification time.
        process = int(re.search(r'node_(\d+)_', path.name)[1])
        current = None
        number = 0
        for line in text.splitlines():
            stamp = re.search(r'\[(\d+\.\d+)\]', line)
            if not stamp:
                continue
            stamp = float(stamp[1])
            if 'Switched from ' in line and ' to trotting' in line:
                number += 1
                current = dict(pid=process, trial_number=number, start_s=stamp,
                               end_s=None, log_path=str(path), log_bytes=raw)
                found.append(current)
            elif 'Switched from trotting to ' in line and current is not None:
                current['end_s'] = stamp
                current = None
    if before is not None:
        found = [v for v in found if v['start_s'] < before]
    return sorted(found, key=lambda v: v['start_s'])


def load_rows(raw, start, end):
    truncated = bool(raw and not raw.endswith(b'\n'))
    if truncated:
        raw = raw[:raw.rfind(b'\n') + 1]
    reader = csv.reader(io.StringIO(raw.decode('utf-8')))
    header = next(reader)
    clock = header.index('system_s')
    selected, malformed = [], 0
    for line in reader:
        if len(line) != len(header):
            malformed += 1
            continue
        try:
            row = [float(v) for v in line]
        except ValueError:
            malformed += 1
            continue
        if start <= row[clock] < end:
            selected.append(row)
    return header, selected, malformed, truncated


def assemble(files, start, end):
    header, rows, sources = None, [], []
    for path in files:
        raw = frozen_bytes(path)
        try:
            names, selected, malformed, partial = load_rows(raw, start, end)
        except (ValueError, StopIteration, UnicodeError):
            continue
        if not selected:
            continue
        if header is not None and names != header:
            raise ValueError('同一轮的CSV列不一致，停止混合分析：' + str(path))
        header = names
        rows.extend(selected)
        sources.append(dict(path=str(path), raw=raw, sha256=hashlib.sha256(raw).hexdigest(),
                            selected_rows=len(selected), malformed_rows=malformed,
                            partial_tail=partial))
    if not rows:
        raise ValueError('找到真机运行日志，但尚无对应CSV；本次不选用别轮数据。')
    array = np.asarray(rows, dtype=float)
    index = {v: i for i, v in enumerate(header)}
    array = array[np.argsort(array[:, index['ros_s']], kind='stable')]
    # Legacy event exports overlap. A conflicting duplicate is reported, not averaged.
    unique, seen, conflicts = [], {}, 0
    for row in array:
        key = (row[index['cycle']], row[index['ros_s']])
        if key in seen:
            previous = unique[seen[key]]
            if not np.allclose(row, previous, equal_nan=True, rtol=1e-10, atol=1e-12):
                conflicts += 1
        else:
            seen[key] = len(unique)
            unique.append(row)
    return header, np.asarray(unique), sources, len(rows) - len(unique), conflicts


class Data:
    def __init__(self, names, rows):
        self.names, self.rows = names, rows
        self.index = {v: i for i, v in enumerate(names)}

    def col(self, name, default=np.nan):
        if name not in self.index:
            return np.full(len(self.rows), default)
        return self.rows[:, self.index[name]]

    def mat(self, name, count):
        return np.column_stack([self.col(f'{name}_{i}') for i in range(count)])


def stats(a):
    a = np.asarray(a)
    finite = a[np.isfinite(a)]
    if not finite.size:
        return dict(rms=None, p95_abs=None, max_abs=None)
    return dict(rms=float(np.sqrt(np.mean(finite ** 2))),
                p95_abs=float(np.percentile(abs(finite), 95)),
                max_abs=float(np.max(abs(finite))))


def axis_stats(a):
    return [stats(a[:, i]) for i in range(a.shape[1])]


def event_peaks(data, values, valid, count=6):
    # values: one scalar magnitude per frame; retain locations for targeted CSV review.
    values = np.where(valid & np.isfinite(values), values, -np.inf)
    indices = np.argsort(values)[-count:][::-1]
    return [dict(cycle=int(data.col('cycle')[i]), ros_s=float(data.col('ros_s')[i]),
                 value=float(values[i])) for i in indices if np.isfinite(values[i])]


def quality(data):
    clock = data.col('ros_s')
    delta = np.diff(clock)
    good_dt = delta[(delta > 0) & np.isfinite(delta)]
    nominal = float(np.median(good_dt)) if len(good_dt) else None
    cycles = data.col('cycle')
    advances = np.diff(cycles)
    gaps = (advances != 1) | (delta <= 0)
    if nominal is not None:
        gaps |= delta > 1.5 * nominal
    revisions = data.col('param_revision', -1)
    # Spectrum windows must never bridge a missing interval or a tuning change.
    breaks = gaps | (revisions[1:] != revisions[:-1])
    contiguous = np.r_[False, ~gaps]
    expected = int(cycles[-1] - cycles[0] + 1)
    missing = int(np.sum(np.maximum(advances - 1, 0)))
    return dict(rows=len(clock), duration_s=float(clock[-1] - clock[0]),
                sample_dt_s=nominal, observed_sampling_hz=1 / nominal if nominal else None,
                missing_cycles=missing, coverage_between_first_last=len(clock) / max(expected, 1),
                queue_dropped_max=stats(data.col('diagnostic_dropped'))['max_abs'],
                first_cycle=int(cycles[0]), last_cycle=int(cycles[-1]),
                gaps=[dict(after_cycle=int(cycles[i]), dt_s=float(delta[i]),
                           cycle_advance=int(advances[i])) for i in np.flatnonzero(gaps)]), contiguous, breaks


def spectrum(data, breaks, sampling_hz):
    """One-second Hann windows; interpolate small jitter only inside intact segments.

    High-frequency energy near planned transitions is a diagnostic association,
    not proof of physical contact, resonance, or causality.
    """
    if sampling_hz is None or sampling_hz < 40:
        return dict(status='insufficient_sampling', channels={})
    upper = min(80.0, 0.4 * sampling_hz)
    length = int(round(sampling_hz))
    window = np.hanning(length)
    freq = np.fft.rfftfreq(length, 1 / sampling_hz)
    band = (freq >= 10) & (freq <= upper)
    contact = data.mat('contact', 4)
    switches = np.any(contact[1:] != contact[:-1], axis=1)
    near = np.zeros(len(contact), dtype=bool)
    radius = int(math.ceil(0.03 * sampling_hz))
    for i in np.flatnonzero(switches) + 1:
        near[max(0, i - radius):i + radius + 1] = True
    channels = {f'imu_{i}': ('gyro_B_rad_s' if i <= 6 else 'acc_B_m_s2') for i in range(4, 10)}
    channels.update({f'{name}_{i}': unit for name, unit, count in
                     [('qd', 'joint_rad_s', 12), ('tau_ff_cmd', 'command_Nm', 12),
                      ('v_unfiltered', 'estimated_m_s', 3), ('v', 'estimated_m_s', 3)]
                     for i in range(count)})
    boundaries = np.r_[0, np.flatnonzero(breaks) + 1, len(data.rows)]
    time_s = data.col('ros_s')
    results = {}
    for name, unit in channels.items():
        if name not in data.index:
            continue  # 新日志不再含旧 KF 的 v_unfiltered；旧记录仍按原列读取。
        signal = data.col(name)
        densities, near_energy, far_energy, near_n, far_n = [], 0.0, 0.0, 0, 0
        for a, b in zip(boundaries[:-1], boundaries[1:]):
            for start in range(a, b - length + 1, max(1, length // 2)):
                stop = start + length
                t = time_s[start:stop] - time_s[start]
                v = signal[start:stop]
                if not np.isfinite(v).all() or not np.isfinite(t).all():
                    continue
                delta = np.diff(t)
                if np.any(delta <= 0) or np.max(abs(delta * sampling_hz - 1)) > 0.25:
                    continue
                target = np.linspace(0, t[-1], length)
                v = np.interp(target, t, v)
                v = v - np.linspace(v[0], v[-1], length)
                transform = np.fft.rfft(v * window)
                density = abs(transform) ** 2 / (sampling_hz * np.sum(window ** 2))
                density[1:-1] *= 2
                densities.append(density)
                # Same band and window for both masks; useful for localization only.
                high = np.fft.irfft(np.where(band, transform, 0), n=length)
                mask = near[start:stop]
                near_energy += float(np.sum(high[mask] ** 2))
                far_energy += float(np.sum(high[~mask] ** 2))
                near_n += int(mask.sum())
                far_n += int((~mask).sum())
        if not densities:
            continue
        mean = np.mean(densities, axis=0)
        band_power = float(np.sum(mean[band]) * sampling_hz / length)
        total_power = float(np.sum(mean[freq >= 1]) * sampling_hz / length)
        results[name] = dict(unit=unit, windows=len(densities), band_rms=math.sqrt(band_power),
                             peak_hz=float(freq[band][np.argmax(mean[band])]) if band_power > 1e-20 else None,
                             band_power_fraction=band_power / total_power if total_power > 1e-20 else None,
                             near_transition_power=near_energy / near_n if near_n else None,
                             away_transition_power=far_energy / far_n if far_n else None)
    return dict(status='ok' if results else 'insufficient_contiguous_data',
                band_hz=[10, upper], window_s=length / sampling_hz,
                channels=results, note='控制采样频率不等于传感器新反馈频率；10Hz起为筛查频带，不是故障判据。')


def analyse(data):
    q, contiguous, breaks = quality(data)
    contact = data.mat('contact', 4)
    switch = np.r_[False, np.any(contact[1:] != contact[:-1], axis=1)] & contiguous
    q_cmd, tau = data.mat('q_cmd', 12), data.mat('tau_ff_cmd', 12)
    q_jump = np.r_[0, np.max(abs(np.diff(q_cmd, axis=0)), axis=1)]
    tau_jump = np.r_[0, np.max(abs(np.diff(tau, axis=0)), axis=1)]
    foot = data.mat('feet_G', 12).reshape(-1, 4, 3)
    goal = data.mat('goal_G', 12).reshape(-1, 4, 3)
    target = data.mat('end_G', 12).reshape(-1, 4, 3)
    lift = np.zeros_like(contact, dtype=bool)
    land = lift.copy()
    lift[1:] = (contact[1:] == 0) & (contact[:-1] == 1) & contiguous[1:, None]
    land[1:] = (contact[1:] == 1) & (contact[:-1] == 0) & contiguous[1:, None]
    swing = contact == 0
    error = goal - foot
    landing_z = np.max(np.where(land, abs(error[:, :, 2]), 0), axis=1)
    target_xy = np.max(abs(target[:, :, :2] - foot[:, :, :2]), axis=2)
    liftoff_xy = np.max(np.where(lift, target_xy, 0), axis=1)
    angle_error = data.mat('rpy_ref', 3) - data.mat('rpy', 3)
    angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
    flags = data.col('wbc_flags', 0).astype(np.int64)
    solver_ids, indices = np.unique(data.col('solver_id'), return_index=True)
    status = data.col('solver_status')[indices]
    result = data.col('solver_result')[indices]
    output = dict(quality=q, orientation_error_deg=axis_stats(np.rad2deg(angle_error)),
                  estimated_position_error_m=axis_stats(data.mat('p_ref', 3) - data.mat('p', 3)),
                  estimated_velocity_m_s=axis_stats(data.mat('v', 3)),
                  joint_position_error_rad=axis_stats(q_cmd - data.mat('q', 12)),
                  swing_tracking_error_m=axis_stats(error[swing]) if swing.any() else None,
                  landing_z_error_m=stats(abs(error[:, :, 2])[land]),
                  liftoff_target_xy_error_m=stats(target_xy[lift]),
                  transition_q_jump_rad=stats(q_jump[switch]),
                  continuous_q_jump_rad=stats(q_jump[contiguous & ~switch]),
                  transition_ff_torque_jump_Nm=stats(tau_jump[switch]),
                  motor_kd_jump=stats(np.diff(data.mat('kd_cmd', 12), axis=0)[contiguous[1:]]),
                  control_update_ms=stats(data.col('update_ms')),
                  observed_period_ms=stats(data.col('wall_dt_ms')),
                  solvers=dict(unique_records=len(solver_ids), failed=int(np.sum(status != 0)),
                               cached=int(np.sum(result == 2)), fallback=int(np.sum(result == 0))),
                  wbc=dict(attempt=int(np.sum((flags & 1) != 0)),
                           failed=int(np.sum(((flags & 1) != 0) & ((flags & 2) == 0))),
                           relaxation_failed=int(np.sum(((flags & 4) != 0) & ((flags & 8) == 0))),
                           applied=int(np.sum((flags & 16) != 0)),
                           rejected=int(np.sum((flags & 32) != 0)),
                           command_clamped=int(np.sum((flags & 64) != 0))),
                  events=dict(joint_jump=event_peaks(data, q_jump, contiguous),
                              torque_jump=event_peaks(data, tau_jump, contiguous),
                              landing_z=event_peaks(data, landing_z, land.any(axis=1)),
                              liftoff_target_xy=event_peaks(data, liftoff_xy, lift.any(axis=1))))
    output['high_frequency'] = spectrum(data, breaks, q['observed_sampling_hz'])
    force = data.mat('ground_force_P', 12).reshape(-1, 4, 3)
    output['planned_force_sum_N'] = axis_stats(force.sum(axis=1))
    applied = (flags & 16) != 0
    corrected = force + data.mat('relax_delta_force', 12).reshape(-1, 4, 3)
    output['applied_model_force_sum_N'] = axis_stats(corrected[applied].sum(axis=1)) if applied.any() else None
    output['interpretation'] = '位置/速度为估计值；contact为计划相位；足力为规划/模型值，均非真实运动或实测接触力。'
    return output


def parameters(files, data, pid, manual=None):
    revisions = sorted(set(data.col('param_revision', -1)))
    records, source_files = {}, []
    for path in files:
        try:
            raw = frozen_bytes(path)
            record = json.loads(raw)
        except (ValueError, OSError):
            continue
        if record.get('format') != 'sysu219_mpc_wbc_snapshot_v1' or record.get('control_pid') != pid:
            continue
        revision = record.get('applied_revision')
        if revision not in revisions or 'run_id' not in record:
            continue
        mask = data.col('param_revision') == revision
        run_ids = set(data.col('run_id')[mask])
        if record['run_id'] not in run_ids:
            continue
        # PID reuse and old files must not silently identify a different experiment.
        if not (data.col('system_s')[0] - 0.1 <= record['system_s'] <= data.col('system_s')[-1] + 0.1):
            continue
        valid = True
        for key, size in [('Q', 13), ('R', 12), ('S', 12)]:
            matrix = np.asarray(record.get(key, []))
            valid &= matrix.shape == (size, size) and np.isfinite(matrix).all()
        for key in GAINS:
            vector = np.asarray(record.get('WBC', {}).get(key, []))
            valid &= vector.shape == (3,) and np.isfinite(vector).all()
        valid &= all(isinstance(record.get('WBC', {}).get(key), (int, float)) and
                     math.isfinite(record['WBC'][key]) for key in
                     ('swing_hip_posture_weight', 'kp_swing_hip_posture', 'kd_swing_hip_posture'))
        if not valid:
            continue
        records[str(int(revision))] = record
        source_files.append((path, raw))
    complete = all(v >= 0 and str(int(v)) in records for v in revisions)
    result = dict(status='runtime_matched' if complete else 'missing_runtime_parameters',
                  revisions=records, observed_revisions=revisions)
    if manual is not None:
        raw = frozen_bytes(manual)
        result['user_supplied_snapshot_unverified'] = json.loads(raw)
        source_files.append((manual, raw))
    return result, source_files


def concise(trial, metrics, params):
    q = metrics['quality']
    lines = [f"真机 PID {trial['pid']}，第{trial['trial_number']}次 trotting，已记录 {q['duration_s']:.2f} 秒。"]
    lines.append('参数：' + ('已按实际生效版本对应。' if params['status'] == 'runtime_matched'
                            else '旧记录缺运行时参数版本，不从当前源码猜参数。'))
    concerns = []
    for field, threshold, label, scale, unit in [
        ('transition_q_jump_rad', LIMITS['q_jump_rad'], '换相关节目标跳变', 180 / np.pi, '°'),
        ('transition_ff_torque_jump_Nm', LIMITS['torque_jump_Nm'], '换相前馈力矩变化', 1, 'N·m'),
        ('landing_z_error_m', LIMITS['landing_z_m'], '计划落地时竖直跟踪误差', 100, 'cm'),
        ('liftoff_target_xy_error_m', LIMITS['lift_target_xy_m'], '落点相对反馈脚的XY偏差', 100, 'cm')]:
        value = metrics[field]['max_abs']
        if value is not None and value > threshold:
            concerns.append(f'{label} {value * scale:.1f}{unit}')
    lines.append('关注：' + ('；'.join(concerns[:3]) if concerns else '未触发当前常规筛查门槛') + '。')
    hf = metrics['high_frequency']
    channels = hf['channels']
    candidates = [(name, v) for name, v in channels.items() if name.startswith('imu_') and
                  v['band_rms'] > (LIMITS['hf_gyro_rad_s'] if int(name[4:]) <= 6 else LIMITS['hf_acc_m_s2'])]
    if candidates:
        name, v = max(candidates, key=lambda pair: pair[1]['band_power_fraction'] or 0)
        lines.append(f"高频：{name} 在筛查频带内有较强分量，峰值 {v['peak_hz']:.1f} Hz；需结合换相与反馈判断来源。")
    elif hf['status'] != 'ok':
        lines.append('高频：连续数据不足，暂不下判断。')
    else:
        lines.append('高频：本次IMU未触发当前筛查门槛，其他信号指标已保留供对比。')
    missing = q['missing_cycles']
    suffix = ('已按指定截止时间截取。' if trial.get('range_clipped') else
              '运行尚未结束，当前为冻结片段。' if trial['end_s'] is None else
              '估计位置和模型足力不当作真值。')
    lines.append(f'数据：记录中间缺 {missing} 个周期；' + suffix)
    return lines


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid', type=int, help='锁定指定真机进程')
    parser.add_argument('--trial', type=int, help='该进程第几次进入trotting；需同时指定--pid')
    parser.add_argument('--before', help='只选此时间以前开始的试验，例：2026-10-26T17:28:00')
    parser.add_argument('--snapshot', type=Path, help='旧数据可附手动快照，但不冒充已验证运行参数')
    parser.add_argument('--log-root', type=Path, default=Path.home() / '.ros/log')
    parser.add_argument('--csv-root', type=Path, default=Path('/tmp'))
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    if args.trial is not None and args.pid is None:
        parser.error('--trial 需要同时指定 --pid，避免选错运行')
    cutoff = dt.datetime.fromisoformat(args.before).timestamp() if args.before else None
    available = trials(args.log_root, args.pid, cutoff)
    if args.trial is not None:
        available = [v for v in available if v['trial_number'] == args.trial]
    if not available:
        raise ValueError('没有找到符合条件的真机trotting记录。')
    trial = available[-1]
    frozen_at = time.time()
    end = min(trial['end_s'] or frozen_at, frozen_at)
    trial['range_clipped'] = cutoff is not None and cutoff < end
    if cutoff is not None:
        end = min(end, cutoff)
    paths = sorted(set(args.csv_root.glob(f'trotting_debug_{trial["pid"]}_*.csv')) |
                   set(args.csv_root.glob(f'trotting_run_{trial["pid"]}_*_chunk*.csv')))
    names, rows, sources, duplicates, conflicts = assemble(paths, trial['start_s'], end)
    data = Data(names, rows)
    if 'use_sim_time' in data.index and np.any(data.col('use_sim_time') != 0):
        raise ValueError('CSV标记为仿真，拒绝作为真机分析。')
    params, parameter_files = parameters(args.csv_root.glob(f'trotting_run_{trial["pid"]}_*_rev*.json'),
                                         data, trial['pid'], args.snapshot)
    manifests = []
    if 'run_id' in data.index:
        for path in args.csv_root.glob(f'trotting_run_{trial["pid"]}_*.json'):
            if '_rev' in path.name:
                continue
            try:
                raw = frozen_bytes(path)
                record = json.loads(raw)
                if record.get('run_id') in set(data.col('run_id')) and \
                        trial['start_s'] <= record.get('last_system_s', 0) <= end:
                    manifests.append((path, raw))
            except (ValueError, OSError):
                continue
    identity = dt.datetime.fromtimestamp(trial['start_s']).strftime('%Y%m%d_%H%M%S')
    base = args.output / f'real_{identity}_pid{trial["pid"]}_trot{trial["trial_number"]}'
    signature = hashlib.sha256(json.dumps(dict(
        tool_version=VERSION, start=trial['start_s'], end=trial['end_s'], selected_end=end,
        inputs=[(v['path'], v['sha256']) for v in sources],
        parameters=[(str(p), hashlib.sha256(raw).hexdigest()) for p, raw in parameter_files],
        manifests=[(str(p), hashlib.sha256(raw).hexdigest()) for p, raw in manifests]
    ), sort_keys=True).encode()).hexdigest()
    for existing in args.output.glob(base.name + '*'):
        try:
            cached = json.loads((existing / 'metrics.json').read_text())
            if cached.get('input_signature') == signature:
                print('\n'.join(cached['spoken_summary']))
                print('归档：' + str(existing))
                return 0
        except (ValueError, OSError):
            continue
    target = base
    if target.exists():
        target = base.with_name(base.name + '_' + str(time.time_ns()))
    target.mkdir(parents=True)
    raw_dir = target / 'raw'
    raw_dir.mkdir()
    # Freeze originals first so the user can continue tuning while metrics run.
    for source in sources:
        (raw_dir / Path(source['path']).name).write_bytes(source['raw'])
    (raw_dir / Path(trial['log_path']).name).write_bytes(trial['log_bytes'])
    for path, raw in parameter_files + manifests:
        (raw_dir / path.name).write_bytes(raw)
    manifest = dict(format='sysu219_real_trial_archive_v1', tool_version=VERSION,
                    frozen_at_s=frozen_at, selected_end_s=end, input_signature=signature,
                    trial={k: v for k, v in trial.items() if k != 'log_bytes'},
                    sources=[{k: v for k, v in source.items() if k != 'raw'} for source in sources],
                    duplicate_rows=duplicates, conflicting_duplicates=conflicts)
    write_json(target / 'manifest.json', manifest)
    write_json(target / 'parameters.json', params)
    metrics = analyse(data)
    metrics['quality'].update(duplicate_rows=duplicates, conflicting_duplicates=conflicts,
                              source_malformed_rows=sum(s['malformed_rows'] for s in sources),
                              partial_source_tails=sum(s['partial_tail'] for s in sources),
                              recorded_exit_marker=any(json.loads(raw).get('complete', False) for _, raw in manifests),
                              io_errors=max([json.loads(raw).get('io_errors', 0) for _, raw in manifests] or [0]))
    metrics['parameter_segments'] = []
    revisions = data.col('param_revision', -1)
    cuts = np.r_[0, np.flatnonzero(revisions[1:] != revisions[:-1]) + 1, len(rows)]
    for a, b in zip(cuts[:-1], cuts[1:]):
        if b - a >= 2:
            metrics['parameter_segments'].append(dict(
                revision=int(revisions[a]), first_cycle=int(data.col('cycle')[a]),
                last_cycle=int(data.col('cycle')[b - 1]), metrics=analyse(Data(names, rows[a:b]))))
    metrics['comparison'] = None
    previous = []
    for path in args.output.glob('real_*/manifest.json'):
        if path.parent == target:
            continue
        try:
            meta = json.loads(path.read_text())
            if meta['trial']['start_s'] < trial['start_s']:
                previous.append((meta['trial']['start_s'], path.parent))
        except (ValueError, OSError, KeyError):
            continue
    if previous:
        older = max(previous)[1]
        try:
            old = json.loads((older / 'metrics.json').read_text())
            old_fs = old['quality']['observed_sampling_hz']
            new_fs = metrics['quality']['observed_sampling_hz']
            if old_fs and new_fs and abs(old_fs / new_fs - 1) < 0.1:
                metrics['comparison'] = dict(archive=str(older), hf_band_rms_ratios={
                    name: current['band_rms'] / old['high_frequency']['channels'][name]['band_rms']
                    for name, current in metrics['high_frequency']['channels'].items()
                    if name in old['high_frequency']['channels'] and
                    old['high_frequency']['channels'][name]['band_rms'] > 1e-12},
                    note='仅描述记录信号变化，未控制步态、命令、参数、持续时间等混杂因素。')
        except (ValueError, OSError, KeyError):
            pass
    metrics['spoken_summary'] = concise(trial, metrics, params)
    metrics['input_signature'] = signature
    metrics['tool_version'] = VERSION
    metrics['screening_limits'] = LIMITS
    write_json(target / 'metrics.json', metrics)
    print('\n'.join(metrics['spoken_summary']))
    print('归档：' + str(target))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError) as error:
        print('未完成归档体检：' + str(error), file=sys.stderr)
        raise SystemExit(2)
