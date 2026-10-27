#!/usr/bin/env python3
"""Synthetic data checks only. No ROS, compiler, hardware, bytecode or build tools."""
import contextlib
import csv
import datetime
import io
import json
from pathlib import Path
import runpy
import tempfile

import numpy as np

tool = runpy.run_path(str(Path(__file__).with_name('real_trial_check.py')))


def trace(start, run):
    names = ['cycle', 'ros_s', 'system_s', 'steady_s', 'param_revision',
             'param_effective_cycle', 'run_id', 'use_sim_time', 'diagnostic_dropped',
             'solver_id', 'solver_status', 'solver_result', 'wbc_flags', 'update_ms', 'wall_dt_ms']
    for prefix, count in [('imu', 10), ('contact', 4), ('q_cmd', 12), ('q', 12),
                          ('qd', 12), ('tau_ff_cmd', 12), ('kd_cmd', 12),
                          ('feet_G', 12), ('goal_G', 12), ('end_G', 12),
                          ('rpy', 3), ('rpy_ref', 3), ('p', 3), ('p_ref', 3),
                          ('v', 3), ('v_unfiltered', 3), ('ground_force_P', 12),
                          ('relax_delta_force', 12)]:
        names.extend(f'{prefix}_{i}' for i in range(count))
    index = {key: i for i, key in enumerate(names)}
    x = np.zeros((800, len(names)))
    elapsed = np.arange(800) / 200
    for name in ('ros_s', 'system_s', 'steady_s'):
        x[:, index[name]] = start + 0.002 + elapsed
    x[:, index['cycle']] = np.arange(800)
    x[:, index['run_id']] = run
    x[400:, index['param_revision']] = 1
    x[400:, index['param_effective_cycle']] = 400
    x[:, index['solver_id']] = np.arange(800) // 2 + 1
    x[:, index['solver_result']] = 1
    x[:, index['wbc_flags']] = 31
    x[:, index['wall_dt_ms']] = 5
    x[:, index['imu_4']] = np.sin(2 * np.pi * 40 * elapsed)
    for leg in range(4):
        x[:, index[f'contact_{leg}']] = 1
    x[200:, index['contact_0']] = 0
    x[200:, index['q_cmd_0']] = 0.2
    return names, x


def save_csv(path, names, rows):
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        writer.writerows(rows)


with tempfile.TemporaryDirectory(prefix='real_trial_check_validation_') as directory:
    root = Path(directory)
    logs, raw, output = (root / name for name in ('logs', 'raw', 'output'))
    for path in (logs, raw):
        path.mkdir()
    (logs / 'ros2_control_node_222_1000000.log').write_text(
        '[INFO] [1000.0] [sysu219_hardware]: connected\n'
        '[INFO] [1001.0] [controller]: Switched from fixed stand to trotting\n'
        '[INFO] [1005.0] [controller]: Switched from trotting to fixed stand\n'
        '[INFO] [1011.0] [controller]: Switched from fixed stand to trotting\n'
        '[INFO] [1015.0] [controller]: Switched from trotting to fixed stand\n')
    (logs / 'ros2_control_node_999_9000000.log').write_text(
        '[INFO] [9000.0] [gazebo]: Switched from fixed stand to trotting\n')
    for start, run in ((1001.0, 1), (1011.0, 2)):
        names, x = trace(start, run)
        if run == 2:
            x = np.delete(x, 350, axis=0)  # A genuine missing sample, not an overlapping export.
        prefix = raw / f'trotting_run_222_{int(start * 1e6)}_run{run}'
        save_csv(prefix.with_name(prefix.name + '_chunk0.csv'), names, x[:500])
        save_csv(prefix.with_name(prefix.name + '_chunk1.csv'), names, x[480:])
        for revision in (0, 1):
            record = dict(format='sysu219_mpc_wbc_snapshot_v1', control_pid=222,
                          run_id=run, applied_revision=revision, effective_cycle=400 * revision,
                          system_s=start + 0.002 + 2 * revision,
                          Q=np.eye(13).tolist(), R=np.eye(12).tolist(), S=np.eye(12).tolist(),
                          WBC={name: [1, 2, 3] for name in tool['GAINS']})
            record['WBC'].update(swing_hip_posture_weight=3.3,
                                 kp_swing_hip_posture=20.0, kd_swing_hip_posture=6.0)
            prefix.with_name(prefix.name + f'_rev{revision}.json').write_text(json.dumps(record))
        prefix.with_suffix('.json').write_text(json.dumps(dict(
            format='sysu219_trotting_run_v1', control_pid=222, run_id=run,
            last_system_s=start + 3.997, complete=True)))
    args = ['--log-root', str(logs), '--csv-root', str(raw), '--output', str(output)]
    with contextlib.redirect_stdout(io.StringIO()):
        assert tool['main'](args) == 0
    archive = next(output.iterdir())
    meta = json.loads((archive / 'manifest.json').read_text())
    result = json.loads((archive / 'metrics.json').read_text())
    params = json.loads((archive / 'parameters.json').read_text())
    assert meta['trial']['pid'] == 222 and meta['trial']['trial_number'] == 2
    assert result['quality']['rows'] == 799
    assert result['quality']['missing_cycles'] == 1
    assert result['quality']['duplicate_rows'] == 20
    assert result['solvers']['unique_records'] == 400  # Held solver status isn't another solve.
    assert params['status'] == 'runtime_matched' and len(params['revisions']) == 2
    assert len(result['parameter_segments']) == 2
    assert abs(result['high_frequency']['channels']['imu_4']['peak_hz'] - 40) < 1e-5
    assert abs(result['transition_q_jump_rad']['max_abs'] - 0.2) < 1e-12
    stamp = (archive / 'metrics.json').stat().st_mtime_ns
    with contextlib.redirect_stdout(io.StringIO()):
        assert tool['main'](args) == 0
    assert len(list(output.iterdir())) == 1
    assert (archive / 'metrics.json').stat().st_mtime_ns == stamp
    with contextlib.redirect_stdout(io.StringIO()):
        assert tool['main'](args + ['--pid', '222', '--trial', '1']) == 0
    first = [p for p in output.iterdir() if p != archive][0]
    first_result = json.loads((first / 'metrics.json').read_text())
    assert first_result['quality']['missing_cycles'] == 0
    # Incomplete last CSV row is ignored, not converted to a zero or bridged in time.
    names, selected, malformed, partial = tool['load_rows'](
        b'cycle,ros_s,system_s\n0,1,1\n1,1.005,1.005', 0, 2)
    assert partial and selected == [[0, 1, 1]]
    # Two short segments separated by a parameter change cannot make one FFT window.
    names, x = trace(2000.0, 3)
    data = tool['Data'](names, np.concatenate((x[:100], x[400:500])))
    checked = tool['analyse'](data)
    assert checked['high_frequency']['status'] == 'insufficient_contiguous_data'
    old = tool['Data'](['cycle', 'ros_s', 'system_s'], np.array([[0, 1, 1], [1, 1.005, 1.005]]))
    unverified, _ = tool['parameters']([], old, 222)
    assert unverified['status'] == 'missing_runtime_parameters'
    cutoff = datetime.datetime.fromtimestamp(1012.0).isoformat()
    with contextlib.redirect_stdout(io.StringIO()):
        assert tool['main'](args + ['--before', cutoff]) == 0
    clipped = [p for p in output.iterdir()
               if json.loads((p / 'manifest.json').read_text())['trial']['range_clipped']]
    assert len(clipped) == 1
    assert json.loads((clipped[0] / 'metrics.json').read_text())['quality']['rows'] == 200
print('PASS: hardware/trial selection, archive isolation, parameter matching/segments, gaps, overlap,')
print('      solver de-duplication, known 40Hz signal, transition jump, cache, cutoff and partial tails.')
