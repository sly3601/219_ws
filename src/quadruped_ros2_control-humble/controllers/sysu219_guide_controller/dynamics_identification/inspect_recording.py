#!/usr/bin/env python3
"""Read-only recording quality report. No ROS, filtering, differentiation, or builds."""

import argparse
import collections
import csv
import json
import math
from pathlib import Path
import statistics


def inspect(folder):
    metadata = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
    intervals = []
    ranges = {}
    segments = collections.Counter()
    flags = collections.Counter()
    previous_ns = previous_seq = None
    first_ns = last_ns = None
    count = gaps = nonmonotonic = repetitions = stopped_with_output = 0
    for joint in ("FL_hip", "FL_thigh", "FL_calf"):
        for field in ("q", "dq", "tau"):
            ranges[f"{joint}_{field}"] = {"min": None, "max": None, "nonfinite": 0}
    with (folder / "samples.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            now_ns, sequence = int(row["steady_ns"]), int(row["sequence"])
            if first_ns is None:
                first_ns = now_ns
            if previous_ns is not None:
                intervals.append((now_ns - previous_ns) * 1e-9)
                nonmonotonic += now_ns <= previous_ns
                gaps += max(0, sequence - previous_seq - 1)
            flag = int(row["flags"])
            for bit in (1, 2, 4, 8):
                flags[str(bit)] += bool(flag & bit)
            repetitions += int(row["repeated_feedback"])
            if int(row["phase"]) == 4:
                segments[f"action_{row['action']}_trial_{row['trial']}"] += 1
            if int(row["phase"]) == 0:
                stopped_with_output += any(
                    abs(float(row[f"{joint}_{field}"])) > 1e-12
                    for joint in ("FL_hip", "FL_thigh", "FL_calf")
                    for field in ("kp", "kd", "tau_ff", "dq_cmd")
                )
            for key, bounds in ranges.items():
                value = float(row[key])
                if not math.isfinite(value):
                    bounds["nonfinite"] += 1
                    continue
                bounds["min"] = value if bounds["min"] is None else min(bounds["min"], value)
                bounds["max"] = value if bounds["max"] is None else max(bounds["max"], value)
            count += 1
            previous_ns = last_ns = now_ns
            previous_seq = sequence
    return {
        "samples": count,
        "closed_cleanly": metadata.get("closed_cleanly", False),
        "writer_failed": metadata.get("writer_failed", True),
        "queue_dropped": metadata.get("queue_dropped"),
        "nonmonotonic_timestamps": nonmonotonic,
        "record_sequence_gaps": gaps,
        "span_s": (last_ns - first_ns) * 1e-9 if count else 0,
        "average_rate_hz": (count - 1) * 1e9 / (last_ns - first_ns)
        if count > 1 and last_ns > first_ns else 0,
        "dt_s": {"min": min(intervals), "median": statistics.median(intervals), "max": max(intervals)}
        if intervals else {},
        "flag_counts": dict(flags),
        "exact_feedback_repetitions": repetitions,
        "stopped_rows_with_nonzero_output": stopped_with_output,
        "experiment_segment_samples": dict(segments),
        "feedback_ranges": ranges,
        "timestamp_note": metadata.get("timestamp_kind"),
        "repeat_note": "Exact repetition alone does not prove stale motor feedback.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    print(json.dumps(inspect(args.folder), ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
