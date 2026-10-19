#!/usr/bin/env python3
"""Record temporal coverage and fail incomplete trajectories (not frame recall)."""
import json
from pathlib import Path
import sys
import yaml

info = yaml.safe_load(Path(sys.argv[1]).read_text())
rows = [line.split() for line in Path(sys.argv[2]).read_text().splitlines() if line and not line.startswith('#')]
times = [float(row[0]) for row in rows]
start, end = float(info['start']), float(info['end'])
assert len(times) >= 2, 'Too few poses'
assert all(b >= a for a, b in zip(times, times[1:])), 'Non-monotonic trajectory'
result = dict(poses=len(times), bag_start=start, bag_end=end, first_pose=times[0], last_pose=times[-1],
              start_gap_s=times[0]-start, end_gap_s=end-times[-1],
              temporal_coverage=(times[-1]-times[0])/(end-start),
              max_pose_gap_s=max(b-a for a, b in zip(times, times[1:])))
print(json.dumps(result, indent=2))
# Fixed tolerances allow estimator initialization and final sensor message timing.
assert -1 <= result['start_gap_s'] <= 30, 'Trajectory misses dataset beginning'
assert -1 <= result['end_gap_s'] <= 10, 'Trajectory misses dataset end'
assert result['max_pose_gap_s'] <= 10, 'Trajectory contains a gap over 10 seconds'
