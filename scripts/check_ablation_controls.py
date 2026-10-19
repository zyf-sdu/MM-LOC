#!/usr/bin/env python3
"""Compare effective ROS settings before playback, allowing only intended changes."""
import math
from pathlib import Path
import sys
import yaml

source, baseline, a, b = sys.argv[1:]
params = yaml.safe_load(Path(source).read_text())
roll = params['roll']
assert roll['globalConstraintWeightMode'] == 'adaptive'
assert math.isclose(roll['globalConstraintInlierWeight'], float(a))
assert math.isclose(roll['globalConstraintStabilityWeight'], float(b))
assert math.isclose(float(a) + float(b), 0.7)
for key in ['globalConstraintInlierWeight', 'globalConstraintStabilityWeight',
            'saveMapDirectory', 'saveKeyframeMapDirectory']:
    del roll[key]
del params['dynamic']['save_traj_dir']
for key in ['run_id', 'roslaunch', 'rosversion', 'rosdistro']:
    params.pop(key, None)
# Anonymous recorder nodes are started only after this comparison.
path = Path(baseline)
if path.exists():
    expected = yaml.safe_load(path.read_text())
    if params != expected:
        import difflib
        print(''.join(difflib.unified_diff(yaml.safe_dump(expected).splitlines(True), yaml.safe_dump(params).splitlines(True))))
        sys.exit('Control parameters differ; playback blocked')
else:
    path.write_text(yaml.safe_dump(params, sort_keys=True))
print('PASS: all non-treatment parameters match baseline')
