#!/usr/bin/env bash
set -u

workspace=/home/myx/project/zyf_learn/BMLL_LOC_xiaorong
package_dir="$workspace/src/BMLL_LOC"
dataset_root=/home/myx/Fighting/0DATASETS/NCLT
run_id="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
weight_mode="${WEIGHT_MODE:-adaptive}"
inlier_weight="${INLIER_WEIGHT:-0.6}"
stability_weight="${STABILITY_WEIGHT:-0.1}"
fixed_pos_sigma="${FIXED_POS_SIGMA:-0.45}"
fixed_rot_sigma="${FIXED_ROT_SIGMA:-0.11}"
case "$weight_mode" in
  adaptive|fixed) ;;
  *) echo "WEIGHT_MODE must be adaptive or fixed" >&2; exit 2 ;;
esac
python3 - "$fixed_pos_sigma" "$fixed_rot_sigma" <<'CHECK_SIGMAS'
import math
import sys
try:
    assert all(math.isfinite(float(x)) and float(x) > 0 for x in sys.argv[1:])
except (ValueError, AssertionError):
    sys.exit("Fixed sigmas must be finite and positive")
CHECK_SIGMAS
[ "$?" -eq 0 ] || exit 2
python3 - "$inlier_weight" "$stability_weight" <<'CHECK_WEIGHTS'
import math, sys
try:
    assert all(math.isfinite(float(x)) and float(x) >= 0 for x in sys.argv[1:])
except (ValueError, AssertionError):
    sys.exit("Adaptive coefficients must be finite and nonnegative")
CHECK_WEIGHTS
[ "$?" -eq 0 ] || exit 2
strategy="${weight_mode}_inlier${inlier_weight}_history0.3_stability${stability_weight}_pos${fixed_pos_sigma}_rot${fixed_rot_sigma}"
suite_dir="${RESULT_DIR:-$package_dir/test_reports/nclt_original_map_${run_id}_${strategy}}"
rate="${RATE:-1.0}"
source "$workspace/devel/setup.bash"
mkdir -p "$suite_dir"
manifest="$suite_dir/weight_strategy.txt"
settings=$(printf 'mode=%s\npos_sigma=%s\nrot_sigma=%s\ninlier_weight=%s\nhistory_weight=0.3\nstability_weight=%s\nrate=%s\ndatasets=%s\n' "$weight_mode" "$fixed_pos_sigma" "$fixed_rot_sigma" "$inlier_weight" "$stability_weight" "$rate" "${DATASETS:-all}")
if [ -f "$manifest" ]; then
  [ "$(cat "$manifest")" = "$settings" ] || { echo "Result directory uses a different weight strategy" >&2; exit 2; }
elif [ -f "$suite_dir/summary.tsv" ]; then
  echo "Existing results have no weight strategy manifest; choose a new RESULT_DIR" >&2
  exit 2
fi
printf '%s\n' "$settings" > "$manifest"
export ROS_HOME="$suite_dir/ros_home"
mkdir -p "$ROS_HOME"
# Use a dedicated master so stale nodes or clocks from other experiments cannot
# satisfy health checks or interfere with this suite.
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT:-11331}"

dates=(2012-03-17 2012-04-29 2012-06-15 2012-08-04 2012-10-28 2012-11-17 2013-01-10)
bags=("12-03-17/2012-03-17_bin.bag" "12-04-29/2012-04-29/2012-04-29_bin.bag" "12-06-15/2012-06-15/2012-06-15_bin.bag" "12-08-04/2012-08-04/2012-08-04_bin.bag" "12-10-28/2012-10-28/2012-10-28_bin.bag" "12-11-17/2012-11-17/2012-11-17_bin.bag" "13-01-10/2013-01-10/2013-01-10_bin.bag")
gts=("12-03-17/2012-03-17/groundtruth_2012-03-17.csv" "12-04-29/2012-04-29/groundtruth_2012-04-29.csv" "12-06-15/2012-06-15/groundtruth_2012-06-15.csv" "12-08-04/2012-08-04/groundtruth_2012-08-04.csv" "12-10-28/2012-10-28/groundtruth_2012-10-28.csv" "12-11-17/2012-11-17/groundtruth_2012-11-17.csv" "13-01-10/2013-01-10/groundtruth_2013-01-10.csv")
long_maps=(20120115 20120202 12-03-17 12-03-17 12-03-17 12-03-17 12-03-17)
mid_maps=(20120202 12-03-17 12-05-11 12-06-15 12-08-04 12-06-15 12-11-17)
guesses=(
"[0.020559761872303, -0.000903047757892, -1.070181276478803, 75.579787639477303, 108.841026421505774, -3.316757230658669]"
"[-0.015404291345797, -0.000618447067820, -0.312421191243955, 76.181700539534063, 108.837027977442645, -3.266038226333669]"
"[-0.054502938206381, -0.025535463484375, -0.192617339278269, 88.661994612088748, 65.113330292655192, -2.807224100861569]"
"[-0.015221811894219, 0.001396211399287, -0.303854522173840, 75.840364680427626, 109.017645300640666, -3.295762322731578]"
"[-0.022181323027525, -0.020090272141793, -0.187628657826380, 76.309999861716037, 108.701643881647712, -3.253837087890719]"
"[-0.022510053101572, -0.009280409924894, -0.248550828547029, 77.430750494550978, 109.263558200969300, -3.198719816568997]"
"[-0.029788434570233, -0.010193528558576, -0.147575680110183, 75.829339527800855, 107.724666286782551, -3.272894625813646]"
)

cleanup() {
  for pid in "${bag_pid:-}" "${rec_pid:-}" "${launch_pid:-}"; do
    [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

summary="$suite_dir/summary.tsv"
[ -f "$summary" ] || printf 'dataset\tlong_map\tmid_map\tposes\trmse_m\tmax_m\tstatus\n' > "$summary"
for i in "${!dates[@]}"; do
  date="${dates[$i]}"
  if [ -n "${DATASETS:-}" ] && [[ " $DATASETS " != *" $date "* ]]; then
    continue
  fi
  out="$suite_dir/$date"; mkdir -p "$out/ros_logs"
  [ -f "$out/complete" ] && { echo "skip_complete=$date"; continue; }
  : > "$out/error.txt"
  bag="$dataset_root/${bags[$i]}"; gt="$dataset_root/${gts[$i]}"
  long="$dataset_root/${long_maps[$i]}/map/block_map/"; mid="$dataset_root/${mid_maps[$i]}/map/block_map/"
  for required in "$bag" "$gt" "$long/corner/000.pcd" "$long/surf/000.pcd" "$mid/corner/000.pcd" "$mid/surf/000.pcd"; do
    [ -f "$required" ] || { echo "missing=$required" | tee "$out/error.txt"; exit 3; }
  done
  python3 - "$package_dir/config/params_nclt_loc.yaml" "$out/run_params.yaml" "${guesses[$i]}" <<'RUN_PARAMS'
import pathlib, sys, yaml
source, dest, guess = sys.argv[1:]
params = yaml.safe_load(pathlib.Path(source).read_text())
roll = params['roll']
roll['initialGuess'] = yaml.safe_load(guess)
# Keep generated logs and keyframes inside this experiment.
root = pathlib.Path(dest).resolve().parent
for key, directory in [('saveMapDirectory', 'map_output'),
                       ('saveKeyframeMapDirectory', 'keyframes')]:
    path = root / directory
    path.mkdir(parents=True, exist_ok=True)
    roll[key] = str(path) + '/'
params['dynamic']['save_traj_dir'] = str(root / 'trajectories') + '/'
(root / 'trajectories').mkdir(exist_ok=True)
pathlib.Path(dest).write_text(yaml.safe_dump(params, sort_keys=False))
RUN_PARAMS
  [ "$?" -eq 0 ] || exit 2
  /usr/bin/python3 - "$bag" "$out/bag_info.yaml" <<'BAG_METADATA'
import rosbag, sys, yaml
with rosbag.Bag(sys.argv[1]) as bag:
    with open(sys.argv[2], 'w') as stream:
        yaml.safe_dump(dict(start=bag.get_start_time(), end=bag.get_end_time()), stream)
BAG_METADATA
  [ "$?" -eq 0 ] || exit 29
  export ROS_LOG_DIR="$out/ros_logs"
  roslaunch roll loc_nclt.launch params_file:="$out/run_params.yaml" visualization:=false long_map_dir:="$long" mid_map_dir:="$mid" \
    global_constraint_weight_mode:="$weight_mode" \
    global_constraint_inlier_weight:="$inlier_weight" \
    global_constraint_stability_weight:="$stability_weight" \
    global_constraint_fixed_pos_sigma:="$fixed_pos_sigma" \
    global_constraint_fixed_rot_sigma:="$fixed_rot_sigma" >"$out/roslaunch.log" 2>&1 & launch_pid=$!
  # Pre-playback gate. The map topics are advertised only after loadAllMap()
  # finishes, so this also proves both complete prior maps are resident.
  ready=0
  for _ in $(seq 1 180); do
    kill -0 "$launch_pid" 2>/dev/null || break
    nodes=$(rosnode list 2>/dev/null || true)
    topics=$(rostopic list 2>/dev/null || true)
    if grep -q '^/laserMapping$' <<<"$nodes" &&
       grep -q '^/roll_mapOptmization$' <<<"$nodes" &&
       grep -q '^/roll_mapServer1$' <<<"$nodes" &&
       grep -q '^/roll_mapServer2$' <<<"$nodes" &&
       grep -q '^/dynamic_corner_map_long$' <<<"$topics" &&
       grep -q '^/dynamic_surf_map_long$' <<<"$topics" &&
       grep -q '^/dynamic_corner_map_mid$' <<<"$topics" &&
       grep -q '^/dynamic_surf_map_mid$' <<<"$topics" &&
       rosnode ping -c 1 /laserMapping >/dev/null 2>&1; then
      ready=1
      break
    fi
    sleep 1
  done
  [ "$ready" -eq 1 ] || { echo "launch_not_ready=$date" | tee "$out/error.txt"; exit 20; }
  rosparam dump "$out/effective_params.yaml" || exit 26
  if [ -n "${CONTROL_BASELINE:-}" ]; then
    python3 "$package_dir/scripts/check_ablation_controls.py" "$out/effective_params.yaml" "$CONTROL_BASELINE" "$inlier_weight" "$stability_weight" > "$out/control_check.txt" 2>&1 || exit 28
  fi
  python3 "$package_dir/scripts/record_odom_tum.py" --output "$out/optimized_trajectory.tum" >"$out/recorder.log" 2>&1 & rec_pid=$!
  sleep 2
  echo "start=$date rate=$rate long=${long_maps[$i]} mid=${mid_maps[$i]}"
  rosbag play -q --clock -r "$rate" "$bag" >"$out/rosbag.log" 2>&1 & bag_pid=$!

  # Active-flow gate: never consume a dataset unless the bag clock, FAST-LIO
  # odometry and final optimized trajectory have all started moving.
  # Large bags on the dataset disk may spend several minutes reading their
  # connection index before publishing the first clock. This is startup I/O,
  # not dataset playback; allow it while the player remains alive.
  timeout 600 rostopic echo -n 1 /clock >"$out/health_clock.txt" 2>&1 & clock_probe_pid=$!
  clock_ready=0
  for _ in $(seq 1 120); do
    if ! kill -0 "$bag_pid" 2>/dev/null; then
      break
    fi
    if ! kill -0 "$clock_probe_pid" 2>/dev/null; then
      wait "$clock_probe_pid" 2>/dev/null && clock_ready=1
      break
    fi
    sleep 5
  done
  if [ "$clock_ready" -ne 1 ]; then
    kill -INT "$clock_probe_pid" 2>/dev/null || true
    wait "$clock_probe_pid" 2>/dev/null || true
    echo "health_failed=no_clock date=$date" | tee "$out/error.txt"
    exit 22
  fi
  if ! timeout 120 rostopic echo -n 1 /Odometry >"$out/health_fastlio_odom.txt" 2>&1; then
    echo "health_failed=no_fastlio_odometry date=$date" | tee "$out/error.txt"
    exit 23
  fi
  trajectory_ready=0
  for _ in $(seq 1 120); do
    kill -0 "$bag_pid" 2>/dev/null || break
    [ -s "$out/optimized_trajectory.tum" ] && { trajectory_ready=1; break; }
    sleep 1
  done
  if [ "$trajectory_ready" -ne 1 ]; then
    echo "health_failed=no_optimized_trajectory date=$date" | tee "$out/error.txt"
    exit 24
  fi
  echo "health_ok=$date first_optimized_pose_recorded"

  # Continuous watchdog: both estimator processes must stay responsive and the
  # trajectory must keep growing. Two consecutive stagnant minutes abort.
  last_size=$(stat -c %s "$out/optimized_trajectory.tum")
  stagnant_checks=0
  watchdog_failure=""
  while kill -0 "$bag_pid" 2>/dev/null; do
    sleep 60
    kill -0 "$bag_pid" 2>/dev/null || break
    if ! kill -0 "$launch_pid" 2>/dev/null ||
       ! rosnode ping -c 1 /laserMapping >/dev/null 2>&1 ||
       ! rosnode ping -c 1 /roll_mapOptmization >/dev/null 2>&1; then
      watchdog_failure="node_died_or_unresponsive"
      break
    fi
    current_size=$(stat -c %s "$out/optimized_trajectory.tum")
    if [ "$current_size" -le "$last_size" ]; then
      stagnant_checks=$((stagnant_checks + 1))
    else
      stagnant_checks=0
    fi
    last_size="$current_size"
    if [ "$stagnant_checks" -ge 2 ]; then
      watchdog_failure="trajectory_stalled_120s"
      break
    fi
  done
  if [ -n "$watchdog_failure" ]; then
    echo "health_failed=$watchdog_failure date=$date" | tee "$out/error.txt"
    kill -INT "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
    exit 25
  fi
  wait "$bag_pid"; bag_status=$?; bag_pid=""
  # Allow queued estimator messages to drain, with a finite upper bound.
  last_size=0; stable=0
  for _ in $(seq 1 24); do
    sleep 5
    current_size=$(stat -c %s "$out/optimized_trajectory.tum")
    if [ "$current_size" -eq "$last_size" ]; then stable=$((stable + 1)); else stable=0; fi
    last_size="$current_size"
    [ "$stable" -ge 3 ] && break
  done
  kill -INT "$rec_pid" 2>/dev/null || true; wait "$rec_pid" 2>/dev/null || true; rec_pid=""
  kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true; launch_pid=""
  [ "$bag_status" -eq 0 ] || { echo "bag_status=$bag_status" | tee "$out/error.txt"; exit "$bag_status"; }
  python3 "$package_dir/scripts/check_trajectory_coverage.py" "$out/bag_info.yaml" "$out/optimized_trajectory.tum" > "$out/coverage.json" 2> "$out/coverage_error.txt" || exit 30
  python3 "$package_dir/scripts/nclt_groundtruth_to_tum.py" "$gt" "$out/groundtruth_imu.tum" >"$out/groundtruth_convert.log"
  evo_ape tum "$out/groundtruth_imu.tum" "$out/optimized_trajectory.tum" -r trans_part --t_max_diff 0.02 --save_results "$out/ape_results.zip" >"$out/evo_ape_stdout.txt" 2>&1 || exit 27
  rmse=$(awk '/rmse/{print $2}' "$out/evo_ape_stdout.txt"); max=$(awk '/max/{print $2}' "$out/evo_ape_stdout.txt"); poses=$(wc -l < "$out/optimized_trajectory.tum")
  printf '%s\t%s\t%s\t%s\t%s\t%s\tcomplete\n' "$date" "${long_maps[$i]}" "${mid_maps[$i]}" "$poses" "$rmse" "$max" >> "$summary"
  touch "$out/complete"
  echo "done=$date poses=$poses rmse=$rmse max=$max"
  if [ -n "${STOP_AFTER:-}" ] && [ "$date" = "$STOP_AFTER" ]; then
    echo "stop_after_reached=$date"
    break
  fi
done

{
  echo '# NCLT 原始地图自动化测试报告'; echo; echo "- 运行目录：\`$suite_dir\`"; echo "- 播放速度：${rate}x"; echo "- 权重策略：$weight_mode；当前项：$inlier_weight；历史项：0.3；稳定项：$stability_weight；固定标准差：${fixed_pos_sigma} m / ${fixed_rot_sigma} rad"; echo '- 定位话题：`/roll/mapping/odometry`'; echo '- 地图加载方式及其余有效参数：见各序列 effective_params.yaml'; echo; echo '| Bag | Long map | Mid map | Poses | RMSE (m) | Max (m) | 状态 |'; echo '|---|---|---|---:|---:|---:|---|';
  tail -n +2 "$summary" | while IFS=$'\t' read -r a b c d e f g; do echo "| $a | $b | $c | $d | $e | $f | $g |"; done
} > "$suite_dir/report.md"
echo "suite_complete=$suite_dir"
