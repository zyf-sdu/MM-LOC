# MM-LOC

**MM-LOC: Robust Long-Term Localization Based on Multi-Temporal Map Fusion**

MM-LOC addresses environmental changes and insufficient prior map coverage during long-term operation. It fuses long-term prior maps, mid-term reference maps, and a real-time local map to provide multi-temporal environmental constraints for LiDAR–IMU localization. Built on ROLL and FAST-LIO, this project implements multi-map matching, adaptive constraint weighting, sliding window optimization, and on-demand map loading.

The public repository accompanying the paper is [zyf-sdu/MM-LOC](https://github.com/zyf-sdu/MM-LOC). This document describes how to configure and run the current code version. The code directory can be named `MM-LOC`.
The current release is a beta version. The final version will be updated after the paper is accepted.

## 1. Method and Code Structure

The system uses frontend LiDAR–IMU odometry to provide continuous motion estimates, matches the current point cloud against maps from different time periods, and passes the resulting localization constraints to the backend optimizer. The map service uses block maps and cube indices to load local point clouds based on the current position.

| Module | Function | Main Code |
| --- | --- | --- |
| LiDAR–IMU frontend | Provides odometry and motion-compensated point clouds | External `fast_lio` package |
| Feature extraction | Extracts edge and planar features | [src/scanRegistration.cpp](src/scanRegistration.cpp) |
| Multi-temporal map localization | Matches against long-term and mid-term maps, incorporates the real-time local map into matching, and assesses matching quality | [src/mapOptmization.cpp](src/mapOptmization.cpp), [include/LOAMmapping.h](include/LOAMmapping.h) |
| Backend optimization | Fuses odometry and map localization constraints and maintains global pose corrections | [src/globalOpt.cpp](src/globalOpt.cpp) |
| Map management | Handles block map indexing, cube loading, and local map publishing and queries | [src/blockMap.cpp](src/blockMap.cpp), [src/globalmap_server_nodelet.cpp](src/globalmap_server_nodelet.cpp) |
| Ground-truth-assisted mapping | Saves prior maps and keyframes using ground-truth poses | [src/mapOptimizationGT.cpp](src/mapOptimizationGT.cpp) |

The current NCLT localization configuration enables matching against both long-term and mid-term maps, the real-time local map, adaptive weighting, and dynamic map loading. Using the real-time local map for localization does not modify the prior maps stored on disk; `roll/mapUpdateEnabled` is currently `false`.

## 2. Dependencies and Build

This project uses ROS 1, catkin, and C++14. According to the current [CMakeLists.txt](CMakeLists.txt), building requires:

- ROS: `roscpp`, `rospy`, `tf`, `tf2`, `tf2_ros`, `roslib`, `cv_bridge`, `pcl_conversions`, and message generation components.
- PCL (minimum version 1.7 declared in CMake), Eigen, OpenCV, GTSAM, Ceres, Boost filesystem, yaml-cpp, and OpenMP.
- ROS `map_server` development dependencies; the map service code uses its image loading interface.
- The `fast_lio` package in the same workspace or a sourced workspace.

The current CMake configuration uses `find_package(Ceres REQUIRED)`, so Ceres is a required dependency for this version. Record the specific dependency versions used in your experimental environment.

Place this package and a version of FAST-LIO adapted to the dataset in a catkin workspace, then run the following from the workspace root:

```bash
cd /path/to/catkin_ws
catkin_make -j2
source devel/setup.bash
rospack find roll
rospack find fast_lio
```

The current launch configuration depends on `fast_lio/launch/mapping_nclt.launch`, and the frontend must publish `cloud_registered_body` and `/Odometry`. When using another FAST-LIO version, first verify the launch files, topics, point cloud fields, extrinsics, and timestamp handling. The FAST-LIO2 commit listed in the baseline document identifies only the corresponding baseline experiment.

## 3. Data and Prior Map Preparation

### 3.1 Data Inputs

The current repository provides launch and configuration entry points for NCLT. Raw NCLT data can be converted using `scripts/nclt_data2bag_BIN.py`; update the data root directory and sequence settings in the script before use.

| Input | Current Interface | Notes |
| --- | --- | --- |
| Raw LiDAR and IMU data | Determined by the external FAST-LIO configuration | Verify dataset topics, timestamps, and sensor extrinsics |
| Frontend point cloud | `cloud_registered_body` | Current `roll/pointCloudTopic` |
| Frontend odometry | `/Odometry` | Frontend odometry subscribed to by the localization node |
| Ground truth | `/ground_truth`, `nav_msgs/Odometry` | Used for ground-truth-assisted mapping; may also be used for localization initialization |
| External initial pose | `/initialpose` | RViz initial pose interface |

Localization initialization first attempts to use a pose from the ground-truth queue. If none is available after several attempts, it uses `roll/initialGuess`. The array order is `[roll, pitch, yaw, x, y, z]`, with angles in rad and positions in m. It must be consistent with the map coordinates and the sensor transform conventions in the code. It also determines where the map server initially loads the local map, so a reasonable initial guess is required even when ground truth is provided.

### 3.2 Map Files

Store long-term and mid-term maps separately. The paths passed to the launch file must point to their respective `block_map/` directories and retain the trailing `/`. In dynamic loading mode, each map must contain both edge and planar feature cubes and their indices:

```text
prior_map/
├── keyframes/
│   ├── poses.txt
│   ├── corner0.pcd
│   └── surf0.pcd
└── block_map/
    ├── corner/
    │   ├── 000.pcd
    │   └── cube/
    │       ├── index.csv
    │       └── ...
    └── surf/
        ├── 000.pcd
        └── cube/
            ├── index.csv
            └── ...
```

Keyframes and large PCD blocks are used for map generation or full map loading, while localization with dynamic loading uses the cube indices and corresponding point clouds. The actual number of files depends on the map.

Existing maps can be used directly for localization. To generate your own maps, the code provides the following stages:

1. Update the save paths in [config/params_nclt_mappingGT.yaml](config/params_nclt_mappingGT.yaml), enable `roll/saveKeyframeMap`, create the output directories, then run `roslaunch roll GTmapping_nclt.launch` and play back the mapping sequence. Keyframe saving is currently disabled by default in this YAML and must be explicitly enabled.
2. Set `map_server/generate_block_map=true` in the localization configuration to organize keyframes into large map blocks. Verify `map_server/global_map_dir` and the keyframe paths. In localization mode, the code resets the keyframe input directory to `global_map_dir + "../keyframes/"`.
3. Set `dynamic/build_dynamic_cube_map=true` to generate cubes. The output directory is derived from `map_server/global_map_dir`; changing `dynamic/output_directory` does not change the actual output path in the current implementation.
4. After generation, reset both generation switches to `false` before running localization.

These switches are part of map preprocessing. Configure the required nodes and directories separately for map generation to avoid directly reusing the dual-map localization launch file with different inputs.

## 4. Running MM-LOC

### 4.1 Check the Localization Configuration

Create an experiment configuration based on [config/params_nclt_loc.yaml](config/params_nclt_loc.yaml). At a minimum, verify the initial pose, output directories, map paths, and sensor parameters. The file contains machine-specific absolute paths that must be replaced when moving to another environment.

The loading paths for the long-term and mid-term map servers are specified by the launch arguments `long_map_dir` and `mid_map_dir`, respectively. The YAML parameter `map_server/global_map_dir` is still used by other map processing logic and must also be checked.

### 4.2 Start Localization

Run the following in a terminal with the workspace environment sourced, replacing the example paths with actual paths:

```bash
roslaunch roll loc_nclt.launch \
  params_file:=/path/to/experiment.yaml \
  long_map_dir:=/path/to/long_prior/block_map/ \
  mid_map_dir:=/path/to/mid_reference/block_map/ \
  visualization:=false
```

In dynamic map mode, the localization node waits for the required query services to become available, with a maximum wait of 120 s per service. Once the maps have loaded successfully and the required services are available, start playback in another terminal with the workspace environment sourced:

```bash
rosbag play /path/to/query_sequence.bag --clock -r 1.0
```

The localization YAML already enables `use_sim_time: true`. For startup issues, see the [map query startup guide](config/map_query_startup_fix.md).


## 5. Core MM-LOC Configuration

The following values come from the current NCLT localization YAML and describe the configuration of this version. Use dataset-specific configurations for other datasets rather than directly reusing the NCLT settings.

| Parameter (full namespace) | Current Value | Description |
| --- | --- | --- |
| `roll/localizationMode` | `true` | Localization mode |
| `roll/mapUpdateEnabled` | `false` | Prior map update switch |
| `dynamic/use_mid_map_to_loc` | `3` | Matches against both long-term and mid-term maps |
| `dynamic/useRealtimeLocalMap` | `true` | Incorporates the real-time local map into matching |
| `dynamic/use_dynamic_map_loading` | `true` | Loads maps dynamically based on position |
| `dynamic/resolution` | `30.0` m | Cube partition size |
| `dynamic/load_radius` | `100` m | Map loading radius |
| `roll/slidingWindowSize` | `100` | Local keyframe window setting |
| `roll/globalMatchMinInlierRatio2` | `0.1` | Global matching quality threshold |
| `roll/globalConstraintWeightMode` | `adaptive` | Weighting mode for long-term/mid-term map constraints |
| `roll/mappingSurfLeafSize` | `0.4` m | Planar feature voxel size |


When `dynamic/use_dynamic_map_loading=false`, the map server loads the full prior map, which can be used for comparison experiments. The corresponding large PCD blocks must also be available.

## 6. Experiments and Ablation Studies

Long-term/mid-term map constraints support both adaptive and fixed standard deviation modes:

```bash
roslaunch roll loc_nclt.launch global_constraint_weight_mode:=adaptive
roslaunch roll loc_nclt.launch global_constraint_weight_mode:=fixed \
  global_constraint_fixed_pos_sigma:=0.45 \
  global_constraint_fixed_rot_sigma:=0.11
```

These commands use the default configuration and map paths. For custom experiments, also provide the configuration and map arguments from Section 4. Fixed mode replaces only the standard deviations of the long-term/mid-term map constraints; see the [fixed covariance ablation guide](config/fixed_weight_ablation.md) for details.

The local script `scripts/run_nclt_original_map_suite.sh` provides an entry point for batch NCLT experiments:

```bash
WEIGHT_MODE=adaptive bash scripts/run_nclt_original_map_suite.sh
WEIGHT_MODE=fixed bash scripts/run_nclt_original_map_suite.sh
```

The script contains machine-specific workspace, dataset, and map paths that must be checked before use. It also requires the Python packages `numpy`, `PyYAML`, and `evo`. For each sequence, the current script generates `run_params.yaml` and saves the effective parameters in `effective_params.yaml`, the weighting strategy, TUM trajectories, coverage checks, and evaluation results. Its APE command evaluates translation error with `--t_max_diff 0.02` and does not include trajectory alignment options.

Record the actual code commit, configuration snapshot, map time periods, initial pose, playback rate, and evaluation method for each experiment. Keep all other conditions consistent across ablation groups.

## 7. Comparison Baselines

The comparison methods include FAST-LIO2, ROLL, BM-LOC, Fast-Lio-Loc, and SuperLoc (SuperOdom in localization mode). The repositories, full commit hashes, and main parameters for NCLT, Boreas, BG, and Real-world are collected in:

**[Core Configurations for Baseline Evaluation](docs/baseline_configurations.md)**

That document records the baseline settings based on the experimental tables and lists the differences between the ROLL baseline table and the NCLT configuration in the current MM-LOC workspace. See Section 5 for the current MM-LOC configuration; the ROLL baseline parameters must not be used as a substitute.

The standard launch files in the current repository primarily target NCLT. Coverage of other datasets in the baseline parameter tables does not imply that this directory contains all the corresponding adaptation code. Instructions for the Boreas transfer package are available in [transfer/boreas_0907/README.md](transfer/boreas_0907/README.md); the archives and external workspaces described there must be prepared separately.

## 8. Acknowledgments and License

This project builds on open-source work including [ROLL](https://github.com/HaisenbergPeng/ROLL) and [FAST-LIO](https://github.com/hku-mars/FAST_LIO), and draws on the map management work in [Block-Map-Based-Localization](https://github.com/YixFeng/Block-Map-Based-Localization). We thank the authors for their contributions.

The repository retains [LICENSE](LICENSE) and the license notices in individual source files.
