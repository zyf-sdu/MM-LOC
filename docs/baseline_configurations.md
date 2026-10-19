# Core Configurations for Baseline Evaluation

This document summarizes the main evaluation configurations for FAST-LIO2, ROLL, BM-LOC, Fast-Lio-Loc, and SuperLoc on NCLT, Boreas, BG, and Real-world to support experiment tracking and reproduction of the paper's results.


## 1. Baseline Implementations and Versions

| Baseline | Source Repository | Commit SHA |
| --- | --- | --- |
| FAST-LIO2 | [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) | `7cc4175de6f8ba2edf34bab02a42195b141027e9` |
| ROLL | [HaisenbergPeng/ROLL](https://github.com/HaisenbergPeng/ROLL) | `97752404f5bbe28492a341a31b2b18ecbabed4a1` |
| BM-LOC | [YixFeng/Block-Map-Based-Localization](https://github.com/YixFeng/Block-Map-Based-Localization) | `bd99dba86c9bafb28fb5501a7f8a876db5da922d` |
| Fast-Lio-Loc | [HViktorTsoi/FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) | `2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2` |
| SuperLoc | [superxslam/SuperOdom](https://github.com/superxslam/SuperOdom) | `3f35d67d4ac3f343cce0e18274ed6a90fad315f4` |

SuperLoc uses the localization mode of SuperOdom.

## 2. FAST-LIO2 Frontend

This table describes only the FAST-LIO2 frontend settings. See Section 5 for the separate frontend configuration of Fast-Lio-Loc. Settings such as the point filtering interval, local map size, and range are not directly interchangeable between the two.

| Parameter (units shown in name) | NCLT | Boreas | BG | Real-world |
| --- | --- | --- | --- | --- |
| `scan_line` | 32 | 128 | 16 | 64 |
| `point_filter_num` | 4 | 4 | 4 | 4 |
| `max_iteration` | 3 | 3 | 3 | 3 |
| Scan/map voxel sizes (m) | 0.5/0.5 | 0.5/0.5 | 0.35/0.35 | 0.5/0.5 |
| `cube_side_length` (m) | 1000 | 1000 | 1000 | 1000 |
| `blind` (m) | 1 | 1 | 0.5 | 1 |
| `det_range` (m) | 100 | 120 | 100 | 120 |

## 3. ROLL

The main ROLL settings cover point cloud preprocessing, transitions into and out of temporary mapping, the sliding window, and local map matching.

| Parameter (units shown in name) | NCLT | Boreas | BG | Real-world |
| --- | --- | --- | --- | --- |
| LiDAR channels | 32 | 128 | 16 | 64 |
| Horizontal scan size | 1800 | 1800 | 1800 | 1024 |
| Scan downsampling rate | 2 | 1 | 1 | 1 |
| Minimum range (m) | 0.2 | 1.0 | 1.0 | 0.5 |
| Maximum range (m) | 150 | 120 | 100 | 150 |
| Global matching rate (Hz) | 5 | 5 | 5 | 5 |
| Sliding window size | 15 | 30 | 30 | 30 |
| Temporary mapping entry inlier ratio | 0.30 | 0.30 | 0.30 | 0.30 |
| Temporary mapping exit inlier ratio | 0.50 | 0.50 | 0.50 | 0.50 |
| Map corner voxel size (m) | 0.2 | 0.4 | 0.2 | 0.2 |
| Map surface voxel size (m) | 0.2 | 0.4 | 0.2 | 0.4 |
| Keyframe translation threshold (m) | 1.0 | 1.0 | 1.0 | 1.0 |
| Keyframe rotation threshold (rad) | 0.20 | 0.15 | 0.15 | 0.20 |
| Surrounding keyframe search radius (m) | 50 | 50 | 50 | 50 |

## 4. BM-LOC

IMU frequency refers to the configured frequency, rather than a measured data rate. Noise terms retain the parameter descriptions from the original table without interpreting them as standard deviations or variances.

| Parameter (units shown in name) | NCLT | Boreas | BG | Real-world |
| --- | --- | --- | --- | --- |
| Minimum range (m) | 2.0 | 1.0 | 0.5 | 2.0 |
| Maximum range (m) | 90 | 120 | 100 | 120 |
| IMU frequency (Hz) | 100 | 200 | 400 | 100 |
| IMU acceleration noise parameter | 0.01 | 0.01 | 0.005 | 0.01 |
| IMU gyroscope noise parameter | 0.01 | 0.001 | 0.002 | 0.01 |
| Initialization cool-down duration (s) | 2.0 | 0.5 | 0.5 | 2.0 |
| Keyframe interval (s) | 0.1 | 0.1 | 0.1 | 0.1 |
| Map query interval (s) | 1.0 | 1.0 | 1.0 | 1.0 |
| NDT resolution (m) | 2.0 | 1.0 | 1.0 | 2.0 |
| NDT neighborhood search | `DIRECT7` | `DIRECT7` | `DIRECT7` | `DIRECT7` |
| NDT convergence parameter (ε) | 0.01 | 0.01 | 0.01 | 0.01 |
| Map downsampling resolution (m) | 0.2 | 0.2 | 0.2 | 0.2 |
| Penalty threshold | 0.6 | 0.6 | 0.6 | 0.6 |
| Penalty weight | 1.0 | 1.0 | 1.0 | 1.0 |
| Odometry translation noise parameter | 0.08 | 0.08 | 0.08 | 0.08 |
| Odometry rotation noise parameter | 0.05 | 0.05 | 0.05 | 0.05 |
| Active factors | 20 | 20 | 20 | 20 |
| Preserved factors | 5 | 5 | 5 | 5 |
| Specified initial pose | Enabled | Enabled | Enabled | Enabled |

## 5. Fast-Lio-Loc

Frontend and global matching parameters are listed separately. The fitness acceptance threshold retains the original experimental values without inferring whether acceptance requires a value above or below the threshold.

| Parameter (units shown in name) | NCLT | Boreas | BG | Real-world |
| --- | --- | --- | --- | --- |
| LiDAR preprocessing type | Velodyne | Velodyne | Velodyne | Ouster |
| LiDAR channels | 32 | 128 | 16 | 64 |
| Point filtering interval | 4 | 4 | 2 | 4 |
| Maximum frontend iterations | 3 | 3 | 3 | 3 |
| Frontend scan voxel size (m) | 0.5 | 0.5 | 0.35 | 0.5 |
| Frontend map voxel size (m) | 0.5 | 0.5 | 0.35 | 0.5 |
| Local map cube side length (m) | 1000 | 1000 | 400 | 1000 |
| Minimum range (m) | 4.0 | 0.2 | 0.5 | 4.0 |
| Detection range (m) | 100 | 150 | 100 | 150 |
| Global matching frequency (Hz) | 0.5 | 0.5 | 0.5 | 0.5 |
| Global matching map voxel size (m) | 0.4 | 0.4 | 0.4 | 0.4 |
| Global matching scan voxel size (m) | 0.1 | 0.1 | 0.1 | 0.1 |
| Global matching field of view (rad) | 2π | 2π | 2π | 2π |
| Global matching far range (m) | 150 | 150 | 150 | 150 |
| Fitness acceptance threshold | 0.95 | 0.80 | 0.80 | 0.95 |

## 6. SuperLoc

SuperOdom is used in localization mode, with the IMU roll/pitch option and pose file reading disabled.

| Parameter (units shown in name) | NCLT | Boreas | BG | Real-world |
| --- | --- | --- | --- | --- |
| LiDAR preprocessing type | Velodyne | Velodyne | Velodyne | Ouster |
| LiDAR channels | 32 | 128 | 16 | 64 |
| Minimum range (m) | 0.2 | 1.0 | 0.2 | 0.2 |
| Point filtering parameter (`filter_point_size`) | 3 | 3 | 3 | 3 |
| Line resolution (m) | 0.1 | 0.2 | 0.1 | 0.1 |
| Plane resolution (m) | 0.2 | 0.4 | 0.2 | 0.2 |
| Maximum iterations | 5 | 5 | 5 | 5 |
| Maximum surface features | 2000 | 2000 | 2000 | 2000 |
| LiDAR correction noise parameter | 0.01 | 0.01 | 0.01 | 0.01 |
| Localization mode | Enabled | Enabled | Enabled | Enabled |
| IMU roll/pitch option | Disabled | Disabled | Disabled | Disabled |
| Read pose file | Disabled | Disabled | Disabled | Disabled |

### SuperLoc Sensor Adaptations

| Dataset | Adaptation Settings |
| --- | --- |
| NCLT | Convert per-point timestamps from microseconds to seconds: `point_time_scale=1e-6`. |
| BG | Per-point timestamps are in seconds: `point_time_scale=1`. |
| Real-world | Use an identity LiDAR-to-IMU extrinsic transform and disable the built-in Ouster sensor transform: `ouster_apply_builtin_sensor_transform=false`. |

