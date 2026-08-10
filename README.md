# MM-LOC

The code for paper of **MM-LOC: Robust Long-Term Localization Based on Multi-Temporal Map Fusion**

## Abstract

High-precision localization with prior maps is essential for autonomous mobile robots to reliably perform long-term tasks. However, real-world environments inevitably undergo local structural changes, and robots may even move beyond mapped areas. As a result, a single prior map can hardly provide reliable localization constraints during long-term operation.

To address this issue, we propose **MM-LOC**, a robust 3D localization method based on multi-temporal map fusion. MM-LOC exploits a long-term prior map, a mid-term reference map, and a real-time local map to provide sufficient environmental constraints across different temporal scales.

During map matching, the system adaptively adjusts the confidence of different constraints according to point-cloud matching quality. These multi-temporal map constraints are then incorporated into a sliding-window factor graph for pose optimization. Furthermore, an egocentric map management method based on a two-level spatial index is introduced to efficiently manage large-scale maps and maintain bounded memory consumption.

Extensive experiments on the public NCLT dataset and real-world environments demonstrate that MM-LOC achieves robust and accurate long-term localization performance with real-time efficiency.

The source code is available at:

https://github.com/zyf-sdu/MM-LOC

---

## Contributors

- Yongxin Ma
- Yifan Zhao
- Jiazhu Zeng
- Yubo Yao
- Jinshun Ou
- Panling Huang
- Jun Zhou

---

## Framework

The overall framework of MM-LOC is illustrated below.

<p align="center">
<img src="./img/framework.png" width="100%">
</p>

The proposed framework consists of:

- **Egocentric Map Management**
  - Prior Map (PPM)
  - Reference Map (RRM)
  - Real-time Local Map (RCM)
  - Two-level spatial indexing
  - On-demand map loading

- **Online Localization Pipeline**
  - LiDAR-inertial odometry front-end
  - Multi-temporal map matching
  - Adaptive localization weighting
  - Sliding-window factor graph optimization

---

# 1. Prerequisites

The environment requirements are consistent with the ROLL project.

- Ubuntu 16.04
- ROS Kinetic
- PCL 1.7
- Eigen 3.3.7
- GTSAM 4.0.2 (`-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF`)
- Ceres 2.0 (optional)

---

# 2. Build

MM-LOC is developed based on **ROLL** and **FAST-LIO**.

## 2.1 Clone MM-LOC

Clone MM-LOC into your ROS workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/zyf-sdu/MM-LOC.git
```

## 2.2 Clone FAST-LIO

Please clone FAST-LIO into the same ROS workspace.

```bash
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
```

## 2.3 Compile

```bash
cd ~/catkin_ws
catkin_make
```

Source the workspace:

```bash
source devel/setup.bash
```

---

# 3. Run MM-LOC

The launch file integrates the LiDAR-inertial odometry module and the MM-LOC localization framework.

Run MM-LOC:

```bash
roslaunch mm_loc xxx.launch
```

For rosbag evaluation:

```bash
rosbag play xxx.bag
```

---

# 4. Rosbag Example

## 4.1 Public Dataset

MM-LOC is evaluated on the **NCLT dataset**.

Dataset:

http://robots.engin.umich.edu/nclt/

The evaluated sequences include:

- 20120317
- 20120429
- 20120615
- 20120804
- 20121028
- 20121117
- 20130110

## 4.2 Real-world Field Dataset

MM-LOC is further evaluated in real-world environments.

Due to data privacy and deployment constraints, the real-world datasets are not publicly released.

---

# 5. Example Results

Coming soon.

---

# Citation

If you use MM-LOC for academic research, please cite our paper.

Coming soon.

---

# Acknowledgements

This project is built upon several excellent open-source projects and previous research works.

We sincerely thank the authors of:

- **FAST-LIO**  
  https://github.com/hku-mars/FAST_LIO

- **ROLL**  
  https://github.com/HaisenbergPeng/ROLL

- **Block-Map-Based-Localization**  
  https://github.com/YixFeng/Block-Map-Based-Localization

---

# License

The source code is released under the GPLv2 license.

http://www.gnu.org/licenses/gpl-2.0.html

---

# TODO

- [ ] Example results updating
- [ ] Refinement and updating
- [ ] Waiting for comments on papers
