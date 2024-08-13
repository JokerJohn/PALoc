
<div align="center">

<h1>PALoc: Advancing SLAM Benchmarking with Prior-Assisted 6-DoF Trajectory Generation and Uncertainty Estimation</h1>

![PALoc Overview](./README/image-20230702133158290.png)
<a href="https://ieeexplore.ieee.org/document/10480308"><img src='https://img.shields.io/badge/IEEE TMECH 2024- PALoc -red' alt='Paper PDF'></a> [![GitHub Stars](https://img.shields.io/github/stars/JokerJohn/PALoc.svg)](https://github.com/JokerJohn/PALoc/stargazers) [![GitHub Issues](https://img.shields.io/github/issues/JokerJohn/PALoc.svg)](https://github.com/JokerJohn/PALoc/issues)[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)<a href="https://github.com/JokerJohn/PALoc/blob/main/"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>

</div>

## Introduction

**PALoc** presents a novel approach for generating high-fidelity, dense 6-DoF ground truth (GT) trajectories, enhancing the evaluation of Simultaneous Localization and Mapping (SLAM) under diverse environmental conditions. This framework leverages prior maps to improve the accuracy of indoor and outdoor SLAM datasets. Key features include:

- **Robustness in Degenerate  Conditions**: Exceptionally handles scenarios frequently encountered in SLAM datasets.
- **Advanced Uncertainty Analysis**: Detailed covariance derivation within factor graphs, enabling precise uncertainty propagation and pose analysis.
- **Open-Source Toolbox**: An [open-source toolbox](https://github.com/JokerJohn/Cloud_Map_Evaluation) is provided for map evaluation, indirectly assessing trajectory precision.
Experimental results demonstrate at least a 30% improvement in map accuracy and a 20% increase in direct trajectory accuracy over the ICP algorithm, across various campus environments.
<div align="center">

![Pipeline](./README/image-20240131044249967.png)
![image-20240323140959367](./README/image-20240323140959367.png)
</div>


## News

- **2024/08/14**: Release codes.
- **2024/03/26**: [Early access](https://ieeexplore.ieee.org/document/10480308) by IEEE/ASME TMECH.
- **2024/02/01**: Preparing codes for release.
- **2024/01/29**: Accepted by 2024 IEEE/ASME TMECH.
- **2023/12/08**: Resubmitted.
- **2023/08/22**: Reject and resubmitted.
- **2023/05/13**: Submitted to IEEE/ASME TRANSACTIONS ON MECHATRONICS (TMECH).
- **2023/05/08**: Accepted by [ICRA 2023 Workshop on Future of Construction](https://construction-robots.github.io/#).

## Dataset

### GEODE Dataset From SYSU

Stairs  scenes with different  types of lidar and **glass noise**. This is very challenging due to **narrow stairs** ,  you need to tune some parameters of **ICP**.

| Sensor setup           | Download link                    |
| ---------------------- | -------------------------------- |
| Velodyne16+ xsense IMU | http://gofile.me/4jm56/yCBxjdEXA |
| Ouster64 + xsense IMU  | http://gofile.me/4jm56/2EoKPDfKi |

| ![image-20240813195354720](./README/image-20240813195354720.png) | ![image-20240812181454491](./README/image-20240812181454491.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |

The prior map and raw map.

| Prior map without glass noise    | raw prior map                    |      |
| -------------------------------- | -------------------------------- | ---- |
| http://gofile.me/4jm56/SfohLpthw | http://gofile.me/4jm56/pK0A9zTJn |      |



### [Fusion Portable Dataset](https://fusionportable.github.io/dataset/fusionportable/)

Our algorithms were rigorously tested on the [Fusion Portable Dataset](https://ram-lab.com/file/site/fusionportable/dataset/fusionportable/). 

| Sequence                                                     | GT Map                           | Scene                                                        |
| ------------------------------------------------------------ | -------------------------------- | ------------------------------------------------------------ |
| [20220216_corridor_day](https://drive.google.com/drive/folders/1Xc6m3WZrbjdhq9OjfWKDepb9cLKJpety) | http://gofile.me/4jm56/mRT2hkB25 | ![image-20240813213307904](./README/image-20240813213307904.png) |



### Self-collected Dataset

Below is our sensor kit setup.
<div align="center">

![image-20240323140835087](./README/image-20240323140835087.png)

| Dataset    | BAG                                                          | GT                                                           |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| redbird_01 | [rosbag](https://hkustconnect-my.sharepoint.com/:u:/g/personal/xhubd_connect_ust_hk/EW-bKP6RkZpPhRHfk2DReeEBwN8MvP2Eq5cfoUIBYglwEQ?e=Rhetkr) | [GT](https://hkustconnect-my.sharepoint.com/:t:/g/personal/xhubd_connect_ust_hk/EYoWWAdX8FZBph3LJZ6lck8BuMj43lcEcab9C0fi4Tmqbg?e=GqPs1D) |
| redbird_02 | [rosbag](https://hkustconnect-my.sharepoint.com/:u:/g/personal/xhubd_connect_ust_hk/EXGbd3lDtLNAr6Q_0QPKiH4B1zDYpA2Qr-RTLcKj36KgYw?e=NJ3XxG) | [GT](https://hkustconnect-my.sharepoint.com/:t:/g/personal/xhubd_connect_ust_hk/EXziPmChz3xGuIwd6_bla0IBbYV5NvCZ92Xff_X17dy9Wg?e=8KoiWr) |

</div>



## Getting Started

The complete code will be released post-journal acceptance. For a sneak peek:
- [Watch on Bilibili](https://www.bilibili.com/video/BV11V4y1a7Fd/)
- [Watch on YouTube](https://www.youtube.com/watch?v=_6a2gWYHeUk)

### Install

- *[Open3d ( >= 0.11)](https://github.com/isl-org/Open3D)*
- PCL
- GTSAM

### Quickly Run

download the demo rosbag and prior map, set the file path in `geode_beta_os64.launch`.

```bash
#ouster64
roslaunch paloc geode_beta_os64.launch

#velodyne 16
roslaunch paloc geode_alpha_vlp16.launch

#os128 fusionportable-corridor
roslaunch paloc fp_corridor.launch
```

then play rosbag:

```bash
# ouster64
rosbag play stairs_bob.bag

#velodyne16
rosbag play stairs_alpha.bag

#os128 fusionportable-corridor
rosbag play 20220216_corridor_day_ref.bag
```

 You can save data. 

```bash
rosservice call /save_map
```

![image-20240813173140476](./README/image-20240813173140476.png)

### For your own dataset

#### Set Parameters

Set your file path in `geode_beta_os64.launch`. Put your prior map file in `prior_map_directory`, the map name must be set as the `sequence`.  The map file size is recommend to down-sampled less than 2Gb.  

```bash
<param name="save_directory" type="string" value="/home/xchu/data/paloc_result/"/>
<param name="prior_map_directory" type="string" value="/home/xchu/data/prior_map/paloc_map_file/"/>

<arg name="sequence" default="stairs_bob"/>
```

![image-20240813184225320](./README/image-20240813184225320.png)

#### Adapt for FAST-LIO2

Set parameters in `geode_beta_os64.yaml`.  Adapt for the FAST-LIO2 first. 

```yaml
common:
  lid_topic: "/ouster/points"
  imu_topic: "/imu/data"

  acc_cov: 1.1118983704388789e-01                     # acc noise and bias
  b_acc_cov: 1.5961182793700285e-03
  gyr_cov: 9.6134865171113148e-02                     # gyro noise and bias
  b_gyr_cov: 7.9993782046705285e-04

  extrinsic_T: [ -0.027172, -0.034873, 0.062643 ]     # from lidar to imu
  extrinsic_R: [ 0.998638, 0.052001, -0.004278,
                 -0.051937, 0.998554, 0.013900,
                 0.004994, -0.013659, 0.999894 ]
lio:
  lidar_type: 8      # 1 for Livox serials LiDAR, 2 for Velodyne LiDAR, 3 for ouster LiDAR,
  scan_line: 64
  scan_rate: 10      # only need to be set for velodyne, unit: Hz,
  timestamp_unit: 3  # 0-second, 1-milisecond, 2-microsecond, 3-nanosecond.
  blind: 0.5         # remove the nearest point cloud

```

#### Initial pose

then set the initial pose.  When you run the launch command, the first point cloud will be saved in `save_directory`,  you can align it with the prior map using [CloudCompare](https://www.danielgm.net/cc/) to get the initial pose.

```yaml
common:
    initial_pose: [ -0.519301, 0.850557, 0.082936 ,-11.347226,
                  -0.852164, -0.522691, 0.024698, 3.002144,
                  0.064357, -0.057849, 0.996249, -0.715776,
                  0.000000, 0.000000, 0.000000, 1.000000 ]
```

![image-20240813185337470](./README/image-20240813185337470.png)

![image-20240807094808981](./README/image-20240807094808981.png)

#### Evaluation with [Cloud Map Evaluation Lib](https://github.com/JokerJohn/Cloud_Map_Evaluation)

We can evaluate the map accuracy of PAloc as follows. Note that when you use the Cloud Map Evaluation library, the map of PALoc or ICP  do not need to set initial pose since they are already in the same frame. But evaluate the map from FAST-LIO2 must to set it. 



| Raw Error Map                                                | Entropy Map                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image-20240813190359418](./README/image-20240813190359418.png) | ![image-20240813190440224](./README/image-20240813190440224.png) |

Evaluation results example can be [downloaded](http://gofile.me/4jm56/JylhSi89S) here.



### TODO

- clean codes
- tutorial and parameters tuning
- support for LIO-SAM
- gravity factor
- adapt for more dataset and lidar 



## Results

### Trajectory Evaluation

<div align="center">

![Trajectory Evaluation](./README/image-20240131044609655.png)
</div>

### Map Evaluation

<div align="center">

![Map Evaluation 1](./README/image-20240131044537891.png)
![image-20240323141038974](./README/image-20240323141038974.png)
</div>

### Degeneracy Analysis
| X Degenerate (Translation)                                   | Yaw Degenerate (Rotation)                                    |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![X Translation Degeneracy](./README/image-20240131044702442.png) | ![Yaw Rotation Degeneracy](./README/image-20240131044518033.png) |
<div align="center">

![Degeneracy Analysis](./README/image-20240131044808306.png)
</div>

### Time Analysis

<div align="center">

![Time Analysis 1](./README/image-20240131044902360.png)
![Time Analysis 2](./README/image-20240131044851437.png)
</div>

To plot the results, you can follow this [scripts](https://github.com/JokerJohn/SLAMTools/blob/main/Run_Time_analysis/time_analysis.py).

## Important Issue

### How do we calculate cov of odom factor in GTSAM?

We follow the assumption of pose Independence as barfoot does(TRO 2014) as equation (13), there is no correlated cov between this two poses. Left-hand convention means a small increments on **world frame**.

![image-20240408203918682](./README/image-20240408203918682.png)

 Since GTSAM follow the right-hand convention on SE(3) , we need to use the adjoint representation as equation (14). 

![image-20240408204125990](./README/image-20240408204125990.png)

## Citations

For referencing our work in PALoc, please use:
```bibtex
@ARTICLE{hu2024paloc,
  author={Hu, Xiangcheng and Zheng, Linwei and Wu, Jin and Geng, Ruoyu and Yu, Yang and Wei, Hexiang and Tang, Xiaoyu and Wang, Lujia and Jiao, Jianhao and Liu, Ming},
  journal={IEEE/ASME Transactions on Mechatronics}, 
  title={PALoc: Advancing SLAM Benchmarking With Prior-Assisted 6-DoF Trajectory Generation and Uncertainty Estimation}, 
  year={2024},
  volume={},
  number={},
  pages={1-12},
  doi={10.1109/TMECH.2024.3362902}
  }
```
The map evaluation metrics of this work follow [Cloud_Map_Evaluation](https://github.com/JokerJohn/Cloud_Map_Evaluation). Please cite:
```bibtex
@article{jiao2024fp,
  author    = {Jianhao Jiao and Hexiang Wei and Tianshuai Hu and Xiangcheng Hu and Yilong Zhu and Zhijian He and Jin Wu and Jingwen Yu and Xupeng Xie and Huaiyang Huang and Ruoyu Geng and Lujia Wang and Ming Liu},
  title     = {FusionPortable: A Multi-Sensor Campus-Scene Dataset for Evaluation of Localization and Mapping Accuracy on Diverse Platforms},
  booktitle = {2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2022}
}
```

## Acknowledgment

The code in this project is adapted from the following projects:

- The odometry  method is adapted from [FAST-LIO2](https://github.com/hku-mars/FAST_LIO).
- The basic framework for pose graph optimization (PGO) is adapted from [SC-A-LOAM](https://github.com/gisbi-kim/SC-A-LOAM).
- The Point-to-Plane registration is adapted from [LOAM](https://github.com/laboshinl/loam_velodyne).



## Contributors

<a href="https://github.com/JokerJohn/PALoc/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=JokerJohn/PALoc" />
</a>
