<div id="top" align="center">

# [PALoc: Advancing SLAM Benchmarking with Prior-Assisted 6-DoF Trajectory Generation and Uncertainty Estimation](https://www.researchgate.net/publication/377778512_PALoc_Advancing_SLAM_Benchmarking_with_Prior-Assisted_6-DoF_Trajectory_Generation_and_Uncertainty_Estimation)
![PALoc Overview](./README/image-20230702133158290.png)
</div>

## Table of Contents
- [Introduction](#introduction)

- [News](#news)

- [Dataset](#dataset)

- [Getting Started](#getting-started)

- [Results](#results)

- [Citations](#citations)

- [License](#license)

  

## Introduction

**PALoc** presents a novel approach for generating high-fidelity, dense 6-DoF ground truth (GT) trajectories, enhancing the evaluation of Simultaneous Localization and Mapping (SLAM) under diverse environmental conditions. This framework leverages prior maps to improve the accuracy of indoor and outdoor SLAM datasets. Key features include:

- **Robustness in Degenerate  Conditions**: Exceptionally handles scenarios frequently encountered in SLAM datasets.
- **Advanced Uncertainty Analysis**: Detailed covariance derivation within factor graphs, enabling precise uncertainty propagation and pose analysis.
- **Open-Source Toolbox**: An [open-source toolbox](https://github.com/JokerJohn/Cloud_Map_Evaluation) is provided for map evaluation, indirectly assessing trajectory precision.
Experimental results demonstrate at least a 30% improvement in map accuracy and a 20% increase in direct trajectory accuracy over the ICP algorithm, across various campus environments.
<div align="center">

![Pipeline](./README/image-20240131044249967.png)
![Daxtor Graph](./README/image-20240131044940524.png)
</div>


## News

- **2024/02/01**: Preparing codes for release.
- **2024/01/29**: Accepted by 2024 IEEE/ASME TRANSACTIONS ON MECHATRONICS (TMECH).
- **2023/12/08**: Resubmitted.
- **2023/08/22**: Reject and resubmitted.
- **2023/05/13**: Submitted to IEEE/ASME TRANSACTIONS ON MECHATRONICS (TMECH).
- **2023/05/08**: Accepted by [ICRA 2023 Workshop on Future of Construction](https://construction-robots.github.io/#).
## Dataset
Our algorithms were rigorously tested on the [FusionPortable Dataset](https://ram-lab.com/file/site/fusionportable/dataset/fusionportable/). Below is our sensor kit setup.
<div align="center">

![Sensor Kit](./README/image-20230702135628428.png)
</div>


## Getting Started

The complete code will be released post-journal acceptance. For a sneak peek:
- [Watch on Bilibili](https://www.bilibili.com/video/BV11V4y1a7Fd/)
- [Watch on YouTube](https://www.youtube.com/watch?v=_6a2gWYHeUk)

## Results

### Trajectory Evaluation

<div align="center">

![Trajectory Evaluation](./README/image-20240131044609655.png)
</div>

### Map Evaluation
<div align="center">

![Map Evaluation 1](./README/image-20240131044537891.png)
![Map Evaluation 2](./README/image-20240131044423905.png)
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

## Citations
For referencing our work in PALoc, please use:
```
@article{article,
author = {Hu, Xiangcheng and Zheng, Linwei and Wu, Jin and Geng, Ruoyu and Wei, Hexiang and Wang, Lujia and Liu, Ming},
year = {2024},
month = {01},
title = {PALoc: Advancing SLAM Benchmarking with Prior-Assisted 6-DoF Trajectory Generation and Uncertainty Estimation},
journal = {IEEE/ASME Transactions on Mechatronics}
}
```
The map evaluation metrics of this work follows [Cloud_Map_Evaluation](https://github.com/JokerJohn/Cloud_Map_Evaluation). Please cite:
```
@article{,
  author    = {Jianhao Jiao and Hexiang Wei and Tianshuai Hu and Xiangcheng Hu and Yilong Zhu and Zhijian He and Jin Wu and Jingwen Yu and Xupeng Xie and Huaiyang Huang and Ruoyu Geng and Lujia Wang and Ming Liu},
  title     = {FusionPortable: A Multi-Sensor Campus-Scene Dataset for Evaluation of Localization and Mapping Accuracy on Diverse Platforms},
  booktitle = {2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2022}
}
```

## License

This project's code is available under the [GNU General Public License v3.0](./LICENSE).
