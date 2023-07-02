<div id="top" align="center">

# [PALoc: Robust Prior-assisted Trajectory Generation for Benchmarking](https://arxiv.org/pdf/2305.13147.pdf)

**[ICRA 2023 Workshop on Future of Construction](https://construction-robots.github.io/#)**

![image-20230702133158290](/home/xchu/.config/Typora/typora-user-images/image-20230702133158290.png)

</div>

## Table of Contents
- [Introduction](#introduction)
- [News](#news)
- [Dataset](#data)
- [Get Started](#get-started)
- [Results](#Results)
- [Citation](#citation)
- [License](#license)

## Introduction
Evaluating simultaneous localization and mapping (SLAM) algorithms necessitates high-precision and dense ground truth (GT) trajectories. But obtaining desirable GT trajectories is sometimes challenging without GT tracking sensors. As an alternative, in this paper, we propose a novel prior-assisted SLAM system to generate a full six-degree-offreedom (6-DOF) trajectory at around 10Hz for benchmarking, under the framework of the factor graph. Our degeneracyaware map factor utilizes a prior point cloud map and LiDAR frame for point-to-plane optimization, simultaneously detecting degeneration cases to reduce drift and enhancing the consistency of pose estimation. Our system is seamlessly integrated with cutting-edge odometry via a loosely coupled scheme to generate high-rate and precise trajectories. Moreover, we propose a norm-constrained gravity factor for stationary cases, optimizing both pose and gravity to boost performance. Extensive evaluations demonstrate our algorithm’s superiority over existing SLAM or map-based methods in diverse scenarios, in terms of precision, smoothness, and robustness. Our approach substantially advances reliable and accurate SLAM evaluation methods, fostering progress in robotics research.

![image-20230702134120453](/home/xchu/.config/Typora/typora-user-images/image-20230702134120453.png)

## News

- [2023/05.08]
  - accepted by ICRA2023 Workshop on Future of Construction.

## Dataset

Our algorithms were tested on our [FusionPortable](https://ram-lab.com/file/site/fusionportable/dataset/fusionportable/) Dataset.

## Get Started

## Results

- Map evaluation

![image-20230702134519531](/home/xchu/.config/Typora/typora-user-images/image-20230702134519531.png)

![image-20230702134345309](/home/xchu/.config/Typora/typora-user-images/image-20230702134345309.png)

## Citation

The map evaluation metrics of this work follows [Cloud_Map_Evaluation](https://github.com/JokerJohn/Cloud_Map_Evaluation). Please cite:

```
@article{,
  author    = {Jianhao Jiao and Hexiang Wei and Tianshuai Hu and Xiangcheng Hu and Yilong Zhu and Zhijian He and Jin Wu and Jingwen Yu and Xupeng Xie and Huaiyang Huang and Ruoyu Geng and Lujia Wang and Ming Liu},
  title     = {FusionPortable: A Multi-Sensor Campus-Scene Dataset for Evaluation of Localization and Mapping Accuracy on Diverse Platforms},
  booktitle = {2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2022}
}
```

For this work,

```
@misc{hu2023paloc,
      title={PALoc: Robust Prior-assisted Trajectory Generation for Benchmarking}, 
      author={Xiangcheng Hu and Jin Wu and Jianhao Jiao and Ruoyu Geng and Ming Liu},
      year={2023},
      eprint={2305.13147},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## License

All code in this project is released under [GNU General Public License v3.0](./LICENSE).
