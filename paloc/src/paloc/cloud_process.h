/*******************************************************
 * * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of FL2SAM (https://github.com/JokerJohn/FL2SAM-GPS).
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Xiangcheng Hu (xhubd@connect.ust.hk.com)
 * Date: ${Date}
 * Description:
 *******************************************************/
#ifndef SRC_POSE_SLAM_PRIOR_SRC_CLOUD_PROCESS_H_
#define SRC_POSE_SLAM_PRIOR_SRC_CLOUD_PROCESS_H_

#include "../../../../../../../usr/local/include/open3d/Open3D.h"
#include "../../../../../../../usr/include/pcl-1.10/pcl/features/normal_3d.h"
#include "../../../../../../../usr/include/pcl-1.10/pcl/filters/radius_outlier_removal.h"
#include "../../../../../../../usr/include/pcl-1.10/pcl/io/pcd_io.h"
#include "../../include/base_type.hpp"

class CloudProcess {
public:
    CloudProcess() {};

    pcl::PointCloud<pcl::PointXYZI>::Ptr RemoveRangeCloud(
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Vector3i axis,
            Eigen::Vector3d threshold, std::string op);

    void CropGlobalCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &globalmap_ptr,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr &temp_map,
                         Eigen::Matrix4d &pose, Eigen::Vector3d size);

    void AddNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                   pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);

    std::shared_ptr<open3d::geometry::PointCloud> GetO3dPointCloudFromPCL(
            pcl::PointCloud<pcl::PointXYZI> &pc);


    void FindNearKeyframes(std::vector<Measurement> &keyMeasures,
                           pcl::PointCloud<PointT>::Ptr &nearKeyframes,
                           const int &key, const int &searchNum);

    bool DoICPVirtualRelative(std::vector<Measurement> &keyMeasures,
                              int loopKeyPre, int loopKeyCur, float &score,
                              int type, Eigen::Matrix4d &trans);

    bool DoICPVirtualRelative2(std::vector<Measurement> &keyMeasures,
                               int loopKeyPre, int loopKeyCur,
                               float &score, int type,
                               Eigen::Matrix4d &trans);

    bool DoICPDegeneracy(std::vector<Measurement> &keyMeasures,
                         int loopKeyPre, int loopKeyCur,
                         float &score, int type,
                         Eigen::Matrix4d &trans);

    Eigen::Matrix6d icp_cov = Eigen::Matrix6d::Identity();


};

#endif  // SRC_POSE_SLAM_PRIOR_SRC_CLOUD_PROCESS_H_
