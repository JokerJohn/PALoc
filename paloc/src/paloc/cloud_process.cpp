/*
* PALoc: Advancing SLAM Benchmarking with Prior-Assisted 6-DoF Trajectory Generation and Uncertainty Estimation
* Copyright (c) 2024 Hu Xiangcheng
*
* This project is licensed under the MIT License.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Author: Hu Xiangcheng
* Contact: xhubd@connect.ust.hk
* Affiliation: The Cheng Kar Shun Robotics Institute (CKSRI), Hong Kong University of Science and Technology (HKUST)
*
*/
#include "cloud_process.h"

pcl::PointCloud<pcl::PointXYZI>::Ptr CloudProcess::RemoveRangeCloud(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Vector3i axis,
        Eigen::Vector3d threshold, std::string op) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
            new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond(
            new pcl::ConditionAnd<pcl::PointXYZI>());
    pcl::ComparisonOps::CompareOp oper;
    if (op == ">")
        oper = pcl::ComparisonOps::LT;
    else
        oper = pcl::ComparisonOps::GT;

    if (axis == Eigen::Vector3i(1, 1, 1)) {
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZI>("x", oper, threshold[0])));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZI>("y", oper, threshold[1])));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZI>("z", oper, threshold[2])));
    } else if (axis == Eigen::Vector3i(1, 1, 0)) {
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZI>("x", oper, threshold[0])));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZI>("y", oper, threshold[1])));
    } else {
        std::cout << "conditional cloud filter can not support this type!"
                  << std::endl;
    }

    pcl::ConditionalRemoval<pcl::PointXYZI> condrem(true);
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(false);
    condrem.filter(*filtered);
    return filtered;
}

void CloudProcess::CropGlobalCloud(
        pcl::PointCloud<pcl::PointXYZI>::Ptr &globalmap_ptr,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &transform_cloud_ptr,
        Eigen::Matrix4d &trans, Eigen::Vector3d size) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_map(
            new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr crop_cloud_map_filtered(
            new pcl::PointCloud<pcl::PointXYZI>);

    pcl::copyPointCloud(*globalmap_ptr, *temp_map);
    // transform point cloud to lidar frame
    // Pose6D map2base_curr = Matrix2Pose6D(Pose6D2Matrix(pose).inverse());
    //  pcl::transformPointCloud(*temp_map, *transform_cloud_ptr,
    //                           pose.inverse().cast<float>());
    *transform_cloud_ptr = *TransformPointCloud(temp_map, trans.inverse());

    // to lidar frame
    //*transform_cloud_ptr = *Local2global(temp_map,
    // Matrix2Pose6D(Pose6D2Matrix(pose).inverse()));
    crop_cloud_map_filtered = RemoveRangeCloud(
            transform_cloud_ptr, Eigen::Vector3i(1, 1, 1), size, ">");
    //  crop_cloud_map_filtered = range_remove(
    //      crop_cloud_map_filtered, Eigen::Vector3i(20, 20, 20), size, "<");

    // map link
    // pcl::PointCloud<PointT>::Ptr trans_cloud_map(new
    // pcl::PointCloud<PointT>());
    transform_cloud_ptr->clear();
    //  pcl::transformPointCloud(*crop_cloud_map_filtered, *transform_cloud_ptr,
    //                           pose.cast<float>());

    *transform_cloud_ptr = *TransformPointCloud(crop_cloud_map_filtered, trans);

    //  publishCloud(pubLaserCloudCrop, transform_cloud_ptr, ros::Time::now(),
    //               odom_link);

    // *transform_cloud_ptr = *trans_cloud_map;
    //    std::cout << BOLDWHITE << "crop global map: " <<
    //    crop_cloud_map_filtered->size() << ", " << transform_cloud_ptr->size()
    //              << std::endl;
}

void CloudProcess::AddNormal(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals) {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(
            new pcl::search::KdTree<pcl::PointXYZI>);
    searchTree->setInputCloud(cloud);

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud);
    normalEstimator.setSearchMethod(searchTree);
    normalEstimator.setKSearch(15);
    normalEstimator.compute(*normals);

    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
}

std::shared_ptr<open3d::geometry::PointCloud>
CloudProcess::GetO3dPointCloudFromPCL(pcl::PointCloud<pcl::PointXYZI> &pc) {
    std::shared_ptr<open3d::geometry::PointCloud> point_cloud_p3d(
            new open3d::geometry::PointCloud());
    point_cloud_p3d->points_.resize(pc.size());
    for (int i = 0; i < pc.size(); ++i) {
        point_cloud_p3d->points_[i][0] = pc.points.at(i).x;
        point_cloud_p3d->points_[i][1] = pc.points.at(i).y;
        point_cloud_p3d->points_[i][2] = pc.points.at(i).z;
        //    point_cloud_p3d->colors_.at(i)
    }
    return point_cloud_p3d;
}


// pcl::PointCloud<pcl::PointXYZ>::Ptr
// CloudProcess::GetPointCloudPCLFromO3d(std::shared_ptr<geometry::PointCloud>
// pc) {
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new
//  pcl::PointCloud<pcl::PointXYZ>); cloud_ptr->resize(pc->points_.size()); for
//  (int i = 0; i < pc->points_.size(); i++) {
//    cloud_ptr->points[i].x = pc->points_[i].x();
//    cloud_ptr->points[i].y = pc->points_[i].y();
//    cloud_ptr->points[i].z = pc->points_[i].z();
//  }
//  return cloud_ptr;
//}

void CloudProcess::FindNearKeyframes(
        std::vector<Measurement> &keyMeasures,
        pcl::PointCloud<PointT>::Ptr &nearKeyframes, const int &key,
        const int &searchNum) {
    // 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合
    nearKeyframes->clear();
    int cloudSize = keyMeasures.size();
    for (int i = -searchNum; i <= searchNum; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize) continue;
        // *nearKeyframes += *transformPointCloud(keyframeLaserClouds[keyNear],
        // &copy_cloudKeyPoses6D->points[keyNear]);
        *nearKeyframes += *TransformPointCloud(keyMeasures[keyNear].lidar,
                                               keyMeasures[keyNear].updated_pose);
    }

    if (nearKeyframes->empty()) return;

    scan_filter_size = 0.2;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterICP;
    downSizeFilterICP.setLeafSize(scan_filter_size, scan_filter_size,
                                  scan_filter_size);

    if (searchNum == 0) {
        *nearKeyframes = *nearKeyframes;
    } else {
        pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>());
        if (useRawCloud)
            downSizeFilterICP.setLeafSize(scan_filter_size, scan_filter_size,
                                          scan_filter_size);

        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }
}

bool CloudProcess::DoICPVirtualRelative(std::vector<Measurement> &keyMeasures,
                                        int loopKeyPre, int loopKeyCur,
                                        float &score, int type,
                                        Eigen::Matrix4d &trans) {
    using namespace open3d;
    // int historyKeyframeSearchNum = 25; // enough. ex. [-25, 25] covers submap
    // length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointT>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr targetKeyframeCloud(
            new pcl::PointCloud<PointT>());
    // loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0,
    // _loop_kf_idx); // use same root of loop kf idx
    // loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx,
    // historyKeyframeSearchNum, _loop_kf_idx); 提取当前关键帧特征点集合，降采样
    FindNearKeyframes(keyMeasures, cureKeyframeCloud, loopKeyCur, 0);
    //    *cureKeyframeCloud =
    //    *Local2global(keyframeLaserRawClouds.at(loopKeyCur),
    //    keyframePosesUpdated[loopKeyCur]);
    // 提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
    FindNearKeyframes(keyMeasures, targetKeyframeCloud, loopKeyPre,
                      historyKeyframeSearchNum);

    if (cureKeyframeCloud->size() < 100 || targetKeyframeCloud->size() < 1000)
        return false;

    //  publishCloud(pubLoopScanLocal, cureKeyframeCloud, ros::Time::now(),
    //               odom_link);
    //  publishCloud(pubLoopSubmapLocal, targetKeyframeCloud, ros::Time::now(),
    //               odom_link);

    // ICP Settings
    // static pcl::IterativeClosestPoint<PointT, PointT> icp;
    //  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
    //  icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
    //  icp.setMaximumIterations(50);
    //  icp.setTransformationEpsilon(1e-6);
    //  icp.setEuclideanFitnessEpsilon(1e-6);
    //  icp.setRANSACIterations(0);
    //
    //  // Align pointclouds
    //  icp.setInputSource(cureKeyframeCloud);
    //  icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
    //  icp.align(*unused_result);

    //*******************open 3d pcl ****************
    //  std::shared_ptr<geometry::PointCloud> source_o3d =
    //      GetPointCloudFromPCL(*cureKeyframeCloud);
    //  std::shared_ptr<geometry::PointCloud> target_o3d =
    //      GetPointCloudFromPCL(*targetKeyframeCloud);
    std::shared_ptr<geometry::PointCloud> source_o3d =
            GetO3dPointCloudFromPCL(*cureKeyframeCloud);
    std::shared_ptr<geometry::PointCloud> target_o3d =
            GetO3dPointCloudFromPCL(*targetKeyframeCloud);
    //  *source_o3d = source_o3d->Transform(guess_matrix.cast<double>());

    source_o3d = source_o3d->VoxelDownSample(scan_filter_size);
    target_o3d = target_o3d->VoxelDownSample(scan_filter_size);

    Eigen::Matrix4d icp_trans = Eigen::Matrix4d::Identity();

    pipelines::registration::RegistrationResult icp;
    auto criteria = pipelines::registration::ICPConvergenceCriteria(30);
    switch (icpO3dType) {
        case 0:  // point-to-point icp
            icp = pipelines::registration::RegistrationICP(
                    *source_o3d, *target_o3d, 1.0, Eigen::Matrix4d::Identity(),
                    pipelines::registration::TransformationEstimationPointToPoint(),
                    criteria);
            break;
        case 1:  // Point-to-plane
            source_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
            target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
            icp = pipelines::registration::RegistrationICP(
                    *source_o3d, *target_o3d, 1.0, Eigen::Matrix4d::Identity(),
                    pipelines::registration::TransformationEstimationPointToPlane(),
                    criteria);
            break;
        case 2:
            source_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
            target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
            icp = pipelines::registration::RegistrationGeneralizedICP(
                    *source_o3d, *target_o3d, 1.0, Eigen::Matrix4d::Identity(),
                    pipelines::registration::TransformationEstimationForGeneralizedICP(),
                    criteria);
            break;
        default:
            std::cout << " evaluation error type!!!!! " << std::endl;
            break;
    }
    icp_trans = icp.transformation_;
    score = icp.inlier_rmse_;
    double overlap = icp.fitness_;

    //  pcl::transformPointCloud(*cureKeyframeCloud, *unused_result,
    //                           trans.cast<float>());
    *unused_result = *TransformPointCloud(cureKeyframeCloud, icp_trans);

    trans = icp_trans * trans;

    if (0) {
        ROS_INFO("LOOP ICP ALIGNED POINTS: %d and %d, %f, %f",
                 cureKeyframeCloud->size(), targetKeyframeCloud->size(), score,
                 overlap);
    }

    if (score > loopFitnessScoreThreshold || overlap < 0.8 || score == 0.0) {
        // do corse to fine icp
        return false;
    }
    return true;
}

bool CloudProcess::DoICPVirtualRelative2(std::vector<Measurement> &keyMeasures,
                                         int loopKeyPre, int loopKeyCur,
                                         float &score, int type,
                                         Eigen::Matrix4d &final_trans) {
    using namespace open3d;
    // int historyKeyframeSearchNum = 25; // enough. ex. [-25, 25] covers submap
    // length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointT>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr targetKeyframeCloud(
            new pcl::PointCloud<PointT>());
    // loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0,
    // _loop_kf_idx); // use same root of loop kf idx
    // loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx,
    // historyKeyframeSearchNum, _loop_kf_idx); 提取当前关键帧特征点集合，降采样
    FindNearKeyframes(keyMeasures, cureKeyframeCloud, loopKeyCur, 0);
    //    *cureKeyframeCloud =
    //    *Local2global(keyframeLaserRawClouds.at(loopKeyCur),
    //    keyframePosesUpdated[loopKeyCur]);
    // 提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
    FindNearKeyframes(keyMeasures, targetKeyframeCloud, loopKeyPre,
                      historyKeyframeSearchNum);

    if (cureKeyframeCloud->size() < 100 || targetKeyframeCloud->size() < 100)
        return false;

    //  publishCloud(pubLoopScanLocal, cureKeyframeCloud, ros::Time::now(),
    //               odom_link);
    //  publishCloud(pubLoopSubmapLocal, targetKeyframeCloud, ros::Time::now(),
    //               odom_link);

    // ICP Settings
    // static pcl::IterativeClosestPoint<PointT, PointT> icp;
    //  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
    //  icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
    //  icp.setMaximumIterations(50);
    //  icp.setTransformationEpsilon(1e-6);
    //  icp.setEuclideanFitnessEpsilon(1e-6);
    //  icp.setRANSACIterations(0);
    //
    //  // Align pointclouds
    //  icp.setInputSource(cureKeyframeCloud);
    //  icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
    //  icp.align(*unused_result);

    //*******************open 3d pcl ****************
    //  std::shared_ptr<geometry::PointCloud> source_o3d =
    //      GetPointCloudFromPCL(*cureKeyframeCloud);
    //  std::shared_ptr<geometry::PointCloud> target_o3d =
    //      GetPointCloudFromPCL(*targetKeyframeCloud);
    std::shared_ptr<geometry::PointCloud> source_o3d =
            GetO3dPointCloudFromPCL(*cureKeyframeCloud);
    std::shared_ptr<geometry::PointCloud> target_o3d =
            GetO3dPointCloudFromPCL(*targetKeyframeCloud);
    //  *source_o3d = source_o3d->Transform(guess_matrix.cast<double>());

//     source_o3d = source_o3d->VoxelDownSample(scan_filter_size);
//    target_o3d = target_o3d->VoxelDownSample(scan_filter_size);
    //  source_o3d = source_o3d->VoxelDownSample(0.1);
    target_o3d = target_o3d->VoxelDownSample(0.4);

    Eigen::Matrix4d icp_trans = Eigen::Matrix4d::Identity();

    double max_correspondence_distance = 2.0; // 根据需求设置
    pipelines::registration::RegistrationResult icp;
    auto criteria = pipelines::registration::ICPConvergenceCriteria(30);
    switch (icpO3dType) {
        case 0:  // point-to-point icp
            icp = pipelines::registration::RegistrationICP(
                    *source_o3d, *target_o3d, max_correspondence_distance, Eigen::Matrix4d::Identity(),
                    pipelines::registration::TransformationEstimationPointToPoint(),
                    criteria);
            break;
        case 1:  // Point-to-plane
            target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(2.0, 20));
            icp = pipelines::registration::RegistrationICP(
                    *source_o3d, *target_o3d, max_correspondence_distance, Eigen::Matrix4d::Identity(),
                    pipelines::registration::TransformationEstimationPointToPlane(),
                    criteria);
            break;
        case 2:
            target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(2.0, 20));
            icp = pipelines::registration::RegistrationGeneralizedICP(
                    *source_o3d, *target_o3d, max_correspondence_distance, Eigen::Matrix4d::Identity(),
                    pipelines::registration::TransformationEstimationForGeneralizedICP(),
                    criteria);
            break;
        default:
            std::cout << " evaluation error type!!!!! " << std::endl;
            break;
    }
    icp_trans = icp.transformation_;
    score = icp.inlier_rmse_;
    double overlap = icp.fitness_;

    //  pcl::transformPointCloud(*cureKeyframeCloud, *unused_result,
    //                           trans.cast<float>());
    *unused_result = *TransformPointCloud(cureKeyframeCloud, icp_trans);
    final_trans = icp_trans * final_trans;
    if (1) {
        std::cout << "LOOP ICP ALIGNED POINTS: " << cureKeyframeCloud->size() << " " << target_o3d->points_.size()
                  << " " << score << " " << overlap << std::endl;
    }
    if (score > loopFitnessScoreThreshold || overlap < 0.7 || score == 0.0) {
        // do corse to fine icp
        std::cout << "LOOP ICP FAILED: " << cureKeyframeCloud->size() << " " << target_o3d->points_.size()
                  << " " << score << " " << overlap << std::endl;
        return false;
    }

    // 使用 EigenMatrixToTensor 转换 Eigen 矩阵到 Open3D Tensor
    open3d::core::Tensor transformation_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(
            icp.transformation_);
    // 将旧版 Open3D 点云转换为新版
    open3d::t::geometry::PointCloud source_o3d_new = open3d::t::geometry::PointCloud::FromLegacy(*source_o3d);
    open3d::t::geometry::PointCloud target_o3d_new = open3d::t::geometry::PointCloud::FromLegacy(*target_o3d);
    bool flag = false;
    try {
        open3d::core::Tensor information_matrix = open3d::t::pipelines::registration::GetInformationMatrix(
                source_o3d_new,
                target_o3d_new,
                max_correspondence_distance,
                transformation_tensor);
        // 将 Tensor 转换为 Eigen 矩阵
        Eigen::MatrixXd ifm_eigen = open3d::core::eigen_converter::TensorToEigenMatrixXd(information_matrix);
        if (ifm_eigen.rows() == 6 && ifm_eigen.cols() == 6) {
            icp_cov = ifm_eigen.inverse() * 1e-3;
            flag = true;
            std::cout << "O3D LOOP ICP COV: \n" << icp_cov.diagonal().transpose() << std::endl;
        } else {
            std::cerr << BOLDRED << "Information matrix is not 6x6. Cannot compute covariance matrix." << std::endl;
        }
    } catch (const std::runtime_error &e) {
        std::cerr << "Runtime error: " << e.what() << std::endl;
        std::cerr
                << "Check if the point clouds have sufficient correspondences and adjust the max_correspondence_distance parameter if necessary."
                << std::endl;
        icp_cov = Eigen::Matrix<double, 6, 6>::Identity();
    }
    std::cout << BOLDGREEN << "ICP COV: " << icp_cov.diagonal().transpose() << std::endl;
    if (!flag)
        return false;

    /*if (score == 0.0 || score > loopFitnessScoreThreshold || overlap < 0.8) {
        // icp failed

        // get corresspondences
        icp_trans.setIdentity();
        geometry::KDTreeFlann kdtree;
        kdtree.SetGeometry(*target_o3d);

        // get corressponding cloud
        shared_ptr<geometry::PointCloud> corresponding_cloud_source(new geometry::PointCloud());
        shared_ptr<geometry::PointCloud> corresponding_cloud_target(new geometry::PointCloud());

        double error2 = 0.0;

        {
            double error2_private = 0.0;
            for (int i = 0; i < (int) source_o3d->points_.size(); i++) {
                std::vector<int> indices(1);
                std::vector<double> dists(1);
                const auto &point = source_o3d->points_[i];
                if (kdtree.SearchHybrid(point, 1.0, 1, indices, dists) > 0) {
                    error2_private += dists[0];
                    corresponding_cloud_source->points_.push_back(
                            source_o3d->points_.at(i));
                    corresponding_cloud_target->points_.push_back(
                            target_o3d->points_.at(indices[0]));
                }
            }
        }

        size_t corres_number = corresponding_cloud_source->points_.size();
        double fitness_ = (double) corres_number / (double) source_o3d->points_.size();
        double inlier_rmse_ = std::sqrt(error2 / (double) corres_number);

        std::cout << "befroe teaser, rmse and overlap: " << inlier_rmse_ << " " << overlap << std::endl;

        // use teaser
        bool global_flag = coarse_reg_teaser(corresponding_cloud_source, corresponding_cloud_target, icp_trans, 1.0,
                                             10);
        if (global_flag != -1) {
            switch (method) {
                case 0:  // point-to-point icp
                    icp = pipelines::registration::RegistrationICP(
                            *source_o3d, *target_o3d, 1.0, icp_trans,
                            pipelines::registration::TransformationEstimationPointToPoint(),
                            criteria);
                    break;
                case 1:  // Point-to-plane
                    source_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
                    target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
                    icp = pipelines::registration::RegistrationICP(
                            *source_o3d, *target_o3d, 1.0, icp_trans,
                            pipelines::registration::TransformationEstimationPointToPlane(),
                            criteria);
                    break;
                case 2:
                    source_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
                    target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
                    icp = pipelines::registration::RegistrationGeneralizedICP(
                            *source_o3d, *target_o3d, 1.0, icp_trans,
                            pipelines::registration::TransformationEstimationForGeneralizedICP(),
                            criteria);
                    break;
                default:
                    std::cout << "evaluation error type!!!!! " << std::endl;
                    break;
            }
            icp_trans = icp.transformation_;
            score = icp.inlier_rmse_;
            double overlap = icp.fitness_;
            std::cout << BOLDBLUE << "GLOBAL REGSITEARATION BEGIN, quality, score and overlap: " <<
                      global_flag << " " << score << " " << overlap << std::endl;
            if (score == 0.0 ||  score > loopFitnessScoreThreshold || overlap < 0.7)
                return false;

            icp_trans = icp.transformation_;
            score = icp.inlier_rmse_;

            final_trans = icp_trans * final_trans;
            std::cout << BOLDBLUE << "GLOBAL REGSITEARATION success, score and overlap: " << score << " "
                      << icp.fitness_ << std::endl;
        } else {
            std::cout << BOLDBLUE << "GLOBAL REGSITEARATION FAILED!!!!!!!" << std::endl;
            return false;
        }
    }*/
    return true;
}


bool CloudProcess::DoICPDegeneracy(std::vector<Measurement> &keyMeasures,
                                   int loopKeyPre, int loopKeyCur,
                                   float &score, int type,
                                   Eigen::Matrix4d &final_trans) {
    using namespace open3d;
    pcl::PointCloud<PointT>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr targetKeyframeCloud(
            new pcl::PointCloud<PointT>());
    //FindNearKeyframes(keyMeasures, cureKeyframeCloud, loopKeyCur, 0);

    *cureKeyframeCloud = *TransformPointCloud(keyMeasures[loopKeyCur].lidar, keyMeasures[loopKeyCur].updated_pose);
    FindNearKeyframes(keyMeasures, targetKeyframeCloud, loopKeyPre,
                      historyKeyframeSearchNum);

    if (cureKeyframeCloud->size() < 100 || targetKeyframeCloud->size() < 100)
        return false;

    pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
    std::shared_ptr<geometry::PointCloud> source_o3d =
            GetO3dPointCloudFromPCL(*cureKeyframeCloud);
    std::shared_ptr<geometry::PointCloud> target_o3d =
            GetO3dPointCloudFromPCL(*targetKeyframeCloud);
    target_o3d = target_o3d->VoxelDownSample(0.4);
    Eigen::Matrix4d icp_trans = Eigen::Matrix4d::Identity();

    pipelines::registration::RegistrationResult icp;
    auto criteria = pipelines::registration::ICPConvergenceCriteria(30);
    switch (icpO3dType) {
        case 0:  // point-to-point icp
            icp = pipelines::registration::RegistrationICP(
                    *source_o3d, *target_o3d, 1.0, Eigen::Matrix4d::Identity(),
                    pipelines::registration::TransformationEstimationPointToPoint(),
                    criteria);
            break;
        case 1:  // Point-to-plane
            source_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
            target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
            icp = pipelines::registration::RegistrationICP(
                    *source_o3d, *target_o3d, 1.0, Eigen::Matrix4d::Identity(),
                    pipelines::registration::TransformationEstimationPointToPlane(),
                    criteria);
            break;
        case 2:
            source_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
            target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(1.0, 30));
            icp = pipelines::registration::RegistrationGeneralizedICP(
                    *source_o3d, *target_o3d, 1.0, Eigen::Matrix4d::Identity(),
                    pipelines::registration::TransformationEstimationForGeneralizedICP(),
                    criteria);
            break;
        default:
            std::cout << " evaluation error type!!!!! " << std::endl;
            break;
    }


    icp_trans = icp.transformation_;
    score = icp.inlier_rmse_;
    double overlap = icp.fitness_;
    *unused_result = *TransformPointCloud(cureKeyframeCloud, icp_trans);

    final_trans = icp_trans * final_trans;
    if (1) {
        ROS_INFO("LOOP ICP ALIGNED POINTS: %d and %d, %f, %f",
                 cureKeyframeCloud->size(), targetKeyframeCloud->size(), score,
                 overlap);
    }

    if (score > loopFitnessScoreThreshold || overlap < 0.7 || score == 0.0) {
        // do corse to fine icp
        return false;
    }
    return true;
}
