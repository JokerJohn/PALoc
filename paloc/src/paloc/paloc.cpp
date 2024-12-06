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
* Author:  Xiangcheng Hu
* Contact: xhubd@connect.ust.hk
* Affiliation: The Cheng Kar Shun Robotics Institute (CKSRI),
* Hong Kong University of Science and Technology (HKUST)
*
*/

#include "paloc.h"

int main(int argc, char **argv) {
    ROS_INFO("\033[1;32m----> PALoc Started.\033[0m");
    ros::init(argc, argv, "PALoc", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    std::cout << "Computer Cpu Core number: " << thread::hardware_concurrency()
              << std::endl;

    auto paloc = std::make_shared<PALoc>(nh);
    std::thread pose_slam{&PALoc::pose_slam, paloc};
    std::thread viz_map{&PALoc::VisaulizationThread, paloc};
    std::thread lc_detection;
    if (useLoopClosure) {
        std::cout << "USE LOOP OPTIMIZATION!" << std::endl;
        lc_detection = std::thread{&PALoc::LoopDection, paloc};
    }
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    pose_slam.join();
    viz_map.join();

    if (useLoopClosure)
        lc_detection.join();

    return EXIT_SUCCESS;
}


void PALoc::pose_slam() {
    while (1) {
        tic_all.tic();
        if (!SyncData(measurement_curr)) {
            continue;
        }
        if (!isInitialized) {
            // Get intial pose frmo global map
            ROS_ERROR("Init System %d", measurement_curr.lidar->size());
            InitSystem(measurement_curr);

            curr_icp_data
                    << eigen_values_ratio_xyz[0], eigen_values_ratio_xyz[1], eigen_values_ratio_xyz[2], eigen_values_ratio_rpy[0], eigen_values_ratio_rpy[1], eigen_values_ratio_rpy[2],
                    iterate_number, total_rmse, final_fitness;
            keyICPData.push_back(curr_icp_data);
            curr_time_data << t1, t2, t3, t4, t5, t1_all, t2_all, t3_all, t4_all;
            keyTimeData.push_back(curr_time_data);
            continue;
        }
        Pose6D pose_curr = getOdom(measurement_curr.odom);

        odom_pose_prev = odom_pose_curr;
        odom_pose_curr = pose_curr;

        Pose6D dis = diffTransformation(odom_pose_prev, odom_pose_curr);
        PublishPose(ros::Time::now(), pubPoseBToMap, odom_link,
                    Pose6D2Matrix(pose_curr));
        measurement_curr.distance = sqrt(dis.x * dis.x + dis.y * dis.y + dis.z * dis.z);
        measurement_curr.key_pose = pose_curr;
        measurement_curr.updated_pose = pose_curr;

        std::unique_lock<std::mutex> kf_guard(mKF);
        keyMeasures.push_back(measurement_curr);
        kf_guard.unlock();

        curr_node_idx = keyMeasures.size() - 1;
        prev_node_idx = curr_node_idx - 1;
        lioState2 = GetStateFromLIO2(curr_node_idx);
        keyLIOState2.push_back(lioState2);

        /*
         * We do not recommend using the IMU factor provided by GTSAM, for the following reasons:
         * 1. The graph size will significantly increase (by 3x), greatly reducing the optimization speed.
         * Although methods similar to LIO-SAM can be used to speed up the process, it may result in discontinuities.
         * 2.The introduction of IMU pre-integration does not directly improve PGO accuracy.
         * In most cases with LIDAR, integration alone is sufficient.
         * */
        {
            TicToc tic;
            AddOdomFactor();
            t1 = tic.toc();
            t1_all += t1;
        }

        if (useGlobalPrior) {
            /*
             * It is important to note that when LIO fails, PALoc may struggle to continue.
             * In method 1, if a failure occurs, parameter tuning are required, particularly
             * max_correspondences_distance and the map sampling size.
             * Often, localization failures result from incorrect correspondence points,
             * so it's essential to check for map noise (e.g., glass)
             * */
            TicToc tic;
            // 0: directly use open3d gicp
            // 1: use point-to-plane icp, recommended;
            int method_prior = 1;
            if (method_prior == 0)
                AddMapPriorFactorO3D();
            else
                AddMapPriorFactor();
            t2 = tic.toc();
            t2_all += t2;
        }

        {
            /*
             * The gravity factor code is not yet ready, as it is closely tied to the IMU.
             * Different IMU configurations demand high code usability,
             * and I'm still fine-tuning it to achieve the most convenient setup.
             * */
            Vector3 lio_gravity = lioState.block(0, 15, 1, 3).transpose();
            newValues.insert(G(curr_node_idx), gtsam::Point3(lio_gravity * -1));

            TicToc tic;
            bool zupt_flag = ZUPTDetector();
            if (zupt_flag) {
                AddNoMotionFactor();
                // TODO: need to fix this bug
                ZUPTIndexContainer[curr_node_idx] = curr_node_idx;
            }
            t3 = tic.toc();
            t3_all += t3;
        }

        {
            if (useLoopClosure) AddLoopFactor();
        }

        {
            TicToc tic;
            GraphOpt();
            t4 = tic.toc();
            t4_all += t4;
        }

        PubPath();

        t5 = tic_all.toc();
        if (curr_node_idx % 1 == 0) {
            t5_all += t5;
            std::cout << "----Frame: " << curr_node_idx
                      << ", Time: " << t5 << " " << t1
                      << " " << t2 << " " << t3 << " " << t4 << std::endl;
            std::cout << "----Average Time: " << t5_all / curr_node_idx
                      << " " << t1_all / curr_node_idx << " " << t2_all / curr_node_idx << " " << t3_all / curr_node_idx
                      << " " << t4_all / curr_node_idx << std::endl;
        }

        // save process log
        curr_icp_data
                << eigen_values_ratio_xyz[0], eigen_values_ratio_xyz[1], eigen_values_ratio_xyz[2], eigen_values_ratio_rpy[0], eigen_values_ratio_rpy[1], eigen_values_ratio_rpy[2],
                iterate_number, total_rmse, final_fitness;
        keyICPData.push_back(curr_icp_data);
        curr_time_data << t1, t2, t3, t4, t5, t1_all, t2_all, t3_all, t4_all;
        keyTimeData.push_back(curr_time_data);

        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}

void PALoc::InitParmeters() {
    initialPose =
            Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
                    initial_pose_vector.data(), 4, 4);
    std::cout << "load initial pose: \n" << initialPose.matrix() << std::endl;

    // set data saved params
    dataSaverPtr = std::make_unique<DataSaver>(saveDirectory, sequence);
    dataSaverPtr->setExtrinc(useImuFrame, saveResultBodyFrame, t_body_sensor, q_body_sensor);
    dataSaverPtr->setConfigDir(configDirectory);
    dataSaverPtr->setKeyframe(saveKeyFrame);

    // scan_filter_size map_viewer_size map_saved_size
    downSizeFilterMapPGO.setLeafSize(map_viewer_size, map_viewer_size,
                                     map_viewer_size);
    downSizeFilterScan.setLeafSize(scan_filter_size, scan_filter_size,
                                   scan_filter_size);
    downSizeFilterICP.setLeafSize(scan_filter_size * 2, scan_filter_size * 2,
                                  scan_filter_size * 2);

    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointT>());
    kdtreeSurfFromLocalmap.reset(new pcl::KdTreeFLANN<PointT>());
    laserCloudMapPGO.reset(new pcl::PointCloud<PointT>());
    globalmap_ptr.reset(new pcl::PointCloud<PointT>());
    globalmap_filter_ptr.reset(new pcl::PointCloud<PointT>());
    laserCloudEffective.reset(new pcl::PointCloud<PointT>());
    laserCloudEffectiveOri.reset(new pcl::PointCloud<PointT>());
    coeffSel.reset(new pcl::PointCloud<PointT>());
    laserCloudOriSurfVec.resize(128 * 2048);
    coeffSelSurfVec.resize(128 * 2048);
    laserCloudOriSurfFlag.resize(128 * 2048);

    // we need to load point cloud map
    if (useGlobalPrior) {
        TicToc ticToc;
        pcl::io::loadPCDFile((priorMapDirectory + sequence + ".pcd").c_str(), *globalmap_ptr);
        std::cout << BOLDGREEN << "Load map file: " << priorMapDirectory + sequence + ".pcd"
                  << ", " << globalmap_ptr->size() << std::endl;
        if (globalmap_ptr->empty()) {
            std::cout << BOLDRED << "failed to load empy map! Pls check your map file path!!" << std::endl;
            ros::shutdown();
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*globalmap_ptr, *global_map);
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(global_map);
        sor.setLeafSize(map_filter_size, map_filter_size, map_filter_size);
        sor.filter(*globalmap_filter_ptr);
        kdtreeSurfFromMap->setInputCloud(globalmap_filter_ptr);
        std::cout << BOLDGREEN << "Load map size and time: " << globalmap_filter_ptr->size() << ", "
                  << ticToc.toc() << " ms." << std::endl;
    }

    // isam params
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    Vector priorPoseNoiseVector6(6);
    priorPoseNoiseVector6 << 1e-8, 1e-8, 1e-8, 1e-6, 1e-6, 1e-6;
    //    priorPoseNoiseVector6 << 1e0, 1e0, 1e0, 1e2, 1e2, 1e2;
    priorPoseNoise =
            gtsam::noiseModel::Diagonal::Variances(priorPoseNoiseVector6);

    Vector priorLIOPoseNoiseVector6(6);
    priorLIOPoseNoiseVector6 << 1e0, 1e0, 1e0, 1e2, 1e2, 1e2;
    priorLIOPoseNoise = noiseModel::Diagonal::Variances(priorLIOPoseNoiseVector6);
    priorVelNoise = noiseModel::Isotropic::Sigma(3, 1e4);  // m/s
    priorBiasNoise = noiseModel::Isotropic::Sigma(6, 1e-2);  // 1e-2 ~ 1e-3 seems to be good

    // odometry
    Vector odomNoiseVector6(6);
    odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2;
    noise_odom_between = noiseModel::Diagonal::Variances(odomNoiseVector6);

    // global map
    Vector priorMapPoseNoiseVector6(6);
    priorMapPoseNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    priorMapPoseNoise = noiseModel::Diagonal::Variances(priorMapPoseNoiseVector6);

    // zero velocity factor
    zero_velocity_prior_noise_ =
            noiseModel::Isotropic::Sigma(3u, zeroVelocitySigma);
    // no motion factor
    Vector6 sigmas;
    sigmas.head<3>().setConstant(noMotionPositionSigma);
    sigmas.tail<3>().setConstant(noMotionRotationSigma);
    no_motion_prior_noise_ = noiseModel::Diagonal::Sigmas(sigmas);
}

void PALoc::InitSystem(Measurement &measurement) {
    std::cout << "PALoc: waiting for initial pose" << std::endl;
    // estimate gravity
    if (!isInitialized) {
        auto imu_deque = measurement.imu_deque;
        std::vector<sensor_msgs::Imu> imu_measurements;

        //get the imu measurements before curr
        //curr_node % (10) -1 ----  prev_node ----curr_node
        // TODO(xchu): make sure the imu measurements are sorted by timestamp
        while (!imu_deque.empty()) {
            imu_measurements.push_back(imu_deque.front());
            imu_deque.pop_front();
        }
        if (imu_deque.empty()) {
            std::cout << BOLDRED << "imu queue empty!!!!!!!!!!! " << std::endl;
        }
        init_gravity.setZero();
        for (const auto &data : imu_measurements) {
            Eigen::Vector3d acc(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z);
            init_gravity += acc;
        }
        init_gravity /= imu_measurements.size();
    }
    std::cout << "Init gravity ! " << init_gravity.transpose() << std::endl;

    // save the first point cloud
    std::string init_pcd_path = saveDirectory + sequence + "/" + sequence + "_init_cloud.pcd";
    pcl::io::savePCDFileASCII(init_pcd_path, *measurement_curr.lidar);
    std::cout << "Saver init cloud: " << init_pcd_path << std::endl;

    // if we do not use global map, then return directly
    if (!useGlobalPrior) {
        initialPose = Eigen::Matrix4d::Identity();
        isInitialized = true;
        std::cout << "Do not use prior map!!!!" << std::endl;
        return;
    }
    publishCloud(pubLocalizationmap, globalmap_ptr, ros::Time::now(), odom_link);

    pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
    std::shared_ptr<geometry::PointCloud> source_o3d =
            cloud_process_.GetO3dPointCloudFromPCL(*measurement_curr.lidar);

    /** globalmap_filter_ptr is the filterd point cloud of the prior map
     *  if you want to be faster, just use this cloud, but there may need to adjust the icp score
     *  to get system initilization success
     * */
        std::shared_ptr<geometry::PointCloud> target_o3d =
                cloud_process_.GetO3dPointCloudFromPCL(*globalmap_ptr);
    //    std::shared_ptr<geometry::PointCloud> target_o3d =
    //            cloud_process_.GetO3dPointCloudFromPCL(*globalmap_filter_ptr);
    Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
    pipelines::registration::RegistrationResult icp;
    auto criteria = pipelines::registration::ICPConvergenceCriteria(30);

    double max_correspondence_distance = 2.0;
    if (initialPose != Eigen::Matrix4d::Identity()) {
        std::cout << "received initial pose from yaml file: \n" << initialPose.matrix() << std::endl;
        switch (icpO3dType) {
            case 0:  // point-to-point icp
                icp = pipelines::registration::RegistrationICP(
                        *source_o3d, *target_o3d, max_correspondence_distance, initialPose.cast<double>(),
                        pipelines::registration::TransformationEstimationPointToPoint(),
                        criteria);
                break;
            case 1:  // Point-to-plane
                target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(2.0, 10));
                icp = pipelines::registration::RegistrationICP(
                        *source_o3d, *target_o3d, max_correspondence_distance, initialPose.cast<double>(),
                        pipelines::registration::TransformationEstimationPointToPlane(),
                        criteria);
                break;
            case 2:
                target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(2.0, 10));
                icp = pipelines::registration::RegistrationGeneralizedICP(
                        *source_o3d, *target_o3d, max_correspondence_distance, initialPose.cast<double>(),
                        pipelines::registration::
                        TransformationEstimationForGeneralizedICP(),
                        criteria);
                break;
            default:
                std::cout << "ICP TYPE error!!!!! " << std::endl;
                break;
        }
        trans = icp.transformation_;
        double score = icp.inlier_rmse_;
        double overlap = icp.fitness_;
        measurement_curr.global_pose = Matrix2Pose6D(trans);
        measurement_curr.global_score = score;

        // publish the transformed cloud
        *unused_result = *TransformPointCloud(measurement_curr.lidar, trans);
        publishCloud(pubInitialCloud, unused_result, ros::Time::now(), odom_link);
        ROS_INFO("Initial ICP ALIGNED POINTS: %d and %d, %f, %f",
                 measurement_curr.lidar->size(), globalmap_ptr->size(), score, overlap);
        /**** Note: in some case, maybe you need to adjust the open3d icp score*/
        if (score > 0.7 || overlap < 0.7 || score == 0.0) {
            std::cout << "check your initial pose in the yaml file" << std::endl;
            priorPose = initialPose = Eigen::Matrix4d::Identity();
            priorScore = loopFitnessScoreThreshold;
            isInitialized = false;
            return;
        }

        // 使用 EigenMatrixToTensor 转换 Eigen 矩阵到 Open3D Tensor
        open3d::core::Tensor transformation_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(
                icp.transformation_);
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
                icp_cov = ifm_eigen.inverse().cast<float>();
                flag = true;
            } else {
                std::cerr << BOLDRED << "Information matrix is not 6x6. Cannot compute covariance matrix." << std::endl;
            }
        } catch (const std::runtime_error &e) {
            std::cerr << "Runtime error: " << e.what() << std::endl;
            std::cerr
                    << "Check if the point clouds have sufficient correspondences and adjust the max_correspondence_distance parameter if necessary."
                    << std::endl;
            icp_cov = Eigen::Matrix<float, 6, 6>::Identity();
        }
        std::cout << BOLDGREEN << "Open3D ICP COV: " << icp_cov.diagonal().transpose() << std::endl;
        if (!flag) return;

        // System initialized successfully
        initialPose = priorPose = trans.matrix().cast<double>();
        priorScore = score;
        PublishPose(ros::Time::now(), pubPoseOdomToMap, odom_link, priorPose.matrix());
        std::cout << BOLDRED << "Initial pose: " << priorPose.matrix() << std::endl;
        std::cout << BOLDRED << "Initial pose cov: " << icp_cov.diagonal().transpose() << std::endl;
        isInitialized = true;
        return;
    } else {
        // get initial pose from rviz, there may exist some bugs
        if (poseReceived) {
            switch (icpO3dType) {
                case 0:  // point-to-point icp
                    icp = pipelines::registration::RegistrationICP(
                            *source_o3d, *target_o3d, 1.0, initialPose.cast<double>(),
                            pipelines::registration::TransformationEstimationPointToPoint(),
                            criteria);
                    break;
                case 1:  // Point-to-plane
                    target_o3d->EstimateNormals(
                            geometry::KDTreeSearchParamHybrid(1.0, 30));
                    icp = pipelines::registration::RegistrationICP(
                            *source_o3d, *target_o3d, 1.0, initialPose.cast<double>(),
                            pipelines::registration::TransformationEstimationPointToPlane(),
                            criteria);
                    break;
                case 2:
                    target_o3d->EstimateNormals(
                            geometry::KDTreeSearchParamHybrid(1.0, 30));
                    icp = pipelines::registration::RegistrationGeneralizedICP(
                            *source_o3d, *target_o3d, 1.0, initialPose.cast<double>(),
                            pipelines::registration::
                            TransformationEstimationForGeneralizedICP(),
                            criteria);
                    break;
                default:
                    std::cout << " evaluation error type!!!!! " << std::endl;
                    break;
            }
            trans = icp.transformation_;
            double score = icp.inlier_rmse_;
            double overlap = icp.fitness_;
            measurement_curr.global_pose = Matrix2Pose6D(trans);
            measurement_curr.global_score = score;

            *unused_result = *TransformPointCloud(measurement_curr.lidar, trans);
            publishCloud(pubInitialCloud, unused_result, ros::Time::now(), odom_link);
            ROS_INFO("initial ICP ALIGNED POINTS: %d and %d, %f, %f",
                     measurement_curr.lidar->size(), globalmap_ptr->size(), score,
                     overlap);
            if (score > 0.6 || overlap < 0.7 || score == 0.0) {
                poseReceived = false;
                isInitialized = false;
                return;
            }

            // base_link->map
            priorPose = trans.matrix();
            PublishPose(ros::Time::now(), pubPoseOdomToMap, odom_link, priorPose.matrix());
            isInitialized = true;
            std::cout << BOLDRED << "Initial pose: " << priorPose.matrix()
                      << std::endl;
            std::cout << BOLDRED << "Initial LIO pose: " << Pose6D2Matrix(getOdom(measurement_curr.odom)).matrix()
                      << std::endl;
            return;
        }
    }
}

void PALoc::GraphOpt() {
    TicToc toc;
    if (curr_node_idx > 0) {
        newValues.insert(X(curr_node_idx), predict_pose);
        newValues.insert(V(curr_node_idx), prevVel);
        newValues.insert(B(curr_node_idx), prevBias);
    }

    try {
        std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
        isam->update(newFactors, newValues);
        isam->update();
        for (int i = 0; i < 5; ++i) {
            isam->update();
        }
        graph_guard.unlock();
    } catch (gtsam::IndeterminantLinearSystemException &e) {
        ROS_ERROR("FORSTER2 gtsam indeterminate linear system exception!");
        std::cerr << e.what() << std::endl;
    }
    newFactors.resize(0);
    newValues.clear();

    currentEstimate = isam->calculateEstimate();
    prevPose = currentEstimate.at<Pose3>(X(curr_node_idx));
    prevVel = currentEstimate.at<Vector3>(V(curr_node_idx));
    prevBias = currentEstimate.at<imuBias::ConstantBias>(B(curr_node_idx));
    auto prevGravity = currentEstimate.at<gtsam::Point3>(G(curr_node_idx));
    prevState = NavState(prevPose, prevVel);

    // caculate pose cov in world frame
    Marginals marginals = Marginals(isam->getFactorsUnsafe(), currentEstimate, Marginals::CHOLESKY);
    bool useJointMarginal = true;
    if (useJointMarginal) {
        try {
            KeyVector keys = {X(curr_node_idx)}; // Example keys for current and previous poses.
            JointMarginal jointCov = marginals.jointMarginalCovariance(keys);
            // Get the covariance matrix between the current and previous pose.
            gtsam::Matrix6 covMatrix = jointCov.at(keys[0], keys[0]);
            poseCovariance = covMatrix;
            std::cout << "PGO COV: " << poseCovariance.diagonal().transpose() << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "Error computing joint marginal covariance: " << e.what() << std::endl;
        }
    }
    if (1) {
        std::cout << "Graph Optimizing Time: " << toc.toc() << " ms" << std::endl;
    }

    std::unique_lock<std::mutex> kf_guard(mKF);
    for (int i = 0; i < keyMeasures.size(); i++) {
        Pose3 pose = currentEstimate.at<Pose3>(X(i));
        Vector3 vel = currentEstimate.at<Vector3>(V(i));
        imuBias::ConstantBias bias = currentEstimate.at<imuBias::ConstantBias>(B(i));

        Pose6D &p = keyMeasures.at(i).updated_pose;
        p.x = pose.translation().x();
        p.y = pose.translation().y();
        p.z = pose.translation().z();
        p.roll = pose.rotation().roll();
        p.pitch = pose.rotation().pitch();
        p.yaw = pose.rotation().yaw();
        p.setBias(bias.vector()[0], bias.vector()[1], bias.vector()[2],
                  bias.vector()[3], bias.vector()[4], bias.vector()[5]);
        p.vx = vel.x();
        p.vy = vel.y();
        p.vz = vel.z();

        KeyVector keys = {X(i)}; // Example keys for current and previous poses.
        JointMarginal jointCov2 = marginals.jointMarginalCovariance(keys);
        gtsam::Matrix6 covMatrix = jointCov2.at(X(i), X(i));
        p.pose_cov = covMatrix;
    }
    kf_guard.unlock();

    aLoopIsClosed = false;
    aGlobalConstrained = false;
}


void PALoc::AddOdomFactor() {
    Pose3 curr_lio_pose = Pose6dTogtsamPose3(keyMeasures.at(curr_node_idx).key_pose);
    Matrix6 pose_cov = lioState2.cov;

    if (curr_node_idx == 0) {
        newValues.insert(V(0), Vector3(0, 0, 0));
        newValues.insert(B(0), imuBias::ConstantBias());
        if (useGlobalPrior) {
            Pose3 global_pose = Pose6dTogtsamPose3(Matrix2Pose6D(priorPose));
            priorMapPoseGaussianNoise = noiseModel::Gaussian::Covariance(icp_cov.matrix().cast<double>());
            PriorFactor<Pose3> map_pose_factor(X(curr_node_idx), global_pose, priorMapPoseGaussianNoise);
            std::unique_lock<std::mutex> graph_guard_1(mtxPosegraph);
            newFactors.add(map_pose_factor);
            graph_guard_1.unlock();
            newValues.insert(X(0), global_pose);
            std::cout << BOLDRED << "ADD Initial pose factor: " << icp_cov.diagonal().transpose() << std::endl;
        } else {
            // only lio, we must fix the first node
            noiseModel::Gaussian::shared_ptr noise_model = noiseModel::Gaussian::Covariance(pose_cov);
            newValues.insert(X(0), curr_lio_pose);
            newFactors.emplace_shared<PriorFactor<Pose3 >>(X(0), curr_lio_pose, noise_model);
        }
        newFactors.emplace_shared<PriorFactor<Vector3 >>(V(0), Vector3(0, 0, 0), priorVelNoise);
        newFactors.emplace_shared<PriorFactor<imuBias::ConstantBias >>(B(0), imuBias::ConstantBias(),
                                                                       priorBiasNoise);
    } else {
        Pose3 prev_lio_pose = Pose6dTogtsamPose3(keyMeasures.at(prev_node_idx).key_pose);
        rel_pose = prev_lio_pose.between(curr_lio_pose);
        predict_pose = prevPose.compose(rel_pose);

        // Adjoint map at relative pose (inverse)
        Matrix6 Adj = rel_pose.inverse().AdjointMap();
        Matrix6 cov1 = keyLIOState2.at(prev_node_idx).cov;
        Matrix6 cov2 = pose_cov;
        // cov propagate, equation 14, but error exits in the paper, we have not fix yet.
        Matrix6 relative_cov = Adj * cov1 * Adj.transpose() + cov2;
        noiseModel::Gaussian::shared_ptr gau_noise_model = noiseModel::Gaussian::Covariance(relative_cov);
        std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
        newFactors.emplace_shared<BetweenFactor<Pose3 >>(X(prev_node_idx), X(curr_node_idx), rel_pose, gau_noise_model);
        graph_guard.unlock();

        if (0) {
            std::cout << "CURR COV:" << pose_cov.diagonal().transpose() << std::endl;
            std::cout << "REL COV: " << relative_cov.diagonal().transpose() << std::endl;
        }
    }
}

void PALoc::AddMapPriorFactorO3D() {
    if (curr_node_idx < 1) return;
    TicToc tic_toc;
    bool flag = GetGlobalICP(keyMeasures.at(curr_node_idx));
    if (flag) {
        gtsam::Pose3 poseGlobal =
                Pose6dTogtsamPose3(keyMeasures.at(curr_node_idx).global_pose);
        priorMapPoseGaussianNoise = noiseModel::Gaussian::Covariance(icp_cov.matrix().cast<double>());
        std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
        newFactors.emplace_shared<PriorFactor<Pose3 >>(X(curr_node_idx), poseGlobal, priorMapPoseGaussianNoise);
        graph_guard.unlock();
        mapIndexContainer[curr_node_idx] = curr_node_idx;
    }
    std::cout << BOLDGREEN << "Global ICP time: " << tic_toc.toc() << " [ms]"
              << std::endl;
}

void PALoc::AddMapPriorFactor() {
    if (curr_node_idx < 1) return;

    TicToc tic_toc;
    pcl::PointCloud<PointT>::Ptr raw_cloud(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*keyMeasures.at(curr_node_idx).lidar, *raw_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map = extractLocalMap(globalmap_filter_ptr, predict_pose.matrix(),
                                                                     mapRadius);
    kdtreeSurfFromMap->setInputCloud(local_map);
    std::cout << "Extracted localmap points: " << local_map->size() << ", " << globalmap_filter_ptr->size()
              << std::endl;
    std::cout << "Extracted localmap cost: " << tic_toc.toc() << " ms." << std::endl;

    double t1 = tic_toc.toc();
    Pose6D predict_pose6d = Matrix2Pose6D(predict_pose.matrix());
    relative_fitness = 0.0;
    relative_rmse = std::numeric_limits<double>::max();
    bool isOptimzed = Point2PlaneICPLM(raw_cloud, local_map, predict_pose6d, correspondenceDis);
    std::cout << "ICP COV:" << icp_cov.diagonal().transpose() << std::endl;

    /** Important: you must make sure the icp result is correct, or the grpah will crashed */
    if (isOptimzed) {
        // update the optimized pose
        Pose3 final_pose = Pose6dTogtsamPose3(predict_pose6d);
        keyMeasures.at(curr_node_idx).global_pose = predict_pose6d;
        keyMeasures.at(curr_node_idx).global_pose.valid = !isDegenerate;
        keyMeasures.at(curr_node_idx).global_score = total_rmse;

        // add map factor
        priorMapPoseGaussianNoise = noiseModel::Gaussian::Covariance(icp_cov.matrix().cast<double>());
        PriorFactor<Pose3> map_pose_factor(X(curr_node_idx), final_pose, priorMapPoseGaussianNoise);
        std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
        newFactors.add(map_pose_factor);
        graph_guard.unlock();

        aGlobalConstrained = true;
        mapIndexContainer[curr_node_idx] = curr_node_idx;
        std::cout << BOLDGREEN << "Global ICP SUCCESS: " << total_rmse << " " << final_fitness << " "
                  << iterate_number << std::endl;
    } else {
        keyMeasures.at(curr_node_idx).global_pose.valid = false;
        std::cout << BOLDCYAN << "Global ICP failed: " << total_rmse << " " << final_fitness << " "
                  << iterate_number << std::endl;
    }

}


pcl::PointCloud<pcl::PointXYZI>::Ptr
PALoc::extractLocalMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &map,
                       const Eigen::Matrix4d &initial_pose_matrix,
                       float radius) {
    // Extract local map
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_map(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*map, *temp_map);
    //    pcl::VoxelGrid<pcl::PointXYZI> sor;
    //    sor.setInputCloud(temp_map);
    //    sor.setLeafSize(0.1, 0.1, 0.1);
    //    sor.filter(*temp_map);

    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZI>);
    // Convert Matrix4d to Affine3f for CropBox compatibility
    Eigen::Matrix3f rotation = initial_pose_matrix.block<3, 3>(0, 0).cast<float>();
    Eigen::Vector3f translation = initial_pose_matrix.block<3, 1>(0, 3).cast<float>();
    Eigen::Affine3f initial_pose;
    initial_pose.linear() = rotation;
    initial_pose.translation() = translation;
    // Define the bounding box dimensions
    Eigen::Vector4f min_pt(-radius, -radius, -std::numeric_limits<float>::infinity(), 1.0);
    Eigen::Vector4f max_pt(radius, radius, std::numeric_limits<float>::infinity(), 1.0);
    // Create the CropBox filter
    pcl::CropBox<pcl::PointXYZI> box_filter;
    box_filter.setInputCloud(temp_map);
    box_filter.setMin(min_pt);
    box_filter.setMax(max_pt);
    box_filter.setTranslation(initial_pose.translation());
    box_filter.setRotation(initial_pose.rotation().eulerAngles(0, 1, 2));
    box_filter.filter(*local_map);
    return local_map;
}


void PALoc::AddNoMotionFactor() {
    if (curr_node_idx < 1) return;

    std::unique_lock<std::mutex> graph_guard1(mtxPosegraph);
    std::cout << BOLDRED << "add zero velocity factor" << std::endl;
    newFactors.emplace_shared<PriorFactor<Vector3 >>(
            V(curr_node_idx), Vector3::Zero(), zero_velocity_prior_noise_);
    graph_guard1.unlock();

    // add no motion factor
    std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
    newFactors.emplace_shared<BetweenFactor<Pose3 >>(
            X(prev_node_idx), X(curr_node_idx), Pose3::Identity(),
            no_motion_prior_noise_);
    graph_guard.unlock();
}

void PALoc::EstimateGravity() {
    if (curr_node_idx < 1) return;

    auto curr_lidar_time = keyMeasures.at(curr_node_idx).lidar_time;
    auto imu_deque = keyMeasures.at(curr_node_idx).imu_deque;
//     std::vector<sensor_msgs::Imu> imu_measurements;
    deque<sensor_msgs::Imu> imu_measurements;

    // get the imu measurements before curr
    //  curr_node % (10) -1 ----  prev_node ----curr_node
    // TODO(xchu): make sure the imu measurements are sorted by timestamp
    for (int i = curr_node_idx - node_rate + 1; i <= curr_node_idx; i++) {
        auto imu_deque_temp = keyMeasures.at(i).imu_deque;

        while (!imu_deque_temp.empty()) {
            if (imu_deque_temp.front().header.stamp.toSec() <= curr_lidar_time) {
                imu_measurements.push_back(imu_deque_temp.front());
                imu_deque_temp.pop_front();
            } else {
                break;
            }
        }
    }

//    if (1) {
//        std::cout << BOLDRED << "sync lidar-imu-front-end time: "
//                  /*<< previous_lidar_time << ", "*/ << curr_lidar_time << ", "
//                  << imu_measurements.front().header.stamp.toSec() << ", "
//                  << imu_measurements.back().header.stamp.toSec() << ", "
//                  << imu_measurements.size() << std::endl;
//    }


    if (imu_deque.empty() || imu_measurements.empty()) {
        std::cout << BOLDRED << "imu queue empty!!!!!!!!!!! " << std::endl;
        return;
    }

// Simulate the accelerometer and gyroscope data from STIM300.
    std::vector<Vector3d> accel_data;
    std::vector<Vector3d> gyro_data;
    std::vector<double> dt_data;


    imuBias::ConstantBias bias = currentEstimate.at<imuBias::ConstantBias>(B(prev_node_idx));

    std::cout << "imu bias: " << bias << std::endl;

    while (!imu_measurements.empty()) {
        // pop and integrate imu data that is between two optimizations
        sensor_msgs::Imu *thisImu = &imu_measurements.front();
        double imuTime = thisImu->header.stamp.toSec();
        // std::cout << " delta_t: " << imuTime -lastImuT_opt << std::endl;
        if (imuTime < curr_lidar_time) {
            double dt = (lastImuTime < 0) ? (1.0 / 200.0) : (imuTime - lastImuTime);

            Vector3 acc_curr(thisImu->linear_acceleration.x - b_acc_cov,
                             thisImu->linear_acceleration.y - b_acc_cov,
                             thisImu->linear_acceleration.z - b_acc_cov);
            Vector3 gyro_curr(thisImu->angular_velocity.x - b_gyr_cov,
                              thisImu->angular_velocity.y - b_gyr_cov,
                              thisImu->angular_velocity.z - b_gyr_cov);


//            Vector3 acc_curr(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y,
//                             thisImu->linear_acceleration.z);
//            Vector3 gyro_curr(thisImu->angular_velocity.x, thisImu->angular_velocity.y,
//                              thisImu->angular_velocity.z);

            accel_data.push_back(acc_curr);
            gyro_data.push_back(gyro_curr);
            dt_data.push_back(dt);

            lastImuTime = imuTime;
            imu_measurements.pop_front();
        } else
            break;
    }

    std::cout << "raw imu data: " << accel_data[0].transpose() << std::endl;



    // Initialize variables for the current orientation and gravity estimate.
    Quaterniond q = Quaterniond::Identity();
    GravityKalmanFilter kalman_filter(3);
    // Process the accelerometer and gyroscope data to estimate the gravity direction.
    for (size_t i = 0; i < accel_data.size(); ++i) {
        // Integrate the gyroscope data to update the current orientation.
        Vector3d omega = gyro_data[i];
        q = integrateGyroscope(q, omega, dt_data[i]);
        // Compensate for the gravity component in the accelerometer data.
        Vector3d accel_body = accel_data[i];
        Vector3d accel_global = q * accel_body - Vector3d(0, 0, 9.81);

        //std::cout << "raw imu data by integrateGyroscope: " << accel_global.transpose() << std::endl;

        // Set up the Kalman filter matrices.
        MatrixXd F = MatrixXd::Identity(3, 3);
        MatrixXd H = MatrixXd::Identity(3, 3);
        MatrixXd Q = MatrixXd::Identity(3, 3) * 1e-2; // Process noise covariance
        MatrixXd R = MatrixXd::Identity(3, 3) * 1e-4; // Measurement noise covariance
        // Perform the Kalman filter prediction step.
        kalman_filter.predict(F, Q);
        // Perform the Kalman filter update step using the compensated accelerometer data.
        kalman_filter.update(accel_global, H, R);
        // Get the gravity estimate from the Kalman filter state.
//        Vector3d gravity_estimate = kalman_filter.x.normalized();
        measured_gravity = kalman_filter.x.normalized();
    }
    std::cout << "Gravity estimate: " << measured_gravity.transpose() << std::endl;

}


Eigen::Vector3d PALoc::estimateGravity(const int n) {
//    auto curr_lidar_time = keyMeasures.at(curr_node_idx).lidar_time;
    auto imu_deque = keyMeasures.at(curr_node_idx).imu_deque;
    std::vector<sensor_msgs::Imu> imu_measurements;
    //deque<sensor_msgs::Imu> imu_measurements;

    //get the imu measurements before curr
    //curr_node % (10) -1 ----  prev_node ----curr_node
    // TODO(xchu): make sure the imu measurements are sorted by timestamp
    //  for (int i = curr_node_idx - node_rate + 1; i <= curr_node_idx; i++) {
    for (int i = curr_node_idx - n; i <= curr_node_idx; i++) {
        auto imu_deque_temp = keyMeasures.at(i).imu_deque;
        while (!imu_deque_temp.empty()) {
            imu_measurements.push_back(imu_deque_temp.front());
            imu_deque_temp.pop_front();
        }
    }

    if (imu_deque.empty()) {
        std::cout << BOLDRED << "imu queue empty!!!!!!!!!!! " << std::endl;
    }

    Eigen::Vector3d gravity(0, 0, 0);
    for (const auto &data : imu_measurements) {
        Eigen::Vector3d acc(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z);
        gravity += acc;
    }
    gravity /= imu_measurements.size();
    return gravity;
}

Eigen::Quaterniond
PALoc::integrateGyroscope(const Eigen::Quaterniond &q, const Eigen::Vector3d &omega, double dt) {
    double norm_omega = omega.norm();
    if (norm_omega > 1e-5) {
        double theta = norm_omega * dt * 0.5;
        Eigen::Quaterniond delta_q(cos(theta), sin(theta) * omega.x() / norm_omega,
                                   sin(theta) * omega.y() / norm_omega,
                                   sin(theta) * omega.z() / norm_omega);
        return (q * delta_q).normalized();
    }
    return q;
}

void PALoc::AddGravityFactor() {
    if (curr_node_idx < 10) return;
    // Add the gravity constraint factor.
    // Here, we assume that the measured_gravity vector has been computed from the IMU data.
    // Vector3 measured_gravity2(0, 0, 1);
    Eigen::Vector3d estimated_gravity = estimateGravity(1);
    std::cout << "Estimated gravity: " << estimated_gravity.transpose() << std::endl;
    std::cout << BOLDRED << "add gravity factor" << std::endl;
    noiseModel::Diagonal::shared_ptr gravity_noise = noiseModel::Diagonal::Variances(
            Vector4(1e-2, 1e-2, 1e-4, 1e-6));

    // 假设传感器规格书或实验中得到了加速度计在各个轴向上的噪声标准偏差
//    Vector3 acc_stddev; // 加速度计噪声标准偏差
//    acc_stddev << acc_cov, acc_cov, acc_cov;
//    // 将标准偏差转换为协方差矩阵
//    Matrix3 acc_cov = acc_stddev.array().square().matrix().asDiagonal();
//    noiseModel::Gaussian::shared_ptr gravity_noise = noiseModel::Gaussian::Covariance(acc_cov);

    std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
    newFactors.emplace_shared<StaticGravityFactor>(X(curr_node_idx), estimated_gravity, gravity_noise);
    //newFactors.emplace_shared<GravityFactorAuto>(X(curr_node_idx), estimated_gravity, gravity_noise);
    //    newFactors.emplace_shared<GravityFactor2>(X(curr_node_idx), G(curr_node_idx), gravity, gravity_noise);
    graph_guard.unlock();
}

void PALoc::AddLoopFactor() {
    if (loopIndexQueue.empty() || curr_node_idx < 1) return;

    std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
    for (int i = 0; i < loopIndexQueue.size(); ++i) {
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Gaussian::shared_ptr noiseBetween = loopGaussianNoiseQueue[i];
        newFactors.emplace_shared<BetweenFactor<Pose3 >>(X(indexFrom), X(indexTo),
                                                         poseBetween, noiseBetween);
    }
    graph_guard.unlock();

    std::unique_lock<std::mutex> loop_guard(mtxLoopContainer);
    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopGaussianNoiseQueue.clear();
    loop_guard.unlock();
    aLoopIsClosed = true;
}

bool PALoc::ZUPTDetector() {
    if (curr_node_idx < 1) return false;

    // motion state from lio
    Pose6D df = diffTransformation(odom_pose_prev, odom_pose_curr);
    double aver_translation = sqrt(df.x * df.x + df.y * df.y + df.z * df.z) / 3;
    double aver_rotation = sqrt(df.roll + df.pitch + df.yaw) / 3;
    if (0) {
        std::cout << "average lio translation and angle: " << aver_translation
                  << ", " << aver_rotation << std::endl;
    }

    auto imu_queue = keyMeasures.at(curr_node_idx).imu_deque;
    if (imu_queue.empty()) return false;

    Eigen::Vector3d gyro_delta_average, acc_deltea_average;
    Eigen::Vector3d acc_average, gyro_average;
    Eigen::Vector3d acc_diff_max, gyro_diff_max;
    Eigen::Vector3d acc_diff_min, gyro_diff_min;

    acc_diff_max << imu_queue.at(0).linear_acceleration.x,
            imu_queue.at(0).linear_acceleration.y,
            imu_queue.at(0).linear_acceleration.z;
    acc_diff_min = acc_diff_max;

    gyro_diff_max << imu_queue.at(0).angular_velocity.x,
            imu_queue.at(0).angular_velocity.y, imu_queue.at(0).angular_velocity.z;
    gyro_diff_min = gyro_diff_max;

    for (auto it_imu = imu_queue.begin(); it_imu < (imu_queue.end() - 1);
         it_imu++) {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        Eigen::Vector3d gyro, acc;
        gyro << head.angular_velocity.x, head.angular_velocity.y,
                head.angular_velocity.z;
        acc << head.linear_acceleration.x, head.linear_acceleration.y,
                head.linear_acceleration.z;
        acc_average += acc;
        gyro_average += gyro;

        acc_diff_max = max(acc_diff_max, acc);
        gyro_diff_max = max(gyro_diff_max, gyro);
        acc_diff_min = min(acc_diff_min, acc);
        gyro_diff_min = min(gyro_diff_min, gyro);

        Eigen::Vector3d gyro_delta, acc_delta;
        gyro_delta << abs(tail.angular_velocity.x - head.angular_velocity.x),
                abs(tail.angular_velocity.y - head.angular_velocity.y),
                abs(tail.angular_velocity.z - head.angular_velocity.z);
        gyro_delta_average += gyro_delta;

        acc_delta << abs(head.linear_acceleration.x - tail.linear_acceleration.x),
                abs(head.linear_acceleration.y - tail.linear_acceleration.y),
                abs(head.linear_acceleration.z - tail.linear_acceleration.z);
        acc_deltea_average += acc_delta;
    }
    gyro_delta_average = gyro_delta_average / (imu_queue.size() - 1);
    acc_deltea_average = acc_deltea_average / (imu_queue.size() - 1);

    // motion state from raw imu data
    acc_average /= (imu_queue.size() - 1);
    gyro_average /= (imu_queue.size() - 1);
    double accel_norm = std::sqrt(
            std::pow(acc_average.x(), 2) +
            std::pow(acc_average.y(), 2) +
            std::pow(acc_average.z(), 2)
    );
    double ang_vel_norm = std::sqrt(
            std::pow(gyro_average.x(), 2) +
            std::pow(gyro_average.y(), 2) +
            std::pow(gyro_average.z(), 2)
    );

    if (0) {
        std::cout << "average acc gyro: " << accel_norm << ", "
                  << ang_vel_norm << std::endl;
        std::cout << "acc_min max: " << acc_diff_min.transpose() << ", "
                  << acc_diff_max.transpose() << std::endl;
        std::cout << "gyro min max: " << gyro_diff_min.transpose() << ", "
                  << gyro_diff_max.transpose() << std::endl;
        std::cout << "delta acc : " << acc_deltea_average.transpose() << ", "
                  << gyro_delta_average.transpose() << std::endl;
    }

    if ((acc_diff_max.x() > 0.01) || (acc_diff_max.y() > 0.01) ||
        (acc_diff_max.z() > 0.01) || (gyro_diff_max.x() > 0.025) ||
        (gyro_diff_max.y() > 0.025) || (gyro_diff_max.z() > 0.025)) {
        // return NONSTATIONARY;
    } else {
        // return STATIONARY;
    }

    //    0.001122458 0.001350456 0.001252326, 0.000002450 0.000026449
    //    0.000050444 0.001134773 0.001257556 0.001234070, 0.000000701
    //    0.000074772 0.000285980 0.001669844 0.003446238 0.002880567,
    //    0.000349555 0.000697232 0.000230081 0.001529575 0.003009357
    //    0.003420752, 0.001184196 0.000416309 0.000945308
    //
    //    0.002358743 0.003992048 0.003210434, 0.001922719 0.000934098
    //    0.008701938 0.001891771 0.003163065 0.003844705, 0.000511198
    //    0.000048277 0.006157734 0.001122092 0.003563314 0.002682264,
    //    0.000727486 0.000303568 0.003597719

    double acc_thro = 0.003, anguler_thro = 0.003;
    //  bool zupt_flag =
    //      (angvel_avr[0] < anguler_thro && angvel_avr[1] < anguler_thro &&
    //       angvel_avr[2] < anguler_thro) ||
    //      (acc_avr[0] < acc_thro && acc_avr[1] < acc_thro && acc_avr[2] <
    //      acc_thro);
    // }
    bool zupt_flag = (aver_translation < 0.01 && aver_rotation < 0.015);
    return zupt_flag;
}


bool PALoc::SyncData(Measurement &measurement) {
    if (odometryBuf.empty() || fullResBuf.empty()) {
        return false;
    }

    // we need to align lidar and odmetry data
    std::unique_lock<std::mutex> data_guard(mutexLock);
    double odom_time = odometryBuf.front()->header.stamp.toSec();
    double cloud_time = fullResBuf.front()->header.stamp.toSec();
    while (!odometryBuf.empty() && (odom_time < cloud_time - 0.5 * 0.1)) {
        ROS_WARN("odometry discarded: %f", odom_time - cloud_time);
        odometryBuf.pop();
        break;
    }
    while (!fullResBuf.empty() && (cloud_time < odom_time - 0.5 * 0.1)) {
        ROS_WARN("pointCloud discarded: %f",
                 cloud_time - odom_time);
        fullResBuf.pop();
        break;
    }

    if (fullResBuf.empty() || odometryBuf.empty()) {
        data_guard.unlock();
        return false;
    }
    sensor_msgs::PointCloud2 raw_cloud_msg = *(fullResBuf.front());
    auto thisOdom = odometryBuf.front();
    fullResBuf.pop();
    odometryBuf.pop();
    data_guard.unlock();

    pcl::PointCloud<PointT>::Ptr thisKeyFrame(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(raw_cloud_msg, *thisKeyFrame);

    // crop cloud
    pcl::PointCloud<PointT>::Ptr thisKeyFrameTemp(new pcl::PointCloud<PointT>());
    thisKeyFrameTemp = cloud_process_.RemoveRangeCloud(
            thisKeyFrame, Eigen::Vector3i(1, 1, 1), Eigen::Vector3d(80, 80, 80), ">");


    // all theae cloud must in imu body frame
    if (useRawCloud) {
        measurement.lidar = thisKeyFrameTemp;
    } else {
        pcl::PointCloud<PointT>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointT>());
        downSizeFilterScan.setInputCloud(thisKeyFrameTemp);
        downSizeFilterScan.filter(*thisKeyFrameDS);
        measurement.lidar = thisKeyFrameDS;
    }
    measurement.lidar_time = cloud_time;
    measurement.odom_time = odom_time;
    measurement.odom = *thisOdom;

    /*** push imu data, and pop from imu buffer ***/
    //  measurement.imu_deque.clear();
    //  std::unique_lock<std::mutex> imu_guard(imuLock);
    //  double imu_time = imuQueue.front().header.stamp.toSec();
    //  while ((!imuQueue.empty()) && (imu_time < cloud_time)) {
    //    imu_time = imuQueue.front().header.stamp.toSec();
    //    if (imu_time > cloud_time) break;
    //    measurement.imu_deque.push_back(imuQueue.front());
    //    imuQueue.pop_front();
    //  }
    //  imu_guard.unlock();
    //  last_imu_deque_ = keyMeasures.back().imu_deque;

    measurement.imu_deque.clear();
    while (!imuQueue.empty()) {
        std::unique_lock<std::mutex> guard(imuLock);
        if (imuQueue.front().header.stamp.toSec() < cloud_time) {
            measurement.imu_deque.push_back(imuQueue.front());
            imuQueue.pop_front();
            guard.unlock();
        } else {
            guard.unlock();
            break;
        }
    }

    if (0) {
        // debug, chech the lidar frame fall into the consective imu frame
        std::cout << BOLDRED << "imu time stamp: "
                  << measurement.imu_deque.front().header.stamp.toSec() << " "
                  << cloud_time << " "
                  << measurement.imu_deque.back().header.stamp.toSec() << std::endl;
        //    std::cout << BOLDWHITE << "filtered cloud size: " <<
        //              thisKeyFrameTemp->size()
        //              << ", " << measurement.lidar->size() << std::endl;
    }

    // align optimized imu pose
    measurement.has_imu = false;
    while (!odometryIMUBuf.empty()) {
        std::unique_lock<std::mutex> guard(mutexLock);
        //        std::cout << "imu odom time: " << odometryIMUBuf.front()->header.stamp.toSec() << " " << cloud_time
        //                  << " " << odom_time << std::endl;
        if (odometryIMUBuf.front()->header.stamp.toSec() < cloud_time - 0.1) {
            // message too old
            odometryIMUBuf.pop();
            guard.unlock();
        } else if (odometryIMUBuf.front()->header.stamp.toSec() > cloud_time + 0.1) {
            // message too new
            guard.unlock();
            break;
        } else {
            measurement.imu_odom = *odometryIMUBuf.front();
            measurement.has_imu = true;
            odometryIMUBuf.pop();
            guard.unlock();
        }
    }

    return true;
}

void PALoc::PubPath() {
    // pub odom and path
    nav_msgs::Odometry odomAftPGO, localizationOdom;
    nav_msgs::Path pathAftPGO, pathLocalization, pathLIO, pathIMU;
    pathAftPGO.header.frame_id = odom_link;
    pathLIO.header.frame_id = odom_link;
    pathIMU.header.frame_id = odom_link;
    odomAftPGO.header.frame_id = odom_link;
    localizationOdom.header.frame_id = odom_link;
    pathLocalization.header.frame_id = odom_link;
    odomAftPGO.child_frame_id = "/aft_pgo";

    for (int node_idx = 0; node_idx < keyMeasures.size(); node_idx++) {
        const Pose6D pose_est = keyMeasures.at(node_idx).updated_pose;
        const Pose6D pose_loc = keyMeasures.at(node_idx).global_pose;
        const Pose6D odom_pose = keyMeasures.at(node_idx).key_pose;

        Pose6D odom_pose_trans;
        if (useGlobalPrior) {
            odom_pose_trans = Matrix2Pose6D(priorPose * Pose6D2Matrix(odom_pose));
        } else
            odom_pose_trans = odom_pose;

        odomAftPGO.header.stamp =
                ros::Time().fromSec(keyMeasures.at(node_idx).odom_time);
        odomAftPGO.pose.pose.position.x = pose_est.x;
        odomAftPGO.pose.pose.position.y = pose_est.y;
        odomAftPGO.pose.pose.position.z = pose_est.z;
        odomAftPGO.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGO.pose.covariance.at(0) = isDegenerate;

        geometry_msgs::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGO.header;
        poseStampAftPGO.pose = odomAftPGO.pose.pose;
        pathAftPGO.header.stamp = odomAftPGO.header.stamp;
        pathAftPGO.poses.push_back(poseStampAftPGO);

        // show lio trajectory
        geometry_msgs::PoseStamped poseStampLIO;
        poseStampLIO.header = odomAftPGO.header;
        poseStampLIO.pose.position.x = odom_pose_trans.x;
        poseStampLIO.pose.position.y = odom_pose_trans.y;
        poseStampLIO.pose.position.z = odom_pose_trans.z;
        poseStampLIO.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                odom_pose_trans.roll, odom_pose_trans.pitch, odom_pose_trans.yaw);
        pathLIO.header.stamp = odomAftPGO.header.stamp;
        pathLIO.poses.push_back(poseStampLIO);

        if (keyMeasures.at(node_idx).global_score != 0) {
            localizationOdom.header.stamp =
                    ros::Time().fromSec(keyMeasures.at(node_idx).odom_time);
            localizationOdom.pose.pose.position.x = pose_loc.x;
            localizationOdom.pose.pose.position.y = pose_loc.y;
            localizationOdom.pose.pose.position.z = pose_loc.z;
            localizationOdom.pose.pose.orientation =
                    tf::createQuaternionMsgFromRollPitchYaw(pose_loc.roll, pose_loc.pitch,
                                                            pose_loc.yaw);
            localizationOdom.pose.covariance.at(0) = pose_loc.cov(0, 0);
            localizationOdom.pose.covariance.at(7) = pose_loc.cov(0, 1);
            localizationOdom.pose.covariance.at(14) = pose_loc.cov(0, 2);
            localizationOdom.pose.covariance.at(21) = pose_loc.cov(0, 3);
            localizationOdom.pose.covariance.at(28) = pose_loc.cov(0, 4);
            localizationOdom.pose.covariance.at(35) = pose_loc.cov(0, 5);

            geometry_msgs::PoseStamped poseStampLoc;
            poseStampLoc.header = localizationOdom.header;
            poseStampLoc.pose = localizationOdom.pose.pose;
            pathLocalization.header.stamp = localizationOdom.header.stamp;
            pathLocalization.poses.push_back(poseStampLoc);
        }
    }

    if (showDegenercy) {
        // show condition number
        odomAftPGO.pose.covariance.at(0) = eigen_values_ratio_xyz[0];
        odomAftPGO.pose.covariance.at(7) = eigen_values_ratio_xyz[1];
        odomAftPGO.pose.covariance.at(14) = eigen_values_ratio_xyz[2];
        odomAftPGO.pose.covariance.at(21) = eigen_values_ratio_rpy[0];
        odomAftPGO.pose.covariance.at(28) = eigen_values_ratio_rpy[1];
        odomAftPGO.pose.covariance.at(35) = eigen_values_ratio_rpy[2];
    } else {
        // show pose uncertainty
        // 调整poseCovariance的列顺序
        Eigen::MatrixXd adjustedCovariance(6, 6);
        adjustedCovariance.block<3, 3>(0, 0) = poseCovariance.block<3, 3>(3, 3); // 平移部分
        adjustedCovariance.block<3, 3>(0, 3) = poseCovariance.block<3, 3>(3, 0); // 平移与旋转之间的协方差
        adjustedCovariance.block<3, 3>(3, 0) = poseCovariance.block<3, 3>(0, 3); // 旋转与平移之间的协方差
        adjustedCovariance.block<3, 3>(3, 3) = poseCovariance.block<3, 3>(0, 0); // 旋转部分
        // 将调整后的矩阵赋值给odomAftPGO.pose.covariance
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                odomAftPGO.pose.covariance.at(i * 6 + j) = adjustedCovariance(i, j);
            }
        }
    }

    pubOdomAftPGO.publish(odomAftPGO);
    pubOdomAftGlobal.publish(localizationOdom);
    pubPathAftPGO.publish(pathAftPGO);
    pubLocalizationPath.publish(pathLocalization);
    pubPathLIO.publish(pathLIO);
    if (pubPathLIO.getNumSubscribers() != 0) {
        pathIMU.header.stamp = ros::Time().fromSec(keyMeasures.at(curr_node_idx).lidar_time);
        pathIMU.header.frame_id = odom_link;
        pubPathIMU.publish(pathIMU);
    }


    // publish current cloud
    pcl::PointCloud<PointT>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointT>);
    *transform_cloud_ptr = *TransformPointCloud(laserCloudEffective, keyMeasures[curr_node_idx].updated_pose);
    publishCloud(pubLaserCloudSurround, transform_cloud_ptr,
                 ros::Time().fromSec(keyMeasures.at(curr_node_idx).odom_time), odom_link);

    // send transform
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftPGO.pose.pose.position.x,
                                    odomAftPGO.pose.pose.position.y,
                                    odomAftPGO.pose.pose.position.z));
    q.setW(odomAftPGO.pose.pose.orientation.w);
    q.setX(odomAftPGO.pose.pose.orientation.x);
    q.setY(odomAftPGO.pose.pose.orientation.y);
    q.setZ(odomAftPGO.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftPGO.header.stamp, odom_link, "/aft_pgo"));
}

void PALoc::PubMap() {
    if (keyMeasures.size() < 5) return;
    int counter = 0;
    laserCloudMapPGO->clear();

    pcl::PointCloud<PointT>::Ptr temp_cloud_ptr(new pcl::PointCloud<PointT>());
    //#pragma omp parallel for num_threads(8)
    for (int node_idx = 0; node_idx < keyMeasures.size(); node_idx++) {
        if (counter % SKIP_FRAMES == 0) {
            // temp_cloud_ptr->clear();
            //    pcl::transformPointCloud(
            //        *keyMeasures[node_idx].lidar, *temp_cloud_ptr,
            //        Pose6D2Matrix(keyMeasures[node_idx].updated_pose).cast<float>());
            //            downSizeFilterScan.setInputCloud(temp_cloud_ptr);
            //            downSizeFilterScan.filter(*temp_cloud_ptr);
            std::unique_lock<std::mutex> kf_guard(mKF);
            *laserCloudMapPGO += *TransformPointCloud(
                    keyMeasures[node_idx].lidar, keyMeasures[node_idx].updated_pose);
            kf_guard.unlock();
            //            *laserCloudMapPGO +=
            //            *Local2global(keyMeasures[node_idx].lidar, Pose6D(0, 0, 0,
            //            0, 0, 0));
            //    *laserCloudMapPGO += *temp_cloud_ptr;
        }
        counter++;
    }

    downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
    downSizeFilterMapPGO.filter(*laserCloudMapPGO);
    publishCloud(pubMapAftPGO, laserCloudMapPGO, ros::Time::now(), odom_link);
}

void PALoc::LoopDection() {
    ros::Rate rate(loopClosureFrequency);
    while (ros::ok()) {
        ros::spinOnce();
        PerformRSLoopClosure();
        rate.sleep();
    }
}

void PALoc::VisaulizationThread() {
    float vizmapFrequency = 0.1;
    ros::Rate rate(vizmapFrequency);
    while (ros::ok()) {
        rate.sleep();
        PubMap();
        VisualizeLoopConstrains(mapIndexContainer, pubGlobalMapConstraintEdge, -2);
        VisualizeLoopConstrains(ZUPTIndexContainer, pubZUPTConstraintEdge, -3);
        if (useLoopClosure) {
            VisualizeLoopConstrains(loopIndexCheckedMap, pubLoopConstraintEdge, 0);
        }
    }
}

void PALoc::SaveData() {
    if (keyMeasures.empty()) {
        ROS_INFO("NO ENOUGH POSE!");
        return;
    }

    int size = keyMeasures.size();
    std::vector<Eigen::Vector3d> lla_vec;
    std::vector<nav_msgs::Odometry> odomUpdated_vec;
    std::vector<double> keyframeTimes;
    std::vector<nav_msgs::Odometry> keyframePosesOdom;
    std::vector<pcl::PointCloud<PointT>::Ptr> keyframeLaserClouds;
    std::vector<sensor_msgs::PointCloud2> keyframeLaserCloudsMsg;
    std::vector<Vector7> pose_vec;
    std::vector<Eigen::Isometry3d> global_pose_vec;
    std::vector<Vector7> global_pose_vec_traj;

    std::string dir_path = saveDirectory + sequence + "/";
    std::ofstream logfile_time(dir_path + "data_time.txt", std::ios_base::out);
    std::ofstream logfile_icp(dir_path + "data_icp.txt", std::ios_base::out);
    std::ofstream outFile(dir_path + "pose_graph_3d_result.txt", std::ios_base::out);
    logfile_time.precision(12);
    logfile_icp.precision(12);
    outFile.precision(12);
    std::cout << "write time file path: " << dir_path + "time_data.txt" << std::endl;
    std::cout << "keyMeasures size: " << size << " " << keyTimeData.size() << " " << keyICPData.size() << std::endl;

    Values result = isam->calculateEstimate();
//    Marginals marginals(isam->getFactorsUnsafe(), result, Marginals::Factorization::CHOLESKY);
    Marginals marginals = Marginals(isam->getFactorsUnsafe(), result, Marginals::CHOLESKY);

    std::unique_lock<std::mutex> kf_guard(mKF);
    for (int i = 0; i < size - 1; ++i) {
        Pose6D p = keyMeasures.at(i).updated_pose;
        double time = keyMeasures.at(i).odom_time;
        double lidar_time = keyMeasures.at(i).lidar_time;
        Vector10 time_vec = keyTimeData.at(i);
        Vector10 icp_vec = keyICPData.at(i);
        keyframeTimes.push_back(time);
        keyframePosesOdom.push_back(keyMeasures.at(i).odom);
        keyframeLaserClouds.push_back(keyMeasures.at(i).lidar);

        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = ros::Time().fromSec(time);
        laserOdometryROS.header.frame_id = "map";
        laserOdometryROS.child_frame_id = "lidar";
        laserOdometryROS.pose.pose.position.x = p.x;
        laserOdometryROS.pose.pose.position.y = p.y;
        laserOdometryROS.pose.pose.position.z = p.z;
        laserOdometryROS.pose.pose.orientation =
                tf::createQuaternionMsgFromRollPitchYaw(p.roll, p.pitch, p.yaw);
        odomUpdated_vec.push_back(laserOdometryROS);

        pose_vec.push_back((Vector7() << p.x, p.y, p.z,
                laserOdometryROS.pose.pose.orientation.x,
                laserOdometryROS.pose.pose.orientation.y,
                laserOdometryROS.pose.pose.orientation.z,
                laserOdometryROS.pose.pose.orientation.w)
                                   .finished());

        // global pose
        if (useGlobalPrior) {
            Pose6D gl_pose = keyMeasures.at(i).global_pose;
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            transform = Pose6D2sometry3d(gl_pose);
            global_pose_vec.push_back(transform);
            tf::Quaternion quaternion =
                    tf::createQuaternionFromRPY(gl_pose.roll, gl_pose.pitch, gl_pose.yaw);
            global_pose_vec_traj.push_back((Vector7() << gl_pose.x, gl_pose.y,
                    gl_pose.z, quaternion.x(), quaternion.y(),
                    quaternion.z(), quaternion.w()).finished());
        }

        logfile_time << time << " ";
        for (int j = 0; j < time_vec.size(); ++j) {
            logfile_time << time_vec(j);
            if (j != time_vec.size() - 1) {
                logfile_time << " ";
            }
        }
        logfile_time << std::endl;

        logfile_icp << time << " ";
        for (int j = 0; j < icp_vec.size(); ++j) {
            logfile_icp << icp_vec(j);
            if (j != icp_vec.size() - 1) {
                logfile_icp << " ";
            }
        }
        logfile_icp << std::endl;

        if (result.exists(X(i))) {
            // 检查键是否存在
            auto pose = result.at<gtsam::Pose3>(X(i));
            // 保存姿态
            gtsam::Point3 translation = pose.translation();
            gtsam::Rot3 rotation = pose.rotation();
            gtsam::Quaternion quaternion = rotation.toQuaternion();
            outFile << time << " "
                    << translation.x() << " " << translation.y() << " " << translation.z() << " "
                    << quaternion.w() << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z();
            // 保存协方差矩阵
//            auto covariance = marginals.marginalCovariance(X(i));
            KeyVector keys = {X(i)}; // Example keys for current and previous poses.
            JointMarginal jointCov = marginals.jointMarginalCovariance(keys);
            gtsam::Matrix6 covariance = jointCov.at(X(i), X(i));
            for (int i = 0; i < covariance.rows(); ++i) {
                for (int j = 0; j < covariance.cols(); ++j) {
                    outFile << " " << covariance(i, j);
                }
            }
            outFile << std::endl;
        }
    }
    kf_guard.unlock();
    logfile_icp.close();
    logfile_time.close();
    outFile.close();

    // 保存结果到文件
    std::cout << "ISAM size: " << result.size() << " "
              << keyframePosesOdom.size() << " " << odomUpdated_vec.size() << " "
              << keyframeLaserClouds.size() << " " << pose_vec.size() << " "
              << global_pose_vec.size() << " " << keyTimeData.size() << " "
              << keyICPData.size() << " " << std::endl;
    std::cout << "size: " << keyframeTimes.size() << " "
              << keyframePosesOdom.size() << " " << odomUpdated_vec.size() << " "
              << keyframeLaserClouds.size() << " " << pose_vec.size() << " "
              << global_pose_vec.size() << " " << keyTimeData.size() << " "
              << keyICPData.size() << " " << std::endl;

    dataSaverPtr->saveTimes(keyframeTimes);
    dataSaverPtr->saveOdometryVerticesTUM(keyframePosesOdom);
    if (useGlobalPrior) {
        dataSaverPtr->saveOptimizedVerticesTUM(global_pose_vec_traj, "icp_tum.txt");
    }
    dataSaverPtr->saveOptimizedVerticesTUM(pose_vec, "optimized_poses_tum.txt");
    std::unique_lock<std::mutex> graph_guard(mtxPosegraph);
    dataSaverPtr->saveGraphGtsam(newFactors, isam, currentEstimate);
    graph_guard.unlock();


    if (saveResultBag) {
        //        dataSaverPtr->saveResultBag(keyframePosesOdom, odomUpdated_vec,
        //                                    keyframeLaserCloudsMsg);
        // dataSaverPtr->saveResultBag(odomUpdated, keyframeLaserClouds);
    }
    dataSaverPtr->savePointCloudMap(odomUpdated_vec, keyframeLaserClouds);

    if (saveKeyFrame) {
        // save lio and icp map
        dataSaverPtr->savePointCloudMapLIO(keyframePosesOdom, keyframeLaserClouds);
        dataSaverPtr->savePointCloudMap(global_pose_vec, keyframeLaserClouds);
    }

}

void PALoc::LidarCallback(
        const sensor_msgs::PointCloud2ConstPtr &pc_msg_ptr) {
    std::unique_lock<std::mutex> guard(mutexLock);
    fullResBuf.push(pc_msg_ptr);
    guard.unlock();
}

void PALoc::OdometryCallback(
        const nav_msgs::OdometryConstPtr &odom_msg_ptr) {
    std::unique_lock<std::mutex> guard(mutexLock);
    odometryBuf.push(odom_msg_ptr);
    guard.unlock();
}

void PALoc::ImuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg) {
    std::unique_lock<std::mutex> imu_guard(imuLock);
    imuQueue.push_back(*imuMsg);
    imu_guard.unlock();
}

void PALoc::InitialCallback(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg_ptr) {
    initialPose = GeoposeToMatrix4d(*pose_msg_ptr);
    std::cout << BOLDBLUE << "Get Rviz Pose estimation: " << initialPose.matrix()
              << std::endl;
    poseReceived = true;
}


bool PALoc::SaveMap(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res) {
    ROS_WARN("SAVE MAP AND G2O..");

    SaveData();

    return true;
}

void PALoc::PerformRSLoopClosure(void) {
    if (keyMeasures.size() < 10) return;
    // 当前关键帧索引，候选闭环匹配帧索引
    int loopKeyCur = keyMeasures.size() - 1;
    int loopKeyPre = -1;
    if (DetectLoopClosureDistance(&loopKeyCur, &loopKeyPre)) {
        Pose6D cur_pose = keyMeasures.at(loopKeyCur).updated_pose;
        Pose6D pre_pose = keyMeasures.at(loopKeyPre).updated_pose;

        float loopScore = std::numeric_limits<double>::max();
        std::unique_lock<std::mutex> icp_guard(mtxICP);
        Eigen::Matrix4d trans_cur2pre = Pose6D2Matrix(cur_pose);
        auto flag = cloud_process_.DoICPVirtualRelative2(
                keyMeasures, loopKeyPre, loopKeyCur, loopScore, 0, trans_cur2pre);
        Eigen::Matrix6d LOOP_cov = cloud_process_.icp_cov;
        icp_guard.unlock();

        if (flag) {
            Pose3 poseTo =
                    Pose6dTogtsamPose3(keyMeasures.at(loopKeyPre).updated_pose);
            Pose6D p = Matrix2Pose6D(trans_cur2pre);
            Pose3 poseFrom(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw),
                           gtsam::Point3(p.x, p.y, p.z));
            //SetLoopscore(loopScore);
            LOOPGaussianNoise = noiseModel::Gaussian::Covariance(LOOP_cov.matrix().cast<double>());

            std::unique_lock<std::mutex> loop_guard(mtxLoopContainer);
            loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
            loopPoseQueue.push_back(poseFrom.between(poseTo));
            //loopNoiseQueue.push_back(robustLoopNoise);
            loopGaussianNoiseQueue.push_back(LOOPGaussianNoise);
            loop_guard.unlock();

            loopIndexCheckedMap[loopKeyCur] = loopKeyPre;
            std::cout << BOLDRED << "ICP Loop detected! " << loopKeyCur << " and " << loopKeyPre << ", "
                      << loopScore << std::endl;
            std::cout << BOLDRED << "ICP Loop COV! " << LOOP_cov.diagonal().transpose() << std::endl;
        }
    }
}

bool PALoc::FilterLoopPairs(int loopKeyCur, int loopKeyPre) {
    if (loopKeyCur == loopKeyPre || loopKeyPre == -1) return false;

    // short time
    if (abs(keyMeasures.at(loopKeyCur).odom_time -
            abs(keyMeasures.at(loopKeyPre).odom_time) <
            historyKeyframeSearchTimeDiff))
        return false;

    // the loop pairs exits in the icp detect container
    auto it = loopIndexCheckedMap.find(loopKeyCur);
    if (it != loopIndexCheckedMap.end()) {
        int loop_pre_candidate = it->second;
        // too closed loop pairs are useless
        if (abs(loop_pre_candidate - loopKeyPre) < filterNodeNum) return false;
    }
//    auto it_sc = loopIndexSCcontainer.find(loopKeyCur);
//    if (it_sc != loopIndexSCcontainer.end()) {
//        int loop_pre_candidate = it_sc->second;
//        if (abs(loop_pre_candidate - loopKeyPre) < filterNodeNum) return false;
//    }
//
//    // the loop pairs exits in the intensity detect container
//    auto it_v = loopVisualIndexCheckedMap.find(loopKeyCur);
//    if (it_v != loopVisualIndexCheckedMap.end()) {
//        int loop_pre_candidate = it_v->second;
//        if (abs(loop_pre_candidate - loopKeyPre) < filterNodeNum) return false;
//    }
//    // the loop pairs exits in the rgb detect container
//    auto it_r = loopRGBVisualIndexCheckedMap.find(loopKeyCur);
//    if (it_r != loopRGBVisualIndexCheckedMap.end()) {
//        int loop_pre_candidate = it_r->second;
//        if (abs(loop_pre_candidate - loopKeyPre) < filterNodeNum) return false;
//    }

    if (abs(lastLoopIndex - loopKeyCur) < filterNodeNum && lastLoopIndex != -1) return false;

    // total accumuted distance bnetween 2 frame
    if (keyMeasures.size() >= loopKeyCur) {
        double distance = 0.0;
        for (int j = loopKeyPre; j < loopKeyCur; ++j) {
            distance += keyMeasures.at(j).distance;
        }
        // LOG(INFO) << "TOTAL DIS:" << distance;
        if (distance < filterDis) {
            std::cout << "CLOSE FRAME MUST FILTER OUT : " << distance << std::endl;
            return false;
        }
    }

    // stair case, when the z_offset is largeer than a threshold, it will not consider as a loop candidate
    double z_offset = abs(keyMeasures.at(loopKeyCur).updated_pose.z -
                          keyMeasures.at(loopKeyPre).updated_pose.z);
    if (z_offset > LOOP_Z_OFFSET) return false;

    lastLoopIndex = loopKeyCur;

    return true;
}

bool PALoc::DetectLoopClosureDistance(int *loopKeyCur,
                                      int *loopKeyPre) {
    // 当前关键帧
    // int loopKeyCur = keyframePoses.size() - 1;
    // int loopKeyPre = -1;
    // 当前帧已经添加过闭环对应关系，不再继续添加
    //    auto it = loopIndexCheckedMap.find(*loopKeyCur);
    //    if (it != loopIndexCheckedMap.end())
    //        return false;
    //
    //    if (abs(lastLoopIndex - *loopKeyCur) < 5 && lastLoopIndex != -1)
    //        return false;


//    std::unique_lock<std::mutex> kf_guard(mKF);
//    keyframePoses2D.clear();
    keyframePoses3D.clear();
    for (int i = 0; i < keyMeasures.size(); ++i) {
        Pose6D pose6D = keyMeasures.at(i).updated_pose;
        keyframePoses3D.push_back(pose6D);
//        pose6D.z = 0;
//        keyframePoses2D.push_back(pose6D);
    }
//    kf_guard.unlock();

    // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合
    pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloudKeyPoses3D =
            vector2pc(keyframePoses3D);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloudKeyPoses2D =
//            vector2pc(keyframePoses2D);

    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(),
                                        historyKeyframeSearchRadius,
                                        pointSearchIndLoop,
                                        pointSearchSqDisLoop,
                                        0);
//    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses2D);
//    kdtreeHistoryKeyPoses->radiusSearch(
//            copy_cloudKeyPoses2D->at(*loopKeyCur), historyKeyframeSearchRadius,
//            pointSearchIndLoop, pointSearchSqDisLoop, 0);

    // 在候选关键帧集合中，找到与当前帧时间相隔较远的帧，设为候选匹配帧
    for (int i = 0; i < pointSearchIndLoop.size(); ++i) {
        int id = pointSearchIndLoop.at(i);
        if (abs(keyMeasures.at(id).odom_time -
                keyMeasures.at(*loopKeyCur).odom_time) >
            historyKeyframeSearchTimeDiff) {
            *loopKeyPre = id;
            break;
        }
    }

    //    LOG(INFO) << "TIMES DISTANCE keyframePoses2D SIZE: " <<
    //              keyframeTimes.size() << ", " << keyframeDistances.size() << ",
    //              " << copy_cloudKeyPoses2D->size() << ", "
    //              << keyframePoses2D.size();
    if (*loopKeyPre == -1 || *loopKeyCur == *loopKeyPre) return false;

    if (!FilterLoopPairs(*loopKeyCur, *loopKeyPre)) return false;

    return true;
}

void PALoc::VisualizeLoopConstrains(std::map<int, int> loopMap,
                                    ros::Publisher &publisher,
                                    int type) {
    if (loopMap.empty()) return;

    visualization_msgs::MarkerArray markerArray;
    // 闭环顶点
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odom_link;  // camera_init
    markerNode.header.stamp = ros::Time().fromSec(keyMeasures.back().odom_time);
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.2;
    markerNode.scale.y = 0.2;
    markerNode.scale.z = 0.2;

    // 闭环边
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = odom_link;
    markerEdge.header.stamp = ros::Time().fromSec(keyMeasures.back().odom_time);
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.15;
    markerEdge.scale.y = 0.15;
    markerEdge.scale.z = 0.15;

    switch (type) {
        case -3:
            markerNode.ns = markerEdge.ns = "gravity";
            markerNode.color.r = markerEdge.color.r = 0.1;
            markerNode.color.g = markerEdge.color.g = 0.7;
            markerNode.color.b = markerEdge.color.b = 0.2;
            markerNode.color.a = markerEdge.color.a = 1;
            break;
        case -2:
            markerNode.ns = markerEdge.ns = "map_constraint_nodes";
            markerNode.color.r = markerEdge.color.r = 0.6;
            markerNode.color.g = markerEdge.color.g = 0.1;
            markerNode.color.b = markerEdge.color.b = 0.2;
            markerNode.color.a = markerEdge.color.a = 1;
            break;
        case 0:
            // icp
            markerNode.ns = markerEdge.ns = "lidar_nodes";
            markerNode.color.r = markerEdge.color.r = 0.9;
            markerNode.color.g = markerEdge.color.g = 0.9;
            markerNode.color.b = markerEdge.color.b = 0;
            markerNode.color.a = markerEdge.color.a = 1;
            break;
        default:
            std::cout << "error visulizer type!!!" << std::endl;
    }

    for (auto it = loopMap.begin(); it != loopMap.end(); ++it) {
        int key_cur = it->first;
        int key_pre = it->second;

        geometry_msgs::Point p;
        p.x = keyMeasures.at(key_cur).updated_pose.x;
        p.y = keyMeasures.at(key_cur).updated_pose.y;
        p.z = keyMeasures.at(key_cur).updated_pose.z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);

        if (type == -2) {
            p.x = keyMeasures.at(key_pre).global_pose.x;
            p.y = keyMeasures.at(key_pre).global_pose.y;
            p.z = keyMeasures.at(key_pre).global_pose.z;
        } else {
            p.x = keyMeasures[key_pre].updated_pose.x;
            p.y = keyMeasures[key_pre].updated_pose.y;
            p.z = keyMeasures[key_pre].updated_pose.z;
        }
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);

    publisher.publish(markerArray);
}

bool PALoc::GetGlobalICP(Measurement &measurement_temp) {

    Eigen::Matrix4d final_trans = Eigen::Matrix4d::Identity();
    //    guess_matrix = (prevPose).matrix();
    //    guess_matrix = predictGlobalState.pose().matrix();

    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map = extractLocalMap(globalmap_filter_ptr, predict_pose.matrix(),
                                                                     mapRadius);
    std::cout << "Extracted localmap points : " << local_map->size() << ", " << globalmap_filter_ptr->size()
              << std::endl;
    if (local_map->empty()) return false;
    publishCloud(pubLaserCloudCrop, local_map, ros::Time::now(), odom_link);

    // calculate covariance
    TicToc tic;
    std::shared_ptr<geometry::PointCloud> source_o3d = cloud_process_.GetO3dPointCloudFromPCL(*measurement_temp.lidar);
    std::shared_ptr<geometry::PointCloud> target_o3d = cloud_process_.GetO3dPointCloudFromPCL(*local_map);
    // double max_correspondence_distance = 2.0;
    pipelines::registration::RegistrationResult icp;
    auto criteria = pipelines::registration::ICPConvergenceCriteria(30);
    switch (icpO3dType) {
        case 0:  // point-to-point icp
            icp = pipelines::registration::RegistrationICP(
                    *source_o3d, *target_o3d, correspondenceDis, predict_pose.matrix(),
                    pipelines::registration::TransformationEstimationPointToPoint(),
                    criteria);
            break;
        case 1:  // Point-to-plane
            target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(2.0, 20));
            icp = pipelines::registration::RegistrationICP(
                    *source_o3d, *target_o3d, correspondenceDis, predict_pose.matrix(),
                    pipelines::registration::TransformationEstimationPointToPlane(),
                    criteria);
            break;
        case 2:
            target_o3d->EstimateNormals(geometry::KDTreeSearchParamHybrid(2.0, 20));
            icp = pipelines::registration::RegistrationGeneralizedICP(
                    *source_o3d, *target_o3d, correspondenceDis, predict_pose.matrix(),
                    pipelines::registration::TransformationEstimationForGeneralizedICP(),
                    criteria);
            break;
        default:
            std::cout << "Evaluation error type!!!!! " << std::endl;
            break;
    }
    final_trans = icp.transformation_;
    double score = icp.inlier_rmse_;
    double overlap = icp.fitness_;

    if (score == 0.0 || score > loopFitnessScoreThreshold || overlap < 0.7) {
        measurement_temp.global_pose.valid = false;
        std::cout << BOLDWHITE << "O3D ICP FAILED: " << score << " " << overlap << std::endl;
        return false;
    }
    measurement_temp.global_pose = Matrix2Pose6D(final_trans);
    measurement_temp.global_score = score;

    // 使用 EigenMatrixToTensor 转换 Eigen 矩阵到 Open3D Tensor
    core::Tensor transformation_tensor = core::eigen_converter::EigenMatrixToTensor(icp.transformation_);
    t::geometry::PointCloud source_o3d_new = t::geometry::PointCloud::FromLegacy(*source_o3d);
    t::geometry::PointCloud target_o3d_new = t::geometry::PointCloud::FromLegacy(*target_o3d);
    bool flag = false;
    try {
        core::Tensor information_matrix = t::pipelines::registration::GetInformationMatrix(
                source_o3d_new, target_o3d_new, correspondenceDis, transformation_tensor);
        // 将 Tensor 转换为 Eigen 矩阵
        Eigen::MatrixXd ifm_eigen = core::eigen_converter::TensorToEigenMatrixXd(information_matrix);
        if (ifm_eigen.rows() == 6 && ifm_eigen.cols() == 6) {
            icp_cov = ifm_eigen.inverse().cast<float>();
            flag = true;
        } else {
            std::cerr << BOLDRED << "Information matrix is not 6x6. Cannot compute covariance matrix." << std::endl;
        }
    } catch (const std::runtime_error &e) {
        std::cerr << "Runtime error: " << e.what() << std::endl;
        std::cerr
                << "Check if the point clouds have sufficient correspondences and adjust the max_correspondence_distance parameter if necessary."
                << std::endl;
        icp_cov = Eigen::Matrix<float, 6, 6>::Identity();
    }
    std::cout << BOLDGREEN << "O3D ICP COV: " << icp_cov.diagonal().transpose() << std::endl;
    if (!flag) return false;

    // publish global odomertry
    if (1) {
        std::cout << BOLDMAGENTA << "Global O3D ICP SUCCESS: "
                  << measurement_temp.lidar->size()
                  << " " << local_map->size() << " "
                  << score << " " << overlap << std::endl;
    }
    return true;
}


bool
PALoc::Point2PlaneICPLM(pcl::PointCloud<PointT>::Ptr measure_cloud,
                        pcl::PointCloud<PointT>::Ptr target_cloud,
                        Pose6D &pose_icp, double SEARCH_RADIUS) {
    bool flag = false;
    total_rmse = 0.0;
    TicToc tic_toc;
    for (int iterCount = 0; iterCount < 20; iterCount++) {
        laserCloudEffective->clear();
        coeffSel->clear();
        total_distance = 0.0;

#pragma omp parallel for num_threads(8)
        for (int i = 0; i < measure_cloud->size(); i++) {
            PointT pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            // transform points in the map using the initial pose
            pointOri = measure_cloud->points[i];
            pointBodyToGlobal(pointOri, pointSel, Pose6D2Matrix(pose_icp));
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            // find correspondences
            if (pointSearchSqDis[4] < SEARCH_RADIUS) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = target_cloud->points[pointSearchInd[j]].x;
                    matA0(j, 1) = target_cloud->points[pointSearchInd[j]].y;
                    matA0(j, 2) = target_cloud->points[pointSearchInd[j]].z;
                }
                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;
                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * target_cloud->points[pointSearchInd[j]].x +
                             pb * target_cloud->points[pointSearchInd[j]].y +
                             pc * target_cloud->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }
                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
                    double distance = fabs(pd2) /
                                      sqrt(pointOri.x * pointOri.x + pointOri.y * pointOri.y +
                                           pointOri.z * pointOri.z);
                    float s = 1 - 0.9 * distance;

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;
                    pointOri.intensity = abs(pd2);
                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                        total_distance += distance;
                    }
                }
            }
        }

        for (int i = 0; i < measure_cloud->size(); ++i) {
            if (laserCloudOriSurfFlag[i] == true) {
                laserCloudEffective->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
                //   residuals_vec.push_back(laserCloudOriSurfVec[i].intensity);
                //    laserCloudEffectiveOri->push_back(raw_cloud->points.at(i));
            }
        }

        // caculate distance
        final_fitness = (double) laserCloudEffective->size() / measure_cloud->size();
        curr_rmse = std::sqrt(total_distance / (double) laserCloudEffective->size());
        total_rmse += curr_rmse;

        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll


        // 0-5 roll pitch yaw x y z
        float srx = sin(pose_icp.pitch);
        float crx = cos(pose_icp.pitch);
        float sry = sin(pose_icp.yaw);
        float cry = cos(pose_icp.yaw);
        float srz = sin(pose_icp.roll);
        float crz = cos(pose_icp.roll);

        int laserCloudSelNum = laserCloudEffective->size();
        if (laserCloudSelNum < 100) {
            std::cout << BOLDGREEN << "NO ENCOUGN ['POINTS']: " << laserCloudSelNum << std::endl;
            return false;
        }

        // matA: Jocbin
        // JtJ*delt_x=-JTf, delt_x->matX
        Eigen::Matrix<float, Eigen::Dynamic, 6> matA(laserCloudSelNum, 6);
        Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, laserCloudSelNum);
        Eigen::Matrix<float, 6, 6> matAtA;
        Eigen::VectorXf matB(laserCloudSelNum);
        Eigen::VectorXf matAtB;
        Eigen::VectorXf matX;

        PointT pointOri, coeff;
        for (int i = 0; i < laserCloudSelNum; i++) {
            pointOri.x = laserCloudEffective->points[i].y;
            pointOri.y = laserCloudEffective->points[i].z;
            pointOri.z = laserCloudEffective->points[i].x;
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;

            // construct the jacobin
            double crx_sry = crx * sry;
            double crz_sry = crz * sry;
            double srx_sry = srx * sry;
            double srx_srz = srx * srz;
            float arx = (crx_sry * srz * pointOri.x + crx * crz_sry * pointOri.y - srx_sry * pointOri.z) * coeff.x
                        + (-srx_srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y
                        + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) *
                          coeff.z;
            float ary = ((cry * srx_srz - crz_sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y +
                         crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx_sry * srz) * pointOri.x
                                                              + (cry * srz - crz * srx_sry) * pointOri.y -
                                                              crx_sry * pointOri.z) * coeff.z;
            float arz = ((crz * srx_sry - cry * srz) * pointOri.x + (-cry * crz - srx_sry * srz) * pointOri.y) *
                        coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y
                        + ((sry * srz + cry * crz * srx) * pointOri.x + (crz_sry - cry * srx_srz) * pointOri.y) *
                          coeff.z;

            matA(i, 0) = arz;
            matA(i, 1) = arx;
            matA(i, 2) = ary;
            matA(i, 3) = coeff.z;
            matA(i, 4) = coeff.x;
            matA(i, 5) = coeff.y;
            matB(i, 0) = -coeff.intensity;
            residuals_vec.push_back(-coeff.intensity);
        }

        matAt = matA.transpose(); // JT   6*laserCloudSelNum
        matAtA = matAt * matA; //  JTJ hessian 6*6
        matAtB = matAt * matB;  // -JT*e
        matX = matAtA.colPivHouseholderQr().solve(matAtB);         // solve AX=b, JTJX=-JTe
        icp_cov = matAtA.inverse();

        // NOTE: Handling with the degeneracy problem according to the paper
        // J. Zhang, M. Kaess and S. Singh, "On degeneracy of optimization-based state estimation problems,"
        // 2016 IEEE International Conference on Robotics and Automation (ICRA), Stockholm, 2016
        // this code block is originlly from LOAM
        if (iterCount == 0) {
            // Eigenvalue decomposition
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> esolver(matAtA);
            Eigen::Matrix<float, 1, 6> matE = esolver.eigenvalues().real();
            Eigen::Matrix<float, 6, 6> matV = esolver.eigenvectors().real();

            // Calculate condition numbers and check minimum eigenvalues for xyz and rpy
            double max_eigval_xyz = matE.tail<3>().maxCoeff();
            double min_eigval_xyz = matE.tail<3>().minCoeff();
            double condition_number_xyz = max_eigval_xyz / min_eigval_xyz;
            double max_eigval_rpy = matE.head<3>().maxCoeff();
            double min_eigval_rpy = matE.head<3>().minCoeff();
            double condition_number_rpy = max_eigval_rpy / min_eigval_rpy;
            eigen_values_ratio_xyz = matE.tail<3>().cast<double>() / min_eigval_xyz;
            eigen_values_ratio_rpy = matE.head<3>().cast<double>() / min_eigval_rpy;
            eigen_values = eigen_values_ratio_xyz;

            // Thresholds for degeneracy judgement
            const double MIN_EIGENVALUE_THRESHOLD = 100.0;
            bool isDegenerate_xyz =
                    condition_number_xyz > DEGENERACY_THRES || min_eigval_xyz < MIN_EIGENVALUE_THRESHOLD;
            bool isDegenerate_rpy =
                    condition_number_rpy > DEGENERACY_THRES || min_eigval_rpy < MIN_EIGENVALUE_THRESHOLD;

            // If any part is degenerate, mark the whole as degenerate
            bool isDegenerate = isDegenerate_xyz || isDegenerate_rpy;
            Eigen::Matrix<float, 6, 6> matV2 = matV;
            if (isDegenerate) {
                for (int i = 5; i >= 0; i--) {
                    matV2.row(i).setZero();
                }
                if (isDegenerate_xyz) { eigen_values = eigen_values_ratio_xyz; }
                if (isDegenerate_rpy) { eigen_values = eigen_values_ratio_rpy; }
                // 计算雅可比误差乘积 (JEP)
                // std::string dir_path = saveDirectory + sequence + "/" + std::to_string(curr_node_idx);
                // std::string filename = dir_path + ".pcd";
                //                    if (pcl::io::savePCDFileASCII(filename, *laserCloudEffective) == 0) {
                //                        std::cout << "Saved " << laserCloudEffective->points.size() << " data points to " << filename
                //                                  << std::endl;
                //                    } else {
                //                        std::cerr << "Error saving PCD file." << std::endl;
                //                    }
            }
            matP_eigen = matV2 * matV.inverse();

            // Output the condition numbers, minimum eigenvalues, and degeneracy status
            std::cout << "Condition number xyz and  rpy: " << condition_number_xyz << " " << condition_number_rpy
                      << std::endl;
            std::cout << "Degeneracy status xyz and rpy: " << (isDegenerate_xyz ? "Degenerate" : "Not Degenerate")
                      << ", " << (isDegenerate_rpy ? "Degenerate" : "Not Degenerate")
                      << std::endl;
            std::cout << "eigen factor xyz: " << matE.matrix() << std::endl;

            // 初始化计数器
            int count_translation_constraints[3] = {0, 0, 0}; // 用于统计对x, y, z贡献最大的correspondence数
            int count_rotation_constraints[3] = {0, 0, 0}; // 用于统计对r, p, y贡献最大的correspondence数
            bool use_translation = true; // 根据这个标志选择使用平移还是旋转部分进行着色
            if (!use_translation)
                eigen_values = eigen_values_ratio_rpy;

            //std::cout << "JT: " << matAt.rows() << " " << matAt.cols() << std::endl;
            for (int i = 0; i < laserCloudSelNum; i++) {
                // 计算雅可比矩阵与误差向量的乘积
                //   Eigen::VectorXf constraint = matA.row(i).transpose() * matB(i);
                Eigen::VectorXf constraint = matAt.col(i).transpose();
                Eigen::Vector3f translation_constraint = constraint.tail<3>();
                Eigen::Vector3f rotation_constraint = constraint.head<3>();
                // 找到平移部分贡献最大的维度
                int max_translation_idx;
                translation_constraint.cwiseAbs().maxCoeff(&max_translation_idx);
                count_translation_constraints[max_translation_idx]++;
                // 找到旋转部分贡献最大的维度
                int max_rotation_idx;
                rotation_constraint.cwiseAbs().maxCoeff(&max_rotation_idx);
                count_rotation_constraints[max_rotation_idx]++;
                // 选择使用平移或旋转部分
                Eigen::Vector3f chosen_constraint = use_translation ? constraint.tail<3>() : constraint.head<3>();
                // 找到最大贡献度的维度
                int max_idx;
                chosen_constraint.cwiseAbs().maxCoeff(&max_idx);
                // 根据最大贡献度的维度上色
                int color = (max_idx == 0) ? 0 : (max_idx == 1) ? 100 : 200;
                // 设置点云的颜色值
                // 假设 laserCloudEffective 是点云指针，且我们可以设置每个点的颜色
                // 这里我们直接设置点的intensity值来表示颜色，你可能需要根据你的点云库的API来调整这部分
                laserCloudEffective->points[i].intensity = color;
            }

            // 计算各个维度的贡献比率
            float total_translation_constraints = laserCloudSelNum;
            float total_rotation_constraints = laserCloudSelNum;
            float translation_constraints_ratio[3];
            float rotation_constraints_ratio[3];
            for (int i = 0; i < 3; ++i) {
                translation_constraints_ratio[i] =
                        count_translation_constraints[i] / total_translation_constraints;
                rotation_constraints_ratio[i] = count_rotation_constraints[i] / total_rotation_constraints;
            }
            std::cout << "Translation constraints ratio: " << translation_constraints_ratio[0] * 100
                      << ", " << translation_constraints_ratio[1] * 100
                      << ", " << translation_constraints_ratio[2] * 100 << std::endl;
            //            std::cout << "Rotation constraints ratio: " << rotation_constraints_ratio[0] * 100
            //                      << ", " << rotation_constraints_ratio[1] * 100
            //                      << ", " << rotation_constraints_ratio[2] * 100 << std::endl;
        }

        if (isDegenerate) {
            // 如果发生退化，就对增量进行修改，退化方向不更新
            Eigen::Matrix<float, 6, 1> matX2(matX);
            matX = matP_eigen * matX2;
        }

        pose_icp.roll += matX(0, 0);
        pose_icp.pitch += matX(1, 0);
        pose_icp.yaw += matX(2, 0);
        pose_icp.x += matX(3, 0);
        pose_icp.y += matX(4, 0);
        pose_icp.z += matX(5, 0);

        // calculate rmse and fitness
        relative_rmse = abs(curr_rmse - prev_rmse);
        relative_fitness = abs(final_fitness - prev_fitness);
        prev_rmse = curr_rmse;
        prev_fitness = final_fitness;

        float deltaR = sqrt(pow(pcl::rad2deg(matX(0, 0)), 2) +
                            pow(pcl::rad2deg(matX(1, 0)), 2) +
                            pow(pcl::rad2deg(matX(2, 0)), 2));
        float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                            pow(matX(4, 0) * 100, 2) +
                            pow(matX(5, 0) * 100, 2));

        // we must restrict the update value till converge,
        // otherwise may lead to local minimum
        if (deltaR < 1e-6 && deltaT < 1e-6 || relative_rmse < 1e-6) {
            //        if (deltaR < 0.05 && deltaT < 0.05 || relative_rmse < 1e-6) {
            flag = true;
            iterate_number = iterCount;
            std::cout << BOLDMAGENTA << "RMSE and overlap: " << total_rmse << " "
                      << final_fitness << " " << relative_rmse << " " << relative_fitness << std::endl;
            std::cout << BOLDMAGENTA << "Time and converge count: " << tic_toc.toc() << " " << iterCount
                      << std::endl;
            break;
        }
    }
    return flag;
}


bool
PALoc::Point2PlaneICPLM2(pcl::PointCloud<PointT>::Ptr measure_cloud,
                         pcl::PointCloud<PointT>::Ptr target_cloud,
                         Pose6D &pose_icp, double SEARCH_RADIUS) {
    bool flag = false;
    total_rmse = 0.0;
    TicToc tic_toc;
    for (int iterCount = 0; iterCount < 30; iterCount++) {
        PrepareClouds(measure_cloud, target_cloud, pose_icp, SEARCH_RADIUS);
        if (laserCloudEffective->size() < 100) {
            std::cout << BOLDGREEN << "NOT ENOUGH POINTS: " << laserCloudEffective->size() << std::endl;
            return false;
        }
        Eigen::Matrix<float, Eigen::Dynamic, 6> matA(laserCloudEffective->size(), 6);
        Eigen::VectorXf matB(laserCloudEffective->size());
        ConstructJacobianMatrix(matA, matB, pose_icp);
        Eigen::Matrix<float, 6, 6> matAtA = matA.transpose() * matA;
        Eigen::VectorXf matAtB = matA.transpose() * matB;
        Eigen::VectorXf matX = matAtA.colPivHouseholderQr().solve(matAtB);
        icp_cov = matAtA.inverse();
        if (iterCount == 0) {
            CheckDegeneracy(matAtA);
        }
        if (isDegenerate) {
            matX = matP_eigen * matX;
        }
        UpdatePose(pose_icp, matX);
        if (CheckConvergence(matX, iterCount)) {
            flag = true;
            std::cout << BOLDMAGENTA << "Time and converge count: " << tic_toc.toc() << " " << iterCount << std::endl;
            break;
        }
    }
    return flag;
}

void PALoc::PrepareClouds(pcl::PointCloud<PointT>::Ptr measure_cloud,
                          pcl::PointCloud<PointT>::Ptr target_cloud,
                          Pose6D &pose_icp, double SEARCH_RADIUS) {
    laserCloudEffective->clear();
    coeffSel->clear();
    total_distance = 0.0;
#pragma omp parallel for num_threads(8)
    for (int i = 0; i < measure_cloud->size(); i++) {
        PointT pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        pointOri = measure_cloud->points[i];
        pointBodyToGlobal(pointOri, pointSel, Pose6D2Matrix(pose_icp));
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
        if (pointSearchSqDis[4] < SEARCH_RADIUS) {
            CalculateCoefficients(target_cloud, pointSearchInd, pointSel, coeff, pointOri, i);
        }
    }
    for (int i = 0; i < measure_cloud->size(); ++i) {
        if (laserCloudOriSurfFlag[i]) {
            laserCloudEffective->push_back(laserCloudOriSurfVec[i]);
            coeffSel->push_back(coeffSelSurfVec[i]);
        }
    }
    final_fitness = static_cast<double>(laserCloudEffective->size()) / measure_cloud->size();
    curr_rmse = std::sqrt(total_distance / static_cast<double>(laserCloudEffective->size()));
    total_rmse += curr_rmse;
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
}

void PALoc::CalculateCoefficients(pcl::PointCloud<PointT>::Ptr target_cloud,
                                  const std::vector<int> &pointSearchInd,
                                  const PointT &pointSel, PointT &coeff, PointT &pointOri, int i) {
    Eigen::Matrix<float, 5, 3> matA0;
    Eigen::Matrix<float, 5, 1> matB0;
    Eigen::Vector3f matX0;
    matA0.setZero();
    matB0.fill(-1);
    matX0.setZero();
    for (int j = 0; j < 5; j++) {
        matA0(j, 0) = target_cloud->points[pointSearchInd[j]].x;
        matA0(j, 1) = target_cloud->points[pointSearchInd[j]].y;
        matA0(j, 2) = target_cloud->points[pointSearchInd[j]].z;
    }
    matX0 = matA0.colPivHouseholderQr().solve(matB0);
    float pa = matX0(0, 0);
    float pb = matX0(1, 0);
    float pc = matX0(2, 0);
    float pd = 1;
    float ps = std::sqrt(pa * pa + pb * pb + pc * pc);
    pa /= ps;
    pb /= ps;
    pc /= ps;
    pd /= ps;
    bool planeValid = true;
    for (int j = 0; j < 5; j++) {
        if (std::fabs(pa * target_cloud->points[pointSearchInd[j]].x +
                      pb * target_cloud->points[pointSearchInd[j]].y +
                      pc * target_cloud->points[pointSearchInd[j]].z + pd) > 0.2) {
            planeValid = false;
            break;
        }
    }
    if (planeValid) {
        float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
        double distance = std::fabs(pd2) /
                          std::sqrt(pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z);
        float s = 1 - 0.9 * distance;
        coeff.x = s * pa;
        coeff.y = s * pb;
        coeff.z = s * pc;
        coeff.intensity = s * pd2;
        pointOri.intensity = std::abs(pd2);
        if (s > 0.1) {
            laserCloudOriSurfVec[i] = pointOri;
            coeffSelSurfVec[i] = coeff;
            laserCloudOriSurfFlag[i] = true;
            total_distance += distance;
        }
    }
}

void PALoc::ConstructJacobianMatrix(Eigen::Matrix<float, Eigen::Dynamic, 6> &matA,
                                    Eigen::VectorXf &matB, Pose6D &pose_icp) {
    float srx = sin(pose_icp.pitch);
    float crx = cos(pose_icp.pitch);
    float sry = sin(pose_icp.yaw);
    float cry = cos(pose_icp.yaw);
    float srz = sin(pose_icp.roll);
    float crz = cos(pose_icp.roll);
    for (int i = 0; i < laserCloudEffective->size(); i++) {
        PointT pointOri = laserCloudEffective->points[i];
        PointT coeff = coeffSel->points[i];
        // Compute the Jacobian
        float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x
                    + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y
                    + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;
        float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y +
                     crx * cry * pointOri.z) * coeff.x
                    + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y -
                       crx * sry * pointOri.z) * coeff.z;
        float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x
                    + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y
                    +
                    ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;
        matA(i, 0) = arx;
        matA(i, 1) = ary;
        matA(i, 2) = arz;
        matA(i, 3) = coeff.z;
        matA(i, 4) = coeff.x;
        matA(i, 5) = coeff.y;
        matB(i, 0) = -coeff.intensity;
        residuals_vec.push_back(-coeff.intensity);
    }
}

void PALoc::CheckDegeneracy(const Eigen::Matrix<float, 6, 6> &matAtA) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> esolver(matAtA);
    Eigen::Matrix<float, 1, 6> matE = esolver.eigenvalues().real();
    Eigen::Matrix<float, 6, 6> matV = esolver.eigenvectors().real();
    double max_eigval_xyz = matE.tail<3>().maxCoeff();
    double min_eigval_xyz = matE.tail<3>().minCoeff();
    double condition_number_xyz = max_eigval_xyz / min_eigval_xyz;
    double max_eigval_rpy = matE.head<3>().maxCoeff();
    double min_eigval_rpy = matE.head<3>().minCoeff();
    double condition_number_rpy = max_eigval_rpy / min_eigval_rpy;
    eigen_values_ratio_xyz = matE.tail<3>().cast<double>() / min_eigval_xyz;
    eigen_values_ratio_rpy = matE.head<3>().cast<double>() / min_eigval_rpy;
    eigen_values = eigen_values_ratio_xyz;
    const double MIN_EIGENVALUE_THRESHOLD = 100.0;
    bool isDegenerate_xyz = condition_number_xyz > DEGENERACY_THRES || min_eigval_xyz < MIN_EIGENVALUE_THRESHOLD;
    bool isDegenerate_rpy = condition_number_rpy > DEGENERACY_THRES || min_eigval_rpy < MIN_EIGENVALUE_THRESHOLD;
    isDegenerate = isDegenerate_xyz || isDegenerate_rpy;
    if (isDegenerate) {
        Eigen::Matrix<float, 6, 6> matV2 = matV;
        for (int i = 5; i >= 0; i--) {
            matV2.row(i).setZero();
        }
        matP_eigen = matV2 * matV.inverse();
    }

    std::cout << "Condition number xyz and rpy: " << condition_number_xyz << " " << condition_number_rpy << std::endl;
    std::cout << "Degeneracy status xyz and rpy: " << (isDegenerate_xyz ? "Degenerate" : "Not Degenerate")
              << ", " << (isDegenerate_rpy ? "Degenerate" : "Not Degenerate") << std::endl;
}

void PALoc::UpdatePose(Pose6D &pose_icp, const Eigen::VectorXf &matX) {
    pose_icp.roll += matX(0, 0);
    pose_icp.pitch += matX(1, 0);
    pose_icp.yaw += matX(2, 0);
    pose_icp.x += matX(3, 0);
    pose_icp.y += matX(4, 0);
    pose_icp.z += matX(5, 0);
}

bool PALoc::CheckConvergence(const Eigen::VectorXf &matX, int iterCount) {
    float deltaR = std::sqrt(std::pow(pcl::rad2deg(matX(0, 0)), 2) +
                             std::pow(pcl::rad2deg(matX(1, 0)), 2) +
                             std::pow(pcl::rad2deg(matX(2, 0)), 2));
    float deltaT = std::sqrt(std::pow(matX(3, 0) * 100, 2) +
                             std::pow(matX(4, 0) * 100, 2) +
                             std::pow(matX(5, 0) * 100, 2));
    relative_rmse = std::abs(curr_rmse - prev_rmse);
    relative_fitness = std::abs(final_fitness - prev_fitness);
    prev_rmse = curr_rmse;
    prev_fitness = final_fitness;
    if (deltaR < 1e-6 && deltaT < 1e-6 || relative_rmse < 1e-6) {
        iterate_number = iterCount;
        std::cout << BOLDMAGENTA << "RMSE and overlap: " << total_rmse << " "
                  << final_fitness << " " << relative_rmse << " " << relative_fitness << std::endl;
        return true;
    }
    return false;
}

void PALoc::SetLoopscore(float loopNoiseScore) {
    Vector6 precious_loop;
    precious_loop.head<3>().setConstant(loopNoiseScore);
    precious_loop.tail<3>().setConstant(loopNoiseScore);
    robustLoopNoise = gtsam::noiseModel::Diagonal::Variances(precious_loop);
}

void PALoc::PublishPose(const ros::Time &time,
                        const ros::Publisher &topic_pub,
                        const std::string &base_frame_id,
                        const Eigen::Matrix4d &transform_matrix) {
    geometry_msgs::PoseWithCovarianceStamped pose_;
    pose_.header.stamp = time;
    pose_.header.frame_id = base_frame_id;

    // set the position
    pose_.pose.pose.position.x = transform_matrix(0, 3);
    pose_.pose.pose.position.y = transform_matrix(1, 3);
    pose_.pose.pose.position.z = transform_matrix(2, 3);

    Eigen::Quaterniond q;
    q = transform_matrix.block<3, 3>(0, 0);
    pose_.pose.pose.orientation.x = q.x();
    pose_.pose.pose.orientation.y = q.y();
    pose_.pose.pose.orientation.z = q.z();
    pose_.pose.pose.orientation.w = q.w();

    topic_pub.publish(pose_);
}


State PALoc::GetStateFromLIO2(const int node_id) {
    const auto &latest_measurement = keyMeasures.at(node_id).odom; // Assuming this is a nav_msgs::Odometry
    const auto &key_measure = keyMeasures.at(node_id);

    State state;
    // Get the orientation quaternion from the message
    const auto &orientation = latest_measurement.pose.pose.orientation;
    gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
    const auto &position = latest_measurement.pose.pose.position;
    gtsam::Point3 translation(position.x, position.y, position.z);

    Pose3 pose_tmp = gtsam::Pose3(rotation, translation);;
    state.pose = pose_tmp;
    const auto &linear_velocity = latest_measurement.twist.twist.linear;
    state.vel = gtsam::Vector3(linear_velocity.x, linear_velocity.y, linear_velocity.z);

    const auto &odom_covariance = latest_measurement.pose.covariance;
    gtsam::Matrix6 pose_cov = Matrix6::Zero();
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            pose_cov(i, j) = odom_covariance[i * 6 + j];
        }
    }
    // IMPORTANT: the pose cov from FASTLIO is described using R3 * SO3, not SE3
    // transform the rotation cov to the SE3 using adjoint
    state.cov = pose_tmp.AdjointMap() * pose_cov * pose_tmp.AdjointMap().transpose();
    return state;
}