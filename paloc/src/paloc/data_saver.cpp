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
#include "data_saver.h"

DataSaver::DataSaver() {}

DataSaver::~DataSaver() {}

DataSaver::DataSaver(string _base_dir, string _sequence_name) {
    this->base_dir = _base_dir;
    this->sequence_name = _sequence_name;

    if (_base_dir.back() != '/') {
        _base_dir.append("/");
    }
    save_directory = _base_dir + sequence_name + '/';
    std::cout << "SAVE DIR:" << save_directory << std::endl;

    auto unused = system((std::string("exec rm -r ") + save_directory).c_str());
    unused = system((std::string("mkdir -p ") + save_directory).c_str());
}

void DataSaver::saveOptimizedVerticesTUM(gtsam::Values _estimates) {
    std::fstream stream(save_directory + "optimized_odom_tum.txt",
                        std::fstream::out);
    stream.precision(15);
    for (int i = 0; i < _estimates.size(); i++) {
        auto &pose = _estimates.at(X(i)).cast<gtsam::Pose3>();
        //        auto &pose = _estimates.at<gtsam::Pose3>(X(i));
        gtsam::Point3 p = pose.translation();
        gtsam::Quaternion q = pose.rotation().toQuaternion();
        stream << keyframeTimes.at(i) << " " << p.x() << " " << p.y() << " "
               << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
               << q.w() << std::endl;
    }
}

void DataSaver::saveOptimizedVerticesTUM(std::vector<Vector7> _estimates) {
    std::fstream stream(save_directory + "optimized_odom_tum.txt",
                        std::fstream::out);
    stream.precision(15);
    // x y z qx qy qz qw
    for (int i = 0; i < _estimates.size(); i++) {
        auto &pose = _estimates.at(i);
        stream << keyframeTimes.at(i) << " " << pose(0) << " " << pose(1) << " "
               << pose(2) << " " << pose(3) << " " << pose(4) << " " << pose(5)
               << " " << pose(6) << std::endl;
    }
}

void DataSaver::saveOptimizedVerticesTUM(std::vector<Vector7> _estimates,
                                         std::string file_name) {
    std::fstream stream(save_directory + file_name, std::fstream::out);
    stream.precision(15);
    // x y z qx qy qz qw
    for (int i = 0; i < _estimates.size(); i++) {
        auto &pose = _estimates.at(i);
        stream << keyframeTimes.at(i) << " " << pose(0) << " " << pose(1) << " "
               << pose(2) << " " << pose(3) << " " << pose(4) << " " << pose(5)
               << " " << pose(6) << std::endl;
    }
}

void DataSaver::setDir(string _base_dir, string _sequence_name) {
    this->base_dir = _base_dir;
    this->sequence_name = _sequence_name;

    if (_base_dir.back() != '/') {
        _base_dir.append("/");
    }
    save_directory = _base_dir + sequence_name + '/';

    auto unused = system((std::string("exec rm -r ") + save_directory).c_str());
    unused = system((std::string("mkdir -p ") + save_directory).c_str());

    //  LOG(INFO) << "SET DIR:" << save_directory;
}

void DataSaver::setConfigDir(string _config_dir) {
    if (_config_dir.back() != '/') {
        _config_dir.append("/");
    }
    this->config_directory = _config_dir;
}

void DataSaver::setKeyframe(bool _save_key_frame) {
    this->save_key_frame = _save_key_frame;

    if (save_key_frame) {
        keyFrmaePath = save_directory + "key_point_frame/";
        //keyColorFrmaePath = save_directory + "key_color_frame/";
        string command = "mkdir -p " + keyFrmaePath;
        //string command2 = "mkdir -p " + keyColorFrmaePath;
        system(command.c_str());
        //system(command2.c_str());
        std::cout << "MKDIR NEW KEYFRAME DIR: " << command << std::endl;
        //std::cout << "MKDIR NEW KEYFRAME DIR: " << command2 << std::endl;
    }
}

void DataSaver::setExtrinc(bool _use_imu, bool _saveResultBodyFrame,
                           Eigen::Vector3d _t_body_sensor,
                           Eigen::Quaterniond _q_body_sensor) {
    this->use_imu_frame = _use_imu;
    this->saveResultBodyFrame = _saveResultBodyFrame;
    this->t_body_sensor = _t_body_sensor;
    this->q_body_sensor = _q_body_sensor;
}

void DataSaver::saveOptimizedVerticesKITTI(gtsam::Values _estimates) {
    std::fstream stream(save_directory + "optimized_odom_kitti.txt",
                        std::fstream::out);
    stream.precision(15);
    //    for (const auto &key_value: _estimates) {
    //        auto p = dynamic_cast<const GenericValue<Pose3>
    //        *>(&key_value.value); if (!p) continue;
    //
    //        const Pose3 &pose = p->value();
    //
    //        Point3 t = pose.translation();
    //        Rot3 R = pose.rotation();
    //        auto col1 = R.column(1); // Point3
    //        auto col2 = R.column(2); // Point3
    //        auto col3 = R.column(3); // Point3
    //
    //        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " <<
    //        t.x() << " "
    //               << col1.y() << " " << col2.y() << " " << col3.y() << " " <<
    //               t.y() << " "
    //               << col1.z() << " " << col2.z() << " " << col3.z() << " " <<
    //               t.z() << std::endl;
    //    }

    for (int i = 0; i < _estimates.size(); i++) {
        auto &pose = _estimates.at(X(i)).cast<gtsam::Pose3>();
        //        gtsam::Point3 p = pose.translation();
        //        gtsam::Quaternion q = pose.rotation().toQuaternion();
        //        stream << keyframeTimes.at(i) << " " << p.x() << " " << p.y()
        //               << " " << p.z() << " "
        //               << q.x() << " " << q.y() << " "
        //               << q.z() << " " << q.w() << std::endl;

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1);
        auto col2 = R.column(2);
        auto col3 = R.column(3);

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
               << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
               << t.y() << " " << col1.z() << " " << col2.z() << " " << col3.z()
               << " " << t.z() << std::endl;
    }
}

void DataSaver::saveOptimizedVerticesKITTI(std::vector<Vector7> _estimates) {
    std::fstream stream(save_directory + "optimized_odom_kitti.txt",
                        std::fstream::out);
    stream.precision(15);
    for (int i = 0; i < _estimates.size(); i++) {
        auto &pose = _estimates.at(i);

        Rot3 R(pose[6], pose[3], pose[4], pose[5]);
        auto col1 = R.column(1);
        auto col2 = R.column(2);
        auto col3 = R.column(3);

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << pose[0]
               << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
               << pose[1] << " " << col1.z() << " " << col2.z() << " " << col3.z()
               << " " << pose[2] << std::endl;
    }
}

void DataSaver::saveOdometryVerticesKITTI(std::string _filename) {
    //  std::fstream stream(_filename.c_str(), std::fstream::out);
    //  stream.precision(15);
    //  for (const auto &_pose6d: keyframePoses) {
    //    gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
    //    Point3 t = pose.translation();
    //    Rot3 R = pose.rotation();
    //    auto col1 = R.column(1); // Point3
    //    auto col2 = R.column(2); // Point3
    //    auto col3 = R.column(3); // Point3
    //
    //    stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
    //    << " "
    //           << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y()
    //           << " "
    //           << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z()
    //           << std::endl;
    //  }
}

void DataSaver::saveOriginGPS(Eigen::Vector3d gps_point) {
    std::fstream originStream(save_directory + "origin.txt", std::fstream::out);
    originStream.precision(15);
    originStream << gps_point[0] << " " << gps_point[1] << " " << gps_point[2]
                 << std::endl;
    originStream.close();
}

void DataSaver::saveTimes(vector<double> _keyframeTimes) {
    if (_keyframeTimes.empty()) {
        //    LOG(ERROR) << "EMPTY KEYFRAME TIMES!";
        return;
    }
    this->keyframeTimes = _keyframeTimes;

    std::fstream pgTimeSaveStream(save_directory + "times.txt",
                                  std::fstream::out);
    pgTimeSaveStream.precision(15);

    // save timestamp
    for (auto const timestamp : keyframeTimes) {
        pgTimeSaveStream << timestamp << std::endl;  // path
    }

    pgTimeSaveStream.close();
}

void DataSaver::saveOdometryVerticesTUM(
        std::vector<nav_msgs::Odometry> keyframePosesOdom) {
    std::fstream stream(save_directory + "odom_tum.txt", std::fstream::out);
    stream.precision(15);
    for (int i = 0; i < keyframePosesOdom.size(); i++) {
        nav_msgs::Odometry odometry = keyframePosesOdom.at(i);
        double time = odometry.header.stamp.toSec();
        // check the size of keyframeTimes
        stream << time << " " << odometry.pose.pose.position.x << " "
               << odometry.pose.pose.position.y << " "
               << odometry.pose.pose.position.z << " "
               << odometry.pose.pose.orientation.x << " "
               << odometry.pose.pose.orientation.y << " "
               << odometry.pose.pose.orientation.z << " "
               << odometry.pose.pose.orientation.w << std::endl;
    }
}

void DataSaver::saveGraphGtsam(gtsam::NonlinearFactorGraph gtSAMgraph,
                               gtsam::ISAM2 *isam,
                               gtsam::Values isamCurrentEstimate) {
    gtsam::writeG2o(gtSAMgraph, isamCurrentEstimate,
                    save_directory + "pose_graph.g2o");
    gtsam::writeG2o(isam->getFactorsUnsafe(), isamCurrentEstimate,
                    save_directory + "pose_graph.g2o");
    std::cout << "WRITE G2O FILE: " << save_directory + "pose_graph.g2o"
              << std::endl;
    std::cout << "Variable size: " << isamCurrentEstimate.size() << std::endl;
    std::cout << "Nonlinear factor size: " << isam->getFactorsUnsafe().size()
              << std::endl;
}

void DataSaver::saveGraph(std::vector<nav_msgs::Odometry> keyframePosesOdom) {
    std::fstream g2o_outfile(save_directory + "odom.g2o", std::fstream::out);
    g2o_outfile.precision(15);
    // g2o_outfile << std::fixed << std::setprecision(9);

    for (int i = 0; i < keyframePosesOdom.size(); i++) {
        nav_msgs::Odometry odometry = keyframePosesOdom.at(i);
        double time = odometry.header.stamp.toSec();

        g2o_outfile << "VERTEX_SE3:QUAT " << std::to_string(i) << " ";
        g2o_outfile << odometry.pose.pose.position.x << " ";
        g2o_outfile << odometry.pose.pose.position.y << " ";
        g2o_outfile << odometry.pose.pose.position.z << " ";
        g2o_outfile << odometry.pose.pose.orientation.x << " ";
        g2o_outfile << odometry.pose.pose.orientation.y << " ";
        g2o_outfile << odometry.pose.pose.orientation.z << " ";
        g2o_outfile << odometry.pose.pose.orientation.w << std::endl;
    }
    //  LOG(INFO) << "WRITE G2O VERTICES: " << keyframePosesOdom.size();
    g2o_outfile.close();
}

void DataSaver::saveLogBag(std::vector<Vector12> logVec) {
    // save log files
    //    rosbag::Bag result_bag;
    //    result_bag.open(save_directory + sequence_name + "_log.bag",
    //    rosbag::bagmode::Write);
    //
    //    if (logVec.size()){
    //        ROS_ERROR("SAVE RESULT BAG FAILED, EMPTY!");
    //        return;
    //    }
    //
    //    for (int i = 0; i < logVec.size(); ++i) {
    //
    //    }
}

void DataSaver::saveResultBag(
        std::vector<nav_msgs::Odometry> allOdometryVec,
        std::vector<nav_msgs::Odometry> updatedOdometryVec,
        std::vector<sensor_msgs::PointCloud2> allResVec) {
    rosbag::Bag result_bag;
    result_bag.open(save_directory + sequence_name + "_result.bag",
                    rosbag::bagmode::Write);

    std::cout << "save bias and velocity " << allOdometryVec.size() << ", "
              << updatedOdometryVec.size() << std::endl;
    for (int i = 0; i < allOdometryVec.size(); i++) {
        nav_msgs::Odometry _laserOdometry = allOdometryVec.at(i);
        result_bag.write("lio_odometry", _laserOdometry.header.stamp,
                         _laserOdometry);

        nav_msgs::Odometry updateOdometry = updatedOdometryVec.at(i);
        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
                         updateOdometry);

        //        sensor_msgs::PointCloud2 _laserCloudFullRes = allResVec.at(i);
        //        result_bag.write("cloud_deskewed",
        //        _laserCloudFullRes.header.stamp, _laserCloudFullRes);
    }
    std::cout << "save bias and velocity " << allOdometryVec.size() << std::endl;

    //    for (int i = 0; i < updatedOdometryVec.size(); i++) {
    //        nav_msgs::Odometry _laserOdometry = updatedOdometryVec.at(i);
    //        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
    //        _laserOdometry);
    //    }

    //    for (int i = 0; i < allResVec.size(); i++) {
    //        sensor_msgs::PointCloud2 _laserCloudFullRes = allResVec.at(i);
    //        result_bag.write("cloud_deskewed", _laserCloudFullRes.header.stamp,
    //        _laserCloudFullRes);
    //    }
    result_bag.close();
}

void DataSaver::saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
                              std::vector<sensor_msgs::PointCloud2> allResVec) {
    rosbag::Bag result_bag;
    result_bag.open(save_directory + sequence_name + "_result.bag",
                    rosbag::bagmode::Write);

    std::cout << "odom and cloud size: " << allOdometryVec.size() << "--"
              << allResVec.size() << std::endl;

    if (allOdometryVec.size() == allOdometryVec.size()) {
        ROS_ERROR("SAVE RESULT BAG FAILED");
        return;
    }

    //  LOG(INFO) << "ODOM AND PCD SIZE:" << allOdometryVec.size() << ", " <<
    //  allResVec.size();
    for (int i = 0; i < allOdometryVec.size(); i++) {
        nav_msgs::Odometry _laserOdometry = allOdometryVec.at(i);
        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
                         _laserOdometry);
    }

    for (int i = 0; i < allResVec.size(); i++) {
        sensor_msgs::PointCloud2 _laserCloudFullRes = allResVec.at(i);
        result_bag.write("cloud_deskewed", _laserCloudFullRes.header.stamp,
                         _laserCloudFullRes);
    }
    result_bag.close();
}

void DataSaver::saveResultBag(
        std::vector<nav_msgs::Odometry> allOdometryVec,
        std::vector<pcl::PointCloud<PointT>::Ptr> allResVec) {
    rosbag::Bag result_bag;
    result_bag.open(save_directory + sequence_name + "_result.bag",
                    rosbag::bagmode::Write);

    //  LOG(INFO) << "ODOM AND PCD SIZE:" << allOdometryVec.size() << ", " <<
    //  allResVec.size();

    for (int i = 0; i < allResVec.size(); i++) {
        nav_msgs::Odometry _laserOdometry = allOdometryVec.at(i);
        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
                         _laserOdometry);

        sensor_msgs::PointCloud2 pointCloud2Msg;
        pcl::PointCloud<PointT>::Ptr _laserCloudFullRes = allResVec.at(i);
        pcl::toROSMsg(*_laserCloudFullRes, pointCloud2Msg);
        pointCloud2Msg.header = _laserOdometry.header;
        pointCloud2Msg.header.frame_id = "camera_init";
        result_bag.write("cloud_deskewed", _laserOdometry.header.stamp,
                         pointCloud2Msg);
    }
    result_bag.close();
    //  LOG(INFO) << "WRITE ROSBAG: " << save_directory + "_result.bag" << ",
    //  SIZE: " << result_bag.getSize();
}

void DataSaver::saveLoopandImagePair(
        std::map<int, int> loopIndexCheckedMap,
        std::vector<std::vector<int>> all_camera_corre_match_pair) {
    std::ofstream loop_outfile;
    loop_outfile.open(save_directory + "lidar_loop.txt", std::ios::out);
    loop_outfile.precision(15);
    //  LOG(INFO) << "WRITE Lidar LOOP FILE: " << save_directory +
    //  "lidar_loop.txt";

    int j = 0;
    for (auto it = loopIndexCheckedMap.begin(); it != loopIndexCheckedMap.end();
         ++it) {
        int curr_node_idx = it->first;
        int prev_node_idx = it->second;

        //    geometry_msgs::Point p;
        //    p.x = keyframePosesUpdated[curr_node_idx].x;
        //    p.y = keyframePosesUpdated[curr_node_idx].y;
        //    p.z = keyframePosesUpdated[curr_node_idx].z;
        //
        //    p.x = keyframePosesUpdated[prev_node_idx].x;
        //    p.y = keyframePosesUpdated[prev_node_idx].y;
        //    p.z = keyframePosesUpdated[prev_node_idx].z;
        //
        //    // we can write some edges to g2o file
        //    //    g2o_out<<"EDGE_SE3:QUAT "<<curr_node_idx<<" "<<prev_node_idx<<"
        //    "
        //    //        <<p.x() <<" "<<p.y() <<" "<<p.z() <<" "
        //    //        <<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<" ";
        //
        //    if (saveLoopdata) {
        //      std::string common_name = std::to_string(curr_node_idx) + "_" +
        //      std::to_string(prev_node_idx);
        //
        //      std::string pcd_name_0 = common_name + "_0.pcd";
        //      std::string pcd_name_1 = common_name + "_1.pcd";
        //      pcl::io::savePCDFileBinary(pgScansDirectory + pcd_name_0,
        //      *keyframeLaserRawClouds[curr_node_idx]);
        //      pcl::io::savePCDFileBinary(pgScansDirectory + pcd_name_1,
        //      *keyframeLaserRawClouds[prev_node_idx]);
        //
        ////      cv::imwrite(pgImageDirectory + common_name + "_0_0.png",
        /// keyMeasures.at(curr_node_idx).camera0.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_1.png",
        /// keyMeasures.at(curr_node_idx).camera1.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_2.png",
        /// keyMeasures.at(curr_node_idx).camera2.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_3.png",
        /// keyMeasures.at(curr_node_idx).camera3.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_4.png",
        /// keyMeasures.at(curr_node_idx).camera4.front()); /      //
        /// cv::imshow("imgCallback", image_mat);
        ////
        ////      cv::imwrite(pgImageDirectory + common_name + "_1_0.png",
        /// keyMeasures.at(prev_node_idx).camera0.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_1.png",
        /// keyMeasures.at(prev_node_idx).camera1.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_2.png",
        /// keyMeasures.at(prev_node_idx).camera2.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_3.png",
        /// keyMeasures.at(prev_node_idx).camera3.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_4.png",
        /// keyMeasures.at(prev_node_idx).camera4.front());
        //      cv::imwrite(pgImageDirectory + common_name + "_1_4.png",
        //      resultMat_vec.at(j));
        //    }
        j++;
        loop_outfile.precision(15);
        loop_outfile << std::to_string(curr_node_idx) << " "
                     << std::to_string(prev_node_idx);
        loop_outfile << std::endl;
    }
    loop_outfile.close();
    //  LOG(INFO) << "SAVE LOOP FILE: " << loopIndexCheckedMap.size();

    // save camera pairs if their correspondences are sufficient
    std::ofstream camera_pair_outfile;
    camera_pair_outfile.open(save_directory + "camera_pair_indices.txt",
                             std::ios::out);
    //  LOG(INFO) << "WRITE CAMERA PAIR FILE: " << save_directory +
    //  "camera_pair_indices.txt"; LOG(INFO) << "Matching camera size: " <<
    //  all_camera_corre_match_pair.size();
    for (const auto &camera_pair : all_camera_corre_match_pair) {
        int lidar_idx_1 = camera_pair[0];
        int lidar_idx_2 = camera_pair[1];
        int cam_idx_1 = camera_pair[2];
        int cam_idx_2 = camera_pair[3];
        int num_corr = camera_pair[4];
        camera_pair_outfile << lidar_idx_1 << " " << lidar_idx_2 << " " << cam_idx_1
                            << " " << cam_idx_2 << " " << num_corr << std::endl;
    }
    camera_pair_outfile.close();
}

void DataSaver::savePointCloudMap(
        std::vector<nav_msgs::Odometry> allOdometryVec,
        std::vector<pcl::PointCloud<PointT>::Ptr> allResVec) {
    std::cout << "odom and cloud size: " << allOdometryVec.size() << ", "
              << allResVec.size() << std::endl;
    if (allOdometryVec.size() != allResVec.size()) {
        std::cout << "point cloud size do not equal to odom size!" << std::endl;
    }

    pcl::PointCloud<PointT>::Ptr laserCloudRaw(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr laserCloudTrans(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr traj_cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
    traj_cloud->width = allOdometryVec.size();
    traj_cloud->height = 1;
    traj_cloud->is_dense = false;

    for (int i = 0; i < allOdometryVec.size(); ++i) {
        nav_msgs::Odometry odom = allOdometryVec.at(i);
        laserCloudRaw = allResVec.at(i);

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.rotate(Eigen::Quaterniond(
                odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
        transform.pretranslate(Eigen::Vector3d(odom.pose.pose.position.x,
                                               odom.pose.pose.position.y,
                                               odom.pose.pose.position.z));
        pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans,
                                 transform.matrix());
        *globalmap += *laserCloudTrans;

        pcl::PointXYZ pt;
        pt.x = odom.pose.pose.position.x;
        pt.y = odom.pose.pose.position.y;
        pt.z = odom.pose.pose.position.z;
        traj_cloud->points.push_back(pt);

        if (save_key_frame) {
            std::cout << " process point cloud: " << i << "/" << allOdometryVec.size()
                      << std::endl;
            if (!laserCloudRaw->empty()) {
                pcl::io::savePCDFileASCII(keyFrmaePath + std::to_string(i) + ".pcd",
                                          *laserCloudRaw);
            } else
                std::cout << "empty key frame!" << std::endl;
        }
    }

    std::cout << " save traj point cloud: " << traj_cloud->size() << std::endl;
    if (!traj_cloud->empty()) {
        pcl::io::savePCDFileASCII(save_directory + "traj_pcd_lidar.pcd",
                                  *traj_cloud);
    }

    std::cout << " save global point cloud: " << globalmap->size() << std::endl;

    // save point cloud in lidar frame
    // if you want to save it in body frame(imu)
    // i will update it later
    if (!globalmap->empty()) {
        globalmap->width = globalmap->points.size();
        globalmap->height = 1;
        globalmap->is_dense = false;

        // all cloud must rotate to body axis
        if (saveResultBodyFrame) {
            if (use_imu_frame) {
                try {
                    for (int j = 0; j < globalmap->points.size(); ++j) {
                        PointT &pt = globalmap->points.at(j);
                        Eigen::Vector3d translation(pt.x, pt.y, pt.z);
                        translation = q_body_sensor * translation + t_body_sensor;

                        pt.x = translation[0];
                        pt.y = translation[1];
                        pt.z = translation[2];
                    }
                    pcl::io::savePCDFileASCII(save_directory + "global_pcd_imu.pcd",
                                              *globalmap);
                } catch (std::exception e) {
                    std::cerr << "save map falied! " << std::endl;
                }

            } else {
                pcl::io::savePCDFileASCII(save_directory + "global_pcd_lidar.pcd",
                                          *globalmap);
            }
        }
    }
    std::cout << "save global map finished! " << globalmap->size() << std::endl;
}

void DataSaver::savePointCloudMap(
        std::vector<Eigen::Isometry3d> allOdometryVec,
        std::vector<pcl::PointCloud<PointT>::Ptr> allResVec) {
    std::cout << "icp odom and cloud size: " << allOdometryVec.size() << ", "
              << allResVec.size() << std::endl;
    if (allOdometryVec.size() != allResVec.size()) {
        std::cout << "icp point cloud size do not equal to odom size!" << std::endl;
    }

    pcl::PointCloud<PointT>::Ptr laserCloudRaw(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr laserCloudTrans(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr traj_cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
    traj_cloud->width = allOdometryVec.size();
    traj_cloud->height = 1;
    traj_cloud->is_dense = false;

    for (int i = 0; i < allOdometryVec.size(); ++i) {
        //    laserCloudRaw->clear();
        //    laserCloudTrans->clear();

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform = allOdometryVec.at(i);
        laserCloudRaw = allResVec.at(i);

        if (i % 100 == 0) {
            std::cout << "laser raw size: " << i << " " << laserCloudRaw->size()
                      << std::endl;
        }

        pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans,
                                 transform.matrix());

        if (i % 100 == 0) {
            std::cout << "laser trans size: " << i << " " << laserCloudTrans->size()
                      << " " << std::endl;
        }

        *globalmap += *laserCloudTrans;

        pcl::PointXYZ pt;
        pt.x = transform.translation().x();
        pt.y = transform.translation().y();
        pt.z = transform.translation().z();
        traj_cloud->points.push_back(pt);

        /*if (save_key_frame) {
          std::cout << " process point cloud: " << i << "/" << allOdometryVec.size()
                    << std::endl;
          if (!laserCloudRaw->empty()) {
            pcl::io::savePCDFileASCII(keyFrmaePath + std::to_string(i) + ".pcd",
                                      *laserCloudRaw);
          } else
            std::cout << "empty key frame " << i << std::endl;
        }*/
    }

    std::cout << "icp save traj point cloud: " << traj_cloud->size() << std::endl;
    if (!traj_cloud->empty()) {
        pcl::io::savePCDFileASCII(save_directory + "traj_icp.pcd",
                                  *traj_cloud);
    }

    std::cout << "icp save global point cloud: " << globalmap->size()
              << std::endl;

    // save point cloud in lidar frame
    // if you want to save it in body frame(imu)
    // i will update it later
    if (!globalmap->empty()) {
        globalmap->width = globalmap->points.size();
        globalmap->height = 1;
        globalmap->is_dense = false;

        // all cloud must rotate to body axis
        if (saveResultBodyFrame) {
            if (use_imu_frame) {
                try {
                    for (int j = 0; j < globalmap->points.size(); ++j) {
                        PointT &pt = globalmap->points.at(j);
                        Eigen::Vector3d translation(pt.x, pt.y, pt.z);
                        translation = q_body_sensor * translation + t_body_sensor;

                        pt.x = translation[0];
                        pt.y = translation[1];
                        pt.z = translation[2];
                    }
                    pcl::io::savePCDFileASCII(save_directory + "map_icp.pcd",
                                              *globalmap);
                } catch (std::exception e) {
                    std::cerr << "save icp map falied! " << std::endl;
                }

            } else {
                pcl::io::savePCDFileASCII(save_directory + "map_icp.pcd",
                                          *globalmap);
            }
        }
    }
    std::cout << "icp save global map finished! " << globalmap->size()
              << std::endl;
}

void DataSaver::savePointCloudMapLIO(
        std::vector<nav_msgs::Odometry> allOdometryVec,
        std::vector<pcl::PointCloud<PointT>::Ptr> allResVec) {
    std::cout << "lio odom and cloud size: " << allOdometryVec.size() << ", "
              << allResVec.size();
    if (allOdometryVec.size() != allResVec.size()) {
        std::cout << "lio point cloud size do not equal to odom size!";
    }
    pcl::PointCloud<PointT>::Ptr laserCloudRaw(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr laserCloudTrans(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr traj_cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
    traj_cloud->width = allOdometryVec.size();
    traj_cloud->height = 1;
    traj_cloud->is_dense = false;

    for (int i = 0; i < allOdometryVec.size(); ++i) {
        nav_msgs::Odometry odom = allOdometryVec.at(i);
        laserCloudRaw = allResVec.at(i);

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.rotate(Eigen::Quaterniond(
                odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
        transform.pretranslate(Eigen::Vector3d(odom.pose.pose.position.x,
                                               odom.pose.pose.position.y,
                                               odom.pose.pose.position.z));

        pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans,
                                 transform.matrix());

        pcl::PointXYZ pt;
        pt.x = odom.pose.pose.position.x;
        pt.y = odom.pose.pose.position.y;
        pt.z = odom.pose.pose.position.z;
        traj_cloud->points.push_back(pt);
        /* if (save_key_frame) {
           std::cout << " process lio point cloud: " << i << "/"
                     << allOdometryVec.size() << std::endl;
           if (!laserCloudRaw->empty())
             pcl::io::savePCDFileASCII(
                 keyFrmaePath + std::to_string(i) + "_lidar_lio.pcd",
                 *laserCloudRaw);
         }*/
        *globalmap += *laserCloudTrans;
    }

    std::cout << " save lio traj point cloud: " << traj_cloud->size()
              << std::endl;
    if (!traj_cloud->empty()) {
        pcl::io::savePCDFileASCII(save_directory + "traj_pcd_lidar_lio.pcd",
                                  *traj_cloud);
    }

    std::cout << " save lio global point cloud: " << globalmap->size()
              << std::endl;

    // save point cloud in lidar frame
    // if you want to save it in body frame(imu)
    // i will update it later
    if (!globalmap->empty()) {
        globalmap->width = globalmap->points.size();
        globalmap->height = 1;
        globalmap->is_dense = false;

        // all cloud must rotate to body axis
        if (saveResultBodyFrame) {
            if (use_imu_frame) {
                try {
                    for (int j = 0; j < globalmap->points.size(); ++j) {
                        PointT &pt = globalmap->points.at(j);
                        Eigen::Vector3d translation(pt.x, pt.y, pt.z);
                        translation = q_body_sensor * translation + t_body_sensor;

                        pt.x = translation[0];
                        pt.y = translation[1];
                        pt.z = translation[2];
                    }
                    pcl::io::savePCDFileASCII(save_directory + "map_lio.pcd",
                                              *globalmap);
                } catch (std::exception e) {
                    std::cerr << "save lio map falied! " << std::endl;
                }

            } else {
                pcl::io::savePCDFileASCII(save_directory + "map_lio.pcd",
                                          *globalmap);
            }
        }
    }
    std::cout << "save lio  map finished! " << globalmap->size()
              << std::endl;
}

void DataSaver::writeDeskedFrame(pcl::PointCloud<PointT>::Ptr pc, int index) {
    std::string path = keyFrmaePath + std::to_string(index) + ".pcd";
    std::cout << " process point cloud: " << index << ", " << path << std::endl;
    if (!pc->empty()) pcl::io::savePCDFileASCII(path.c_str(), *pc);
}

void DataSaver::saveColorCloudMap(
        std::vector<nav_msgs::Odometry> allOdometryVec,
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> allResVec) {
    std::cout << "odom and cloud size: " << allOdometryVec.size() << ", "
              << allResVec.size();

    if (allOdometryVec.size() != allResVec.size()) {
        std::cout << "point cloud size do not equal to odom size!";
        //  return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudRaw(
            new pcl::PointCloud<pcl::PointXYZRGB>());  // giseop
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudTrans(
            new pcl::PointCloud<pcl::PointXYZRGB>());  // giseop
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalmap(
            new pcl::PointCloud<pcl::PointXYZRGB>());  // giseop
    for (int i = 0; i < allOdometryVec.size(); ++i) {
        laserCloudRaw->clear();
        laserCloudTrans->clear();
        nav_msgs::Odometry odom = allOdometryVec.at(i);
        laserCloudRaw = allResVec.at(i);

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.rotate(Eigen::Quaterniond(
                odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
        transform.pretranslate(Eigen::Vector3d(odom.pose.pose.position.x,
                                               odom.pose.pose.position.y,
                                               odom.pose.pose.position.z));

        pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans,
                                 transform.matrix());

        if (save_key_frame) {
            std::cout << " process rgb point cloud: " << i << "/"
                      << allOdometryVec.size() << std::endl;
            if (!laserCloudRaw->empty())
                pcl::io::savePCDFileASCII(
                        keyColorFrmaePath + std::to_string(i) + ".pcd", *laserCloudRaw);
        }
        *globalmap += *laserCloudTrans;
    }

    // save point cloud in lidar frame
    // if you want to save it in body frame(imu)
    // i will update it later
    if (!globalmap->empty()) {
        globalmap->width = globalmap->points.size();
        globalmap->height = 1;
        globalmap->is_dense = false;

        // all cloud must rotate to body axis
        if (saveResultBodyFrame) {
            if (use_imu_frame) {
                for (int j = 0; j < globalmap->points.size(); ++j) {
                    pcl::PointXYZRGB &pt = globalmap->points.at(j);
                    Eigen::Vector3d translation(pt.x, pt.y, pt.z);
                    translation = q_body_sensor * translation + t_body_sensor;

                    pt.x = translation[0];
                    pt.y = translation[1];
                    pt.z = translation[2];
                }
                pcl::io::savePCDFileASCII(save_directory + "global_colormap_imu.pcd",
                                          *globalmap);
            } else {
                pcl::io::savePCDFileASCII(save_directory + "global_colormap_lidar.pcd",
                                          *globalmap);
            }
        } else {
            pcl::io::savePCDFileASCII(save_directory + "global_colormap_lidar.pcd",
                                      *globalmap);
        }
    } else
        std::cout << "EMPTY POINT CLOUD";
}

void DataSaver::saveColorCloudMap(pcl::PointCloud<pcl::PointXYZRGB> cloud_ptr) {
    if (cloud_ptr.empty()) {
        std::cout << "empty color map cloud!" << std::endl;
        return;
    }
    pcl::io::savePCDFileASCII(save_directory + "globalmap_color.pcd", cloud_ptr);
}
