/*******************************************************
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 *
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
#ifndef SRC_PALOC_SRC_PALOC_H_
#define SRC_PALOC_SRC_PALOC_H_

#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot2.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/inference/inferenceExceptions.h"
#include "gtsam/navigation/CombinedImuFactor.h"
#include "gtsam/navigation/GPSFactor.h"
#include "gtsam/navigation/ImuFactor.h"
#include "gtsam/nonlinear/DoglegOptimizer.h"
#include "gtsam/nonlinear/GaussNewtonOptimizer.h"
#include "gtsam/nonlinear/GncOptimizer.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/slam/dataset.h"

// pcl and open3d
#include "open3d/Open3D.h"
#include "pcl/features/normal_3d.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/io/pcd_io.h"


//ros
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_srvs/Empty.h"

#include "chrono"
#include "csignal"
#include "fstream"
#include "iomanip"
#include "memory"
#include "string"
#include "thread"
#include "unordered_map"
#include "utility"
#include "vector"
#include "iostream"
#include "vector"
#include "cmath"
#include "eigen3/Eigen/Core"
#include "algorithm"

#include "base_type.hpp"
#include "cloud_process.h"
#include "data_saver.h"
#include "factors/gravity_factor.h"
#include "factors/gravity_kalman_filter.h"
#include "tic_toc.h"

using namespace gtsam;
using namespace open3d;
using symbol_shorthand::G;
typedef Eigen::Matrix<double, 2, 18> Vector28;

struct State {
    Pose3 pose;
    Velocity3 vel;
    Matrix6 cov;

    State() {
        pose = Pose3::Identity();
        vel = Velocity3::Zero();
        cov = Matrix6::Identity();
    };

};
class PALoc {
public:
    PALoc(ros::NodeHandle &nh) {
        LoadRosParams(nh);

        srvSaveMap = nh.advertiseService("/save_map", &PALoc::SaveMap, this);
        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/current_cloud", 10);
        pubLaserCloudCrop = nh.advertise<sensor_msgs::PointCloud2>("/crop_cloud", 10);
        pubOdomAftPGO = nh.advertise<nav_msgs::Odometry>("/pgo_odom", 100);
        pubOdomAftGlobal = nh.advertise<nav_msgs::Odometry>("/global_odom", 100);
        pubPathAftPGO = nh.advertise<nav_msgs::Path>("/fusion_path", 100);
        pubPathLIO = nh.advertise<nav_msgs::Path>("/lio_path", 100);
        pubPathIMU = nh.advertise<nav_msgs::Path>("/imu_path", 100);
        pubLocalizationPath = nh.advertise<nav_msgs::Path>("/global_path", 100);
        pubMapAftPGO = nh.advertise<sensor_msgs::PointCloud2>("/pgo_map", 100);
        pubGlobalMapConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/map_constraint", 1);
        pubZUPTConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/zupt_constraint", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/lio_loops", 1);
        pubLoopScanLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_scan_local", 100);
        pubLoopSubmapLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_submap_local", 100);
        pubLocalizationmap = nh.advertise<sensor_msgs::PointCloud2>("/localization_map", 100);
        pubInitialCloud = nh.advertise<sensor_msgs::PointCloud2>("/initial_cloud", 100);
        pubPoseBToMap = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/base_link2map", 100);
        pubPoseOdomToMap = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/odom2map", 100);

        subImu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 2000, &PALoc::ImuCallback, this,
                                                ros::TransportHints().tcpNoDelay());
        subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered_body", 10000,
                                                                      &PALoc::LidarCallback, this);

        // if you want to use liosam
        //                "/cloud_deskewed", 10000, &PALoc::LidarCallback, this);

        //        subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>(
        //                "/cloud_effected", 10000, &PALoc::LidarCallback, this);

        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/lidar_odometry", 10000, &PALoc::OdometryCallback, this);
        //                "/lio_sam_6axis/mapping/odometry", 10000, &PALoc::OdometryCallback, this);


        subInitialPose = nh.subscribe("/initialpose", 10000, &PALoc::InitialCallback, this);

        // load prior pose
        nh.param<vector<double>>("common/initial_pose", initial_pose_vector, vector<double>());
        InitParmeters();
    }

    ~PALoc() {}

    void pose_slam();

    void LoopDection();

    void VisaulizationThread();


private:
    void InitParmeters();

    void InitSystem(Measurement &measurement);

    bool SyncData(Measurement &measurement);

    void AddOdomFactor();

    pcl::PointCloud<pcl::PointXYZI>::Ptr extractLocalMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &global_map,
                                                         const Eigen::Matrix4d &initial_pose_matrix,
                                                         float radius);

    void AddMapPriorFactorO3D();

    void AddMapPriorFactor();

    void EstimateGravity();

    Eigen::Vector3d estimateGravity(const int n);

    void AddNoMotionFactor();

    Eigen::Quaterniond integrateGyroscope(const Eigen::Quaterniond &q, const Eigen::Vector3d &omega, double dt);

    void AddGravityFactor();

    void AddLoopFactor();

    bool ZUPTDetector();

    void GraphOpt();

    void PerformRSLoopClosure();

    void SaveData();

    bool SaveMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg_ptr);

    void OdometryCallback(const nav_msgs::OdometryConstPtr &odom_msg_ptr);

    void ImuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg);

    void InitialCallback(
            const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg_ptr);

    void PubPath(void);

    void PubMap(void);

    void VisualizeLoopConstrains(std::map<int, int> loopMap,
                                 ros::Publisher &publisher, int type);

    bool FilterLoopPairs(int loopKeyCur, int loopKeyPre);

    bool DetectLoopClosureDistance(int *loopKeyCur, int *loopKeyPre);


    bool GetGlobalICP(Measurement &measurement_temp);

    bool Point2PlaneICPLM(pcl::PointCloud<PointT>::Ptr measure_cloud, pcl::PointCloud<PointT>::Ptr target_cloud,
                          Pose6D &transform, double search_radius);

    void SetLoopscore(float loopNoiseScore);


    void PublishPose(const ros::Time &time, const ros::Publisher &topic_pub,
                     const std::string &base_frame_id,
                     const Eigen::Matrix4d &transform_matrix);

    State GetStateFromLIO2(const int node_id);

    inline Eigen::Affine3f Pose6dToAffine3f(Pose6D pose) {
        return pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch,
                                      pose.yaw);
    }

    inline gtsam::Pose3 Pose6dTogtsamPose3(Pose6D pose) {
        return gtsam::Pose3(
                gtsam::Rot3::RzRyRx(double(pose.roll), double(pose.pitch),
                                    double(pose.yaw)),
                gtsam::Point3(double(pose.x), double(pose.y), double(pose.z)));
    }

    inline float pointDistance(PointT p1, PointT p2) {
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
                    (p1.z - p2.z) * (p1.z - p2.z));
    }

    inline Eigen::Vector3d max(Eigen::Vector3d v1, Eigen::Vector3d v2) {
        return Eigen::Vector3d(std::max(v1.x(), v2.x()), std::max(v1.y(), v2.y()),
                               std::max(v1.z(), v2.z()));
    }

    inline Eigen::Vector3d min(Eigen::Vector3d v1, Eigen::Vector3d v2) {
        return Eigen::Vector3d(std::min(v1.x(), v2.x()), std::min(v1.y(), v2.y()),
                               std::min(v1.z(), v2.z()));
    }

    inline void pointBodyToGlobal(PointT const pi, PointT &po, Eigen::Matrix4d pose) {
        Eigen::Vector3d p_body(pi.x, pi.y, pi.z);
        Eigen::Vector3d p_global(pose.block<3, 3>(0, 0) * p_body +
                                 pose.block<3, 1>(0, 3));

        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
    }

    ros::Subscriber subLaserCloudFullRes;
    ros::Subscriber subImu;
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subLoop;
    ros::Subscriber subInitialPose;

    ros::Publisher pubMapAftPGO, pubOdomAftPGO, pubOdomAftGlobal;
    ros::Publisher pubPathLIO, pubPathAftPGO, pubLocalizationPath,
            pubPathBeforPGO, pubPathIMU;
    ros::Publisher pubLaserCloudSurround, pubLoopScanLocal, pubLoopSubmapLocal;
    ros::Publisher pubLocalizationmap, pubInitialCloud;
    ros::Publisher pubPoseOdomToMap, pubPoseBToMap, pubLaserCloudCrop;

    ros::Publisher pubLoopConstraintEdge,
            pubGlobalMapConstraintEdge, pubZUPTConstraintEdge;
    ros::Publisher pubLoopScanLocalRegisted, pubPathGnssCoords;

    ros::ServiceServer srvSaveMap;

    std::mutex mKF;
    std::mutex mutexLock;
    std::mutex mtxPosegraph;
    std::mutex mtxICP;
    std::mutex imuLock;

    Measurement measurement_curr;
    double lastImuTime = -1;
    std::deque<sensor_msgs::Imu> last_imu_deque_;
    Measurement last_measurement;

    map<int, int> mapIndexContainer;
    map<int, int> ZUPTIndexContainer;
    // for loop closure detection
    int lastLoopIndex = -1;
    map<int, int> loopIndexCheckedMap;

    bool isInitialized = false;
    bool poseReceived = false;
    Eigen::Vector3d init_gravity = Eigen::Vector3d::Zero();

    bool aLoopIsClosed = false;
    bool aGlobalConstrained = false;


    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeHistoryKeyPoses;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfFromMap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfFromLocalmap;
    pcl::PointCloud<PointT>::Ptr laserCloudEffective;
    pcl::PointCloud<PointT>::Ptr laserCloudEffectiveOri;
    pcl::PointCloud<PointT>::Ptr coeffSel;
    Eigen::Matrix<float, 6, 6> matP_eigen;
    std::vector<double> residuals_vec;

    int iterate_number = 0;
    Vector3d measured_gravity;
    std::mutex mtxLoopContainer;
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    vector<noiseModel::Gaussian::shared_ptr> loopGaussianNoiseQueue;

    int prev_node_idx = 0, curr_node_idx = 0;
    const int node_rate = 1;  // 10s jifenyici

    std::vector<Pose6D> keyframePoses2D;
    std::vector<Pose6D> keyframePoses3D;
    std::vector<Vector10> keyICPData;
    Vector10 curr_icp_data;
    std::vector<Vector10> keyTimeData;
    Vector10 curr_time_data;

    std::vector<Measurement> keyMeasures;
    std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
    std::queue<nav_msgs::Odometry::ConstPtr> odometryIMUBuf;
    std::deque<sensor_msgs::Imu> imuQueue;
    Matrix4d initialPose = Eigen::Matrix4d::Identity();
    double priorScore = 0.0;
    Vector28 lioState;
    State lioState2;
    std::vector<State> keyLIOState2;

    bool isDegenerate = false;
    Eigen::Vector3d eigen_values{0, 0, 0};
    Eigen::Vector3d eigen_values_ratio_rpy{0, 0, 0};
    Eigen::Vector3d eigen_values_ratio_xyz{0, 0, 0};

    std::vector<double> initial_pose_vector;
    Eigen::Matrix4d priorPose = Eigen::Matrix4d::Identity();


    NavState propState;
    Pose3 rel_pose;
    Pose3 predict_pose;
    Pose3 prevPose;
    Vector3 prevVel;
    NavState prevState;
    imuBias::ConstantBias prevBias;

    Pose6D odom_pose_prev{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Pose6D odom_pose_curr{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    TicToc tic_all;
    double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0.0;
    double t1_all = 0, t2_all = 0, t3_all = 0, t4_all = 0, t5_all = 0.0;


    std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
    pcl::PointCloud<PointT>::Ptr laserCloudMapPGO;
    pcl::PointCloud<PointT>::Ptr globalmap_ptr;
    pcl::PointCloud<PointT>::Ptr globalmap_filter_ptr;

    // surf point holder for parallel computation
    std::vector<PointT> laserCloudOriSurfVec;
    std::vector<PointT> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    double final_fitness = 0.0, prev_fitness = 0.0;
    double total_rmse = 0.0, prev_rmse = 0.0, curr_rmse = 0.0;
    double total_distance = 0.0;
    double relative_rmse = std::numeric_limits<double>::max();
    double relative_fitness = 1.0;
    Eigen::Matrix<float, 6, 6> icp_cov;

    pcl::VoxelGrid<PointT> downSizeFilterMapPGO;
    pcl::VoxelGrid<PointT> downSizeFilterICP;
    pcl::VoxelGrid<PointT> downSizeFilterScan;

    NonlinearFactorGraph newFactors;
    Values newValues;
    ISAM2 *isam;
    Values currentEstimate;
    Eigen::MatrixXd poseCovariance;

    double zeroVelocitySigma = 0.001;
    double noMotionPositionSigma = 1e-3;
    double noMotionRotationSigma = 1e-4;

    //! No motion factors settings.
    SharedNoiseModel zero_velocity_prior_noise_;
    SharedNoiseModel no_motion_prior_noise_;
    SharedGaussian priorMapPoseGaussianNoise;
    SharedGaussian LOOPGaussianNoise;
    SharedGaussian noise_odom_between;

    //  noiseModel::Diagonal::shared_ptr priorOdomNoise;
    noiseModel::Diagonal::shared_ptr priorPoseNoise;
    noiseModel::Diagonal::shared_ptr priorLIOPoseNoise;
    noiseModel::Diagonal::shared_ptr priorMapPoseNoise;
    noiseModel::Diagonal::shared_ptr priorVelNoise;
    noiseModel::Diagonal::shared_ptr priorBiasNoise;
    noiseModel::Diagonal::shared_ptr noiseModelBetweenBias;
    noiseModel::Diagonal::shared_ptr robustLoopNoise;

    CloudProcess cloud_process_;
    std::unique_ptr<DataSaver> dataSaverPtr;
};

#endif  // SRC_PALOC_SRC_PALOC_H_
