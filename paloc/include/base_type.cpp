//
// Created by xchu on 27/5/2022.
//

#include "base_type.hpp"

std::string toString(Pose6D &pose) {
    return "(" + std::to_string(pose.x) + ", " + std::to_string(pose.y) + ", " +
           std::to_string(pose.z) + ", " + std::to_string(pose.roll) + ", " +
           std::to_string(pose.pitch) + ", " + std::to_string(pose.yaw) + ")";
}

extern Pose6D getOdom(nav_msgs::Odometry _odom) {
    auto tx = _odom.pose.pose.position.x;
    auto ty = _odom.pose.pose.position.y;
    auto tz = _odom.pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion quat = _odom.pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w))
            .getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw};
    //    return Pose6D{tx, ty, tz, roll, pitch, yaw, _odom.header.seq};
}

extern Pose6D diffTransformation(const Pose6D &_p1, const Pose6D &_p2) {
    Eigen::Affine3f SE3_p1 =
            pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 =
            pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta;
    SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles(SE3_delta, dx, dy, dz, droll, dpitch, dyaw);

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)),
                  double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
}

extern Eigen::Matrix4d Pose6D2Matrix(const Pose6D &p) {
    Eigen::Translation3d tf_trans(p.x, p.y, p.z);
    Eigen::AngleAxisd rot_x(p.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y(p.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z(p.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d mat = (tf_trans * rot_z * rot_y * rot_x).matrix();
    return mat;
}

extern Pose6D Pose6D2Body(const Pose6D &p, const Eigen::Matrix4d &matrix) {
    Eigen::Translation3d tf_trans(p.x, p.y, p.z);
    Eigen::AngleAxisd rot_x(p.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y(p.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z(p.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d mat = (tf_trans * rot_z * rot_y * rot_x).matrix();

    // lidar -> imu T_imu_lidar * Pose_lidar
    Eigen::Matrix4d imu_pose = matrix * mat;
    Eigen::Vector3d pos = imu_pose.block<3, 1>(0, 3).matrix();
    Eigen::Matrix3d rot = imu_pose.block<3, 3>(0, 0).matrix();
    Eigen::Quaterniond quat(rot);

    Pose6D p2;
    p2.x = pos(0);
    p2.y = pos(1);
    p2.z = pos(2);
    tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()))
            .getRPY(p2.roll, p2.pitch, p2.yaw);
    return p;
}

extern Pose6D Matrix2Pose6D(const Eigen::Matrix4d &matrix) {
    Eigen::Vector3d pos = matrix.block<3, 1>(0, 3).matrix();
    Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0).matrix();
    Eigen::Quaterniond quat(rot);

    Pose6D p;
    p.x = pos(0);
    p.y = pos(1);
    p.z = pos(2);
    tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()))
            .getRPY(p.roll, p.pitch, p.yaw);
    return p;
}

extern Pose6D Isometry3d2Pose6D(const Eigen::Isometry3d &matrix) {
    Eigen::Matrix3d rot = matrix.rotation().matrix();
    Eigen::Quaterniond quat(rot);
    // quat.normalize();

    Pose6D p;
    p.x = matrix.translation().x();
    p.y = matrix.translation().y();
    p.z = matrix.translation().z();
    tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()))
            .getRPY(p.roll, p.pitch, p.yaw);
    return p;
}

extern Eigen::Isometry3d Pose6D2sometry3d(const Pose6D &p) {
    Eigen::Vector3d tf_trans(p.x, p.y, p.z);
    //  Eigen::AngleAxisd rollAngle1(p.roll, Eigen::Vector3d::UnitX());
    //  Eigen::AngleAxisd pitchAngle1(p.pitch, Eigen::Vector3d::UnitY());
    //  Eigen::AngleAxisd yawAngle1(p.yaw, Eigen::Vector3d::UnitZ());
    //  Eigen::Quaterniond quat = yawAngle1 * pitchAngle1 * rollAngle1;

    tf::Quaternion quat = tf::createQuaternionFromRPY(p.roll, p.pitch, p.yaw);
    Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
    mat.rotate(Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z())
                       .toRotationMatrix());
    mat.pretranslate(tf_trans);

    return mat;
}

extern Eigen::Isometry3d OdomToIsometry3d(const nav_msgs::Odometry &pose_geo) {
    Eigen::Isometry3d pose_eigen = Eigen::Isometry3d::Identity();
    pose_eigen.rotate(Eigen::Quaterniond(
            pose_geo.pose.pose.orientation.w, pose_geo.pose.pose.orientation.x,
            pose_geo.pose.pose.orientation.y, pose_geo.pose.pose.orientation.z));
    pose_eigen.pretranslate(Eigen::Vector3d(pose_geo.pose.pose.position.x,
                                            pose_geo.pose.pose.position.y,
                                            pose_geo.pose.pose.position.z));
    return pose_eigen;
}

extern Eigen::Isometry3d GeoposeToIsometry3d(
        const geometry_msgs::PoseWithCovarianceStamped &pose_geo) {
    Eigen::Isometry3d pose_eigen = Eigen::Isometry3d::Identity();
    pose_eigen.rotate(Eigen::Quaterniond(
            pose_geo.pose.pose.orientation.w, pose_geo.pose.pose.orientation.x,
            pose_geo.pose.pose.orientation.y, pose_geo.pose.pose.orientation.z));
    pose_eigen.pretranslate(Eigen::Vector3d(pose_geo.pose.pose.position.x,
                                            pose_geo.pose.pose.position.y,
                                            pose_geo.pose.pose.position.z));
    return pose_eigen;
}

extern Eigen::Matrix4d GeoposeToMatrix4d(
        const geometry_msgs::PoseWithCovarianceStamped &pose_geo) {
    Eigen::Quaterniond test_q(
            pose_geo.pose.pose.orientation.w, pose_geo.pose.pose.orientation.x,
            pose_geo.pose.pose.orientation.y, pose_geo.pose.pose.orientation.z);

    Eigen::Matrix4d test_guess = Eigen::Matrix4d::Identity();
    test_guess.block<3, 3>(0, 0) = test_q.toRotationMatrix();
    test_guess.block<3, 1>(0, 3) = Eigen::Vector3d(pose_geo.pose.pose.position.x,
                                                   pose_geo.pose.pose.position.y,
                                                   pose_geo.pose.pose.position.z);
    return test_guess;
}

pcl::PointCloud<PointT>::Ptr TransformPointCloud(
        const pcl::PointCloud<PointT>::Ptr &cloudIn, const Pose6D &tf) {
    pcl::PointCloud<PointT>::Ptr cloudOut(new pcl::PointCloud<PointT>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur =
            pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

    int numberOfCores = 16;
#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i) {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x +
                                transCur(0, 1) * pointFrom.y +
                                transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x +
                                transCur(1, 1) * pointFrom.y +
                                transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x +
                                transCur(2, 1) * pointFrom.y +
                                transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

pcl::PointCloud<PointT>::Ptr TransformPointCloud(
        const pcl::PointCloud<PointT>::Ptr &cloudIn,
        const Eigen::Matrix4d &transformIn) {
    pcl::PointCloud<PointT>::Ptr cloudOut(new pcl::PointCloud<PointT>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    //  Eigen::Affine3f transCur = pcl::getTransformation(
    //      transformIn->x, transformIn->y, transformIn->z, transformIn->roll,
    //      transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(8)
    for (int i = 0; i < cloudSize; ++i) {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transformIn(0, 0) * pointFrom.x +
                                transformIn(0, 1) * pointFrom.y +
                                transformIn(0, 2) * pointFrom.z + transformIn(0, 3);
        cloudOut->points[i].y = transformIn(1, 0) * pointFrom.x +
                                transformIn(1, 1) * pointFrom.y +
                                transformIn(1, 2) * pointFrom.z + transformIn(1, 3);
        cloudOut->points[i].z = transformIn(2, 0) * pointFrom.x +
                                transformIn(2, 1) * pointFrom.y +
                                transformIn(2, 2) * pointFrom.z + transformIn(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc(
        const std::vector<Pose6D> vectorPose6d) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto p : vectorPose6d) {
        res->points.emplace_back(p.x, p.y, p.z);
    }
    return res;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc2d(
        const std::vector<Pose6D> vectorPose6d) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto p : vectorPose6d) {
        res->points.emplace_back(p.x, p.y, 0);
    }
    return res;
}

bool saveKeyFrame;
bool useLoopClosure;
bool saveResultBag;
bool saveResultBodyFrame;
bool useImuFrame;

bool useRawCloud;
bool useGlobalPrior;
bool showDegenercy;

int icpO3dType;
int DEGENERACY_THRES;
double correspondenceDis;
double mapRadius;
double map_filter_size;

double historyKeyframeSearchRadius;
double historyKeyframeSearchTimeDiff;
double loopFitnessScoreThreshold;
int historyKeyframeSearchNum;
int loopClosureFrequency;

double filterDis;
double LOOP_Z_OFFSET;
int filterNodeNum;
int SKIP_FRAMES;

std::string configDirectory;
std::string priorMapDirectory;
std::string saveDirectory;
std::string sequence;
std::string odom_link;
std::string imu_topic;

double scan_filter_size;
double map_viewer_size;
double map_saved_size;

double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
vector<double> b_gyr_cov_n;
vector<double> b_acc_cov_n;
vector<double> extrinT;
vector<double> extrinR;
Eigen::Quaterniond q_body_sensor;
Eigen::Vector3d t_body_sensor;
Eigen::Matrix3d rot_body_sensor;
Eigen::Matrix4d T_body_lidar;


void LoadRosParams(ros::NodeHandle &nh) {


    nh.param<std::string>("config_directory", configDirectory, "~/Download/paloc");
    nh.param<std::string>("save_directory", saveDirectory, "~/Download/paloc");
    nh.param<std::string>("prior_map_directory", priorMapDirectory, "~/Download/paloc");

    nh.param<std::string>("common/sequence", sequence, "exp05");
    nh.param<std::string>("common/odom_link", odom_link, "camera_init");
    nh.param<bool>("common/useLoopClosure", useLoopClosure, false);

    // get extrinsics between  lidar to imu
    nh.param<double>("common/gyr_cov", gyr_cov, 0.01);
    nh.param<double>("common/acc_cov", acc_cov, 0.001);
    nh.param<double>("common/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("common/b_acc_cov", b_acc_cov, 0.0000001);
    nh.param<vector<double>>("common/extrinsic_T", extrinT, vector<double>(3));
    nh.param<vector<double>>("common/extrinsic_R", extrinR, vector<double>(9));
    nh.param<vector<double>>("common/b_gyr_cov_n", b_gyr_cov_n, vector<double>(3));
    nh.param<vector<double>>("common/b_acc_cov_n", b_acc_cov_n, vector<double>(3));

    t_body_sensor = Eigen::Vector3d::Identity();
    q_body_sensor = Eigen::Quaterniond::Identity();
    rot_body_sensor = Eigen::Matrix3d::Identity();
    rot_body_sensor =
            Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extrinR.data(), 3, 3);
    q_body_sensor = Eigen::Quaterniond(1, 0, 0, 0);
    q_body_sensor = Eigen::Quaterniond(rot_body_sensor);
    std::cout << "extrinsics R: " << rot_body_sensor.matrix() << std::endl;
    t_body_sensor = Eigen::Vector3d(extrinT[0], extrinT[1], extrinT[2]);
    std::cout << "extrinsics T: " << t_body_sensor.transpose() << std::endl;
    T_body_lidar.block<3, 1>(0, 3) = t_body_sensor.matrix();
    T_body_lidar.block<3, 3>(0, 0) = rot_body_sensor.matrix();

    // save map data
    nh.param<bool>("common/saveResultBag", saveResultBag, false);
    nh.param<bool>("common/saveResultBodyFrame", saveResultBodyFrame, false);
    nh.param<bool>("common/saveKeyFrame", saveKeyFrame, false);

    // voxigrid size
    nh.param<double>("common/scan_filter_size", scan_filter_size, 0.1);
    nh.param<double>("common/map_viewer_size", map_viewer_size, 0.5);
    nh.param<double>("common/map_saved_size", map_saved_size, 0.2);

    // if we need to save key frame
    // for loop closure detection
    nh.param<int>("pgo/SKIP_FRAMES", SKIP_FRAMES, 5);
    nh.param<bool>("pgo/useRawCloud", useRawCloud, false);
    nh.param<bool>("pgo/useGlobalPrior", useGlobalPrior, true);
    nh.param<bool>("pgo/showDegenercy", showDegenercy, false);
    nh.param<int>("pgo/icpO3dType", icpO3dType, 2);
    nh.param<int>("pgo/degeneracy_thres", DEGENERACY_THRES, 50);
    nh.param<double>("pgo/correspondence_dis", correspondenceDis, 2.0);
    nh.param<double>("pgo/map_radius", mapRadius, 80.0);
    nh.param<double>("pgo/map_filter_size", map_filter_size, 0.1);

    // for stairs
    nh.param<double>("pgo/loopZOffset", LOOP_Z_OFFSET, 10.0);

    nh.param<double>("pgo/filterDis", filterDis, 20.0);
    nh.param<int>("pgo/filterNodeNum", filterNodeNum, 10);

    nh.param<double>("pgo/historyKeyframeSearchRadius",
                     historyKeyframeSearchRadius, 20.0);
    nh.param<double>("pgo/historyKeyframeSearchTimeDiff",
                     historyKeyframeSearchTimeDiff, 10.0);
    nh.param<int>("pgo/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    nh.param<int>("pgo/loopClosureFrequency", loopClosureFrequency, 1);
    nh.param<double>("pgo/loopFitnessScoreThreshold", loopFitnessScoreThreshold, 0.5);
}
