
#ifndef IMAGING_LIDAR_PLACE_RECOGNITION_PGO_INCLUDE_BASE_TYPE_H_
#define IMAGING_LIDAR_PLACE_RECOGNITION_PGO_INCLUDE_BASE_TYPE_H_

//pcl
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/impl/search.hpp>

//ros
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <condition_variable>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <queue>

#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

using namespace std;
typedef pcl::PointXYZI PointT;

extern bool useLoopClosure;
extern bool saveResultBag;
extern bool saveResultBodyFrame;
extern int DEGENERACY_THRES;

extern bool useImuFrame;
extern bool saveKeyFrame;
extern bool showDegenercy;

extern bool useRawCloud;
extern bool useGlobalPrior;
extern int icpO3dType;
extern double correspondenceDis;
extern double mapRadius;
extern double map_filter_size;

extern double historyKeyframeSearchRadius;
extern double historyKeyframeSearchTimeDiff;
extern double loopFitnessScoreThreshold;
extern int historyKeyframeSearchNum;
extern int loopClosureFrequency;

extern double filterDis;
extern double LOOP_Z_OFFSET;
extern int filterNodeNum;
extern int SKIP_FRAMES;

extern std::string configDirectory;
extern std::string priorMapDirectory;
extern std::string saveDirectory;
extern std::string sequence;
extern std::string odom_link;
extern std::string imu_topic;

extern double scan_filter_size;
extern double map_viewer_size;
extern double map_saved_size;

extern double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
extern vector<double> b_gyr_cov_n;
extern vector<double> b_acc_cov_n;

extern vector<double> extrinT;
extern vector<double> extrinR;
extern Eigen::Quaterniond q_body_sensor;
extern Eigen::Matrix3d rot_body_sensor;
extern Eigen::Vector3d t_body_sensor;

struct Pose6D {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

    double vx;
    double vy;
    double vz;

    double bax;
    double bay;
    double baz;

    double bgx;
    double bgy;
    double bgz;
    bool valid;

    Eigen::Matrix<double, 6, 1> cov;
    Eigen::Matrix<double, 6, 6> pose_cov;
    Eigen::Quaterniond quaternion;

    Pose6D() {
        x = y = z = roll = pitch = yaw = 0.0;
        vx = vy = vz = 0.0;
        bax = bay = baz = 0.0;
        bgx = bgy = bgz = 0.0;
        quaternion = Eigen::Quaterniond(1, 0, 0, 0);
        cov = Eigen::Matrix<double, 6, 1>::Identity();
        valid = true;
    }

    Pose6D(double _x, double _y, double _z, double _roll, double _pitch,
           double _yaw)
            : x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw) {
        tf::Quaternion quaternion1 = tf::createQuaternionFromRPY(roll, pitch, yaw);
        quaternion = Eigen::Quaterniond(quaternion1.w(), quaternion1.x(),
                                        quaternion1.y(), quaternion1.z());
        pose_cov.setIdentity();
    };

    void setBias(double _bax, double _bay, double _baz, double _bgx, double _bgy,
                 double _bgz) {
        this->bax = _bax;
        this->bay = _bay;
        this->baz = _baz;
        this->bgx = _bgx;
        this->bgy = _bgy;
        this->bgz = _bgz;
    }

    void setRPY(double _roll, double _pitch, double _yaw) {
        roll = _roll;
        pitch = _pitch;
        yaw = _yaw;
        tf::Quaternion quaternion1 =
                tf::createQuaternionFromRPY(_roll, _pitch, _yaw);
        quaternion = Eigen::Quaterniond(quaternion1.w(), quaternion1.x(),
                                        quaternion1.y(), quaternion1.z());
    }

    Pose6D &operator=(const Pose6D &s) {
        if (this == &s) return *this;
        this->x = s.x;
        this->y = s.y;
        this->z = s.z;
        this->roll = s.roll;
        this->pitch = s.pitch;
        this->yaw = s.yaw;

        this->vx = s.vx;
        this->vy = s.vy;
        this->vz = s.vz;

        this->bax = s.bax;
        this->bay = s.bay;
        this->baz = s.baz;
        this->bgx = s.bgx;
        this->bgy = s.bgy;
        this->bgz = s.bgz;
        this->quaternion = s.quaternion;
        this->cov = s.cov;
        this->valid = s.valid;
        return *this;
    }
};

struct Measurement  // Lidar data and imu dates for the curent process
{
    Measurement() {
        lidar_time = odom_time = distance = 0.0;
        gps.header.seq = 1;
        global_score = 0;
        this->lidar.reset(new pcl::PointCloud<pcl::PointXYZI>());
        this->has_gps = false;
    }
    double lidar_time;
    double odom_time;

    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar;
    nav_msgs::Odometry odom;
    nav_msgs::Odometry imu_odom;
    nav_msgs::Odometry updated_odom;
    nav_msgs::Odometry gps;
    bool has_gps;
    bool has_imu;

    std::deque<sensor_msgs::Imu> imu_deque;

    double distance;
    Pose6D key_pose;
    Pose6D updated_pose;
    Pose6D global_pose;
    double global_score;
};

std::string toString(Pose6D &pose);

extern Pose6D getOdom(nav_msgs::Odometry _odom);

extern Pose6D diffTransformation(const Pose6D &_p1, const Pose6D &_p2);

extern Eigen::Matrix4d Pose6D2Matrix(const Pose6D &p);

extern Pose6D Matrix2Pose6D(const Eigen::Matrix4d &matrix);

extern Pose6D Pose6D2Body(const Pose6D &p, const Eigen::Matrix4d &matrix);

extern Pose6D Isometry3d2Pose6D(const Eigen::Isometry3d &matrix);

extern Eigen::Isometry3d Pose6D2sometry3d(const Pose6D &p);

extern Eigen::Isometry3d OdomToIsometry3d(const nav_msgs::Odometry &pose_geo);

extern Eigen::Isometry3d GeoposeToIsometry3d(
        const geometry_msgs::PoseWithCovarianceStamped &pose_geo);

extern Eigen::Matrix4d GeoposeToMatrix4d(
        const geometry_msgs::PoseWithCovarianceStamped &pose_geo);

pcl::PointCloud<PointT>::Ptr TransformPointCloud(
        const pcl::PointCloud<PointT>::Ptr &cloudIn, const Pose6D &tf);

pcl::PointCloud<PointT>::Ptr TransformPointCloud(
        const pcl::PointCloud<PointT>::Ptr &cloudIn,
        const Eigen::Matrix4d &transformIn);

pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc(
        const std::vector<Pose6D> vectorPose6d);

pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc2d(
        const std::vector<Pose6D> vectorPose6d);


void LoadRosParams(ros::NodeHandle &nh);

template<typename T>
T readParam(ros::NodeHandle &n, std::string name) {
    T ans;
    if (n.getParam(name, ans)) {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    } else {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher &thisPub,
                                      const T &thisCloud, ros::Time thisStamp,
                                      std::string thisFrame) {
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0) thisPub.publish(tempCloud);
    return tempCloud;
}

#endif