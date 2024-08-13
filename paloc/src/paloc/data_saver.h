
#ifndef FAST_LIO_SRC_PGO_SRC_DATASAVER_H_
#define FAST_LIO_SRC_PGO_SRC_DATASAVER_H_

#include <nav_msgs/Odometry.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>
#include <iostream>
#include <pcl/search/impl/search.hpp>

// save kml files
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/dataset.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xmlmemory.h>
#include <libxml/xmlstring.h>
#include <libxml/xpath.h>

using namespace std;
using namespace gtsam;
using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G;  // GPS pose
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

using PointT = pcl::PointXYZI;

class DataSaver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DataSaver();

    ~DataSaver();

    DataSaver(string _base_dir, string _sequence_name);

    void setDir(string _base_dir, string _sequence_name);

    void setConfigDir(string _configDirectory);

    void setKeyframe(bool _save_key_frame);

    void setExtrinc(bool _use_imu, bool _saveResultBodyFrame,
                    Eigen::Vector3d _t_body_sensor,
                    Eigen::Quaterniond _q_body_sensor);

    void saveOptimizedVerticesKITTI(gtsam::Values _estimates);

    void saveOptimizedVerticesKITTI(std::vector<Vector7> _estimates);

    void saveOdometryVerticesKITTI(std::string _filename);

    void saveOriginGPS(Eigen::Vector3d gps_point);

    void saveTimes(vector<double> keyframeTimes);

    void saveOptimizedVerticesTUM(gtsam::Values _estimates);

    void saveOptimizedVerticesTUM(std::vector<Vector7> _estimates);

    void saveOptimizedVerticesTUM(std::vector<Vector7> _estimates,
                                  std::string file_name);

    void saveOdometryVerticesTUM(
            std::vector<nav_msgs::Odometry> keyframePosesOdom);

    void saveGraphGtsam(gtsam::NonlinearFactorGraph gtSAMgraph,
                        gtsam::ISAM2 *isam, gtsam::Values isamCurrentEstimate);

    void saveGraph(std::vector<nav_msgs::Odometry> keyframePosesOdom);

    void saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
                       std::vector<nav_msgs::Odometry> updatedOdometryVec,
                       std::vector<sensor_msgs::PointCloud2> allResVec);

    void saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
                       std::vector<sensor_msgs::PointCloud2> allResVec);

    void saveLogBag(std::vector<Vector12> logVec);

    void writeDeskedFrame(pcl::PointCloud<PointT>::Ptr pc, int size);

    void saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
                       std::vector<pcl::PointCloud<PointT>::Ptr> allResVec);

    void saveLoopandImagePair(
            std::map<int, int> loopIndexCheckedMap,
            std::vector<std::vector<int>> all_camera_corre_match_pair);

    void savePointCloudMap(std::vector<nav_msgs::Odometry> allOdometryVec,
                           std::vector<pcl::PointCloud<PointT>::Ptr> allResVec);

    void savePointCloudMap(std::vector<Eigen::Isometry3d> allOdometryVec,
                           std::vector<pcl::PointCloud<PointT>::Ptr> allResVec);

    void savePointCloudMapLIO(
            std::vector<nav_msgs::Odometry> allOdometryVec,
            std::vector<pcl::PointCloud<PointT>::Ptr> allResVec);

    void saveColorCloudMap(
            std::vector<nav_msgs::Odometry> allOdometryVec,
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> allResVec);

    void saveColorCloudMap(pcl::PointCloud<pcl::PointXYZRGB> cloud_ptr);

    int ReadParameter();

    int SaveKMLTrajectory(const std::vector<Eigen::Vector3d> lla_vec);

    int SaveKMLTrajectory(vector<pair<double, double>> WGSBL,
                          vector<double> altitude,
                          vector<pair<int, string>> segmentColor);

public:
    string save_directory, config_directory;

private:
    string base_dir, sequence_name;
    string keyFrmaePath, keyColorFrmaePath;

    vector<string> configParameter;

    bool use_imu_frame = false;
    bool saveResultBodyFrame = false;
    bool save_key_frame = false;

    Eigen::Quaterniond q_body_sensor;
    Eigen::Vector3d t_body_sensor;

    vector<double> keyframeTimes;
};

#endif
