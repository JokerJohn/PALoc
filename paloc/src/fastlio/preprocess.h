#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
//#include <livox_ros_driver/CustomMsg.h>
//#include <fast_lio/CustomMsg.h>
//#include <fast_lio/CustomPoint.h>
#include <fast_lio/fast_lio/CustomMsg.h>
#include <fast_lio/fast_lio/CustomPoint.h>
#include <fast_lio/ikd-Tree/ikd_Tree.h>

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE {
    AVIA = 1, VELO16 = 2, OUST64 = 3, HESAI = 4, MULTI_VELO = 5, MID70 = 6, ROBOSENSE = 7, OUS128 = 8
}; //{1, 2, 3, 4, 5, 6}
enum TIME_UNIT {
    SEC = 0, MS = 1, US = 2, NS = 3
};
enum Feature {
    Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint
};
enum Surround {
    Prev, Next
};
enum E_jump {
    Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind
};

struct orgtype {
    double range;
    double dista;
    double angle[2];
    double intersect;
    E_jump edj[2];
    Feature ftype;

    orgtype() {
        range = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype = Nor;
        intersect = 2;
    }
};
namespace pandar_ros {
    struct Point {
        PCL_ADD_POINT4D

        float intensity;
        double timestamp;
        uint16_t ring;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (double, timestamp, timestamp)
                                          (uint16_t, ring, ring)
)
namespace velodyne_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        float time;
        uint16_t ring;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (float, time, time)
                                          (uint16_t, ring, ring)
)

namespace livox_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t ring;
        float time;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (uint16_t, ring, ring)
                                          (float, time, time)

)


// rostopic echo /lidar_odom_node/meta_cloud/fields
namespace multi_velodyne_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;                 ///< laser intensity reading
        uint16_t ring;                      ///< laser ring number
        float time;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(multi_velodyne_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (uint16_t, ring, ring)
                                          (float, time, time)
)


// own lidar point cloud formate for RAMLAB
namespace ouster_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t ambient;
        float time;
//        uint16_t noise;
//        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
// use std::uint32_t to avoid conflicting with pcl::uint32_t
                (std::uint16_t, reflectivity, reflectivity)
                (std::uint8_t, ring, ring)
                (std::uint16_t, ambient, ambient)
                (float, time, time)
//                (uint16_t, noise, noise)
//                (uint32_t, range, range)
)


// common ouster lidar formate
//namespace ouster_ros {
//  struct EIGEN_ALIGN16 Point {
//      PCL_ADD_POINT4D;
//      float intensity;
//      uint32_t t;
//      uint16_t reflectivity;
//      uint8_t  ring;
//      uint16_t ambient;
//      uint32_t range;
//      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//  };
//}  // namespace ouster_ros
//

//POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
//    (float, x, x)
//    (float, y, y)
//    (float, z, z)
//    (float, intensity, intensity)
//    // use std::uint32_t to avoid conflicting with pcl::uint32_t
//    (std::uint32_t, t, t)
//    (std::uint16_t, reflectivity, reflectivity)
//    (std::uint8_t, ring, ring)
//    (std::uint16_t, ambient, ambient)
//    (std::uint32_t, range, range)
//)
// own lidar point cloud formate for RAMLAB FPV1
namespace ouster_ros_fp1 {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t ambient;
        float time;  // when having  time channel , add this
//        uint16_t noise;
//        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros_fp1::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
// use std::uint32_t to avoid conflicting with pcl::uint32_t
                (std::uint16_t, reflectivity, reflectivity)
                (std::uint8_t, ring, ring)
                (std::uint16_t, ambient, ambient)
                (float, time, time)
//                (uint16_t, noise, noise)
//                (uint32_t, range, range)
)

namespace rs_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D

        float intensity;
        uint16_t ring;                   ///< laser ring number
        double timestamp;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(rs_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)
                                          (double, timestamp, timestamp)
                                          (uint16_t, ring, ring))



// own lidar point cloud formate for RAMLAB
namespace ouster_ros_raw {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t ambient;
        //      float time;  // when having  time channel , add this
//        uint16_t noise;
//        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros_raw::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
// use std::uint32_t to avoid conflicting with pcl::uint32_t
                (std::uint16_t, reflectivity, reflectivity)
                (std::uint8_t, ring, ring)
                (std::uint16_t, ambient, ambient)
//               (float, time, time)
//                (uint16_t, noise, noise)
//                (uint32_t, range, range)
)


// common ouster lidar formate, The newer colleage dataset
namespace ouster_ros_nc {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t ambient;  //os64 do not have, but os128 have this fields
        uint32_t range;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace ouster_ros


POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros_nc::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                          (std::uint32_t, t, t)
                                          (std::uint16_t, reflectivity, reflectivity)
                                          (std::uint8_t, ring, ring)
                                          (std::uint16_t, ambient, ambient)
                                          (std::uint32_t, range, range)
)



class Preprocess {
public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Preprocess();

    ~Preprocess();

//    void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
    void process(const fast_lio::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

    void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

    void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out,
                 PointCloudXYZI::Ptr &pcl_out_full);

    void set(bool feat_en, int lid_type, double bld, int pfilt_num);

    // sensor_msgs::PointCloud2::ConstPtr pointcloud;
    PointCloudXYZI pl_full, pl_corn, pl_surf;
    PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
    vector<orgtype> typess[128]; //maximum 128 line lidar
    float time_unit_scale;
    int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
    double blind;
    bool feature_enabled, given_offset_time;
    ros::Publisher pub_full, pub_surf, pub_corn;


private:
//    void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);

    void avia_handler(const fast_lio::CustomMsg::ConstPtr &msg);

    void livox_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void pandar_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void multi_velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void oust128_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void rs32_handler(
            const sensor_msgs::PointCloud2::ConstPtr &msg);

    void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);

    void pub_func(PointCloudXYZI &pl, const ros::Time &ct);

    int
    plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);

    bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex,
                     Eigen::Vector3d &curr_direct);

    bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

    int group_size;
    double disA, disB, inf_bound;
    double limit_maxmid, limit_midmin, limit_maxmin;
    double p2l_ratio;
    double jump_up_limit, jump_down_limit;
    double cos160;
    double edgea, edgeb;
    double smallp_intersect, smallp_ratio;
    double vx, vy, vz;
};
