#include <utility>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <random>
#include <sys/times.h>
#include <sys/vtimes.h>
#include <thread>
#include <malloc.h>
#include <condition_variable>
#include <optional>

// ROS
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#define PCL_NO_PRECOMPILE
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

class Feature {
    public:
        typedef std::pair<Eigen::Vector3f, Eigen::Quaternionf> Pose;
        typedef Eigen::VectorXf Descriptor;

        Feature(int scan_index) : m_scan_index(scan_index), m_init_flags(Feature::EMPTY) {}

        inline int get_scan_index() const { return m_scan_index; }
        inline ros::Time get_timestamp() const { return m_timestamp; }
        inline const Pose& get_pose() const { return m_pose; }
        inline const Descriptor& get_descriptor() const { return m_descriptor; }
        inline std::string get_cloud_path() const { return m_cloud_path; }
        inline float get_element_weighting() const { return m_element_weighting; }
        inline const std::optional<Pose> get_del_pose() const { return m_del_pose; }
        inline const unsigned int get_init_flags() const { return m_init_flags; }

        inline bool is_fully_init() const { return m_init_flags == FULLY_INIT; }

        inline void set_timestamp(ros::Time time) { m_timestamp = time; m_init_flags |= TIME_INIT; }
        inline void set_pose(const Pose& pose) { m_pose = pose; m_init_flags |= POSE_INIT; }
        inline void set_descriptor(const Descriptor& desc) { m_descriptor = desc; m_init_flags |= DESCRIPTOR_INIT; }
        inline void set_cloud_path(std::string cloud_path) { m_cloud_path = cloud_path; m_init_flags |= CLOUD_INIT; }
        inline void set_element_weighting(float weight) { m_element_weighting = weight; }
        inline void set_del_pose(const Pose& del_pose) { m_del_pose = del_pose; }

        enum INIT_FLAGS
        {
            EMPTY = 0,
            TIME_INIT = (1u << 0),
            POSE_INIT = (1u << 1),
            DESCRIPTOR_INIT = (1u << 2),
            CLOUD_INIT = (1u << 3),
            FULLY_INIT = (TIME_INIT | POSE_INIT | DESCRIPTOR_INIT | CLOUD_INIT)
        };
    
    private:
        int m_scan_index;

        ros::Time m_timestamp;
        Pose m_pose;
        Descriptor m_descriptor;
        std::string m_cloud_path;

        float m_element_weighting;
        std::optional<Pose> m_del_pose;

        // bitmask of `INIT_FLAGS` for which fields have been initialized
        unsigned int m_init_flags;
};