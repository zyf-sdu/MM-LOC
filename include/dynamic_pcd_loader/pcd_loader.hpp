#ifndef DYNAMIC_PCD_LOADER_PCD_LOADER_HPP
#define DYNAMIC_PCD_LOADER_PCD_LOADER_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>
#include "dynamic_pcd_loader/octree_index.hpp"
#include "utility.h"

class PCDLoader : public ParamServer{
public:
    PCDLoader(ros::NodeHandle& nh);
    ~PCDLoader();

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void loadPCDBlocks(const std::vector<PCDBlock>& blocks);
    void publishPointCloud();

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cloud_pub_;

    std::string input_pcd_;
    std::string input_pcd_name_ = "000";
    
    double resolution_;
    std::string output_directory_;
    double load_radius_;
    std::string map_frame_;

    OctreeIndex octree_index_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    std::mutex cloud_mutex_;
};

#endif // DYNAMIC_PCD_LOADER_PCD_LOADER_HPP