#include "dynamic_pcd_loader/pcd_loader.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_pcd_loader");
    ros::NodeHandle nh("~");  // Use the private namespace.

    try {
        PCDLoader loader(nh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }

    return 0;
}