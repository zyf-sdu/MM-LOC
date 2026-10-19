#include "dynamic_pcd_loader/pcd_loader.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>

namespace fs = boost::filesystem;

PCDLoader::PCDLoader(ros::NodeHandle& nh) : nh_(nh), current_cloud_(new pcl::PointCloud<pcl::PointXYZ>) {
    // Declare and retrieve parameters.
    // nh_.param<std::string>("dynamic/input_pcd", input_pcd_, "input_pcd");
    // nh_.param<double>("dynamic/resolution", resolution_, 1.0);
    // nh_.param<std::string>("dynamic/output_directory", output_directory_, "output");
    // nh_.param<double>("dynamic/load_radius", load_radius_, 5.0);
    // nh_.param<std::string>("dynamic/map_frame", map_frame_, "map");
    // nh_.param<std::string>("dynamic/input_pcd_name", input_pcd_name_, "map");
    input_pcd_ = input_pcd_path_;
    resolution_ = cube_resolution_;
    output_directory_ = cube_block_path_;
    load_radius_ = cube_load_radius_;
    map_frame_ = cube_map_frame_; 
    ROS_INFO("input_pcd_name_ = %s", input_pcd_name_.c_str());

    // Resolve the package path and handle relative paths.
    // std::string pkg_path = ros::package::getPath("dynamic_pcd_loader");
    // if (!output_directory_.empty() && output_directory_[0] != '/') {
    //     output_directory_ = pkg_path + "/" + output_directory_;
    // }
    // if (!input_pcd_.empty() && input_pcd_[0] != '/') {
    //     input_pcd_ = pkg_path + "/" + input_pcd_;
    // }

    // Load the octree index.
    std::string index_file_name = input_pcd_name_ + "_"+"octree_index.csv";
    std::string index_path = (boost::filesystem::path(cube_block_path_) / index_file_name).string();

    std::string pcd_name = input_pcd_name_ + ".pcd";
    std::string input_pcd_path = (boost::filesystem::path(input_pcd_) / pcd_name).string();
    if (!octree_index_.loadFromFile(index_path)) {
        ROS_WARN("Failed to load octree index path = %s, building a new one...",index_path.c_str());
        octree_index_.setSaveCubeBlockPath(output_directory_, input_pcd_name_);
        if (!octree_index_.buildFromPCD(input_pcd_path, resolution_)) {
            ROS_ERROR("Failed to build octree index!,pcd index = %s", input_pcd_path.c_str());
            throw std::runtime_error("Index building failed");
        }
    }

    // Initialize ROS interfaces.
    odom_sub_ = nh_.subscribe("/odom", 10, &PCDLoader::odomCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_map", 10);

    ROS_INFO("Dynamic PCD Loader initialized");
}

PCDLoader::~PCDLoader() {}

void PCDLoader::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    // Get the vehicle position.
    Eigen::Vector3f car_pos(
        static_cast<float>(msg->pose.pose.position.x),
        static_cast<float>(msg->pose.pose.position.y),
        static_cast<float>(msg->pose.pose.position.z));

    // Query nearby PCD blocks.
    auto nearby_blocks = octree_index_.queryNearbyBlocks(car_pos, load_radius_);
    
    // Load and merge point clouds.
    loadPCDBlocks(nearby_blocks);

    // Publish the merged point cloud.
    publishPointCloud();
}

void PCDLoader::loadPCDBlocks(const std::vector<PCDBlock>& blocks) {
    current_cloud_->clear();

    for (const auto& block : blocks) {
        fs::path block_path = fs::path(output_directory_) / block.file_path;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr block_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(block_path.string(), *block_cloud) == -1) {
            ROS_WARN("Failed to load block: %s", block.file_path.c_str());
            continue;
        }

        *current_cloud_ += *block_cloud;
    }

    ROS_INFO("Loaded %zu blocks, total points: %zu", 
             blocks.size(), current_cloud_->size());
}

void PCDLoader::publishPointCloud() {
    if (current_cloud_->empty()) {
        ROS_WARN("No point cloud data to publish");
        return;
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*current_cloud_, cloud_msg);
    cloud_msg.header.frame_id = map_frame_;
    cloud_msg.header.stamp = ros::Time::now();

    cloud_pub_.publish(cloud_msg);
}