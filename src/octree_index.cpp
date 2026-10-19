#include "dynamic_pcd_loader/octree_index.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_pointcloud.h>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <pcl/common/centroid.h>


// Legacy CSV indices do not store voxel resolution. Previously this member
// was indeterminate when loading them, sometimes expanding the query to the
// entire map. Preserve the requested-radius behavior of the legacy baseline;
// buildFromPCD still sets the explicit resolution for newly built indices.
OctreeIndex::OctreeIndex() : resolution_(0.0) {}
OctreeIndex::~OctreeIndex() {}


bool OctreeIndex::buildFromPCD(const std::string& pcd_path, float resolution) {
    resolution_ = resolution;

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType>(pcd_path, *cloud) == -1) {
        ROS_ERROR("Failed to load PCD file: %s", pcd_path.c_str());
        return false;
    }

    if (cloud->empty()) {
        ROS_ERROR("Loaded empty point cloud");
        return false;
    }

    Eigen::Vector4d curr_centroid;
    pcl::compute3DCentroid(*cloud, curr_centroid);
    block_info_.center = Eigen::Vector3f(curr_centroid[0], curr_centroid[1], curr_centroid[2]);

    pcl::octree::OctreePointCloud<PointType> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    blocks_.clear();

    for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it) {
        Eigen::Vector3f min_corner, max_corner;
        octree.getVoxelBounds(it, min_corner, max_corner);

        PCDBlock block;
        block.min_corner = min_corner;
        block.max_corner = max_corner;
        block.center = (min_corner + max_corner) * 0.5;
        
        block.file_path = save_cube_pcd_prefix_ + "_block_" + std::to_string(blocks_.size()) + ".pcd";
        blocks_.push_back(block);

        pcl::PointCloud<PointType>::Ptr block_cloud(new pcl::PointCloud<PointType>);
        std::vector<int> point_indices;
        it.getLeafContainer().getPointIndices(point_indices);

        for (const auto& idx : point_indices) {
            block_cloud->push_back(cloud->points[idx]);
        }

        if (!block_cloud->empty()) {
            // std::string pkg_path = ros::package::getPath("dynamic_pcd_loader");
            // std::string output_path = pkg_path + "/output/" + block.file_path;
            boost::filesystem::path output_dir(save_cube_pcd_out_put_path_);
            // output_dir /= "output";  // Append the path component.
            output_dir /= block.file_path;
            std::string output_path = output_dir.string();

            if (pcl::io::savePCDFileBinary(output_path, *block_cloud) == -1) {
                ROS_ERROR("Failed to save block PCD file: %s", block.file_path.c_str());
                return false;
            }
        }
    }

    if (!blocks_.empty()) {
        // std::string pkg_path = ros::package::getPath("dynamic_pcd_loader");
        std::string index_path = save_cube_pcd_out_put_path_+"/" + save_cube_pcd_prefix_+"_" +"octree_index.csv";
        if (!saveToFile(index_path)) {
            ROS_ERROR("Failed to save index file");
            return false;
        }
    }
    block_info_.file_path = save_cube_pcd_prefix_+"_" +"octree_index.csv";
    initKdtree();
    ROS_INFO("Successfully built octree index with %zu blocks", blocks_.size());
    return true;
}
//Each large map block has its own index.
bool OctreeIndex::loadFromFile(const std::string& index_path) {
    std::ifstream in_file(index_path);
    if (!in_file.is_open()) {
        ROS_ERROR("OctreeIndex:Failed to open index file: %s", index_path.c_str());
        perror("错误原因");
        return false;
    }

    blocks_.clear();
    std::string line;
    std::getline(in_file, line);  // Skip the header row.
    while (std::getline(in_file, line)) {
        std::istringstream iss(line);
        PCDBlock block;
        char sep;
        iss >> block.center[0] >> sep >> block.center[1] >> sep >> block.center[2]>> sep
            >> block.file_path;
        blocks_.push_back(block);
    }

    initKdtree();

    return true;
}

bool OctreeIndex::saveToFile(const std::string& index_path) const {
    std::ofstream out_file(index_path);
    if (!out_file.is_open()) {
        ROS_ERROR("Failed to create index file: %s", index_path.c_str());
        return false;
    }

    for (const auto& block : blocks_) {
        out_file << block.center[0] << "," << block.center[1] << "," << block.center[2] << ","
                 << block.file_path << "\n";
    }

    return true;
}
//Switch to k-d tree queries.
//A better indexing scheme is needed.
//Index block centers instead of max_corner/min_corner; associate filenames with the index.
std::vector<PCDBlock> OctreeIndex::queryNearbyBlocks(const Eigen::Vector3f& position, 
                                                   float radius) const {
    std::vector<PCDBlock> result;

    double additional_dist = resolution_ * 1.73;
    double serach_radius = additional_dist + radius;

    std::vector<int> indices; 
    std::vector<float> sqr_dists; 

    PointType target_point;
    target_point.x = position[0];
    target_point.y = position[1];
    target_point.z = position[2];

    int found = kdtreeSurroundingBlocksPtr->radiusSearch(target_point, serach_radius, indices, sqr_dists);

    if(found > 0){

        for(const auto& idx : indices){
            result.push_back(blocks_[idx]);
        }
    }else{
        ROS_WARN("don't detect any cube block in pose(%f ,%f, %f), serach_radiu = %f", 
                    target_point.x, target_point.y, target_point.z, serach_radius);
    }

    //old version
    // for (const auto& block : blocks_) {
    //     Eigen::Vector3f center = (block.min_corner + block.max_corner) * 0.5f;
    //     float distance = (position - center).norm();
        
    //     float block_radius = (block.max_corner - block.min_corner).norm() / 2.0f;
    //     if (distance <= (radius + block_radius)) {
    //         if (!block.file_path.empty()) {
    //             result.push_back(block);
    //         } else {
    //             ROS_WARN("Found nearby block with empty file_path");
    //         }
    //     }
    // }


    return result;
}

void OctreeIndex::initKdtree()
{
    pcl::PointCloud<PointType>::Ptr cube_block_centers(new pcl::PointCloud<PointType>());
    for(const auto& block : blocks_){
        PointType point;
        point.x = block.center[0];
        point.y = block.center[1];
        point.z = block.center[2];

        cube_block_centers->push_back(point);
    }

    if(0 == cube_block_centers->size()){
        ROS_ERROR("cube block is empty, initKdtree fail");
        return;
    }

    kdtreeSurroundingBlocksPtr.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurroundingBlocksPtr->setInputCloud(cube_block_centers);
    ROS_INFO("dynamic map kdtree build success, there has %d", cube_block_centers->size());
}
