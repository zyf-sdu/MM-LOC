#ifndef DYNAMIC_PCD_LOADER_OCTREE_INDEX_HPP
#define DYNAMIC_PCD_LOADER_OCTREE_INDEX_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

// Use ROS 1 headers instead of the ROS 2 rclcpp header.
#include <ros/ros.h>

// PCD block structure storing point cloud bounds and the file path.
struct PCDBlock {
    Eigen::Vector3f min_corner;  // Minimum corner of the block.
    Eigen::Vector3f max_corner;  // Maximum corner of the block.

    Eigen::Vector3f center;       // Center of the block.
    std::string file_path;       // Path to the PCD file for this block.
};


typedef pcl::PointXYZI PointType;

class OctreeIndex {
public:
    OctreeIndex();
    ~OctreeIndex();

    /**
     * Build an octree index from a PCD file and partition the point cloud into blocks.
     * @param pcd_path Full path to the input PCD file.
     * @param resolution Octree resolution (block size).
     * @return True if building succeeds, false otherwise.
     */
    bool buildFromPCD(const std::string& pcd_path, float resolution);

    /**
     * Load an existing octree index from a CSV file.
     * @param index_path Path to the index CSV file.
     * @return True if loading succeeds, false otherwise.
     */
    bool loadFromFile(const std::string& index_path);

    /**
     * Save the octree index to a CSV file.
     * @param index_path Path for saving the index.
     * @return True if saving succeeds, false otherwise.
     */
    bool saveToFile(const std::string& index_path) const;

    /**
     * Query point cloud blocks near the specified position.
     * @param position Target position (vehicle position).
     * @param radius Query radius.
     * @return List of matching point cloud blocks.
     */
    std::vector<PCDBlock> queryNearbyBlocks(const Eigen::Vector3f& position, float radius) const;

    //Set the output path and filename prefix.
    void setSaveCubeBlockPath(const std::string& path, const std::string& prefix){
        save_cube_pcd_out_put_path_ = path;
        save_cube_pcd_prefix_ = prefix;
    }    

    void getCurrentLayerBlockInfo(std::string& index_file_name, Eigen::Vector3f& center){
        
        index_file_name = block_info_.file_path;
        center = block_info_.center;
    }


private:

    void initKdtree();

    std::vector<PCDBlock> blocks_;  // Information for all point cloud blocks.


    std::string save_cube_pcd_out_put_path_; //Output path for cube blocks.
    std::string save_cube_pcd_prefix_; //Output filename prefix for cube blocks.

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingBlocksPtr;

    double resolution_;

    PCDBlock  block_info_;     // Center coordinates of the map block.
};

#endif // DYNAMIC_PCD_LOADER_OCTREE_INDEX_HPP