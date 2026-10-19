#pragma once

#include "dynamic_pcd_loader/octree_index.hpp"
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <unistd.h>  // Provides access(). 

typedef pcl::PointXYZI PointType;
class BlockMapManager{



public: 
    BlockMapManager();

    ~BlockMapManager();

    //Load all block PCD files and generate all cube PCD files.
    int buildCubesFromBlockPCDFolder(const std::string& load_pcd_folder_path,
                                     const std::string& save_cube_folder_path,
                                     float resolution);



    //Generate cube blocks from a single map block.
    //pcd_path[in] Directory containing the input block PCD files.
    //save_cube_folder_path[in] Directory for saving cubes.
    //prefix_name[in] Filename prefix for PCD files.
    //resolution Resolution of the generated cubes.
    int buildCubesFromBlockPCD( const std::string& load_pcd_file_path, 
                                const std::string& save_cube_folder_path,
                                const std::string& prefix_name, 
                                float resolution);
    

    //Load the map.
    bool loadBlocksFromIndexFile(const std::string& index_folder_path);


    

    pcl::PointCloud<PointType> getCurrentPointMap(const Eigen::Vector3d& query_pose, double search_radius);

    

private:

    //Return the nearest map found by the query.
    bool queryNearbyBlocks(const Eigen::Vector3d& query_pose, 
                            double search_radius, 
                            std::set<int>& nearby_blocks_idx);

    int updateNearbyBlockMaps(const Eigen::Vector3d& query_pose, double search_radius);
    
    void initKdtree();

    //Save the block map index file.
    bool saveIndexToFile(const std::string& index_path);

    pcl::PointCloud<PointType> loadPcdCubes(const std::vector<PCDBlock>& blocks);

private:

    std::vector<std::string> index_files_;
    std::vector<Eigen::Vector3f> block_centers_;

    std::string cube_index_file_path_;       // Index file for the cubes belonging to this block.


    pcl::KdTreeFLANN<PointType>::Ptr kdtree_blocks_ptr_;  //All map blocks in the map.

    std::string map_frame_id_;

    std::vector<int> cur_block_map_index_;

    std::map<int, OctreeIndex> cur_block_maps_;

    std::string block_index_file_name_ = "index.csv";
};