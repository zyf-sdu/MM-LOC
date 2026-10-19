#include "blockMap.h"
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

BlockMapManager::BlockMapManager()
{
    kdtree_blocks_ptr_.reset(new pcl::KdTreeFLANN<PointType>());
}

BlockMapManager::~BlockMapManager()
{
}

int 
BlockMapManager::buildCubesFromBlockPCD( const std::string& load_pcd_file_path, 
                            const std::string& save_cube_folder_path,
                            const std::string& prefix_name, 
                            float resolution){
    
    OctreeIndex cubes;

    cubes.setSaveCubeBlockPath(save_cube_folder_path, prefix_name);
    if(!cubes.buildFromPCD(load_pcd_file_path, resolution)){

        ROS_ERROR("build fail!!!! load %s,  build cubes in %s , using predix_name: %s,", 
            load_pcd_file_path.c_str(), 
            save_cube_folder_path.c_str(),
            prefix_name.c_str());
        return -1;
    }

    std::string index_file_name;
    Eigen::Vector3f center;
    cubes.getCurrentLayerBlockInfo(index_file_name, center);
    index_files_.push_back(index_file_name);
    block_centers_.push_back(center);
    

    ROS_INFO("load %s,  build cubes in %s , using predix_name: %s,", 
                load_pcd_file_path.c_str(), 
                save_cube_folder_path.c_str(),
                prefix_name.c_str());
    return 0;
}

int 
BlockMapManager::buildCubesFromBlockPCDFolder(const std::string& load_pcd_folder_path,
                                    const std::string& save_cube_folder_path,
                                    float resolution){

    boost::format load_format("%03d");

    for (int map_id = 0; ; map_id++)
    {
        pcl::PointCloud<PointType>::Ptr tmp_cloud(new pcl::PointCloud<PointType>());
        std::string block_index_name = (load_format % map_id).str();

        std::string prefix_name = (load_format % map_id).str();
        std::string map_path = load_pcd_folder_path+"/" + (load_format % map_id).str() +".pcd";

        if(-1 == access(map_path.c_str(), F_OK)){
            ROS_WARN("cant open %s file", map_path.c_str());
            break;
        }

        if(0 != buildCubesFromBlockPCD(map_path, save_cube_folder_path, prefix_name, resolution)){
            ROS_ERROR("buildCubesFromBlockPCD fail !!!");
            break;
        }
    }

    if(index_files_.empty() || block_centers_.empty()){
        ROS_ERROR("index_files_  block_centers_ is empty !!!");
        return -2;
    }

    std::string index_name = save_cube_folder_path + "/"+ block_index_file_name_;
    ROS_INFO("index_name %s", index_name.c_str());
    if(!saveIndexToFile(index_name)){
        ROS_ERROR("saveIndexToFile fail !!!");
        return -3;
    }


    ROS_INFO("buildCubesFromBlockPCDFolder done!!!");
    return 0;
}

bool BlockMapManager::saveIndexToFile(const std::string& index_path) {
    std::ofstream out_file(index_path);
    if (!out_file.is_open()) {
        ROS_ERROR("Failed to create index file: %s", index_path.c_str());
        return false;
    }

    if(index_files_.empty() || block_centers_.empty()){
        ROS_ERROR("index_files_  block_centers_ is empty !!!");
        return false;
    }
    if(index_files_.size() != block_centers_.size()){
        ROS_ERROR(" index_files_.size() != block_centers_.size() !!!");
        return false;
    }


    for(int i = 0 ; i < index_files_.size(); i++){
        out_file << block_centers_[i][0] << "," << block_centers_[i][1] << "," << block_centers_[i][2] << ","
                 << index_files_[i] << "\n";
    }

    return true;
}


bool 
BlockMapManager::loadBlocksFromIndexFile(const std::string& index_folder_path)
{
    cube_index_file_path_ = index_folder_path;
    fs::path index_file_path = fs::path(index_folder_path) / block_index_file_name_;
    std::ifstream in_file(index_file_path.string());
    if (!in_file.is_open()) {
        ROS_ERROR("block:Failed to open index file: %s", index_file_path.c_str());
        return false;
    }

    index_files_.clear();
    block_centers_.clear();

    std::string line;
    // std::getline(in_file, line);  // Skip the header row.
    while (std::getline(in_file, line)) {
        std::istringstream iss(line);
        Eigen::Vector3f center;
        std::string cube_file_path;
        char sep;
        iss >> center[0] >> sep >> center[1] >> sep >> center[2]>>sep
            >> cube_file_path;

        index_files_.push_back(cube_file_path);
        block_centers_.push_back(center);
    }
    for(const auto& center : block_centers_){
        ROS_INFO("center(%f, %f ,%f)", center[0], center[1], center[2]);
    }

    initKdtree();

    ROS_INFO("loadBlocksFromIndexFile done !!!!");

    return true;
}

void 
BlockMapManager::initKdtree()
{
    pcl::PointCloud<PointType>::Ptr block_centers(new pcl::PointCloud<PointType>());
    for(const auto& block : block_centers_){
        PointType point;
        point.x = block[0];
        point.y = block[1];
        point.z = block[2];

        block_centers->push_back(point);
    }

    if(0 == block_centers->size()){
        ROS_ERROR("cube block is empty, initKdtree fail");
        return;
    }

    kdtree_blocks_ptr_.reset(new pcl::KdTreeFLANN<PointType>());

    kdtree_blocks_ptr_->setInputCloud(block_centers);
    ROS_INFO("dynamic map kdtree build success, there has %d", block_centers->size());
}

bool 
BlockMapManager::queryNearbyBlocks(const Eigen::Vector3d& query_pose, 
                        double search_radius, 
                        std::set<int>& nearby_blocks_idx)
{
    PointType search_point;
    search_point.x = query_pose.x();
    search_point.y = query_pose.y();
    search_point.z = query_pose.z();

    std::vector<int> point_idx_knn_search;
    std::vector<float> point_knn_square_distance;

    int found = kdtree_blocks_ptr_->nearestKSearch(search_point, 2, point_idx_knn_search, point_knn_square_distance); 

    if(found <=0){
        ROS_WARN("queryNearbyBlocks:don't detect any block in pose(%f ,%f, %f)", query_pose.x(), query_pose.y(), query_pose.z());
        return false;
    }

    nearby_blocks_idx.clear();
    for(const auto& id : point_idx_knn_search){
        nearby_blocks_idx.insert(id);
    }

    return true;
}

 
int BlockMapManager::updateNearbyBlockMaps(const Eigen::Vector3d& query_pose, double search_radius)
{
    std::set<int> nearby_blocks_idx;
    if(!queryNearbyBlocks(query_pose, search_radius, nearby_blocks_idx)){
        ROS_ERROR("query Nearby Blocks fail !!!");
        return -1;
    }
    // Retain the same selected indices without copying their complete KD-trees.
    bool changed = false;
    for (auto it = cur_block_maps_.begin(); it != cur_block_maps_.end();) {
        if (!nearby_blocks_idx.count(it->first)) {
            it = cur_block_maps_.erase(it);
            changed = true;
        } else ++it;
    }
    for (const auto id : nearby_blocks_idx) {
        if (cur_block_maps_.count(id)) continue;
        auto& index = cur_block_maps_[id];
        const auto path = cube_index_file_path_ + "/" + index_files_[id];
        if (!index.loadFromFile(path)) {
            cur_block_maps_.erase(id); // Retry failed loads on the next query.
            return -1;
        }
        changed = true;
    }
    return changed ? 0 : -1;
}

pcl::PointCloud<PointType> 
BlockMapManager::getCurrentPointMap(const Eigen::Vector3d& query_pose, double search_radius)
{
    if(0 == updateNearbyBlockMaps(query_pose, search_radius)){
        ROS_INFO("update block maps!");
    }
    Eigen::Vector3f query_posef = Eigen::Vector3f(static_cast<float>(query_pose.x()), 
                                                  static_cast<float>(query_pose.y()), 
                                                  static_cast<float>(query_pose.z()));

    pcl::PointCloud<PointType> pcd_map;

    std::vector<PCDBlock> result;
    
    for(auto& block : cur_block_maps_){

        Eigen::Vector3f center = block_centers_[block.first];

        ROS_INFO("cur_block_maps_ %d, (%f, %f,%f)", block.first, center[0], center[1], center[2]);
        auto nearby_cubes = block.second.queryNearbyBlocks(query_posef, search_radius);
        result.insert(result.end(), nearby_cubes.begin(), nearby_cubes.end());
    }
    
    ROS_INFO("load result size = %d", result.size());
    //todo Filter duplicate point clouds loaded at the boundary between two maps.

    return loadPcdCubes(result);
}

pcl::PointCloud<PointType> BlockMapManager::loadPcdCubes(const std::vector<PCDBlock>& blocks) {

    pcl::PointCloud<PointType> current_cloud;
    for (const auto& block : blocks) {
        fs::path block_path = fs::path(cube_index_file_path_) / block.file_path;
        
        pcl::PointCloud<PointType>::Ptr block_cloud(new pcl::PointCloud<PointType>);
        if (pcl::io::loadPCDFile<PointType>(block_path.string(), *block_cloud) == -1) {
            ROS_WARN("Failed to load block: %s", block.file_path.c_str());
            continue;
        }

        current_cloud += *block_cloud;
    }
    current_cloud.header.frame_id = "odom";

    ROS_INFO("Loaded %zu blocks, total points: %zu", 
             blocks.size(), current_cloud.size());

    return current_cloud;
}
