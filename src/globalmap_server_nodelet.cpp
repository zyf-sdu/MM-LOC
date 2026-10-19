#include "resource_timing.h"
#include "utility.h"
#include "roll/queryMap.h"

#include <map_server/image_loader.h>
#include <yaml-cpp/yaml.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include "blockMap.h"
#include <mutex>

#ifdef HAVE_NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif


namespace block_localization {

class GlobalmapServerNodelet : public ParamServer
{
public:

    struct Params{
    std::string global_corner_map_topic;
    std::string global_surfer_map_topic;
    std::string dynamic_corner_map_topic;
    std::string dynamic_surf_map_topic;
    std::string dymaic_map_query;
    std::string global_map_dir;
    std::string type;
    };

public:
    GlobalmapServerNodelet(const Params& params) {
    
        params_ = params;
        onInit();
    }
    virtual ~GlobalmapServerNodelet() {
    
    }

    void onInit(){
        // nh = getNodeHandle();

        if (use_dynamic_map_loading_) {
            corner_block_map_manager_ptr_ = std::make_shared<BlockMapManager>();
            surf_block_map_manager_ptr_   = std::make_shared<BlockMapManager>();
            if(build_dynamic_cube_map_){
                ROS_INFO("start to build cubes");
                buildBlockMaps(input_pcd_path_, cube_resolution_);
            }

            loadAllBlockMaps(params_.global_map_dir);
            //This can also be read from the configuration.
            Eigen::Vector3d init_pose(initialGuess[3], initialGuess[4], initialGuess[5]);
            getCurrentPointMap(init_pose, cube_load_radius_);
        }


        // Cube-map localization does not need a second complete legacy map.
        if (!use_dynamic_map_loading_) {
            loadAllMap();
            legacy_maps_loaded_ = true;
            if (globalmap_vec.empty() || global_surf_map_vec.empty()) {
                throw std::runtime_error("No prior corner/surf map loaded");
            }
            // Keep existing subscribers unchanged: static prior maps are also
            // published on the dynamic-map topics used by mapOptmization.
            for (const auto& map : globalmap_vec) {
                cur_corner_map_ += *map;
            }
            for (const auto& map : global_surf_map_vec) {
                cur_surf_map_ += *map;
            }
            globalmap_vec.clear();
            global_surf_map_vec.clear();
            ROS_INFO("Dynamic map loading disabled: loaded complete prior map (%zu corner, %zu surf points).",
                     cur_corner_map_.size(), cur_surf_map_.size());
        }

        
        // service for changing BM
        std::string map_query_topic = "/mapQuery";
        if(params_.type == "mid"){
            map_query_topic += "_mid"; 
           
        }
        mapQueryServer = nh.advertiseService(map_query_topic, &GlobalmapServerNodelet::mapQueryCB, this);
        

        
        // publish globalmap with "latched" publisher
        globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>(params_.global_corner_map_topic, 5, true);
        globalmap_pub_timer = nh.createWallTimer(ros::WallDuration(mapqry_interval_), &GlobalmapServerNodelet::pubMapOnce, this, false, true);

        global_surf_map_pub =  nh.advertise<sensor_msgs::PointCloud2>(params_.global_surfer_map_topic, 5, true);
        global_surf_map_pub_timer = nh.createWallTimer(ros::WallDuration(mapqry_interval_), &GlobalmapServerNodelet::pubSurfMapOnce, this, false, true);

        dynamic_map_query_server_ = nh.advertiseService(params_.dymaic_map_query, &GlobalmapServerNodelet::dynamicMapQueryCB, this);
        dynamic_corner_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(params_.dynamic_corner_map_topic, 5, true);
        dynamic_corner_map_pub_timer_ = nh.createWallTimer(ros::WallDuration(mapqry_interval_), &GlobalmapServerNodelet::pubDynamicCornerMapOnce, this, false, true);

        dynamic_surf_map_pub_ =  nh.advertise<sensor_msgs::PointCloud2>(params_.dynamic_surf_map_topic, 5, true);
        dynamic_surf_map_pub_timer_ = nh.createWallTimer(ros::WallDuration(mapqry_interval_), &GlobalmapServerNodelet::pubDynamicSurfMapOnce, this, false, true);



        
    }

private:
    void pubMapOnce(const ros::WallTimerEvent& event) {
        sensor_msgs::PointCloud2 ros_cloud;
        ros_cloud.header.frame_id = mapFrame;

        if (!use_dynamic_map_loading_) {
            if (!cur_corner_map_.empty()) {
                pcl::toROSMsg(cur_corner_map_, ros_cloud);
                globalmap_pub.publish(ros_cloud);
            }
            return;
        }

        if (globalmap_pub.getNumSubscribers() == 0) return;
        ensureLegacyMaps();
        if (!globalmap.empty()) {
            pcl::toROSMsg(globalmap, ros_cloud);
            globalmap_pub.publish(ros_cloud);
        }
    }

    void pubSurfMapOnce(const ros::WallTimerEvent& event) {
        sensor_msgs::PointCloud2 ros_cloud;
        ros_cloud.header.frame_id = mapFrame;

        if (!use_dynamic_map_loading_) {
            if (!cur_surf_map_.empty()) {
                pcl::toROSMsg(cur_surf_map_, ros_cloud);
                global_surf_map_pub.publish(ros_cloud);
            }
            return;
        }

        if (global_surf_map_pub.getNumSubscribers() == 0) return;
        ensureLegacyMaps();
        if (!global_surf_map.empty()) {
            pcl::toROSMsg(global_surf_map, ros_cloud);
            global_surf_map_pub.publish(ros_cloud);
        }else{
            ROS_ERROR("pubSurfMapOnce: global_surf_map is empty");
        }
    }


    bool mapQueryCB(roll::queryMap::Request &req,
                    roll::queryMap::Response &res) {
        if (!use_dynamic_map_loading_) {
            res.success = true;
            return true;
        }

        ensureLegacyMaps();
        PointT searchPoint;
        searchPoint.x = req.position.x;
        searchPoint.y = req.position.y;
        searchPoint.z = req.position.z;
        
        static int print_cnt = 0;
        
        // find nearest block map
        pointIdxNKNSearch.clear();
        pointNKNSquareDistance.clear();
        int k_nearest = centroid_kdtree.nearestKSearch(searchPoint, 2, pointIdxNKNSearch, pointNKNSquareDistance);
        if (k_nearest == 2) {
            globalmap.clear();
            globalmap = (*loadMapFromIdx(pointIdxNKNSearch[0])) + (*loadMapFromIdx(pointIdxNKNSearch[1]));

            if(0 == print_cnt++ % 100){
                ROS_INFO("K-nearest neighbor search at (%f, %f, %f).", searchPoint.x, searchPoint.y, searchPoint.z);
                ROS_INFO("serach 2 block map(%d, %d ), dist = (%f, %f)", pointIdxNKNSearch[0], pointIdxNKNSearch[1], 
                        pointNKNSquareDistance[0], pointNKNSquareDistance[1]);

            }


            global_surf_map.clear();
            global_surf_map = (*loadSurfMapFromIdx(pointIdxNKNSearch[0])) + (*loadSurfMapFromIdx(pointIdxNKNSearch[1]));
            
        } else if (k_nearest == 1) {
            globalmap.clear();
            globalmap = *loadMapFromIdx(pointIdxNKNSearch[0]);

            ROS_INFO("serach 1 block map(%d, %d )", pointIdxNKNSearch[0]);

            global_surf_map.clear();
            global_surf_map = (*loadSurfMapFromIdx(pointIdxNKNSearch[0]));
        } // else BM doesn't need to be changed

        res.success = true;
        return true;
    }



    void ensureLegacyMaps() {
        if (legacy_maps_loaded_) return;
        loadAllMap();
        legacy_maps_loaded_ = true;
        if (!globalmap_vec.empty()) globalmap = *globalmap_vec.front();
        if (!global_surf_map_vec.empty()) global_surf_map = *global_surf_map_vec.front();
    }

    void loadAllMap() {
        auto t1 = ros::WallTime::now();
        boost::format load_format("%03d.pcd");

        cal_centroid_cloud.reset(new pcl::PointCloud<PointT>());

        for (int map_id = 0; ; map_id++)
        {
            pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
            std::string map_name = params_.global_map_dir+ global_corner_map_dir_+"/"  + (load_format % map_id).str();

            if (pcl::io::loadPCDFile(map_name, *tmp_cloud) == -1) {
                map_id -= 1;
                map_name = params_.global_map_dir + (load_format % map_id).str();
                // NODELET_WARN("The last map is: %s", map_name.c_str());
                break;
            }

            tmp_cloud->header.frame_id = mapFrame;

            // downsampling
            boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
            voxelgrid->setLeafSize(downsample_resolution_, downsample_resolution_, downsample_resolution_);
            voxelgrid->setInputCloud(tmp_cloud);
            pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
            voxelgrid->filter(*filtered_cloud);
            tmp_cloud = filtered_cloud;
            
            globalmap_vec.push_back(tmp_cloud);
        }
        
        //Load the surface map.
        for (int map_id = 0; ; map_id++)
        {
            pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
            std::string map_name = params_.global_map_dir+ gloabl_surf_map_dir_+"/"  + (load_format % map_id).str();

            if (pcl::io::loadPCDFile(map_name, *tmp_cloud) == -1) {
                map_id -= 1;
                map_name = params_.global_map_dir + (load_format % map_id).str();
                // NODELET_WARN("The last map is: %s", map_name.c_str());
                break;
            }

            tmp_cloud->header.frame_id = mapFrame;

            // downsampling
            boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
            voxelgrid->setLeafSize(downsample_resolution_, downsample_resolution_, downsample_resolution_);
            voxelgrid->setInputCloud(tmp_cloud);
            pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
            voxelgrid->filter(*filtered_cloud);
            tmp_cloud = filtered_cloud;
            
            global_surf_map_vec.push_back(tmp_cloud);

            Eigen::Vector4d tmp_center;
            pcl::compute3DCentroid(*tmp_cloud, tmp_center);
            PointT temp_center_point;
            temp_center_point.x = tmp_center[0];
            temp_center_point.y = tmp_center[1];
            temp_center_point.z = tmp_center[2];
            cal_centroid_cloud->push_back(temp_center_point);
            ROS_INFO("center %d ,pose = (%f, %f, %f)", map_id, tmp_center[0], tmp_center[1], tmp_center[2]);
        }

        auto t2 = ros::WallTime::now();
        ROS_INFO("Globalmap server has already loaded %ld surf maps. Time cost: %f [msec]", global_surf_map_vec.size(), (t2 - t1).toSec() * 1000.0);
        centroid_kdtree.setInputCloud(cal_centroid_cloud);
    }


    inline pcl::PointCloud<PointT>::Ptr loadMapFromIdx(int map_idx) {
        // ROS_INFO("Globalmap NO.%03d is already loaded!", map_idx);
        return globalmap_vec[map_idx];
    }

    inline pcl::PointCloud<PointT>::Ptr loadSurfMapFromIdx(int map_idx) {
        // ROS_INFO("Globalmap NO.%03d is already loaded!", map_idx);
        return global_surf_map_vec[map_idx];
    }


    void loadMapCentroids() {
        std::string centroid_filename = params_.global_map_dir + "CentroidCloud.pcd";
        
        centroid_cloud.reset(new pcl::PointCloud<PointT>());
        if (pcl::io::loadPCDFile(centroid_filename, *centroid_cloud) == -1) {
            ROS_ERROR("Fail to load the centroid cloud! Please check your source!");
            ros::shutdown();
        }

        ROS_INFO("Already loaded %ld centroids of block maps.", centroid_cloud->points.size());

        if (centroid_cloud->empty()) {
            ROS_ERROR("Fail to build a KD-Tree! Please check your centroid pointcloud!");
            ros::shutdown();
        }
        for(const auto& point : (*centroid_cloud).points){
            ROS_INFO("center %d, %f, %f, %f", point.x, point.y, point.z);
        }

        // build KD-Tree
        centroid_kdtree.setInputCloud(centroid_cloud);
    }


    void pubGridmap() {
        YAML::Node doc = YAML::LoadFile(yaml_path_);
        try
        {
            doc["resolution"] >> res_;
            doc["origin"][0] >> origin_[0];
            doc["origin"][1] >> origin_[1];
            doc["origin"][2] >> origin_[2];
            doc["negate"] >> negate_;
            doc["occupied_thresh"] >> occ_th_;
            doc["free_thresh"] >> free_th_;
        }
        catch(YAML::InvalidScalar)
        {
            ROS_ERROR("The .yaml does not contain tags required or they are invalid.");
            ros::shutdown();
        }
        mode_ = TRINARY;

        std::cout << "	       map name: " << map_name_
                    << "\n	     resolution: " << res_
                    << "\n	         origin: " << origin_[0] << ", " << origin_[1] << ", " << origin_[2]
                    << "\n	         negate: " << negate_
                    << "\n	occupied thresh: " << occ_th_
                    << "\n	    free thresh: " << free_th_ << std::endl;
        map_server::loadMapFromFile(&map_resp_, pgm_path_.c_str(), res_, negate_, occ_th_, free_th_, origin_, mode_);
        map_resp_.map.header.frame_id = "gridmap_link";
        map_publisher_ = nh.advertise<nav_msgs::OccupancyGrid> ("/gridmap", 1, true);
        map_publisher_.publish(map_resp_.map);
    }

private:
//dynamic pcd loader

    void buildBlockMaps(const std::string& block_folder_path, double resolution){

        std::string corner_pcd_file_path = block_folder_path + "corner";
        std::string corner_cube_file_path = block_folder_path + "corner/cube";
        std::string surf_pcd_file_path = block_folder_path + "surf";
        std::string surf_cube_file_path = block_folder_path + "surf/cube";

        boost::filesystem::create_directories(corner_pcd_file_path);
        boost::filesystem::create_directories(corner_cube_file_path);
        boost::filesystem::create_directories(surf_pcd_file_path);
        boost::filesystem::create_directories(surf_cube_file_path);

        corner_block_map_manager_ptr_->buildCubesFromBlockPCDFolder(corner_pcd_file_path, corner_cube_file_path, resolution);
        surf_block_map_manager_ptr_->buildCubesFromBlockPCDFolder(surf_pcd_file_path, surf_cube_file_path, resolution);

        ROS_INFO("build block maps done !!!");

    }
    void loadAllBlockMaps(const std::string& index_folder_path)
    {
        std::string corner_index_folder_path = index_folder_path + "corner/cube";
        corner_block_map_manager_ptr_->loadBlocksFromIndexFile(corner_index_folder_path);

        std::string surf_index_folder_path = index_folder_path + "surf/cube";
        surf_block_map_manager_ptr_->loadBlocksFromIndexFile(surf_index_folder_path);

        ROS_INFO("load block maps index file done!!!");
    }


    void getCurrentPointMap(const Eigen::Vector3d& query_pose, double search_radius){
        resource_metrics::Scope timer("map_load_" + params_.type, ros::Time::now().toSec());
        auto new_corner_map = corner_block_map_manager_ptr_->getCurrentPointMap(query_pose, search_radius);
        ROS_INFO("get cur_corner_map_ done,  %d points", new_corner_map.size());
        auto new_surf_map = surf_block_map_manager_ptr_ ->getCurrentPointMap(query_pose, search_radius);
        ROS_INFO("get cur_surf_map_ done,  %d points", new_surf_map.size());

        std::lock_guard<std::mutex> lock(map_mutex_);
        cur_corner_map_ = std::move(new_corner_map);
        cur_surf_map_ = std::move(new_surf_map);
        corner_message_dirty_ = surf_message_dirty_ = true;
    }

    bool dynamicMapQueryCB(roll::queryMap::Request &req,
                    roll::queryMap::Response &res)
    {
        if (!use_dynamic_map_loading_) {
            res.success = true;
            return true;
        }

        Eigen::Vector3d searchPoint;

        searchPoint.x() = req.position.x;
        searchPoint.y() = req.position.y;
        searchPoint.z() = req.position.z;

        getCurrentPointMap(searchPoint, cube_load_radius_);

        res.success = true;
        return true;
    }


    void publishDynamicMap(bool corner) {
        // Keep publication cadence and filtering unchanged. Cache only the
        // serialized message until the next map query replaces its source.
        std::lock_guard<std::mutex> lock(map_mutex_);
        auto& cloud = corner ? cur_corner_map_ : cur_surf_map_;
        auto& message = corner ? corner_message_ : surf_message_;
        auto& dirty = corner ? corner_message_dirty_ : surf_message_dirty_;
        if (cloud.empty()) return;
        if (dirty) {
            if (use_dynamic_map_loading_) {
                pcl::VoxelGrid<PointType> voxelgrid;
                const auto leaf = corner ? downsample_resolution_ / 2 : downsample_resolution_;
                voxelgrid.setLeafSize(leaf, leaf, leaf);
                voxelgrid.setInputCloud(cloud.makeShared());
                pcl::PointCloud<PointType> filtered;
                voxelgrid.filter(filtered);
                pcl::toROSMsg(filtered, message);
            } else pcl::toROSMsg(cloud, message);
            message.header.frame_id = mapFrame;
            dirty = false;
        }
        (corner ? dynamic_corner_map_pub_ : dynamic_surf_map_pub_).publish(message);
    }

    void pubDynamicCornerMapOnce(const ros::WallTimerEvent&) { publishDynamicMap(true); }
    void pubDynamicSurfMapOnce(const ros::WallTimerEvent&) { publishDynamicMap(false); }


private:
    // map settings
    pcl::PointCloud<PointT> globalmap;
    std::vector<pcl::PointCloud<PointT>::Ptr> globalmap_vec;

    pcl::PointCloud<PointT> global_surf_map;
    std::vector<pcl::PointCloud<PointT>::Ptr> global_surf_map_vec;

    ros::ServiceServer mapQueryServer;
    ros::Publisher globalmap_pub;
    ros::WallTimer globalmap_pub_timer;

    ros::Publisher global_surf_map_pub;
    ros::WallTimer global_surf_map_pub_timer;

    // block map centroids
    pcl::PointCloud<PointT>::Ptr centroid_cloud;


    pcl::PointCloud<PointT>::Ptr cal_centroid_cloud;

    // KD-Tree for block maps
    pcl::KdTreeFLANN<PointT> centroid_kdtree;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquareDistance;

    MapMode mode_;
    std::string map_name_;
    double res_;
    double origin_[3];
    int negate_;
    double occ_th_, free_th_;
    nav_msgs::GetMap::Response map_resp_;
    ros::Publisher map_publisher_;

    ros::ServiceServer dynamic_map_query_server_;

    std::shared_ptr<BlockMapManager>  corner_block_map_manager_ptr_;
    std::shared_ptr<BlockMapManager>  surf_block_map_manager_ptr_;
    std::mutex map_mutex_;
    pcl::PointCloud<PointType> cur_corner_map_;
    pcl::PointCloud<PointType> cur_surf_map_;

    ros::Publisher dynamic_corner_map_pub_;
    ros::WallTimer dynamic_corner_map_pub_timer_;

    ros::Publisher dynamic_surf_map_pub_;
    ros::WallTimer dynamic_surf_map_pub_timer_;

    bool legacy_maps_loaded_ = false;
    bool corner_message_dirty_ = true, surf_message_dirty_ = true;
    sensor_msgs::PointCloud2 corner_message_, surf_message_;
    Params params_;
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "temp_globalmap_node", false);
    ros::NodeHandle nh("~"); // Use the private namespace to avoid parameter conflicts.


    std::string global_corner_map_topic;
    std::string global_surfer_map_topic;
    std::string dynamic_corner_map_topic;
    std::string dynamic_surf_map_topic;
    std::string dymaic_map_query;
    std::string global_map_dir;
    std::string type;

    nh.param<std::string>("global_corner_map_topic", global_corner_map_topic, "global_corner_map"); 
    nh.param<std::string>("global_surfer_map_topic", global_surfer_map_topic, "global_surfer_map");
    nh.param<std::string>("dynamic_corner_map", dynamic_corner_map_topic, "dynamic_corner_map");
    nh.param<std::string>("dynamic_surf_map", dynamic_surf_map_topic, "dynamic_surf_map");
    nh.param<std::string>("dymaic_map_query", dymaic_map_query, "dymaic_map_query");
    nh.param<std::string>("global_map_dir", global_map_dir, "global_map_dir");
    nh.param<std::string>("type", type, "");

    block_localization::GlobalmapServerNodelet::Params params;
    params.global_corner_map_topic = global_corner_map_topic;
    params.global_surfer_map_topic = global_surfer_map_topic;
    params.dynamic_corner_map_topic = dynamic_corner_map_topic;
    params.dynamic_surf_map_topic = dynamic_surf_map_topic;
    params.dymaic_map_query = dymaic_map_query;
    params.global_map_dir = global_map_dir;
    params.type = type;

    block_localization::GlobalmapServerNodelet map_server(params);

    ROS_INFO("\033[1;32m---->map_server Started.\033[0m");

    ros::spin();

    return 0;
}


// PLUGINLIB_EXPORT_CLASS(block_localization::GlobalmapServerNodelet, nodelet::Nodelet)
