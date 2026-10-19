#include "resource_timing.h"
#include "odometry_publication_state.h"
#include "bounded_history.h"
#include <ros/callback_queue.h>
#include "utility.h"
#include <stdexcept>
#include "roll/cloud_info.h"
#include "roll/save_map.h"
#include "roll/queryMap.h"
#include "roll/SaveTraj.h"

#include"LOAMmapping.h"
#include "globalOpt.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <boost/format.hpp> 

using namespace gtsam;

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;
typedef geometry_msgs::PoseWithCovarianceStampedConstPtr rvizPoseType;



// for trajectory alignment
GlobalOptimization globalEstimator(500);

struct keyFrame{
    double timestamp;
    PointTypePose point6d;
    PointType point3d;
    int idx;
    int is_indoor;
};


class mapOptimization : public ParamServer
{

public:

    // // time log
    // ofstream time_log_file(saveMapDirectory);
    ofstream pose_log_file;
    float fitnessScore = 0;
    ofstream gps_file;

    Eigen::Affine3f lastOdometryPose;
    nav_msgs::Path global_path_gtsam;

    // indoor outdoor keyframe detection
    vector<int> isIndoorKeyframe;
    vector<int> isIndoorKeyframeTMM;

    Eigen::Affine3f affine_lidar_to_imu;
    Eigen::Affine3f affine_imu_to_body;
    Eigen::Affine3f affine_lidar_to_body;
    Eigen::Affine3f affine_gps_to_body;
    //gps
    double rns;
    double rew;
    bool relocSuccess = false;
    bool tryReloc = false;
    bool mapLoaded = false;

    // for temporary mapping mode
    double rosTimeStart = -1;
    bool temporaryMappingMode = false;
    float transformBeforeMapped[6];
    bool goodToMergeMap = false;
    int startTemporaryMappingIndex = -1;
    float mergeNoise;
    bool frameTobeAbandoned = false;
    
    int TMMcount = 0;
    
    float TMMx = 0,TMMy = 0;
    float maxErrorX = 0, maxErrorY = 0, maxError = -1.0;


    // // use intensity channel of K pointclouds to record the error counts: K=5 for now
    // int K = 5;
    // vector<pcl::PointCloud<PointType>> errorCounts; // not the best way to do this 

    // data analysis
    int iterCount = 0;
    vector<float> mappingTimeVec;
    int edgePointCorrNum = 0;
    int surfPointCorrNum = 0;
    float maxEdgeIntensity = -1;
    float maxSurfIntensity = -1;
    float maxIntensity = -1;

    Eigen::Affine3f correctedPose;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr dynamic_cloudKeyPoses3D;
    // for kitti pose save
    Eigen::Affine3f H_init;
    vector<Eigen::Affine3f> pose_kitti_vec;

    bool doneSavingMap = false;

    Eigen::Affine3f affine_imu_to_odom; // convert points in lidar frame to odom frame
    Eigen::Affine3f affine_imu_to_map;
    Eigen::Affine3f affine_odom_to_map;

    vector<vector<double>> mappingLogs;
    vector<float> noiseVec;
    pcl::PointCloud<PointType>::Ptr submap;
    vector<rvizPoseType> poseEstVec;
    Eigen::Affine3f relocCorrection;

    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::Publisher pub_global_path_gtsam;
    ros::Publisher pubLidarCloudSurround;
    ros::Publisher pubLidarOdometryGlobal;
    ros::Publisher pubLidarOdometryGlobalFusion;
    ros::Publisher vinsFusion;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;
    ros::Publisher pubPathFusion;
    ros::Publisher pubPathFusionVINS;

    ros::Publisher gpsTrajPub;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubMergedMap;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubLoopConstraintEdge;
    ros::Publisher pubKeyPosesTmp;

    ros::Subscriber subCloud;
    ros::Subscriber subGPS;
    ros::Subscriber subGT;
    ros::Subscriber subLoop;
    ros::Subscriber subLidarOdometry;
    ros::Subscriber initialpose_sub;
    ros::ServiceServer srvSaveMap;

    std::deque<nav_msgs::Odometry> gpsQueue;
    std::deque<nav_msgs::Odometry> gtQueue;

    roll::cloud_info cloudInfo;
    queue<roll::cloud_infoConstPtr> cloudInfoBuffer;
    queue<nav_msgs::Odometry::ConstPtr> lidarOdometryBuffer;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    vector<pcl::PointCloud<PointType>::Ptr> temporaryCornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> temporarySurfCloudKeyFrames;


    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr temporaryCloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr temporaryCloudKeyPoses6D;


    pcl::PointCloud<PointType>::Ptr lidarCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr lidarCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr lidarCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr lidarCloudSurfLastDS; // downsampled surf featuer set from odoOptimization



    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> lidarCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr lidarCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr lidarCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr lidarCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr lidarCloudSurfFromMapDS;

    pcl::PointCloud<PointType>::Ptr lidarCloudCornerFromMidMapDS;
    pcl::PointCloud<PointType>::Ptr lidarCloudSurfFromMidMapDS;

    pcl::PointCloud<PointType>::Ptr lidarCloudCornerFromDynamicMap;//Dynamic map.
    pcl::PointCloud<PointType>::Ptr lidarCloudSurfFromDynamicMap;
    pcl::PointCloud<PointType>::Ptr lidarCloudCornerFromDynamicMapDS;
    pcl::PointCloud<PointType>::Ptr lidarCloudSurfFromDynamicMapDS;





    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses_dynamic;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    pcl::VoxelGrid<PointType> downSizeFilterSavingKeyframes; // for surrounding key poses of scan-to-map optimization

    pcl::VoxelGrid<PointType> downSizeFilterCorner_map;
    pcl::VoxelGrid<PointType> downSizeFilterSurf_map;
    
    ros::Time timeLidarInfoStamp;
    double cloudInfoTime;


    float transformTobeMapped[6];
    float transformTobeMapped_last[6];
    bool hasLastGlobalPose = false;

    std::deque<float> inlier_ratio_history;
    const int history_window_size = 10;
    float last_inlier_ratio = 0.5f;

    std::mutex mtx;
    std::mutex mtxInit;
    std::mutex mtxLoopInfo;
    std::mutex pose_estimator_mutex;
    // std::mutext mtxReloc;
    std::mutex mtx_map_;
    std::mutex mtx_save_traj_;
    std::mutex mtx_fusion_path_;
    std::mutex mtx_vins_path_;
    ros::CallbackQueue vins_publication_queue_;
    ros::NodeHandle vins_publication_nh_;
    ros::Subscriber subRealtimeVins_;
    OdometryCorrectionSnapshot odometry_correction_;
    MatchingKeyframeGate matching_keyframe_gate_;
    bool force_matching_pending_ = false; // matching-worker owned; survives map readiness waits
    ros::CallbackQueue odometry_publication_queue_;
    ros::NodeHandle odometry_publication_nh_;
    ros::Subscriber subRealtimeOdometry_;

    

    int lidarCloudCornerFromMidMapDSNum = 0;
    int lidarCloudSurfFromMidMapDSNum = 0;
    int lidarCloudCornerFromMapDSNum = 0;
    int lidarCloudSurfFromMapDSNum = 0;
    int lidarCloudCornerLastDSNum = 0;
    int lidarCloudSurfLastDSNum = 0;

    int lidarCloudCornerFromDynamicMapDSNum = 0;
    int lidarCloudSurfFromDynamicMapDSNum = 0;

    bool aLoopIsClosed = false;
    
    multimap<int,int>    loopIndexContainer;
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    
    deque<std_msgs::Float64MultiArray> loopInfoVec;
    vector<nav_msgs::Odometry> globalOdometry;
    
    nav_msgs::Path globalPath;
    nav_msgs::Path globalPathFusion;
    nav_msgs::Path globalPathFusionVINS;


    bool poseGuessFromRvizAvailable = false;
    float rvizGuess[6];

    
    pcl::PointCloud<PointType>::Ptr lidarCloudRaw; 
    
    //---map server---//
    ros::ServiceClient mapQuery_client;
    ros::ServiceClient mapQuery_mid_client;
    ros::Subscriber globalmap_sub;
    ros::Publisher real_time_odom_pub;
    ros::ServiceClient dynamic_mapQuery_client;
    ros::ServiceClient dynamic_mid_mapQuery_client;

    pcl::PointCloud<PointT>::Ptr prev_globalmap;
    pcl::PointCloud<PointT>::Ptr globalmap;

    pcl::PointCloud<PointT>::Ptr prev_globalmap_mid;
    pcl::PointCloud<PointT>::Ptr globalmap_mid;


    ros::Subscriber global_surf_map_sub;
    ros::Subscriber global_surf_map_sub_from_mid;
    ros::Subscriber global_corner_map_sub_from_mid;
    pcl::PointCloud<PointT>::Ptr prev_global_surf_map;
    pcl::PointCloud<PointT>::Ptr global_surf_map;
    pcl::PointCloud<PointT>::Ptr prev_global_surf_map_from_mid;
    pcl::PointCloud<PointT>::Ptr global_surf_map_from_mid;

    gtsam::Vector4 prev_centroid;
    gtsam::Vector4 curr_centroid;
    bool corner_map_init = false;
    bool surf_map_init = false;

    bool corner_mid_map_init = false;
    bool surf_mid_map_init = false;

    double points_pre_time, points_curr_time, mapqry_pre_time;
    bool systemInitialized = true;

    bool init_surf_map_success_flag_;
    bool init_corner_map_success_flag_;

    bool init_surf_mid_map_success_flag_;
    bool init_corner_mid_map_success_flag_;

    std::vector<std::string> tum_trajs_;
    ros::ServiceServer save_traj_srv_;

    std::string save_traj_name_;
    bool save_tum_traj_done_flag_ = false;

    bool use_static_map_current_scan_ = true;

    int loc_begin_scan_cnt = 0;




    mapOptimization()
    {
        if ((globalConstraintWeightMode != "adaptive" && globalConstraintWeightMode != "fixed") ||
            !std::isfinite(globalConstraintFixedPosSigma) || globalConstraintFixedPosSigma <= 0.0f ||
            !std::isfinite(globalConstraintFixedRotSigma) || globalConstraintFixedRotSigma <= 0.0f) {
            ROS_FATAL("Invalid global constraint settings: mode must be adaptive/fixed and fixed sigmas must be finite and positive");
            throw std::invalid_argument("Invalid global constraint weight settings");
        }
        if (!std::isfinite(globalConstraintInlierWeight) || globalConstraintInlierWeight < 0.0f ||
            !std::isfinite(globalConstraintStabilityWeight) || globalConstraintStabilityWeight < 0.0f) {
            throw std::invalid_argument("Adaptive coefficients must be finite and nonnegative");
        }
        ROS_INFO("[AdaptiveCoefficients] inlier=%.6f history=0.3 stability=%.6f (no normalization; confidence clipped to [0,1])",
                 globalConstraintInlierWeight, globalConstraintStabilityWeight);
        ROS_INFO("[GlobalConstraintWeight] mode=%s fixed_pos_sigma=%.6f m fixed_rot_sigma=%.6f rad (long/mid maps)",
                 globalConstraintWeightMode.c_str(), globalConstraintFixedPosSigma, globalConstraintFixedRotSigma);
        pose_log_file.open(saveMapDirectory + "/global_matching_pose_log.txt");
        pose_log_file.setf(ios::fixed, ios::floatfield);  // Use fixed-point notation for floating-point values.
        pose_log_file.precision(6); // Use six decimal places.

        gpsTrajPub      = nh.advertise<nav_msgs::Odometry> ("/roll/gps_odom", 1);
        gps_file.setf(ios::fixed, ios::floatfield);  // Use fixed-point notation for floating-point values.
        gps_file.precision(6); // Use six decimal places.
        gps_file.open(saveMapDirectory+"/gps.txt",ios::out);
        if(!gps_file.is_open())
        {
            cout<<"Cannot open"<<saveMapDirectory+"/gps.txt"<<endl;
        }
        // // ISAM2Params parameters;
        // // parameters.relinearizeThreshold = 0.1;
        // // parameters.relinearizeSkip = 1;
        // // isam = new ISAM2(parameters);

        pub_global_path_gtsam = nh.advertise<nav_msgs::Path>("global_path_gtsam", 100);
        initialpose_sub = nh.subscribe("/initialpose", 1, &mapOptimization::initialpose_callback, this);

        pubKeyPosesTmp                 = nh.advertise<sensor_msgs::PointCloud2>("/roll/mapping/tmp_key_poses", 1);
        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>("/roll/mapping/key_poses", 1);
        pubLidarCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("/roll/mapping/map_global", 1);
        pubLidarOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("/roll/mapping/odometry", 1);
        pubLidarOdometryGlobalFusion      = nh.advertise<nav_msgs::Odometry> ("/roll/mapping/odometry_fusion", 1);
        vinsFusion      = nh.advertise<nav_msgs::Odometry> ("/roll/mapping/vins_fusion", 1);
        pubPath                     = nh.advertise<nav_msgs::Path>("/roll/mapping/path", 1);
        pubPathFusion               = nh.advertise<nav_msgs::Path>("/roll/mapping/path_fusion", 1);
        pubPathFusionVINS               = nh.advertise<nav_msgs::Path>("/roll/mapping/path_fusion_vins", 1);

        pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>("/roll/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>("/roll/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/roll/mapping/loop_closure_constraints", 1);

        pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>("/roll/mapping/map_local", 1);

        pubMergedMap = nh.advertise<sensor_msgs::PointCloud2>("/roll/mapping/merged_map", 1); 

        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>("/roll/mapping/cloud_registered", 1);

        subCloud = nh.subscribe<roll::cloud_info>("/roll/feature/cloud_info", 5, &mapOptimization::lidarCloudInfoHandler, this);
        subGPS   = nh.subscribe<sensor_msgs::NavSatFix> (gpsTopic, 200, &mapOptimization::gpsHandler, this);
        subGT   = nh.subscribe<nav_msgs::Odometry> (gtTopic, 200, &mapOptimization::gtHandler, this);
        subLidarOdometry = nh.subscribe<nav_msgs::Odometry> ("/Odometry", 200, &mapOptimization::lidarOdometryHandler,this);
        // The primary high-rate pose must not share the map/backend callback queue.
        // Start its single spinner in main only after construction is complete.
        odometry_publication_nh_.setCallbackQueue(&odometry_publication_queue_);
        subRealtimeOdometry_ = odometry_publication_nh_.subscribe<nav_msgs::Odometry>(
            "/Odometry", 200, &mapOptimization::publishRealtimeOdometry, this,
            ros::TransportHints().tcpNoDelay());
        vins_publication_nh_.setCallbackQueue(&vins_publication_queue_);
        subRealtimeVins_ = vins_publication_nh_.subscribe<nav_msgs::Odometry>(
            "/Odometry", 200, &mapOptimization::publishRealtimeVins, this,
            ros::TransportHints().tcpNoDelay());

        

        srvSaveMap  = nh.advertiseService("/roll/save_map", &mapOptimization::saveMapService, this);

      

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

        downSizeFilterCorner_map.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf_map.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);


        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        // gps parameter calculation
        double earthEqu = 6378135;
        double earthPolar = 6356750;
        double tmp = sqrt(earthEqu*earthEqu*cos(deg2rad(lati0))*cos(deg2rad(lati0)) +earthPolar*earthPolar*sin(deg2rad(lati0))*sin(deg2rad(lati0)));
        rns = earthEqu*earthEqu*earthPolar*earthPolar/tmp/tmp/tmp;
        rew = earthEqu*earthEqu/tmp;

        // Map servers load their maps before advertising services. roslaunch starts
        // sibling nodes asynchronously, therefore create clients only after the
        // required services exist. The previous `this` argument was converted to
        // bool and accidentally created persistent clients before the servers were
        // ready; those clients remained invalid for the entire run.
        std::vector<std::string> required_map_services;
        // Offline block-map generation only reads the saved keyframes and writes
        // block PCDs; it does not query a running map server. Requiring
        // /mapQuery here deadlocks that standalone conversion workflow.
        if (localizationMode && !generate_block_map_) {
            required_map_services.push_back("/mapQuery");
        }
        if (localizationMode && !generate_block_map_ && use_dynamic_map_loading_) {
            required_map_services.push_back(dymaic_map_query_name_);
        }
        if (localizationMode && !generate_block_map_ && use_mid_map_to_loc_ != 0) {
            required_map_services.push_back("/mapQuery_mid");
            if (use_dynamic_map_loading_) {
                required_map_services.push_back(dymaic_mid_map_query_name_);
            }
        }
        for (const auto& service_name : required_map_services) {
            ROS_INFO("[MapQuery] waiting for service %s", service_name.c_str());
            if (!ros::service::waitForService(service_name, ros::Duration(120.0))) {
                ROS_FATAL("[MapQuery] service %s unavailable after 120 seconds", service_name.c_str());
                throw std::runtime_error("Required map query service unavailable: " + service_name);
            }
        }
        mapQuery_client = nh.serviceClient<roll::queryMap>("/mapQuery", false);
        mapQuery_mid_client = nh.serviceClient<roll::queryMap>("/mapQuery_mid", false);
        dynamic_mapQuery_client = nh.serviceClient<roll::queryMap>(dymaic_map_query_name_, false);
        dynamic_mid_mapQuery_client = nh.serviceClient<roll::queryMap>(dymaic_mid_map_query_name_, false);
        if (localizationMode) {
            ROS_INFO("[MapQuery] all required services are ready; using reconnectable clients");
        }
        ROS_INFO("dymaic_map_query_name_ = %s, dymaic_mid_map_query_name_ = %s", dymaic_map_query_name_.c_str(), dymaic_mid_map_query_name_.c_str());
        save_traj_srv_ = nh.advertiseService("save_traj", &mapOptimization::saveTrajService, this);
        //block_map
        // globalmap_sub = nh.subscribe("/globalmap", 1, &mapOptimization::globalCornerMapCallBack, this, ros::TransportHints().tcpNoDelay());
        // global_surf_map_sub = nh.subscribe("/global_surf_map", 1, &mapOptimization::globalSurfMapCallBack, this, ros::TransportHints().tcpNoDelay());
        //dynamic cube map
        globalmap_sub = nh.subscribe(dynamic_corner_map_long_topic_, 1, &mapOptimization::globalCornerMapCallBack, this, ros::TransportHints().tcpNoDelay());
        global_surf_map_sub = nh.subscribe(dynamic_surf_map_long_topic_, 1, &mapOptimization::globalSurfMapCallBack, this, ros::TransportHints().tcpNoDelay());

        global_surf_map_sub_from_mid = nh.subscribe(dynamic_surf_map_mid_topic_, 1, &mapOptimization::globalSurfMidMapCallBack, this, ros::TransportHints().tcpNoDelay());
        global_corner_map_sub_from_mid = nh.subscribe(dynamic_corner_map_mid_topic_, 1, &mapOptimization::globalCornerMapMidMapCallBack, this, ros::TransportHints().tcpNoDelay());
        real_time_odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);

        allocateMemory();

        auto saveBlockMap =[&](const vector<pcl::PointCloud<PointType>::Ptr>& sur_keyframes, 
                        const vector<pcl::PointCloud<PointType>::Ptr>& corner_keyframes, 
                        const pcl::PointCloud<PointTypePose>::Ptr& poses6d,
                        const pcl::PointCloud<PointType>::Ptr& poses3d,
                        double dist_save_ths)
        {
            if(sur_keyframes.empty() || corner_keyframes.empty() || poses6d->points.empty() || poses3d->points.empty()){
                return false;
            }

            pcl::KdTreeFLANN<PointType>::Ptr bm_kdtree;
            bm_kdtree.reset(new pcl::KdTreeFLANN<PointType>());
            // bm_kdtree->setInputCloud(cloudKeyPoses3D); // create kd-tree
        
            // bm_kdtree->radiusSearch(pt, (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
            // *lidarCloudCornerFromMap += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            vector<pcl::PointCloud<PointType>>sur_BM;
            vector<pcl::PointCloud<PointType>> corner_BM; 
            pcl::PointCloud<PointType>::Ptr centers(new pcl::PointCloud<PointType>());

            int key_frame_num = poses6d->size();
            pcl::PointCloud<PointType>::Ptr tmp_centers(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType> tmp_sur_BM;
            pcl::PointCloud<PointType> tmp_corner_BM;

            pcl::PointCloud<PointType> tmp_next_sur_BM;
            pcl::PointCloud<PointType> tmp_next_corner_BM;

            PointType bm_start_pose = poses3d->points[0];
            
            bool first_bm_build_flag = false;
            PointType tmp_center;
            for(int i = 0; i < key_frame_num; i++)
            {
                float bm_dist = pointDistance(bm_start_pose, poses3d->points[i]);

                
                tmp_sur_BM += *transformPointCloud(sur_keyframes[i], &poses6d->points[i]);
                tmp_corner_BM += *transformPointCloud(corner_keyframes[i], &poses6d->points[i]);


                tmp_center.x += poses3d->points[i].x;
                tmp_center.y += poses3d->points[i].y;
                tmp_center.z += poses3d->points[i].z;

                if(bm_dist >= 0.8 * dist_save_ths){
                    tmp_next_sur_BM +=*transformPointCloud(sur_keyframes[i], &poses6d->points[i]);
                    tmp_next_corner_BM += *transformPointCloud(corner_keyframes[i], &poses6d->points[i]);
                }
                
                
                if(bm_dist >= dist_save_ths){

                    sur_BM.push_back(tmp_sur_BM);
                    corner_BM.push_back(tmp_corner_BM);

                    int cur_key_frames = tmp_sur_BM.size();

                    tmp_center.x /= (cur_key_frames * 1.0);
                    tmp_center.y /= (cur_key_frames * 1.0);
                    tmp_center.z /= (cur_key_frames * 1.0);

                    tmp_centers->push_back(tmp_center);

                    //reset
                    PointType init_center;
                    tmp_center = init_center;

                    tmp_sur_BM.clear();
                    tmp_corner_BM.clear();

                    tmp_sur_BM = tmp_next_sur_BM;
                    tmp_corner_BM = tmp_next_corner_BM;

                    tmp_next_sur_BM.clear();
                    tmp_next_corner_BM.clear();
                    
                    bm_start_pose = i+1 < poses3d->points.size()? poses3d->points[i+1]: poses3d->points[i];



                    // if(!first_bm_build_flag){
                        //tood Merge adjacent maps.
                        // std::vector<int> point_search_ind;
                        // std::vector<float> point_search_sqDis;

                        // double surrounding_map_search_radius = 50;
                        // bm_kdtree->setInputCloud(tmp_centers);
                        // bm_kdtree->radiusSearch(tmp_center, surrounding_map_search_radius, point_search_ind, point_search_sqDis);
                        
                        // if(point_search_ind.empty()){
                        //     continue;
                        // }
                    // }

                    
                }

                
            }
            ROS_INFO("generate %d block_map, %d corner map, %d surf map", tmp_centers->size(), corner_BM.size(), sur_BM.size());

            boost::format save_format("%03d.pcd");
            for(int i = 0; i < tmp_centers->size(); i++){


                std::string corner_map_name = global_map_dir_+ global_corner_map_dir_+"/"+ (save_format % i).str();
                ROS_INFO("corner_map_name = %s, point size = %d", corner_map_name.c_str(), corner_BM[i].points.size());
                pcl::io::savePCDFileBinary(corner_map_name, corner_BM[i]);


                std::string surf_map_name = global_map_dir_ + gloabl_surf_map_dir_+ "/" + (save_format % i).str();
                ROS_INFO("surf_map_name = %s point size = %d ", surf_map_name.c_str(), sur_BM[i].points.size());
                pcl::io::savePCDFileBinary(surf_map_name, sur_BM[i]);

            }

            std::string center_pose_file_name = global_map_dir_ + "CentroidCloud";
            ROS_INFO("center_pose_file_name = %s, tmp_centers point size = %d", center_pose_file_name.c_str(), tmp_centers->size());
            pcl::io::savePCDFileBinary(center_pose_file_name + ".pcd", *tmp_centers);
            ROS_INFO("block map save done");
            //  pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
        };

        if (localizationMode && generate_block_map_)
        { // even ctrl+C won't terminate loading process
            std::lock_guard<std::mutex> lock(mtxInit);
            // load keyframe map

            if (!mapLoaded){


                ROS_INFO("************************loading keyframe map************************");
                string filePath = loadKeyframeMapDirectory + "/poses.txt";
                ifstream fin(filePath);
                if (!fin.is_open()) {
                    cout << filePath << " is not valid!" << endl;
                    // Stop processing if opening the file fails to prevent subsequent errors.
                    return; // Alternatively, use exit(1); adapt to the function return type.
                }

                // Read all nine fields before processing each record, including the final line.
                PointTypePose point;
                PointType point3;
                double timestamp; // Read the timestamp in the first column even if unused.
                int key_frame_id; // Store the field in the eighth column (meaning unspecified).
                int tmp;

                // Process a record only if all nine fields were read successfully; otherwise stop.
                std::vector<keyFrame> key_frames;
                int i = 0;
                while (fin >> timestamp >> point.x >> point.y >> point.z 
                        >> point.roll >> point.pitch >> point.yaw 
                        >> point.intensity >> tmp) {

                    keyFrame tmp_key_frame;

                    point3.x = point.x;
                    point3.y = point.y;
                    point3.z = point.z;
                    point3.intensity = point.intensity;

                    tmp_key_frame.point6d = point;
                    tmp_key_frame.point3d = point3;
                    tmp_key_frame.timestamp = timestamp;
                    tmp_key_frame.is_indoor = tmp;
                    tmp_key_frame.idx = point.intensity;
                    
                    // Append the data without a peek() check so the final line is retained.
                    // cloudKeyPoses6D->push_back(point);
                    // cloudKeyPoses3D->push_back(point3);
                    // isIndoorKeyframe.push_back(tmp);

                    key_frames.push_back(tmp_key_frame);
                }

                std::sort(key_frames.begin(), key_frames.end(), 
                    [](const keyFrame& frame_a, const keyFrame& frame_b) {
                        // Sort by timestamp in ascending order.
                        return frame_a.timestamp < frame_b.timestamp;
                    }
                );

                for(const auto& frame : key_frames){

                    PointTypePose point = frame.point6d;
                    PointType point3 = frame.point3d;
                    int tmp = frame.is_indoor;

                    cloudKeyPoses6D->push_back(point);
                    cloudKeyPoses3D->push_back(point3);
                    isIndoorKeyframe.push_back(tmp);

                }


                int keyframeN = (int)cloudKeyPoses6D->size();
                ROS_INFO("There are in total %d keyframes", keyframeN);
                for (int i = 0; i < key_frames.size(); i++ ){

                    int idx = key_frames[i].idx;
                    pcl::PointCloud<PointType>::Ptr cornerKeyFrame(new pcl::PointCloud<PointType>());
                    pcl::PointCloud<PointType>::Ptr surfKeyFrame(new pcl::PointCloud<PointType>());
                    string cornerFileName = loadKeyframeMapDirectory + "/corner"+ to_string(idx) + ".pcd";
                    string surfFileName = loadKeyframeMapDirectory + "/surf"+ to_string(idx) + ".pcd";
                    if (pcl::io::loadPCDFile<PointType> (cornerFileName, *cornerKeyFrame) == -1) 
                       cout<< "Couldn't read file"+ cornerFileName <<endl;
                    if (pcl::io::loadPCDFile<PointType> (surfFileName, *surfKeyFrame) == -1) 
                       cout<< "Couldn't read file"+ surfFileName <<endl;
                    cornerCloudKeyFrames.push_back(cornerKeyFrame);
                    surfCloudKeyFrames.push_back(surfKeyFrame);
                    if (i%100 == 0)
                        cout << "\r" << std::flush << "Loading feature cloud " << i << " of " << keyframeN-1 << " ...\n";
                }
                ROS_INFO("************************Keyframe map loaded************************");
                mapLoaded=true;

                //Convert these maps to Block_map format.
                if(generate_block_map_){
                    saveBlockMap(surfCloudKeyFrames, cornerCloudKeyFrames, cloudKeyPoses6D,
                                         cloudKeyPoses3D, block_map_generate_dist_ths_);
                }
                
            }
            
        }
        
    }

    bool saveTrajService(roll::SaveTraj::Request &req, roll::SaveTraj::Response &res){
        

        save_traj_name_ = req.filename;
        save_traj_dir_ += save_traj_name_ + ".tum";
        ROS_INFO("[saveTrajService]: save traj in %s", save_traj_dir_.c_str());

        std::ofstream tum_file;
        tum_file.open(save_traj_dir_);
        if (!tum_file.is_open()) {
            ROS_ERROR("无法打开TUM轨迹文件: %s", save_traj_dir_.c_str());
            res.success = false;
            save_tum_traj_done_flag_ = false;
            return false;
        }

        std::vector<std::string> temp_tum_trajs;
        {
            std::lock_guard<std::mutex> lock(mtx_save_traj_);
            temp_tum_trajs =tum_trajs_; 
        }

        for(const auto& tum : tum_trajs_){
            tum_file<< tum;
        }
        res.success = true;
        save_tum_traj_done_flag_ = true;
        return true;
        
    }

    void initBlockMap(const sensor_msgs::PointCloud2ConstPtr& points_msg, pcl::VoxelGrid<PointType>& down_size_filter,
                    pcl::PointCloud<PointT>::Ptr& global_block_map, pcl::PointCloud<PointT>::Ptr& pre_global_block_map, 
                    pcl::PointCloud<PointType>::Ptr& lidar_cloud_block_from_map_ds,int lidar_cloud_block_from_map_ds_num, 
                    bool& map_init_flag){

        auto mapConvert= [&](pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out, int& point_cloud_num)
        {
            if(nullptr == cloud_in){
                ROS_ERROR("map convert cloud in is nullptr");
                return false;
            }
            if(0 == cloud_in->size()){
                ROS_ERROR("cloud_in points is empty !!!");
                return false;
            }
            down_size_filter.setInputCloud(cloud_in);
            down_size_filter.filter(*cloud_out);
            
            // *cloud_out = *cloud_in;
            point_cloud_num = cloud_out->size();

            return true;
        };
        
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*points_msg, *cloud);
        if(!map_init_flag) {
            global_block_map = cloud;
            pre_global_block_map = global_block_map;
            // registration->setInputTarget(globalmap);
            pcl::compute3DCentroid(*global_block_map, curr_centroid);
            prev_centroid = curr_centroid;
            map_init_flag = true;

            mapConvert(global_block_map, lidar_cloud_block_from_map_ds, lidar_cloud_block_from_map_ds_num);
            ROS_INFO("initBlockMap: global_block_map size = %d,  lidar_cloud_block_from_map_ds size = %d", 
                    global_block_map->size(), lidar_cloud_block_from_map_ds->size());
        } else {
            global_block_map = cloud;
            pcl::compute3DCentroid(*global_block_map, curr_centroid);

            mapConvert(global_block_map, lidar_cloud_block_from_map_ds,lidar_cloud_block_from_map_ds_num);
        }
        static int print_cnt = 0;
        if(0 == print_cnt++ % 100 ){
            ROS_INFO("global map received!, the map centroid = (%f, %f, %f)", curr_centroid[0], curr_centroid[1], curr_centroid[2]);
        }
        
    }

    bool isPointCloudEmpty(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    // 1. Check for a null pointer before dereferencing it.
    if (!points_msg) {
        ROS_ERROR("points_msg is nullptr");
        return true; // Treat a null pointer as an empty point cloud.
    }
    // 2. Compute the total point count: width * height.
    size_t total_points = points_msg->width * points_msg->height;
    // 3. A zero point count indicates an empty cloud.
    return (total_points == 0);
    }


    void globalCornerMapCallBack(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
        
        std::lock_guard<std::mutex> lock(mtx_map_);

        if(isPointCloudEmpty(points_msg)){
            ROS_ERROR("global corner map point cloud is empty");
            return ;
        }

        initBlockMap(points_msg, downSizeFilterCorner_map, globalmap, prev_globalmap, 
                        lidarCloudCornerFromMapDS, lidarCloudCornerFromMapDSNum, corner_map_init);
        if(corner_map_init)
             init_corner_map_success_flag_ = true;
        else{
            ROS_ERROR("global corner map load fail");
        }

    }

    void globalCornerMapMidMapCallBack(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
        
        std::lock_guard<std::mutex> lock(mtx_map_);

        if(isPointCloudEmpty(points_msg)){
            ROS_ERROR("global corner map for mid point cloud is empty");
            return ;
        }

        initBlockMap(points_msg, downSizeFilterCorner_map, globalmap_mid, prev_globalmap_mid, 
                        lidarCloudCornerFromMidMapDS, lidarCloudCornerFromMidMapDSNum, corner_mid_map_init);
        // ROS_INFO("lidarCloudCornerFromMidMapDSNum = %d, lidarCloudCornerFromMidMapDS = %d", 
        //              lidarCloudCornerFromMidMapDSNum, lidarCloudCornerFromMidMapDS->size());
        if(corner_mid_map_init)
             init_corner_mid_map_success_flag_ = true;
        else{
            ROS_ERROR("global corner mid map load fail");
        }

    }

    void globalSurfMapCallBack(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
        
        std::lock_guard<std::mutex> lock(mtx_map_);

        if(isPointCloudEmpty(points_msg)){
            ROS_ERROR("global surf map point cloud is empty");
            return;
        }

        initBlockMap(points_msg, downSizeFilterSurf_map, global_surf_map, prev_global_surf_map, 
                            lidarCloudSurfFromMapDS, lidarCloudSurfFromMapDSNum, surf_map_init);
        if(surf_map_init){
            init_surf_map_success_flag_ = true;
        }else{
            ROS_ERROR("global surf map load fail");
        }
        
    }

    void globalSurfMidMapCallBack(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
        
        std::lock_guard<std::mutex> lock(mtx_map_);

        if(isPointCloudEmpty(points_msg)){
            ROS_ERROR("global surf map point cloud is empty");
            return;
        }

        initBlockMap(points_msg, downSizeFilterSurf_map, global_surf_map_from_mid, prev_global_surf_map_from_mid, 
                            lidarCloudSurfFromMidMapDS, lidarCloudSurfFromMidMapDSNum, surf_mid_map_init);
        if(surf_mid_map_init){
            init_surf_mid_map_success_flag_ = true;
        }else{
            ROS_ERROR("global surf mid map load fail");
        }
        
    }


    void queryMapSwitch(gtsam::Pose3 pose) {
        //todo Update points_curr_time.
        double time_elapsed = points_curr_time - mapqry_pre_time;

        if((time_elapsed < mapqry_interval_) && !systemInitialized)
        {
            ROS_WARN("queryMapSwitch return time_elapsed %f < mapqry_interval_ %f, systemInitialized = %d",
                     time_elapsed, mapqry_interval_, systemInitialized);
            return;
        }
            
        mapqry_pre_time = points_curr_time;

        ROS_INFO("request pose:(%f, %f, %f)", pose.x(), pose.y(), pose.z());

        TicToc time_call_Service;
        const auto call_map_query = [&pose](ros::ServiceClient& client, const std::string& service_name) {
            roll::queryMap query;
            query.request.position.x = pose.x();
            query.request.position.y = pose.y();
            query.request.position.z = pose.z();
            if (!client.call(query) || !query.response.success) {
                ROS_WARN_THROTTLE(1.0, "[MapQuery] call failed: %s", service_name.c_str());
                return false;
            }
            return true;
        };
        bool queries_ok = true;
        if (!use_dynamic_map_loading_)
            queries_ok = call_map_query(mapQuery_client, "/mapQuery");
        if (use_dynamic_map_loading_) {
            queries_ok = call_map_query(dynamic_mapQuery_client, dymaic_map_query_name_) && queries_ok;
        }
        if (use_mid_map_to_loc_ != 0) {
            if (use_dynamic_map_loading_) {
                queries_ok = call_map_query(dynamic_mid_mapQuery_client, dymaic_mid_map_query_name_) && queries_ok;
            }
            if (!use_dynamic_map_loading_)
                queries_ok = call_map_query(mapQuery_mid_client, "/mapQuery_mid") && queries_ok;
        }
        if (queries_ok) {
            ROS_INFO_THROTTLE(10.0, "[MapQuery] all required queries succeeded at (%.3f, %.3f, %.3f)",
                              pose.x(), pose.y(), pose.z());
        }
        if(debugMode) cout<<"time_call_Service: "<<time_call_Service.toc()<<endl;

        nav_msgs::Odometry real_time_odom;
        real_time_odom.pose.pose.position.x = pose.x();
        real_time_odom.pose.pose.position.y = pose.y();
        real_time_odom.pose.pose.position.z = pose.z();
        real_time_odom_pub.publish(real_time_odom);

    }

    void allocateMemory()
    {        
        resetISAM();
        affine_imu_to_body = pcl::getTransformation(-0.11, -0.18, -0.71, 0.0, 0.0, 0.0);
        affine_lidar_to_body = pcl::getTransformation(0.002, -0.004, -0.957, 0.014084807063594,0.002897246558311,-1.583065991436417);
        affine_gps_to_body = pcl::getTransformation(-0.24, 0,-1.24, 0,0,0);

        affine_lidar_to_imu = affine_imu_to_body.inverse()*affine_lidar_to_body;

        affine_imu_to_map = Eigen::Affine3f::Identity();
        affine_imu_to_odom = Eigen::Affine3f::Identity();
        affine_odom_to_map = Eigen::Affine3f::Identity();
        odometry_correction_.update(affine_odom_to_map, !localizationMode);
        correctedPose = Eigen::Affine3f::Identity();

        globalEstimator.inputIMUToBodyAffine(affine_imu_to_body);

        submap.reset(new pcl::PointCloud<PointType>()); // why dot when it is pointer type

        lidarCloudRaw.reset(new pcl::PointCloud<PointType>()); 

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        temporaryCloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        temporaryCloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        kdtreeSurroundingKeyPoses_dynamic.reset(new pcl::KdTreeFLANN<PointType>());

        lidarCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        lidarCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        lidarCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        lidarCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization


        lidarCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        lidarCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        lidarCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        lidarCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        lidarCloudSurfFromMidMapDS.reset(new pcl::PointCloud<PointType>());
        lidarCloudCornerFromMidMapDS.reset(new pcl::PointCloud<PointType>());

        lidarCloudCornerFromDynamicMap.reset(new pcl::PointCloud<PointType>());//Dynamic map.
        lidarCloudSurfFromDynamicMap.reset(new pcl::PointCloud<PointType>());
        lidarCloudCornerFromDynamicMapDS.reset(new pcl::PointCloud<PointType>());
        lidarCloudSurfFromDynamicMapDS.reset(new pcl::PointCloud<PointType>());

        for (int i = 0; i < 6; ++i){
            transformBeforeMapped[i] = 0;
            transformTobeMapped[i] = 0;
            transformTobeMapped_last[i] = 0;
        }
        hasLastGlobalPose = false;
        
        // cv::Mat matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void odometryMsgToAffine3f(nav_msgs::Odometry msgIn,Eigen::Affine3f &trans)
    {
        tf::Quaternion tfQ(msgIn.pose.pose.orientation.x,msgIn.pose.pose.orientation.y,msgIn.pose.pose.orientation.z,msgIn.pose.pose.orientation.w);
        double roll,pitch,yaw;
        tf::Matrix3x3(tfQ).getRPY(roll,pitch,yaw);
        // think about why not update affine_imu_to_odom and affine_imu_to_map here!!!
        trans = pcl::getTransformation(msgIn.pose.pose.position.x,
        msgIn.pose.pose.position.y,msgIn.pose.pose.position.z, float(roll),float(pitch),float(yaw));

        
    }
     void odometryMsgToTrans(const nav_msgs::Odometry::ConstPtr& msgIn,float trans[6])
    {
        tf::Quaternion tfQ(msgIn->pose.pose.orientation.x,msgIn->pose.pose.orientation.y,msgIn->pose.pose.orientation.z,msgIn->pose.pose.orientation.w);
        double roll,pitch,yaw;
        tf::Matrix3x3(tfQ).getRPY(roll,pitch,yaw);
        trans[0] = roll;
        trans[1] = pitch;
        trans[2] = yaw;
        trans[3] = msgIn->pose.pose.position.x;
        trans[4] = msgIn->pose.pose.position.y;
        trans[5] = msgIn->pose.pose.position.z;
    }

    void lidarCloudInfoHandler(const roll::cloud_infoConstPtr& msgIn)
    {
        // === DEBUG: monitor cloudInfo frame interval ===
        {
            static double prevCloudStamp = 0;
            static int cloudFrameCnt = 0;
            double curStamp = msgIn->header.stamp.toSec();
            if (prevCloudStamp > 0) {
                double dt = curStamp - prevCloudStamp;
                if (dt > 0.5) {
                    ROS_ERROR("[CloudInfo] LARGE dt=%.4f s between frames! frame#%d, stamp=%.6f, bufSize=%d",
                              dt, cloudFrameCnt, curStamp, (int)cloudInfoBuffer.size());
                }
                if (cloudFrameCnt % 200 == 0) {
                    ROS_INFO("[CloudInfo] frame#%d, dt=%.4f s (%.1f Hz), stamp=%.3f, bufSize=%d",
                             cloudFrameCnt, dt, 1.0/dt, curStamp, (int)cloudInfoBuffer.size());
                }
            }
            prevCloudStamp = curStamp;
            cloudFrameCnt++;
        }

        mtx.lock();
        cloudInfoBuffer.push(msgIn);
        if (cloudInfoBuffer.size()>5)
            resource_metrics::event("matching_queue",msgIn->header.stamp.toSec(),"drop_capacity",cloudInfoBuffer.size()-5);
        trimQueue(cloudInfoBuffer, 5);
        resource_metrics::event("matching_queue", msgIn->header.stamp.toSec(), "enqueue", cloudInfoBuffer.size());
        mtx.unlock();
            }

    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& msgIn)
    {
        // === DEBUG: monitor LIO odom frame interval ===
        {
            static double prevOdomStamp = 0;
            static int odomFrameCnt = 0;
            double curStamp = msgIn->header.stamp.toSec();
            if (prevOdomStamp > 0) {
                double dt = curStamp - prevOdomStamp;
                if (dt > 0.5) {
                    ROS_ERROR("[LIO_Odom] LARGE dt=%.4f s between frames! frame#%d, stamp=%.6f",
                              dt, odomFrameCnt, curStamp);
                } else if (dt < 0.001) {
                    ROS_WARN("[LIO_Odom] TINY dt=%.6f s, possible duplicate frame#%d", dt, odomFrameCnt);
                }
                // print every 200 frames for heartbeat
                if (odomFrameCnt % 200 == 0) {
                    ROS_INFO("[LIO_Odom] frame#%d, dt=%.4f s (%.1f Hz), stamp=%.3f, bufSize=%d",
                             odomFrameCnt, dt, 1.0/dt, curStamp, (int)lidarOdometryBuffer.size());
                }
            }
            prevOdomStamp = curStamp;
            odomFrameCnt++;
        }

        mtx.lock();
        lidarOdometryBuffer.push(msgIn);
        trimQueue(lidarOdometryBuffer, maxSensorQueueSize);
        mtx.unlock();
        
        // in loc mode, publish lidar_to_map tf so pointcloud can be visualized in rviz
        Eigen::Affine3f legacy_correction;
        if (!odometry_correction_.read(legacy_correction)) return;

        Eigen::Affine3f affine_imu_to_odom_tmp;
        Eigen::Affine3f affine_imu_to_odom_tmp1;
        odometryMsgToAffine3f(*msgIn, affine_imu_to_odom_tmp1);

        // // use raw, motion-skewed clouds
        // affine_imu_to_odom_tmp =affine_imu_to_body*affine_imu_to_odom*affine_lidar_to_imu;
        // use deskewed, imu-centered clouds
        affine_imu_to_odom_tmp =affine_imu_to_body*affine_imu_to_odom_tmp1; 
        // Backend history ingestion may wait for optimization. Publishing must not.
        globalEstimator.inputOdom(msgIn->header.stamp.toSec(),affine_imu_to_odom_tmp.matrix().cast<double>());
    }

    void publishRealtimeVins(const nav_msgs::Odometry::ConstPtr& msgIn)
    {
        resource_metrics::event("fusion_pose", msgIn->header.stamp.toSec(), "odom_receive");
        Eigen::Affine3f legacy_correction;
        if (!odometry_correction_.read(legacy_correction)) {
            resource_metrics::event("fusion_pose", msgIn->header.stamp.toSec(), "skip_uninitialized");
            return;
        }
        Eigen::Affine3f raw_odometry;
        odometryMsgToAffine3f(*msgIn, raw_odometry);
        const Eigen::Affine3f affine_imu_to_odom_tmp = affine_imu_to_body * raw_odometry;
        const Eigen::Affine3f affine_imu_to_map_tmp = legacy_correction * affine_imu_to_odom_tmp;
        // testing fusion
        Eigen::Vector3d odomP;
        Eigen::Quaterniond odomQ;
        globalEstimator.projectRealtimeOdom(affine_imu_to_odom_tmp.matrix().cast<double>(), odomP, odomQ);

        // === DEBUG: detect fusion trajectory jump ===
        {
            static Eigen::Vector3d prevFusionP(0, 0, 0);
            static double prevFusionTime = 0;
            static bool fusionInitialized = false;
            double curTime = msgIn->header.stamp.toSec();
            if (fusionInitialized) {
                double dt = curTime - prevFusionTime;
                double dist = (odomP - prevFusionP).norm();
                double speed = (dt > 1e-6) ? dist / dt : 0;
                if (dist > 2.0) {
                    ROS_ERROR("[FusionJump] LARGE POSITION JUMP detected! dist=%.3f m, dt=%.4f s, speed=%.2f m/s", dist, dt, speed);
                    ROS_ERROR("[FusionJump]   prev: (%.3f, %.3f, %.3f) -> cur: (%.3f, %.3f, %.3f)",
                              prevFusionP.x(), prevFusionP.y(), prevFusionP.z(), odomP.x(), odomP.y(), odomP.z());
                }
                if (speed > 30.0) {
                    ROS_ERROR("[FusionJump] ABNORMAL SPEED! speed=%.2f m/s, dist=%.3f m, dt=%.4f s", speed, dist, dt);
                }
            }
            prevFusionP = odomP;
            prevFusionTime = curTime;
            fusionInitialized = true;
        }

        // === DEBUG: print input odom vs fusion output for comparison ===
        {
            float inputTrans[6];
            Affine3f2Trans(affine_imu_to_odom_tmp, inputTrans);
            float mapTrans[6];
            Affine3f2Trans(affine_imu_to_map_tmp, mapTrans);
            static int fusionPrintCnt = 0;
            if (fusionPrintCnt++ % 50 == 0) {
                ROS_INFO("[FusionDebug] inputOdom=(%.3f,%.3f,%.3f) odom2map=(%.3f,%.3f,%.3f) fusion=(%.3f,%.3f,%.3f)",
                         inputTrans[3], inputTrans[4], inputTrans[5],
                         mapTrans[3], mapTrans[4], mapTrans[5],
                         odomP.x(), odomP.y(), odomP.z());
            }
        }

        static ros::Time last_vins_fusion_time_stamp = msgIn->header.stamp;

        ros::Duration diff  = msgIn->header.stamp - last_vins_fusion_time_stamp;
        double ms_double = diff.toSec() * 1000;
        if(ms_double >1000){
            ROS_ERROR("time stamp is diff than 1000, curr time = %f, last time = %f", msgIn->header.stamp .toSec(), last_vins_fusion_time_stamp.toSec());
        }

        nav_msgs::Odometry odomFusion;
        odomFusion.header.frame_id = mapFrame;
        odomFusion.header.stamp = msgIn->header.stamp;
        odomFusion.pose.pose.orientation.x = odomQ.x();
        odomFusion.pose.pose.orientation.y = odomQ.y();
        odomFusion.pose.pose.orientation.z = odomQ.z();
        odomFusion.pose.pose.orientation.w = odomQ.w();
        odomFusion.pose.pose.position.x = odomP[0];
        odomFusion.pose.pose.position.y = odomP[1];
        odomFusion.pose.pose.position.z = odomP[2];

        vinsFusion.publish(odomFusion);
        resource_metrics::event("fusion_pose", msgIn->header.stamp.toSec(), "publish");
        last_vins_fusion_time_stamp = odomFusion.header.stamp;


        geometry_msgs::PoseStamped pose_stampedF;
        pose_stampedF.pose = odomFusion.pose.pose;
        pose_stampedF.header.frame_id = mapFrame;
        pose_stampedF.header.stamp = msgIn->header.stamp;
        std::lock_guard<std::mutex> path_lock(mtx_vins_path_);
        globalPathFusionVINS.poses.push_back(pose_stampedF);
        trimFront(globalPathFusionVINS.poses, maxTrajectoryHistorySize);
        globalPathFusionVINS.header.stamp = msgIn->header.stamp;
        globalPathFusionVINS.header.frame_id = mapFrame;
        if (pubPathFusionVINS.getNumSubscribers()) pubPathFusionVINS.publish(globalPathFusionVINS);

        // std::string tum_pose;
        // tum_pose += ss.str();
        // {
        //     std::lock_guard<std::mutex> lock(mtx_save_traj_);
        //     tum_trajs_.push_back(tum_pose);
        // }
        

        

    }

    void publishRealtimeOdometry(const nav_msgs::Odometry::ConstPtr& msgIn)
    {
        resource_metrics::event("odometry_fusion_pose", msgIn->header.stamp.toSec(), "odom_receive");
        Eigen::Affine3f correction;
        if (!odometry_correction_.read(correction)) {
            resource_metrics::event("odometry_fusion_pose", msgIn->header.stamp.toSec(), "skip_uninitialized");
            return;
        }
        Eigen::Affine3f raw_odometry;
        odometryMsgToAffine3f(*msgIn, raw_odometry);
        const Eigen::Affine3f affine_imu_to_map_tmp = correction * affine_imu_to_body * raw_odometry;

        float array_imu_to_map[6];
        Affine3f2Trans(affine_imu_to_map_tmp, array_imu_to_map);
        tf::Quaternion q = tf::createQuaternionFromRPY(array_imu_to_map[0],array_imu_to_map[1],array_imu_to_map[2]);
        
        nav_msgs::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = mapFrame;
        odomAftMapped.header.stamp = msgIn->header.stamp;
        odomAftMapped.pose.pose.orientation.x = q.x();
        odomAftMapped.pose.pose.orientation.y = q.y();
        odomAftMapped.pose.pose.orientation.z = q.z();
        odomAftMapped.pose.pose.orientation.w = q.w();
        odomAftMapped.pose.pose.position.x = array_imu_to_map[3];
        odomAftMapped.pose.pose.position.y = array_imu_to_map[4];
        odomAftMapped.pose.pose.position.z = array_imu_to_map[5];
        pubLidarOdometryGlobalFusion.publish(odomAftMapped);
        // Keep this endpoint separate from vinsFusion: the trajectories differ.
        resource_metrics::event("odometry_fusion_pose", msgIn->header.stamp.toSec(), "publish");

        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(array_imu_to_map[0], array_imu_to_map[1], array_imu_to_map[2]),
                                                      tf::Vector3(array_imu_to_map[3], array_imu_to_map[4], array_imu_to_map[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, msgIn->header.stamp, mapFrame, lidarFrame);
        br.sendTransform(trans_odom_to_lidar);
        // // sometimes it cannot find tf when bagtime is not the same with the sensor time
        // tf::TransformListener tf_;
        // tf::StampedTransform stamped_lidar_to_baselink;
        // try
        // {
        //     tf_.lookupTransform(baselinkFrame, lidarFrame, msgIn->header.stamp, stamped_lidar_to_baselink);
        // }
        // catch(tf::TransformException e)
        // {
        //     ROS_ERROR("Failed to compute lidar_to_baselink: (%s)", e.what());
        //     return;
        // }
        // tf::Transform t_odom_to_baselink = t_odom_to_lidar*stamped_lidar_to_baselink;
        // child frame 'lidar_link' expressed in parent_frame 'mapFrame'
        // tf::StampedTransform trans_odom_to_baselink = tf::StampedTransform(t_odom_to_baselink, msgIn->header.stamp, mapFrame, baselinkFrame);
        // br.sendTransform(trans_odom_to_baselink);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose = odomAftMapped.pose.pose;
        pose_stamped.header.frame_id = mapFrame;
        pose_stamped.header.stamp = msgIn->header.stamp;
        {
            std::lock_guard<std::mutex> lock(mtx_fusion_path_);
            globalPathFusion.poses.push_back(pose_stamped);
            trimFront(globalPathFusion.poses, maxTrajectoryHistorySize);
            globalPathFusion.header.stamp = msgIn->header.stamp;
            globalPathFusion.header.frame_id = mapFrame;
            if (pubPathFusion.getNumSubscribers()) pubPathFusion.publish(globalPathFusion);
        }




        static bool init_flag = true;
        if (init_flag==true)
        {
            H_init = affine_imu_to_map_tmp;
            init_flag=false;
        }
        Eigen::Affine3f H_rot;
        
        // for benchmarking in kitti
        // how kitti camera frame (xright y down z forward) rotates into REP103
        // kitti x uses -y from roll
        // H_rot.matrix() << 0,-1,0,0, 
        //                     0,0,-1,0,
        //                     1,0,0,0,
        //                     0,0,0,1;

        // for fusion_pose output
        H_rot.matrix() << 1,0,0,0, 
                            0,1,0,0,
                            0,0,1,0,
                            0,0,0,1;
        Eigen::Affine3f H = affine_imu_to_map_tmp;
        H = H_rot*H_init.inverse()*H; //to get H12 = H10*H02 , 180 rot according to z axis
        pose_kitti_vec.push_back(H);
        trimFront(pose_kitti_vec, maxTrajectoryHistorySize);
    }

    void transformUpdate()
    {
        mtx.lock();
        // === DEBUG: detect affine_odom_to_map jump ===
        {
            static Eigen::Vector3f prevOdomToMapTrans(0,0,0);
            static bool odomToMapInit = false;
            Eigen::Affine3f newOdomToMap = affine_imu_to_map*affine_imu_to_odom.inverse();
            Eigen::Vector3f curTrans = newOdomToMap.translation();
            if (odomToMapInit) {
                float change = (curTrans - prevOdomToMapTrans).norm();
                if (change > 1.0) {
                    ROS_ERROR("[transformUpdate] affine_odom_to_map JUMP! change=%.3f m", change);
                    ROS_ERROR("[transformUpdate]   prev_trans=(%.3f,%.3f,%.3f) -> cur_trans=(%.3f,%.3f,%.3f)",
                              prevOdomToMapTrans.x(), prevOdomToMapTrans.y(), prevOdomToMapTrans.z(),
                              curTrans.x(), curTrans.y(), curTrans.z());
                }
            }
            prevOdomToMapTrans = curTrans;
            odomToMapInit = true;
        }
        affine_odom_to_map = affine_imu_to_map*affine_imu_to_odom.inverse();
        odometry_correction_.update(affine_odom_to_map, !localizationMode || relocSuccess);

        // cout<<"affine_imu_to_map "<<affine_imu_to_map.matrix()<<endl;
        // cout<<"affine_odom_to_map "<<affine_odom_to_map.matrix()<<endl;
        // // Publish TF
        // static tf::TransformBroadcaster br;
        // tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
        //                                               tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        // // child frame 'lidar_link' expressed in parent_frame 'mapFrame'
        // tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLidarInfoStamp, mapFrame, lidarFrame);
        // br.sendTransform(trans_odom_to_lidar);
        mtx.unlock();
    }  
std::string getCurrentTimeStringLegacy(const std::string& format = "%Y%m%d_%H%M%S")
{
    // Get the current time in seconds.
    std::time_t now = std::time(NULL);
    // Convert to local time.
    std::tm* local_now = std::localtime(&now);

    // Allocate a buffer large enough for the timestamp string.
    const int buf_size = 128;
    char buffer[buf_size];
    // Format the timestamp using the updated default format.
    std::strftime(buffer, buf_size, format.c_str(), local_now);

    return std::string(buffer);
}
    void run()
    {
        ros::Rate matchingRate(globalMatchingRate);
        while(ros::ok()){ // why while(1) is not okay???
            while (!cloudInfoBuffer.empty() && !lidarOdometryBuffer.empty())
            {
                mtx.lock();
                while (!lidarOdometryBuffer.empty() && lidarOdometryBuffer.front()->header.stamp.toSec() < cloudInfoBuffer.front()->header.stamp.toSec())
                {
                    lidarOdometryBuffer.pop();
                }
                if (lidarOdometryBuffer.empty()){
                    mtx.unlock();
                    break;
                }

                // lower the global matching frequency to speed up
                timeLidarInfoStamp = cloudInfoBuffer.front()->header.stamp;

                cloudInfoTime = cloudInfoBuffer.front()->header.stamp.toSec();

                double lidarOdometryTime = lidarOdometryBuffer.front()->header.stamp.toSec();

                if(debugMode) cout<<setiosflags(ios::fixed)<<setprecision(3)<<"cloud time: "<<cloudInfoTime-rosTimeStart<<endl;

                if (rosTimeStart < 0) rosTimeStart = cloudInfoTime;

                if (abs(lidarOdometryTime - cloudInfoTime) > 0.05) // normally >, so pop one cloud_info msg
                {
                     ROS_WARN("Unsync message!");
                    cloudInfoBuffer.pop();  // pop the old one,otherwise it  will go to dead loop, different from aloam 
                    resource_metrics::event("matching_queue", cloudInfoTime, "drop_unsync", 1);
                    mtx.unlock();
                    break;
                }

                // extract info and feature cloud
                roll::cloud_infoConstPtr cloudInfoMsg = cloudInfoBuffer.front();
                nav_msgs::Odometry::ConstPtr lidarOdometryMsg =  lidarOdometryBuffer.front();

                lidarCloudRaw.reset(new pcl::PointCloud<PointType>()); 
                cloudInfo = *cloudInfoMsg;
                Eigen::Affine3f tmp;
                odometryMsgToAffine3f(*lidarOdometryMsg,tmp);
                // // // use raw, motion-skewed clouds
                // affine_imu_to_odom =affine_imu_to_body*affine_imu_to_odom*affine_lidar_to_imu;
                // use deskewed, imu-centered clouds
                affine_imu_to_odom =affine_imu_to_body*tmp;

                pcl::fromROSMsg(cloudInfoMsg->cloud_corner,  *lidarCloudCornerLast);
                pcl::fromROSMsg(cloudInfoMsg->cloud_surface, *lidarCloudSurfLast);
                pcl::fromROSMsg(cloudInfoMsg->cloud_raw,  *lidarCloudRaw);
                // clear
                lidarOdometryBuffer.pop(); 
                resource_metrics::event("matching_queue", cloudInfoTime, "dequeue", cloudInfoBuffer.size());
                resource_metrics::event("matching_queue", cloudInfoTime, "drop_backlog", cloudInfoBuffer.size()-1);
                while (!cloudInfoBuffer.empty())
                {
                    cloudInfoBuffer.pop();
                    // ROS_INFO_STREAM("popping old cloud_info messages for real-time performance");
                }
                mtx.unlock();

                resource_metrics::Scope resource_timer("mapping_cycle", cloudInfoTime);
                TicToc mapping;
                
                force_matching_pending_ = force_matching_pending_ || poseGuessFromRvizAvailable || !relocSuccess;
                updateInitialGuess(); // actually the same as ALOAM
                odometry_correction_.update(affine_odom_to_map, !localizationMode || relocSuccess);

                if ((tryReloc == true || relocSuccess == true))
                {
                    if(!init_corner_map_success_flag_ || !init_surf_map_success_flag_){
                        //todoConsider handling this after obtaining the block map.
                        TicToc extract;
                        extractNearbyBM();
                        ROS_WARN("wait for block map init, init_corner_map_success_flag_ = %d, init_surf_map_success_flag_ = %d",
                                            init_corner_map_success_flag_, init_surf_map_success_flag_);
                        continue;
                    }
                }

                if ((tryReloc == true || relocSuccess == true))
                {
                    TicToc extract;

                    // saveFrame() only controls map storage AFTER matching. Gate
                    // expensive localization matching here, using the existing
                    // distance-only keyframe policy (NCLT default: 1 metre).
                    if (localizationMode && !matching_keyframe_gate_.shouldMatch(
                            tmp.translation(), surroundingkeyframeAddingDistThreshold, force_matching_pending_)) {
                        resource_metrics::event("matching_selection", cloudInfoTime, "skip_non_keyframe");
                        continue;
                    }
                    resource_metrics::event("matching_selection", cloudInfoTime, "selected_keyframe");

                    TicToc extratNearbyDynamicMap1;
                    if(useRealtimeLocalMap_){
                        extratNearbyDynamicMap();
                        if(debugMode) cout<<"extratNearbyDynamicMap: "<<extratNearbyDynamicMap1.toc()<<endl;
                    }


                    
                    if(debugMode) cout<<"extract: "<<extract.toc()<<endl;
                    TicToc downsample;
                    downsampleCurrentScan();

                    if(debugMode) cout<<"downsample: "<<downsample.toc()<<endl;
                    TicToc opt;
                    ROS_INFO("before scan2MapOptimization: current scan size = corner %ld, surf %ld, map size corner %ld, surf %ld, lidarCloudCornerFromDynamicMapDS = %ld, lidarCloudSurfFromDynamicMapDS = %ld", 
                        lidarCloudCornerLastDS->size(), lidarCloudSurfLastDS->size(), 
                        lidarCloudCornerFromMapDS->size(), lidarCloudSurfFromMapDS->size(),
                        lidarCloudCornerFromDynamicMapDS->size(), lidarCloudSurfFromDynamicMapDS->size());
                    // scan2MapOptimization();

                    // if(use_static_map_current_scan_ || loc_begin_scan_cnt < 10){
                    //     scan2MapOptimizationStaticMap();

                    //     use_static_map_current_scan_ = false;
                    // }else
                    {
                        resource_metrics::Scope pure_matching_timer("global_matching", cloudInfoTime);
                        resource_metrics::event("matching_task", cloudInfoTime, "start");
                        // determine dynamic map pointers based on useRealtimeLocalMap_ config
                        auto dynCorner = useRealtimeLocalMap_ ? lidarCloudCornerFromDynamicMapDS : nullptr;
                        auto dynSurf   = useRealtimeLocalMap_ ? lidarCloudSurfFromDynamicMapDS   : nullptr;

                        if (use_mid_map_to_loc_ == 0 || (use_mid_map_to_loc_ == 2 && loc_begin_scan_cnt % 2 == 0)){
                            ROS_INFO("use long map");
                            scan2MapOptimizationDynamicMap(lidarCloudCornerLastDS, lidarCloudSurfLastDS,
                                                            lidarCloudCornerFromMapDS, lidarCloudSurfFromMapDS, affine_imu_to_map,
                                                            false, dynCorner, dynSurf);
                        }else if(use_mid_map_to_loc_ == 3){
                            ROS_INFO("use both long and mid map");
                            scan2MapOptimizationDynamicMap(lidarCloudCornerLastDS, lidarCloudSurfLastDS,
                                                            lidarCloudCornerFromMapDS, lidarCloudSurfFromMapDS, affine_imu_to_map,
                                                            false, dynCorner, dynSurf);
                            scan2MapOptimizationDynamicMap(lidarCloudCornerLastDS, lidarCloudSurfLastDS,
                                lidarCloudCornerFromMidMapDS, lidarCloudSurfFromMidMapDS, affine_imu_to_map, true,
                                dynCorner, dynSurf);
                        }else{
                            ROS_INFO("use mid map");
                            bool use_mid_map = true;
                            scan2MapOptimizationDynamicMap(lidarCloudCornerLastDS, lidarCloudSurfLastDS,
                                lidarCloudCornerFromMidMapDS, lidarCloudSurfFromMidMapDS, affine_imu_to_map, use_mid_map,
                                dynCorner, dynSurf);
                        }
                        resource_metrics::event("matching_task", cloudInfoTime, "finish");
                    }
                    matching_keyframe_gate_.commit(tmp.translation());
                    force_matching_pending_ = false;
                    loc_begin_scan_cnt++;
                    
                    
                    float optTime = opt.toc();
                    resource_metrics::record("matching_stage", cloudInfoTime, optTime);
                    if(debugMode)  cout<<"optimization: "<<optTime<<endl; // > 90% of the total time
                    
                    TicToc optPose;
                    if (localizationMode)
                    {

                        saveTemporaryKeyframes();
                        updatePathRELOC(cloudInfoMsg);  // for visualizing in rviz
                    }
                    else
                    {//Within the sliding window, skip LiDAR odometry prediction and odometry constraints.
                        saveKeyFramesAndFactor();
                        correctPoses();
                    }
                    float optPoseTime = optPose.toc();
                    if(debugMode)  cout<<"pose opt. takes "<< optPoseTime<<endl;
                    TicToc publish;
                    // publishLocalMap();
                    publishOdometry();
                    transformUpdate();
                    if(debugMode)  cout<<"publish. takes "<< publish.toc()<<endl;
                    
                    frameTobeAbandoned = false;
                    // cout<<"publish: "<<publish.toc()<<endl;
                    // printTrans("after mapping: ",transformTobeMapped);
                    mappingTimeVec.push_back(mapping.toc());
                    trimFront(mappingTimeVec, maxMappingTimeHistorySize);

                    // ROS_INFO_STREAM("At time "<< cloudInfoTime - rosTimeStart);
                    if (goodToMergeMap)
                    {
                        //Local map optimization with only the first-frame prior cannot correct accumulated drift without loop closures.
                        if (mapUpdateEnabled)
                            mergeMap();
                        // downsize temporary maps to slidingWindowSize
                        auto iteratorKeyPoses3D = temporaryCloudKeyPoses3D->begin();
                        auto iteratorKeyPoses6D = temporaryCloudKeyPoses6D->begin();
                        auto iteratorKeyFramesC = temporaryCornerCloudKeyFrames.begin();
                        auto iteratorKeyFramesS = temporarySurfCloudKeyFrames.begin();
                        auto iteratorKeyFramesI = isIndoorKeyframeTMM.begin();
                        // usually added cloud would not be big so just leave the sparsification to savingMap
                        // ROS_INFO_STREAM("At time "<< cloudInfoTime - rosTimeStart<< " sec, Merged map has "<<(int)temporaryCloudKeyPoses3D->size()<< " key poses");
                        while ((int)temporaryCloudKeyPoses3D->size() > slidingWindowSize)
                        {
                            // automatically +1
                            temporaryCloudKeyPoses3D->erase(iteratorKeyPoses3D);
                            temporaryCloudKeyPoses6D->erase(iteratorKeyPoses6D);
                            temporaryCornerCloudKeyFrames.erase(iteratorKeyFramesC);
                            temporarySurfCloudKeyFrames.erase(iteratorKeyFramesS);
                            isIndoorKeyframeTMM.erase(iteratorKeyFramesI);
                        }
                        // cout<<temporaryCloudKeyPoses3D->size()<<endl;
                        // cout<<"reindexing: key poses and key frames are corresponding with respect to the adding sequence"<<endl;
                        for (int i = 0 ; i< (int)temporaryCloudKeyPoses3D->size(); i++)
                        {
                            temporaryCloudKeyPoses3D->points[i].intensity = i;
                            temporaryCloudKeyPoses6D->points[i].intensity = i;
                        }
                        goodToMergeMap = false;
                        temporaryMappingMode = false;
                    }
                    if(debugMode)  cout<<"mapping time: "<<mappingTimeVec.back()<<endl;
                    matchingRate.sleep();
                }

             
                
            }
            if(!save_tum_traj_done_flag_ && tum_trajs_.size() >100)
            {
                ROS_INFO("run is done, start save tum traj to file");
                {
                    save_traj_name_ = getCurrentTimeStringLegacy();
                    save_traj_dir_ += save_traj_name_ + ".tum";
                    ROS_INFO("[saveTrajService]: save traj in %s", save_traj_dir_.c_str());

                    std::ofstream tum_file;
                    tum_file.open(save_traj_dir_);
                    if (!tum_file.is_open()) {
                        ROS_ERROR("无法打开TUM轨迹文件: %s", save_traj_dir_.c_str());
                        continue;
                    }
                    ROS_INFO("tum_trajs_ size = %d", tum_trajs_.size());

                    std::vector<std::string> temp_tum_trajs;
                    {
                        std::lock_guard<std::mutex> lock(mtx_save_traj_);
                        temp_tum_trajs =tum_trajs_; 
                    }

                    for(const auto& tum : temp_tum_trajs){
                        tum_file<< tum;
                    }
                    save_tum_traj_done_flag_ = true;
                }

            }

        }

    }

    void mergeMap()
    {
        cout<<" DO gtsam optimization here"<<endl;
        TicToc t_merge;
        int priorNode = 0;
        
        // gtsam
        NonlinearFactorGraph gtSAMgraphTM;
        Values initialEstimateTM;        
        Values isamCurrentEstimateTM;
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        // Each merge owns its temporary optimizer; release it on every exit path.
        ISAM2 isamTM(parameters);

        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, 1e-2, 1e-1, 1e-1, 1e-1).finished()); // rad*rad, meter*meter
        gtsam::Pose3 posePrior = pclPointTogtsamPose3(temporaryCloudKeyPoses6D->points[priorNode]);
        gtSAMgraphTM.add(PriorFactor<Pose3>(priorNode, posePrior, priorNoise));
        initialEstimateTM.insert(priorNode, posePrior);

        int tempSize = temporaryCloudKeyPoses6D->points.size();
        if (tempSize < 3 ) return;
        for (int i = priorNode; i < tempSize - 2; i++)
        {
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) <<1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3 ).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(temporaryCloudKeyPoses6D->points[i]);
            gtsam::Pose3 poseTo   = pclPointTogtsamPose3(temporaryCloudKeyPoses6D->points[i+1]);
            gtSAMgraphTM.add(BetweenFactor<Pose3>(i,i+1, poseFrom.between(poseTo), odometryNoise));
            initialEstimateTM.insert(i+1, poseTo);
            // update iSAM
            isamTM.update(gtSAMgraphTM, initialEstimateTM);
            isamTM.update();
            gtSAMgraphTM.resize(0);
            initialEstimateTM.clear();
        }

        // Eigen::Affine3f wrongPose = pclPointToAffine3f(temporaryCloudKeyPoses6D->points[tempSize -1 ]);
        gtsam::Pose3 poseCorr = Affine3f2gtsamPose(correctedPose);

        // cout<<" add prior factor instead to constrain the covariances"<<endl;
        // cannot put it in the loop above
        // odomFactor needs to be a smooth one!!!
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(temporaryCloudKeyPoses6D->points[tempSize - 2 ]);
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(temporaryCloudKeyPoses6D->points[tempSize - 1 ]);
        gtSAMgraphTM.add(BetweenFactor<Pose3>(tempSize - 2 , tempSize -1, poseFrom.between(poseTo), odometryNoise));

        noiseModel::Diagonal::shared_ptr corrNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3).finished()); // rad*rad, meter*meter
        gtSAMgraphTM.add(PriorFactor<Pose3>(tempSize - 1, poseCorr, corrNoise));
        initialEstimateTM.insert(tempSize - 1, poseCorr);
        
        // cout<<"before opt. "<< poseCorr.translation().x()<<" "<< poseCorr.translation().y()<<" "<< poseCorr.translation().z()<<endl;

        // update iSAM
        isamTM.update(gtSAMgraphTM, initialEstimateTM);
        isamTM.update();
        isamTM.update();
        isamTM.update();
        gtSAMgraphTM.resize(0);
        initialEstimateTM.clear();

        
        isamCurrentEstimateTM = isamTM.calculateEstimate();

        // cout<<"pose correction: "<<isamCurrentEstimateTM.size()<<endl;

        std::vector<int> keyPoseSearchIdx;
        std::vector<float> keyPoseSearchDist;
        for (int i = priorNode; i < tempSize; i++)
        {
            // change every loop
            pcl::KdTreeFLANN<PointType>::Ptr keyPosesTree(new pcl::KdTreeFLANN<PointType>());
            
            mtx.lock();
            keyPosesTree->setInputCloud(cloudKeyPoses3D);
            mtx.unlock();

            auto poseCov = isamTM.marginalCovariance(i);

            temporaryCloudKeyPoses3D->points[i].x = isamCurrentEstimateTM.at<Pose3>(i).translation().x();
            temporaryCloudKeyPoses3D->points[i].y = isamCurrentEstimateTM.at<Pose3>(i).translation().y();
            temporaryCloudKeyPoses3D->points[i].z = isamCurrentEstimateTM.at<Pose3>(i).translation().z();

            temporaryCloudKeyPoses6D->points[i].x = temporaryCloudKeyPoses3D->points[i].x;
            temporaryCloudKeyPoses6D->points[i].y = temporaryCloudKeyPoses3D->points[i].y;
            temporaryCloudKeyPoses6D->points[i].z = temporaryCloudKeyPoses3D->points[i].z;
            temporaryCloudKeyPoses6D->points[i].roll  = isamCurrentEstimateTM.at<Pose3>(i).rotation().roll();
            temporaryCloudKeyPoses6D->points[i].pitch = isamCurrentEstimateTM.at<Pose3>(i).rotation().pitch();
            temporaryCloudKeyPoses6D->points[i].yaw   = isamCurrentEstimateTM.at<Pose3>(i).rotation().yaw();

            // temporaryCloudKeyPoses3D->points[i].intensity = i; // no change here actually
            // temporaryCloudKeyPoses6D->points[i].intensity = i;

            keyPosesTree->nearestKSearch(temporaryCloudKeyPoses3D->points[i],1, keyPoseSearchIdx, keyPoseSearchDist);
            // cout<<"nearest: "<<keyPoseSearchDist.size()<<endl;
            mtx.lock();
            if (keyPoseSearchDist[0] < 2*surroundingKeyframeDensity)
            {
                // cout<<keyPoseSearchIdx[0]<<endl;
                // TODO:cloudKeyPoses3D is not loaded yet; merge maps by adding them directly to block_map. 
                // cloudKeyPoses3D->erase(cloudKeyPoses3D->begin() + keyPoseSearchIdx[0]);
                // cloudKeyPoses6D->erase(cloudKeyPoses6D->begin() + keyPoseSearchIdx[0]);
                cornerCloudKeyFrames.erase(cornerCloudKeyFrames.begin() + keyPoseSearchIdx[0]);
                surfCloudKeyFrames.erase(surfCloudKeyFrames.begin() + keyPoseSearchIdx[0]);
                isIndoorKeyframe.erase(isIndoorKeyframe.begin() + keyPoseSearchIdx[0]);
            }
            mtx.unlock();
        }

        pcl::PointCloud<PointType>::Ptr cloudLocal(new pcl::PointCloud<PointType>());
        for (int i=0;i<(int)tempSize;i++)
        {
            int idx = temporaryCloudKeyPoses3D->points[i].intensity;
            *cloudLocal += *transformPointCloud(temporarySurfCloudKeyFrames[idx],&temporaryCloudKeyPoses6D->points[i]);
            *cloudLocal += *transformPointCloud(temporaryCornerCloudKeyFrames[idx],&temporaryCloudKeyPoses6D->points[i]);
        }
        publishCloud(&pubMergedMap, cloudLocal, timeLidarInfoStamp, mapFrame);



        for (int i = priorNode; i < tempSize; i++)
        {
            cloudKeyPoses3D->push_back(temporaryCloudKeyPoses3D->points[i]); // no "points." in between!!!
            cloudKeyPoses6D->push_back(temporaryCloudKeyPoses6D->points[i]);
            cornerCloudKeyFrames.push_back(temporaryCornerCloudKeyFrames[i]);
            surfCloudKeyFrames.push_back(temporarySurfCloudKeyFrames[i]); 
            isIndoorKeyframe.push_back(isIndoorKeyframeTMM[i]);
        }
        cout<<"map merge takes "<<t_merge.toc()<< " ms"<<endl; // negligible

        // reindexing due to the erasing operation
        for (int i = 0; i < (int) cloudKeyPoses3D->size(); i++)
        {
            cloudKeyPoses3D->points[i].intensity = i;
            cloudKeyPoses6D->points[i].intensity = i;
        }

    }

    void updatePathRELOC(const roll::cloud_infoConstPtr& msgIn){
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = msgIn->header.stamp;
        pose_stamped.header.frame_id = mapFrame;
        pose_stamped.pose.position.x = transformTobeMapped[3];
        pose_stamped.pose.position.y = transformTobeMapped[4];
        pose_stamped.pose.position.z = transformTobeMapped[5];
        tf::Quaternion q = tf::createQuaternionFromRPY(transformTobeMapped[0],transformTobeMapped[1],transformTobeMapped[2]);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();
        globalPath.poses.push_back(pose_stamped);
        trimFront(globalPath.poses, maxTrajectoryHistorySize);
    }

    void gpsHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg)
    {
        if (gpsMsg->status.status != 0 || isnan(gpsMsg->latitude) || isnan(gpsMsg->longitude) || isnan(gpsMsg->altitude)) 
        {
            if(debugMode) cout<<"Bad message, status: "<<gpsMsg->status.status<<endl;
            return;
        }
        // string gpsFrame = "/gps";
        // tf::TransformListener listener;
        // tf::StampedTransform transformStamped_lidar_to_gps;
        // try
        // {
        //     // ERROR] [1639030193.661625830, 1521156822.162099906]: "lidar_link" passed to lookupTransform argument target_frame does not exist
        //     listener.waitForTransform(lidarFrame, mapFrame, gpsMsg->header.stamp, ros::Duration(0.1));
        //     listener.lookupTransform(lidarFrame, mapFrame, gpsMsg->header.stamp, transformStamped_lidar_to_gps);
        // }
        // catch (tf::TransformException &ex) 
        // {
        //     ROS_ERROR("%s",ex.what());
        //     return;
        // }

        float x,y,z;
        x = sin(deg2rad(gpsMsg->latitude - lati0))*rns;
        y = sin(deg2rad(gpsMsg->longitude - longi0))*rew*cos(deg2rad(lati0));
        z = alti0 - gpsMsg->altitude;

        Eigen::Affine3f trans_gps_to_gpsM = pcl::getTransformation(x,y,z,0,0,0);
        Eigen::Affine3f trans_imu_to_map = trans_gps_to_gpsM;
        // Eigen::Affine3f trans_imu_to_map = affine_gps_to_body*trans_gps_to_gpsM*affine_gps_to_body.inverse()*affine_imu_to_body;
        // cout<<"after conversion x y z: "<<trans_imu_to_map(0,3)<<" "<<trans_imu_to_map(1,3)<<" "<<trans_imu_to_map(2,3)<<endl;
        nav_msgs::Odometry odomGPS;
        // notice PoseWithCovariance order: # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis; float64[36] covariance
        // different from GTSAM pose covariance order

        odomGPS.pose.covariance[0] = gpsMsg->position_covariance[0];
        odomGPS.pose.covariance[7] = gpsMsg->position_covariance[4];
        odomGPS.pose.covariance[14] = gpsMsg->position_covariance[8];
        odomGPS.pose.pose.position.x = trans_imu_to_map(0,3);
        odomGPS.pose.pose.position.y = trans_imu_to_map(1,3);
        odomGPS.pose.pose.position.z = trans_imu_to_map(2,3);
        odomGPS.pose.pose.orientation.x = 0;
        odomGPS.pose.pose.orientation.y = 0;
        odomGPS.pose.pose.orientation.z = 0;
        odomGPS.pose.pose.orientation.w = 1.0;
        odomGPS.header.stamp = gpsMsg->header.stamp;
        odomGPS.header.frame_id = "/map";
        gpsQueue.push_back(odomGPS);
        trimFront(gpsQueue, maxSensorQueueSize);
        gpsTrajPub.publish(odomGPS);

        gps_file<<odomGPS.header.stamp.toSec()<<" "<<odomGPS.pose.pose.position.x<<" "<<odomGPS.pose.pose.position.y<<" "<<odomGPS.pose.pose.position.z<<
            " "<<odomGPS.pose.covariance[0]<<" "<<odomGPS.pose.covariance[7]<<" "<<odomGPS.pose.covariance[14]<<"\n";
    }


    void gtHandler(const nav_msgs::Odometry::ConstPtr& gtMsg)
    {
        if (isnan(gtMsg->pose.pose.position.x) || isnan(gtMsg->pose.pose.position.y) || isnan(gtMsg->pose.pose.position.z))
        {
            return;
        }
        Eigen::Affine3f trans_body_to_map,trans_imu_to_map;
        odometryMsgToAffine3f(*gtMsg, trans_body_to_map);
        // // use raw, motion-skewed clouds
        // trans_lidar_to_map = trans_body_to_map*affine_imu_to_body;
        // use deskewed, imu-centered clouds
        trans_imu_to_map = trans_body_to_map*affine_imu_to_body;
        float roll,pitch,yaw,x,y,z;
        pcl::getTranslationAndEulerAngles(trans_imu_to_map,x,y,z,roll,pitch,yaw);
        tf::Quaternion q = tf::createQuaternionFromRPY(roll,pitch,yaw);
        nav_msgs::Odometry gtMsgNew;
        gtMsgNew.header = gtMsg->header;
        gtMsgNew.pose.covariance = gtMsg->pose.covariance;
        gtMsgNew.pose.pose.position.x = x;
        gtMsgNew.pose.pose.position.y = y;
        gtMsgNew.pose.pose.position.z = z;
        gtMsgNew.pose.pose.orientation.x = q.x();
        gtMsgNew.pose.pose.orientation.y = q.y();
        gtMsgNew.pose.pose.orientation.z = q.z();
        gtMsgNew.pose.pose.orientation.w = q.w();
        gtQueue.push_back(gtMsgNew);
        trimFront(gtQueue, maxSensorQueueSize);
    }


    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        // #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    gtsam::Pose3 Affine3f2gtsamPose(Eigen::Affine3f aff){
        float transformOut[6] = {0};
        Affine3f2Trans(aff, transformOut);
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformOut[0], transformOut[1], transformOut[2]), 
                                  gtsam::Point3(transformOut[3], transformOut[4], transformOut[5]));
    }
    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    
    bool saveMapService(roll::save_mapRequest& req, roll::save_mapResponse& res)
    {        
        float resMap,resPoseIndoor,resPoseOutdoor,overlapThre;
        
        if(req.resolutionMap != 0)            resMap = req.resolutionMap;
        else         resMap = 0.4;

        if(req.resPoseIndoor != 0)            resPoseIndoor = req.resPoseIndoor;
        else         resPoseIndoor = 2.0;

        if(req.resPoseOutdoor != 0)            resPoseOutdoor = req.resPoseOutdoor;
        else         resPoseOutdoor = 5.0;

        if(req.overlapThre != 0)            overlapThre = req.overlapThre;
        else         overlapThre = 0.9;

        float mappingTime = accumulate(mappingTimeVec.begin(),mappingTimeVec.end(),0.0);
        cout<<"Average time consumed by mapping is :"<<mappingTime/mappingTimeVec.size()<<" ms"<<endl;
        if (localizationMode) cout<<"Times of entering TMM is :"<<TMMcount<<endl;

        // saving pose estimates and GPS signals
        if (savePose)
        {
            // ofstream pose_file;
            cout<<"Recording trajectory..."<<endl;
            // string fileName = saveMapDirectory+"/pose_fusion_kitti.txt";
            // pose_file.open(fileName,ios::out);
            // pose_file.setf(std::ios::scientific, std::ios::floatfield);
            // pose_file.precision(6);
            // if(!pose_file.is_open())
            // {
            //     cout<<"Cannot open "<<fileName<<endl;
            //     return false;
            // }
            // // keyposes: kitti form (z forward x right y downward)
            // cout<<"Number of poses: "<<pose_kitti_vec.size()<<endl;
            // for (int i = 0; i <(int)pose_kitti_vec.size(); ++i)
            // {
            //     Eigen::Affine3f pose_kitti = pose_kitti_vec[i];
            //     pose_file<<pose_kitti(0,0)<<" "<<pose_kitti(0,1)<<" "<<pose_kitti(0,2)<<" "<<pose_kitti(0,3)<<" "
            //         <<pose_kitti(1,0)<<" "<<pose_kitti(1,1)<<" "<<pose_kitti(1,2)<<" "<<pose_kitti(1,3)<<" "
            //         <<pose_kitti(2,0)<<" "<<pose_kitti(2,1)<<" "<<pose_kitti(2,2)<<" "<<pose_kitti(2,3)<<"\n";

            // }
            // pose_file.close();


            // // 2nd: keyposes
            // int pointN = (int)globalPath.poses.size();
            // cout<< "There are "<<pointN<<" keyframes in total"<<endl;
            // for (int i = 0; i < pointN; ++i)
            // {
            //     geometry_msgs::PoseStamped tmp = globalPath.poses[i];
            //     pose_file<<tmp.pose.position.x<<" "<<tmp.pose.position.y<<" "<<tmp.pose.position.z<<"\n";
            // }
            // // 3rd: odometry msgs
            // int pointN = (int)globalOdometry.size();
            // cout<< "There are "<<pointN<<" in total"<<endl;
            // for (int i = 0; i < pointN; ++i)
            // {
            //     nav_msgs::Odometry tmp = globalOdometry[i];
            //     pose_file<<tmp.pose.pose.position.x<<" "<<tmp.pose.pose.position.y<<"\n";
            // }
            // pose_file.close();

            // 4th: stamped pose for odometry gt
            mtx.lock();
            ofstream pose_file2;
            string fileName2 = saveMapDirectory+"/path_mapping.txt";
            pose_file2.open(fileName2,ios::out);
            pose_file2.setf(ios::fixed, ios::floatfield);  // Use fixed-point notation for floating-point values.
            pose_file2.precision(6); // Use six decimal places.
            int pointN = (int)globalOdometry.size();
            cout<<"mapping pose size: "<<pointN<<endl;
            for (int i = 0; i < pointN; ++i)
            {
                nav_msgs::Odometry tmp = globalOdometry[i];
                double r,p,y;
                tf::Quaternion q(tmp.pose.pose.orientation.x,tmp.pose.pose.orientation.y, tmp.pose.pose.orientation.z,tmp.pose.pose.orientation.w);
                tf::Matrix3x3(q).getRPY(r,p,y);
                // save it in micro sec to compare it with the nclt gt
                pose_file2<<tmp.header.stamp.toSec()*1e+6<<" "<<tmp.pose.pose.position.x<<" "<<tmp.pose.pose.position.y<<" "<<
                tmp.pose.pose.position.z<<" "<<r<<" "<<p<<" "<<y<<" "<<"\n";
            }
            pose_file2.close();
            mtx.unlock();

            // This path is written only by the dedicated primary-pose callback.
            nav_msgs::Path savedFusionPath;
            {
                std::lock_guard<std::mutex> lock(mtx_fusion_path_);
                savedFusionPath = globalPathFusion;
            }
            // higher frequency odometry
            ofstream pose_file3;
            string fileName3 = saveMapDirectory+"/path_fusion.txt";
            pose_file3.open(fileName3,ios::out);
            pose_file3.setf(ios::fixed, ios::floatfield);  // Use fixed-point notation for floating-point values.
            pose_file3.precision(6); // Use six decimal places.
            int pointN3 = (int)savedFusionPath.poses.size();
            cout<<"fusion pose size: "<<pointN3<<endl;
            for (int i = 0; i < pointN3; ++i)
            {
                geometry_msgs::PoseStamped tmp = savedFusionPath.poses[i];
                double r,p,y;
                tf::Quaternion q(tmp.pose.orientation.x,tmp.pose.orientation.y, tmp.pose.orientation.z,tmp.pose.orientation.w);
                tf::Matrix3x3(q).getRPY(r,p,y);
                // save it in nano sec to compare it with the nclt gt
                pose_file3<<tmp.header.stamp.toSec()*1e+6<<" "<<tmp.pose.position.x<<" "<<tmp.pose.position.y<<" "<<
                tmp.pose.position.z<<" "<<r<<" "<<p<<" "<<y<<" "<<"\n";
            }
            pose_file3.close();

            nav_msgs::Path savedVinsPath;
            {
                std::lock_guard<std::mutex> lock(mtx_vins_path_);
                savedVinsPath = globalPathFusionVINS;
            }
            // higher frequency odometry
            ofstream pose_file4;
            string fileName4 = saveMapDirectory+"/path_vinsfusion.txt";
            pose_file4.open(fileName4,ios::out);
            pose_file4.setf(ios::fixed, ios::floatfield);  // Use fixed-point notation for floating-point values.
            pose_file4.precision(6); // Use six decimal places.
            int pointN4 = (int)savedVinsPath.poses.size();
            cout<<"vinsfusion pose size: "<<pointN4<<endl;
            for (int i = 0; i < pointN4; ++i)
            {
                geometry_msgs::PoseStamped tmp = savedVinsPath.poses[i];
                double r,p,y;
                tf::Quaternion q(tmp.pose.orientation.x,tmp.pose.orientation.y, tmp.pose.orientation.z,tmp.pose.orientation.w);
                tf::Matrix3x3(q).getRPY(r,p,y);
                // save it in nano sec to compare it with the nclt gt
                pose_file4<<tmp.header.stamp.toSec()*1e+6<<" "<<tmp.pose.position.x<<" "<<tmp.pose.position.y<<" "<<
                tmp.pose.position.z<<" "<<r<<" "<<p<<" "<<y<<" "<<"\n";
            }
            pose_file4.close();
            cout<<"Trajectory recording finished!"<<endl;
        }


        // save keyframe map: for every keyframe, save keyframe pose, edge point pcd, surface point pcd
        // keyframe pose in one file
        // every keyframe has two other files: cornerI.pcd surfI.pcd
        if (savePCD)
        {
            cout << "****************************************************" << endl;
            cout << "Saving map to pcd files ..." << endl;

            cout << "Save destination: " << saveMapDirectory << endl;

            // save key frame transformations
            pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
            pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);
            // extract global point cloud map
            pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
            for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
                int idx = cloudKeyPoses3D->points[i].intensity;
                *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[idx],  &cloudKeyPoses6D->points[idx]);
                *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[idx],    &cloudKeyPoses6D->points[idx]);
                // cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...\n";
            }

            cout << "\n\nSave resolution: " << resMap << endl;

            // down-sample and save corner cloud
            downSizeFilterCorner.setInputCloud(globalCornerCloud);
            downSizeFilterCorner.setLeafSize(resMap, resMap, resMap);
            downSizeFilterCorner.filter(*globalCornerCloudDS);
            pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloudDS);
            // down-sample and save surf cloud
            downSizeFilterSurf.setInputCloud(globalSurfCloud);
            downSizeFilterSurf.setLeafSize(resMap, resMap, resMap);
            downSizeFilterSurf.filter(*globalSurfCloudDS);
            pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);

            // save global point cloud map
            *globalMapCloud += *globalCornerCloud;
            *globalMapCloud += *globalSurfCloud;

            int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);
            res.success = ret == 0;

            cout << "Saving map to pcd files completed\n" << endl;
        }
        
        if(saveKeyframeMap){ 
            int keyframeN = (int)cloudKeyPoses6D->size();

            cout<<"Map Saving to "+saveKeyframeMapDirectory<<endl;
            cout<<"There are "<<keyframeN<<" keyframes before downsampling"<<endl;

            cout<<"********************Saving keyframes and poses one by one**************************"<<endl;
            pcl::PointCloud<PointType>::Ptr cloudKeyPoses3DDS(new pcl::PointCloud<PointType>());

            
            keyframeSparsification(cloudKeyPoses3DDS,resMap,resPoseIndoor,resPoseOutdoor,overlapThre);
            // keyframeSparsification(cloudKeyPoses3DDS,resMap,resPoseIndoor,resPoseOutdoor);

            int keyframeNDS = cloudKeyPoses3DDS->size();
            cout<<"There are "<<keyframeNDS<<" keyframes after downsampling"<<endl;
            ofstream pose_file;
            pose_file.open(saveKeyframeMapDirectory+"/poses.txt",ios::out); // downsampled
            if(!pose_file.is_open())
            {
                std::cout<<"Cannot open"<<saveKeyframeMapDirectory+"/poses.txt"<<std::endl;
                return false;
            }
            std::vector<int> keyframeSearchIdx;
            std::vector<float> keyframeSearchDist;
            pcl::KdTreeFLANN<PointType>::Ptr kdtreeKeyframes(new pcl::KdTreeFLANN<PointType>());
            kdtreeKeyframes->setInputCloud(cloudKeyPoses3D);
            int i = 0;
            
            // recover downsampled intensities
            for(auto& pt:cloudKeyPoses3DDS->points)
            {                
                kdtreeKeyframes->nearestKSearch(pt,1,keyframeSearchIdx,keyframeSearchDist); 
                pt.intensity = cloudKeyPoses6D->points[keyframeSearchIdx[0]].intensity;  
                pcl::io::savePCDFileBinary(saveKeyframeMapDirectory + "/corner" + std::to_string(i)+".pcd", *cornerCloudKeyFrames[pt.intensity]);
                pcl::io::savePCDFileBinary(saveKeyframeMapDirectory + "/surf" + std::to_string(i)+".pcd", *surfCloudKeyFrames[pt.intensity]);
                pose_file<<cloudKeyPoses6D->points[pt.intensity].x<<" "<<cloudKeyPoses6D->points[pt.intensity].y<<" "<<cloudKeyPoses6D->points[pt.intensity].z
                <<" "<<cloudKeyPoses6D->points[pt.intensity].roll<<" "<<cloudKeyPoses6D->points[pt.intensity].pitch<<" "<<cloudKeyPoses6D->points[pt.intensity].yaw
                << " " << i<<" "<<isIndoorKeyframe[pt.intensity]<<"\n";
                i++;
                if((i+1)%100 == 0) cout<<i<<" keyframes saved!"<<endl;
            }
            pose_file.close();
            cout<<"Keyframes Saving Finished!"<<endl;

        }
        res.success = true;
        doneSavingMap = true;
        return true;
    }

   
    void keyframeSparsification(pcl::PointCloud<PointType>::Ptr  &cloudKeyPoses3DDS, float resMap, float resPoseIndoor, float resPoseOutdoor,
                                float overlapThre)
    {

        TicToc sparsiTime;
        pcl::PointCloud<PointType>::Ptr  cloudKeyPoses3DDSinit(new pcl::PointCloud<PointType>());

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalKeyPoses(new pcl::KdTreeFLANN<PointType>());
        kdtreeGlobalKeyPoses->setInputCloud(cloudKeyPoses3D);

        // separate indoor or outdoor, sparsify crudely
        pcl::PointCloud<PointType>::Ptr keyPosesIndoor(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr keyPosesOutdoor(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr keyPosesIndoorDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr keyPosesOutdoorDS(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)cloudKeyPoses3D->points.size();i++)
        {
            if (isIndoorKeyframe[cloudKeyPoses3D->points[i].intensity] == 1) 
                keyPosesIndoor->push_back(cloudKeyPoses3D->points[i]);
            else 
                keyPosesOutdoor->push_back(cloudKeyPoses3D->points[i]);
        }

        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;      
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPosesI;

        //indoor
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPosesO;
        downSizeFilterGlobalMapKeyPosesO.setLeafSize(resPoseIndoor,resPoseIndoor,resPoseIndoor);
        downSizeFilterGlobalMapKeyPosesO.setInputCloud(keyPosesIndoor);
        downSizeFilterGlobalMapKeyPosesO.filter(*keyPosesIndoorDS);

        // fix the keyframe downsample bug, keep intensity as an index of adding sequence
        for(auto& pt : keyPosesIndoorDS->points)
        {
            kdtreeGlobalKeyPoses->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
            cloudKeyPoses3DDSinit->push_back(pt);
        }
        cout<<"indoor: "<<keyPosesIndoorDS->size()<<" frames" <<endl;
        //outdoor
        downSizeFilterGlobalMapKeyPosesI.setLeafSize(resPoseOutdoor,resPoseOutdoor,resPoseOutdoor);
        downSizeFilterGlobalMapKeyPosesI.setInputCloud(keyPosesOutdoor);
        downSizeFilterGlobalMapKeyPosesI.filter(*keyPosesOutdoorDS);
        // fix the keyframe downsample bug
        for(auto& pt : keyPosesOutdoorDS->points)
        {
            kdtreeGlobalKeyPoses->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
            cloudKeyPoses3DDSinit->push_back(pt);
        }
         cout<<"outdoor: "<<keyPosesOutdoorDS->size()<<" frames" <<endl;

        //starting from small indexes, add frames with overlap less than miu (with no consideration of scene changes!)
        float searchR = 30.0;
        //init chosen key poses
        cloudKeyPoses3DDS->push_back(cloudKeyPoses3DDSinit->points[0]);
        for(int i = 1; i < (int)cloudKeyPoses3DDSinit->size(); i++)
        {
            
            // find chosen key poses within searchR for the current key pose
            pcl::KdTreeFLANN<PointType>::Ptr tmpTree(new pcl::KdTreeFLANN<PointType>());
            vector<int> idxes;
            vector<float> distances;
            tmpTree->setInputCloud(cloudKeyPoses3DDS);
            tmpTree->radiusSearch(cloudKeyPoses3DDSinit->points[i],searchR,idxes, distances);

            if (idxes.empty())           
            {
                // cout<<"no nearby key poses, wierd"<<endl;
                cloudKeyPoses3DDS->push_back(cloudKeyPoses3DDSinit->points[i]);
                continue;
            }

            // build kdtree of the local map using found keyframes
            pcl::PointCloud<PointType>::Ptr  localMap(new pcl::PointCloud<PointType>());
            for (int j = 0; j < (int)idxes.size(); j++)
            {
                int thisKeyInd = (int)cloudKeyPoses3DDS->points[idxes[j]].intensity;
                *localMap += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
                *localMap += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
            }

            // pcl::io::savePCDFileBinary(saveMapDirectory + "/1.pcd",*localMap);
            // cout<<"nearby key pose size: "<<idxes.size()<<endl;
            pcl::KdTreeFLANN<PointType>::Ptr tmpTree2(new pcl::KdTreeFLANN<PointType>());
            vector<int> idxes2;
            vector<float> distances2;
            tmpTree2->setInputCloud(localMap);

            // get overlap ratio of the current keyframe in the local map
            int cnt = 0;
            int currentIdx = cloudKeyPoses3DDSinit->points[i].intensity;

            int sizeCorner = cornerCloudKeyFrames[currentIdx]->size();
            pcl::PointCloud<PointType>::Ptr  cornerCloud(new pcl::PointCloud<PointType>());
            cornerCloud = transformPointCloud(cornerCloudKeyFrames[currentIdx],  &cloudKeyPoses6D->points[currentIdx]);
            for (int k = 0; k < sizeCorner; k++)
            {
                tmpTree2->nearestKSearch(cornerCloud->points[k],1,idxes2, distances2);
                if (distances2[0] < resMap) cnt++;
            }

            int sizeSurf = surfCloudKeyFrames[currentIdx]->size();
            pcl::PointCloud<PointType>::Ptr  surfCloud(new pcl::PointCloud<PointType>());
            surfCloud = transformPointCloud(surfCloudKeyFrames[currentIdx],  &cloudKeyPoses6D->points[currentIdx]);
            for (int k = 0; k < sizeSurf; k++)
            {
                tmpTree2->nearestKSearch(surfCloud->points[k],1,idxes2, distances2);
                if (distances2[0] < resMap) cnt++;
            }

            float overlap = (float)(cnt)/(sizeCorner + sizeSurf);

            if (overlap < overlapThre) 
            {
                cloudKeyPoses3DDS->push_back(cloudKeyPoses3DDSinit->points[i]);
            }

            // cout<<"overlap: "<<overlap<<" finishing "<<i<<endl;
            if (i%100 == 0)
            cout<<"finished processed"<< i <<"keyframes"<<endl;

        }
        cout<<"after sparsification: "<<cloudKeyPoses3DDS->size()<<endl;
        cout<<"sparsification takes "<<sparsiTime.toc()/1000<<" sec "<<endl;
        
    }

    void visualizeGlobalMapThread()
    {
        ros::Rate rate(0.2);
        
        while (ros::ok())
        {
            publishCloud(&pubKeyPoses, cloudKeyPoses3D, timeLidarInfoStamp, mapFrame);           
            publishGlobalMap();
            rate.sleep();
        }

        if (savePCD == false && saveKeyframeMap == false && savePose == false)
            return;
        
        {
            roll::SaveTraj::Request req_traj;
            roll::SaveTraj::Response res_traj;

            // Use the current timestamp as the filename in YYYYMMDD_HHMMSS format.
            auto now = std::chrono::system_clock::now();
            auto tt = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&tt), "%Y%m%d_%H%M%S");
            cout << "save traj: " << ss.str() << endl;
            req_traj.filename = ss.str();
            if(!save_tum_traj_done_flag_){
                if(!saveTrajService(req_traj, res_traj))   cout << "Fail to save tum traj" << endl;
            }

        }


        roll::save_mapRequest  req;
        roll::save_mapResponse res;

        if (!doneSavingMap)
        {   ROS_INFO("save map before exit");
            if(!saveMapService(req, res))   cout << "Fail to save map" << endl;
        }
    }

    void publishGlobalMap()
    {
        if (pubLidarCloudSurround.getNumSubscribers() == 0 || cloudKeyPoses3D->empty() == true)
        {
            return;
        }
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key poses 
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        // fix the keyframe downsample bug
        for(auto& pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        // only for visualization
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
        {
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points: why it is not working
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS); 
        // cout<<globalMapKeyFramesDS->size()<<endl;       
        publishCloud(&pubLidarCloudSurround, globalMapKeyFramesDS, timeLidarInfoStamp, mapFrame);

    }

    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false )
            return;

        ros::Rate rate(loopClosureFrequency);
        while (ros::ok())
        {
            rate.sleep();
            if (!localizationMode) 
            {
                performLoopClosure();
                visualizeLoopClosure();
            }
            
        }
    }

    void queryMapThread()
    {
        ros::Rate rate(loopClosureFrequency);
        while(ros::ok()){
            if((tryReloc == true || relocSuccess == true)){
                TicToc extractNearbyBM1;
                extractNearbyBM();
                if(debugMode) cout<<"extractNearbyBM: "<<extractNearbyBM1.toc()<<endl;
            }
            rate.sleep();
        }
    }


    void performLoopClosure()
    {
        if (cloudKeyPoses3D->empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        //Close in space but far apart in time.
        if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
            return;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
        loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
        if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
            return;
        if (pubHistoryKeyFrames.getNumSubscribers() != 0)
            publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLidarInfoStamp, mapFrame);

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> registration;
        registration.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        registration.setMaximumIterations(100);
        registration.setTransformationEpsilon(1e-6);
        registration.setEuclideanFitnessEpsilon(1e-6);
        registration.setRANSACIterations(0);

        // // gicp is way better than icp
        // static pcl::GeneralizedIterativeClosestPoint<PointType,PointType> registration;

        // Align clouds
        registration.setInputSource(cureKeyframeCloud);
        registration.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        registration.align(*unused_result);

        fitnessScore = registration.getFitnessScore();

        if (registration.hasConverged() == false ||  fitnessScore > historyKeyframeFitnessScore)
            return;
        
        if(debugMode) cout<<"found loop, fitness score: "<< fitnessScore<<endl;
        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, registration.getFinalTransformation());
            publishCloud(&pubIcpKeyFrames, closed_cloud, timeLidarInfoStamp, mapFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = registration.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);

        Vector6 << fitnessScore, fitnessScore, fitnessScore, fitnessScore, fitnessScore, fitnessScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        noiseVec.push_back(fitnessScore);
        mtx.unlock();

        // add loop constriant
        // loopIndexContainer[loopKeyCur] = loopKeyPre;
        loopIndexContainer.insert(std::pair<int, int>(loopKeyCur, loopKeyPre)); // giseop for multimap
    }

    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        // reset before usage!
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        // cout<<copy_cloudKeyPoses6D->points[id].time-cloudKeyPoses6D->points[id].time<<" "<<copy_cloudKeyPoses6D->points[id].time-cloudInfoTime <<endl;
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            
            if (abs(copy_cloudKeyPoses6D->points[id].time - cloudInfoTime) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, bool doFiltering = true)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;
            // it may happen in temporary mapping mode
            float dist = pointDistance(copy_cloudKeyPoses3D->points[key],copy_cloudKeyPoses3D->points[keyNear]);
            if (  dist > surroundingKeyframeSearchRadius)
                continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);  
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        if (doFiltering){
            pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
            downSizeFilterICP.setInputCloud(nearKeyframes);
            downSizeFilterICP.filter(*cloud_temp);
            *nearKeyframes = *cloud_temp;
        }
    }
    // copied to the original, so that reloc part can use this func
    void loopFindNearKeyframesReloc(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, bool doFiltering = true)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;
                // copied to the original, so that reloc part can use this func too
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &cloudKeyPoses6D->points[keyNear]);  
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        if (doFiltering){
            pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
            downSizeFilterICP.setInputCloud(nearKeyframes);
            downSizeFilterICP.filter(*cloud_temp);
            *nearKeyframes = *cloud_temp;
        }
    }

    void visualizeLoopClosure()
    {
        if (loopIndexContainer.empty())
            return;
        
        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = mapFrame;
        markerNode.header.stamp = timeLidarInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = mapFrame;
        markerEdge.header.stamp = timeLidarInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
        // ROS_INFO_STREAM("Loop noise: "<<noiseVec[noiseVec.size()-1]);

    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, Eigen::Affine3f transCur)
    {
        int cloudSize = cloudIn->size();
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        cloudOut->resize(cloudSize);
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    void initialpose_callback(const rvizPoseType& pose_msg) {
        ROS_INFO("initial pose received!!");
        std::lock_guard<std::mutex> lock(pose_estimator_mutex);
        poseEstVec.push_back(pose_msg);
        if (relocByRviz()){
            ROS_INFO("Got pose estimate");
            poseGuessFromRvizAvailable = true;
        }
    }


    bool relocByRviz() {
        if (!poseEstVec.empty())
        {
            // use mutex for copy process to prevent concurrent change to the cloudKeyPoses3D,seems no need
            Eigen::Affine3f transBackup;
            mtx.lock(); 
            transBackup = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
            mtx.unlock();

            kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
            kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
            PointType pt;
            vector<int> pointSearchIndLoop;
            vector<float> pointSearchSqDisLoop;
            rvizPoseType pose_msg = poseEstVec.back();
            const auto& p = pose_msg->pose.pose.position;
            const auto& q = pose_msg->pose.pose.orientation;
            tf::Quaternion q_tf(q.x,q.y,q.z,q.w);
            double roll,pitch,yaw;
            tf::Matrix3x3(q_tf).getRPY(roll,pitch,yaw);
            Eigen::Affine3f affineGuess = pcl::getTransformation(p.x, p.y, p.z, (float)roll,(float)pitch,(float)yaw);
            relocCorrection = affineGuess*transBackup.inverse(); 
            poseEstVec.clear();
            return true;     
        }
        return false;
    }





    void updateInitialGuess()
    {
        static Eigen::Affine3f lastImuTransformation;
        static Eigen::Affine3f lastLidarTransformation;
        
        // only use rviz in loc mode
        if(localizationMode && poseGuessFromRvizAvailable)
        {
            Eigen::Affine3f tWrong = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f tCorrect = relocCorrection*tWrong;
            pcl::getTranslationAndEulerAngles(tCorrect, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
            for(int i = 0;i < 6;i++){
                rvizGuess[i] = transformTobeMapped[i];
                transformBeforeMapped[i] = transformTobeMapped[i];
            }
            printTrans("The reloc pose given by rviz: ",transformTobeMapped);
            poseGuessFromRvizAvailable = false;
            tryReloc = true;
            return;
        }

        // initialization 
        if (localizationMode && relocSuccess == false)
        {
            //  use ground truth for initialization in both modes!
            //  use ground truth all along for building a map
            int tryRuns = 3; // tryRuns = 0 means using initialGuess directly,sometimes gt is too delayed to be used!
            static int count = 0;
            if (!gtQueue.empty() && count < tryRuns)
            {
                odometryMsgToAffine3f(gtQueue.front(),affine_imu_to_map);
                gtQueue.pop_front();
                Affine3f2Trans(affine_imu_to_map,transformTobeMapped);
                affine_odom_to_map = affine_imu_to_map*affine_imu_to_odom.inverse();
                for(int i=0;i<6;i++)
                {
                    transformBeforeMapped[i] = transformTobeMapped[i];
                }
                printTrans("Initial: ",transformTobeMapped);
                                globalEstimator.setTgl(affine_odom_to_map.matrix().cast<double>());
                globalEstimator.inputGlobalLocPose(cloudInfoTime, affine_imu_to_map.matrix().cast<double>(), 0.5, 0.1);
                relocSuccess = true; // only apply to bag testing
                
            }
            else if (count < tryRuns)
            {
                count++;
                ROS_INFO("Initializing: %d",count);
            }
            else
            {
                // tired of waiting for gt, try initial guess given. 
                ROS_WARN("Waiting for correct initial guess, probably need rviz for reloc");
                for(int i=0;i<6;i++){
                    transformTobeMapped[i] = initialGuess[i];
                    transformBeforeMapped[i] = initialGuess[i];
                }
                Eigen::Affine3f affine_body_to_map = trans2Affine3f(transformTobeMapped);
                affine_imu_to_map = affine_body_to_map*affine_imu_to_body;
                affine_odom_to_map = affine_imu_to_map*affine_imu_to_odom.inverse();
                printTrans("Initial: ",transformTobeMapped); //no more waiting for rviz guess
                                globalEstimator.setTgl(affine_odom_to_map.matrix().cast<double>());     
                globalEstimator.inputGlobalLocPose(cloudInfoTime, affine_imu_to_map.matrix().cast<double>(), 0.5, 0.1);
                 relocSuccess = true;      
            }
            return;          
        }
        else
        {
            relocSuccess = true;
        }

        
        float arrayTmp[6];
        affine_imu_to_map = affine_odom_to_map*affine_imu_to_odom;
        Affine3f2Trans(affine_imu_to_map, arrayTmp);
        
        for (int i=0;i<6;i++)                
        {
            transformTobeMapped[i] = arrayTmp[i];
            transformBeforeMapped[i] = transformTobeMapped[i];
        }
        return;
    }

    void extractNearbyBM(){

        PointType pt;
        pt.x=transformTobeMapped[3];
        pt.y=transformTobeMapped[4];
        pt.z=transformTobeMapped[5];


        
        gtsam::Rot3 R_xyz = gtsam::Rot3::Ypr(transformTobeMapped[2], transformTobeMapped[1], transformTobeMapped[0]);
        gtsam::Point3 t(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
        gtsam::Pose3 lidar_pose(R_xyz, t);

        TicToc queryMapSwitch_time;
        queryMapSwitch(lidar_pose);
        if(debugMode) cout<<"queryMapSwitch_time: "<<queryMapSwitch_time.toc()<<endl;

        //todo Choose a method for scan-to-map matching.
        //Candidate methods: NDT, ICP, and LOAM.


    }

    void extratNearbyDynamicMap()
    {
        if(temporaryCloudKeyPoses3D->empty() == true){
            ROS_WARN(" temporaryCloudKeyPoses3D is empty() !!!");
            return ;
        }

        TicToc time_kd_tree_build;

        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        double surroundingKeyframeSearchRadius_dynamic = 20.0;

        kdtreeSurroundingKeyPoses_dynamic->setInputCloud(temporaryCloudKeyPoses3D);
        if(debugMode) cout<<"time_kd_tree_build: "<<time_kd_tree_build.toc()<<endl;

        PointType pt;
        pt.x=transformTobeMapped[3];
        pt.y=transformTobeMapped[4];
        pt.z=transformTobeMapped[5];

        TicToc time_kdtree_search;
        kdtreeSurroundingKeyPoses_dynamic->radiusSearch(pt,
                                                         surroundingKeyframeSearchRadius_dynamic, 
                                                         pointSearchInd, 
                                                         pointSearchSqDis);
        if(debugMode) cout<<"kdtree_search: "<<time_kdtree_search.toc()<<endl;

        if(pointSearchInd.empty()){
            ROS_WARN("pointSearchInd is empty !!!");
            return ;
        }
        
        for(int i = 0; i < (int)pointSearchInd.size(); ++i){
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(temporaryCloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

        TicToc time_kdtree_search_again;
        for(auto& pt : surroundingKeyPosesDS->points){

            kdtreeSurroundingKeyPoses_dynamic->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = temporaryCloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }
        if(debugMode) cout<<"time_kdtree_search_again: "<<time_kdtree_search_again.toc()<<endl;

        {
            ROS_INFO("temporaryCornerCloudKeyFrames size = %ld, temporarySurfCloudKeyFrames size = %ld, temporaryCloudKeyPoses6D size = %d",
                     temporaryCornerCloudKeyFrames.size(), temporarySurfCloudKeyFrames.size(), temporaryCloudKeyPoses6D->size());
            // fuse the map
            lidarCloudCornerFromDynamicMap->clear();
            lidarCloudSurfFromDynamicMap->clear(); 
            // TicToc transPC;
            for (int i = 0; i < (int)surroundingKeyPosesDS->size(); ++i)
            {
                int thisKeyInd = (int)surroundingKeyPosesDS->points[i].intensity;
                //todo Save the dynamic map.
                *lidarCloudCornerFromDynamicMap += *transformPointCloud(temporaryCornerCloudKeyFrames[thisKeyInd],  &temporaryCloudKeyPoses6D->points[thisKeyInd]);
                *lidarCloudSurfFromDynamicMap   += *transformPointCloud(temporarySurfCloudKeyFrames[thisKeyInd],    &temporaryCloudKeyPoses6D->points[thisKeyInd]);    
            }
            
            // Downsample the surrounding corner key frames (or map)
            downSizeFilterCorner.setInputCloud(lidarCloudCornerFromDynamicMap);
            downSizeFilterCorner.filter(*lidarCloudCornerFromDynamicMapDS);
            lidarCloudCornerFromDynamicMapDSNum = lidarCloudCornerFromDynamicMapDS->size();
            // Downsample the surrounding surf key frames (or map)
            downSizeFilterSurf.setInputCloud(lidarCloudSurfFromDynamicMap);
            downSizeFilterSurf.filter(*lidarCloudSurfFromDynamicMapDS);
            lidarCloudSurfFromDynamicMapDSNum = lidarCloudSurfFromDynamicMapDS->size();
        }

    }

    void extractNearby()
    {
        if (cloudKeyPoses3D->empty() == true) 
            return; 
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        PointType pt;
        pt.x=transformTobeMapped[3];
        pt.y=transformTobeMapped[4];
        pt.z=transformTobeMapped[5];
        
        kdtreeSurroundingKeyPoses->radiusSearch(pt, (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);

        if (pointSearchInd.empty()) 
        {
            // cout<<pt.x<< " "<<pt.y <<endl;
            // ROS_WARN("No nearby keyposes within %f meters",surroundingKeyframeSearchRadius);
            return;
        }
        //Find the nearest neighboring keyframes.
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        // downsampling is important especially at places where trajectories overlap when doing slam
        
        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

        for(auto& pt : surroundingKeyPosesDS->points) // recover the intensity field averaged by voxel filter
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        if (!localizationMode)
        {
            // also extract some latest key frames in case the robot rotates in one position
            // more recent ones matches better with current frames
            int numPoses = cloudKeyPoses3D->size();
            for (int i = numPoses-1; i >= 0; --i)
            {
                if (cloudInfoTime - cloudKeyPoses6D->points[i].time < 10.0)
                    surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
                else
                    break;
            }
        }
        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        lidarCloudCornerFromMap->clear();
        lidarCloudSurfFromMap->clear(); 
        // TicToc transPC;
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            int thisKeyInd = (int)cloudToExtract->points[i].intensity;

            *lidarCloudCornerFromMap += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            *lidarCloudSurfFromMap   += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);    
        }
        
        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(lidarCloudCornerFromMap);
        downSizeFilterCorner.filter(*lidarCloudCornerFromMapDS);
        lidarCloudCornerFromMapDSNum = lidarCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(lidarCloudSurfFromMap);
        downSizeFilterSurf.filter(*lidarCloudSurfFromMapDS);
        lidarCloudSurfFromMapDSNum = lidarCloudSurfFromMapDS->size();
        
    }


    void downsampleCurrentScan()
    {
        // if (cloudKeyPoses3D->empty() )
        //     return;
        // Downsample cloud from current scan
        lidarCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(lidarCloudCornerLast);
        downSizeFilterCorner.filter(*lidarCloudCornerLastDS);
        lidarCloudCornerLastDSNum = lidarCloudCornerLastDS->size();
        // cout<<"After sampling: "<<lidarCloudCornerLastDSNum<<endl;
        lidarCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(lidarCloudSurfLast);
        downSizeFilterSurf.filter(*lidarCloudSurfLastDS);
        lidarCloudSurfLastDSNum = lidarCloudSurfLastDS->size();
    }

    void scan2MapOptimization()
    {
        std::lock_guard<std::mutex> lock(mtx_map_);
        // no clouds nearby
        {

            
            //cloudKeyPoses3D->empty() || todo,cloudKeyPoses3D is not loaded yet.
            if (lidarCloudCornerFromMapDS->empty() || lidarCloudSurfFromMapDS->empty())
            {

                ROS_INFO("pose or map point is empty!, pose size = %ld, corner map size = %ld, surf map size = %ld", 
                        cloudKeyPoses3D->size(), lidarCloudCornerFromMapDS->size(), lidarCloudSurfFromMapDS->size());
                return;
            }

        }   
        // ROS_INFO("scan2MapOptimization: lidarCloudCornerFromMapDS size() = %d, lidarCloudSurfFromMapDS size = %d", 
        //             lidarCloudCornerFromMapDS->size(), lidarCloudSurfFromMapDS->size());            


        // cout<<"corner, surf points: "<<lidarCloudCornerLastDSNum<<" "<<lidarCloudSurfLastDSNum<<endl;
        if (lidarCloudCornerLastDSNum > edgeFeatureMinValidNum && lidarCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            // get a copy of those clouds might lower down speed
            //TODO：No or too few matches indicate an unmapped area; using a block map without point clouds can crash.
            //todo:The dynamic map is not used.

            if(0 == lidarCloudCornerFromMapDS->size() || 0 == lidarCloudSurfFromMapDS->size() 
            || 0 == lidarCloudCornerFromDynamicMapDS->size() || 0 == lidarCloudSurfFromDynamicMapDS->size()){

                ROS_ERROR("map size  == 0");
                ROS_INFO("error: current scan size = corner %ld, surf %ld, map size corner %ld, surf %ld, lidarCloudCornerFromDynamicMapDS = %ld, lidarCloudSurfFromDynamicMapDS = %ld", 
                        lidarCloudCornerLastDS->size(), lidarCloudSurfLastDS->size(), 
                        lidarCloudCornerFromMapDS->size(), lidarCloudSurfFromMapDS->size(),
                        lidarCloudCornerFromDynamicMapDS->size(), lidarCloudSurfFromDynamicMapDS->size());
            }

            LOAMmapping::ScanMatchInput scan_match_input;
            scan_match_input.lidarCloudCornerLastDSnew = lidarCloudCornerLastDS;
            scan_match_input.lidarCloudSurfLastDSnew   = lidarCloudSurfLastDS;
            scan_match_input.lidarCloudCornerFromMapDSnew = lidarCloudCornerFromMapDS;
            scan_match_input.lidarCloudSurfFromMapDSnew = lidarCloudSurfFromMapDS;
            scan_match_input.lidarCloudCornerFromDynamicMapDSnew = lidarCloudCornerFromDynamicMapDS;
            scan_match_input.lidarCloudSurfFromDynamicMapDSnew = lidarCloudSurfFromDynamicMapDS;
            LOAMmapping LM(scan_match_input, affine_imu_to_map);
            if(0 == lidarCloudCornerFromDynamicMapDS->size()|| 0 == lidarCloudSurfFromDynamicMapDS->size()){
                // LOAMmapping LM(lidarCloudCornerLastDS, lidarCloudSurfLastDS, lidarCloudCornerFromMapDS, lidarCloudSurfFromMapDS, affine_imu_to_map);

                ROS_INFO("scan match without dynamic map: current scan size = corner %ld, surf %ld, map size corner %ld, surf %ld, lidarCloudCornerFromDynamicMapDS = %ld, lidarCloudSurfFromDynamicMapDS = %ld", 
                        lidarCloudCornerLastDS->size(), lidarCloudSurfLastDS->size(), 
                        lidarCloudCornerFromMapDS->size(), lidarCloudSurfFromMapDS->size(),
                        lidarCloudCornerFromDynamicMapDS->size(), lidarCloudSurfFromDynamicMapDS->size());
                LM.match();

            }else{


                ROS_INFO("scan_match_input: lidarCloudCornerLastDS size = %ld, lidarCloudSurfLastDS size = %ld, lidarCloudCornerFromMapDS = %ld, lidarCloudSurfFromMapDS size = %ld, lidarCloudCornerFromDynamicMapDS = %ld, lidarCloudSurfFromDynamicMapDS = %ld", 
                        lidarCloudCornerLastDS->size(), lidarCloudSurfLastDS->size(), lidarCloudCornerFromMapDS->size(), lidarCloudSurfFromMapDS->size(),
                        lidarCloudCornerFromDynamicMapDS->size(), lidarCloudSurfFromDynamicMapDS->size());
                LM.matchwithStaticAndDynamicMap();

            }

            
            // // for relocalization in loc mode: only needed when used in actual world
            // if (LM.inlier_ratio > 0.4 && tryReloc == true)
            // {
            //     ROS_INFO_STREAM("At time "<< cloudInfoTime - rosTimeStart <<" sec, relocalization succeeds!");
            //     relocSuccess = true;
            //     tryReloc = false;
            //     transformUpdate(); // for giving fusion pose immediate map_odom
            // }

            const bool accept_global_match = shouldAddGlobalMatchConstraint(LM, "long/static");

            if (relocSuccess == true && localizationMode == true)
            {
                // for entering TMM: 
                //inlier_ratio2 is more sensitive to map extension, and will suffice
                if ( LM.inlier_ratio2 < startTemporaryMappingInlierRatioThre && temporaryMappingMode == false)
                {
                    ROS_INFO_STREAM("At time "<< cloudInfoTime - rosTimeStart <<" sec, Entering temporary mapping mode due to poor mapping performace");
                    ROS_INFO_STREAM("Inlier ratio2: "<< LM.inlier_ratio2);                        
                    temporaryMappingMode = true; // here is the case for outdated map
                    startTemporaryMappingIndex = temporaryCloudKeyPoses3D->size();
                    frameTobeAbandoned = true;
                    TMMcount++;                   
                }
                
                // more strict to exit TMM for map updating
                if (LM.inlier_ratio2 > exitTemporaryMappingInlierRatioThre && int(temporaryCloudKeyPoses3D->size()) > slidingWindowSize + 10 && temporaryMappingMode == true)
                {
                    correctedPose = LM.affine_out;// notice: the correction cannot be simply the correction for last keyframe!
                    affine_imu_to_map = LM.affine_out;
                    // LM.getTransformation(transformTobeMapped); // don't change it here, need original one as odom factor
                    goodToMergeMap = true;
                    cout<<"Now it is okay to merge the temporary map"<<endl;
                }
            }
            // only correct pose from fastlio when mapping quality is good
            // if (temporaryMappingMode == false && LM.isDegenerate == false)  // worse
            // when building a map, it is better to fuse LIO and global matching
            if ((temporaryMappingMode == false || localizationMode == false) && accept_global_match)
            {
                // fusion with gtsam 
                // // TicToc opt_gtsam;
                // Eigen::Affine3f affine_imu_to_map_smooth = gtsamOptimize(affine_imu_to_map,LM.affine_out, 0.5*(1-LM.inlier_ratio));
                // // cout<<"gtsam opt takes:"<<opt_gtsam.toc()<<" ms"<<endl;
                

                // // primitive fusion
                // affine_imu_to_map = LM.affine_out;
                            
                // fusion with ceres: currently only for smoothing, not for pose guess.
                // cannot update Tgl immediately because opt takes time, getting a delayed Tgl is rather forfeiting it

                if (goodToMergeMap) // reset for the frame of merging
                    globalEstimator.resetOptimization(LM.affine_out.matrix().cast<double>());
                else
                    globalEstimator.inputGlobalLocPose(cloudInfoTime, LM.affine_out.matrix().cast<double>(), 0.5, 0.1);

                // === DEBUG: check global match quality vs current Tgl-based pose ===
                {
                    Eigen::Vector3f matchPos = LM.affine_out.translation();
                    Eigen::Affine3f tglPose = affine_odom_to_map * affine_imu_to_odom;
                    Eigen::Vector3f tglPos = tglPose.translation();
                    float posDiff = (matchPos - tglPos).norm();
                    ROS_INFO("[MatchQuality] inlier=%.3f, inlier2=%.3f, regiErr=%.4f, matchPos=(%.2f,%.2f,%.2f), tglPos=(%.2f,%.2f,%.2f), diff=%.3f m",
                             LM.inlier_ratio, LM.inlier_ratio2, LM.regiError,
                             matchPos.x(), matchPos.y(), matchPos.z(),
                             tglPos.x(), tglPos.y(), tglPos.z(), posDiff);
                    if (posDiff > 2.0) {
                        ROS_ERROR("[MatchQuality] LARGE diff=%.3f m between global match and Tgl-based pose! Possible bad match. inlier=%.3f, inlier2=%.3f, regiErr=%.4f",
                                  posDiff, LM.inlier_ratio, LM.inlier_ratio2, LM.regiError);
                    }
                }

                affine_imu_to_map = LM.affine_out;
                Affine3f2Trans(affine_imu_to_map,transformTobeMapped);
                                // printTrans("trans: ",transformTobeMapped);
            }

            if(saveLog)
            {
                // for recording mapping logs
                double mappingTime = mappingTimeVec.empty()?0:mappingTimeVec.back();
                mtx.lock(); // need lock for fitnessScore
                pose_log_file<<setw(20)<<cloudInfoTime<<" "<<transformTobeMapped[0]<<" "<<transformTobeMapped[1]<<" "<<transformTobeMapped[2]<<" "
                    <<transformTobeMapped[3]<<" "<<transformTobeMapped[4]<<" "<<transformTobeMapped[5]<<" "<< LM.inlier_ratio<<" "
                    <<LM.inlier_ratio2<<" " <<LM.regiError<<" "<<temporaryMappingMode<<" "<<mappingTime<<" "<<fitnessScore<<"\n";
                mtx.unlock();
            }

            ROS_INFO("match result: timestamp: %f, angle(%f, %f, %f), pose(%f, %f, %f)",
                        cloudInfoTime, transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2],
                        transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);

            // ROS_INFO_STREAM("error: "<<regiError <<" inlier ratio: "<<  inlier_ratio);
              
        } 
        else 
        {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", lidarCloudCornerLastDSNum, lidarCloudSurfLastDSNum);
        }
    }


   void scan2MapOptimizationStaticMap()
    {
        std::lock_guard<std::mutex> lock(mtx_map_);
        // no clouds nearby
        if (lidarCloudCornerFromMapDS->empty() || lidarCloudSurfFromMapDS->empty())
        {

            ROS_INFO("scan2MapOptimizationStaticMap:pose or map point is empty!, pose size = %ld, corner map size = %ld, surf map size = %ld", 
                    cloudKeyPoses3D->size(), lidarCloudCornerFromMapDS->size(), lidarCloudSurfFromMapDS->size());
            return;
        }
        
        // cout<<"corner, surf points: "<<lidarCloudCornerLastDSNum<<" "<<lidarCloudSurfLastDSNum<<endl;
        if (lidarCloudCornerLastDSNum > edgeFeatureMinValidNum && lidarCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            // get a copy of those clouds might lower down speed
            LOAMmapping LM(lidarCloudCornerLastDS, lidarCloudSurfLastDS, lidarCloudCornerFromMapDS, lidarCloudSurfFromMapDS, affine_imu_to_map);
            LM.match();
            
            // // for relocalization in loc mode: only needed when used in actual world
            // if (LM.inlier_ratio > 0.4 && tryReloc == true)
            // {
            //     ROS_INFO_STREAM("At time "<< cloudInfoTime - rosTimeStart <<" sec, relocalization succeeds!");
            //     relocSuccess = true;
            //     tryReloc = false;
            //     transformUpdate(); // for giving fusion pose immediate map_odom
            // }

            const bool accept_global_match = shouldAddGlobalMatchConstraint(LM, "static");

            if (relocSuccess == true && localizationMode == true)
            {
                // for entering TMM: 
                //inlier_ratio2 is more sensitive to map extension, and will suffice
                if ( LM.inlier_ratio2 < startTemporaryMappingInlierRatioThre && temporaryMappingMode == false)
                {
                    ROS_INFO_STREAM("At time "<< cloudInfoTime - rosTimeStart <<" sec, Entering temporary mapping mode due to poor mapping performace");
                    ROS_INFO_STREAM("Inlier ratio2: "<< LM.inlier_ratio2);                        
                    temporaryMappingMode = true; // here is the case for outdated map
                    startTemporaryMappingIndex = temporaryCloudKeyPoses3D->size();
                    frameTobeAbandoned = true;
                    TMMcount++;                   
                }
                
                // more strict to exit TMM for map updating
                if (LM.inlier_ratio2 > exitTemporaryMappingInlierRatioThre && int(temporaryCloudKeyPoses3D->size()) > slidingWindowSize + 10 && temporaryMappingMode == true)
                {
                    correctedPose = LM.affine_out;// notice: the correction cannot be simply the correction for last keyframe!
                    affine_imu_to_map = LM.affine_out;
                    // LM.getTransformation(transformTobeMapped); // don't change it here, need original one as odom factor
                    goodToMergeMap = true;
                    cout<<"Now it is okay to merge the temporary map"<<endl;
                }
            }
            // only correct pose from fastlio when mapping quality is good
            // if (temporaryMappingMode == false && LM.isDegenerate == false)  // worse
            // when building a map, it is better to fuse LIO and global matching
            if ((temporaryMappingMode == false || localizationMode == false) && accept_global_match)
            {
                // fusion with gtsam 
                // // TicToc opt_gtsam;
                // Eigen::Affine3f affine_imu_to_map_smooth = gtsamOptimize(affine_imu_to_map,LM.affine_out, 0.5*(1-LM.inlier_ratio));
                // // cout<<"gtsam opt takes:"<<opt_gtsam.toc()<<" ms"<<endl;
                

                // // primitive fusion
                // affine_imu_to_map = LM.affine_out;
                            
                // fusion with ceres: currently only for smoothing, not for pose guess.
                // cannot update Tgl immediately because opt takes time, getting a delayed Tgl is rather forfeiting it

                if (goodToMergeMap) // reset for the frame of merging
                    globalEstimator.resetOptimization(LM.affine_out.matrix().cast<double>());
                else 
                    globalEstimator.inputGlobalLocPose(cloudInfoTime, LM.affine_out.matrix().cast<double>(), 0.5, 0.1);
                                
                affine_imu_to_map = LM.affine_out;
                Affine3f2Trans(affine_imu_to_map,transformTobeMapped);
                                ROS_INFO("static pose: angle:(%f, %f, %f), pose:(%f, %f, %f)", 
                transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2], 
                transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
                // printTrans("trans: ",transformTobeMapped);
            }
     
            if(saveLog)
            {
                // for recording mapping logs
                double mappingTime = mappingTimeVec.empty()?0:mappingTimeVec.back();
                mtx.lock(); // need lock for fitnessScore
                pose_log_file<<setw(20)<<cloudInfoTime<<" "<<transformTobeMapped[0]<<" "<<transformTobeMapped[1]<<" "<<transformTobeMapped[2]<<" "
                    <<transformTobeMapped[3]<<" "<<transformTobeMapped[4]<<" "<<transformTobeMapped[5]<<" "<< LM.inlier_ratio<<" "
                    <<LM.inlier_ratio2<<" " <<LM.regiError<<" "<<temporaryMappingMode<<" "<<mappingTime<<" "<<fitnessScore<<"\n";
                mtx.unlock();
            }

            // ROS_INFO_STREAM("error: "<<regiError <<" inlier ratio: "<<  inlier_ratio);
              
        } 
        else 
        {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", lidarCloudCornerLastDSNum, lidarCloudSurfLastDSNum);
        }
    }

    void constrainGlobalObservationPose(Eigen::Affine3f& affine_pose)
    {
        float pose[6];
        Affine3f2Trans(affine_pose, pose);

        if (!hasLastGlobalPose) {
            for (int i = 0; i < 6; ++i)
                transformTobeMapped_last[i] = pose[i];
            hasLastGlobalPose = true;
            return;
        }

        float dz = pose[5] - transformTobeMapped_last[5];
        float droll = pose[0] - transformTobeMapped_last[0];
        float dpitch = pose[1] - transformTobeMapped_last[1];

        if (dz < -0.02) pose[5] = transformTobeMapped_last[5] - 0.02;
        else if (dz > 0.02) pose[5] = transformTobeMapped_last[5] + 0.02;

        if (droll < -0.02) pose[0] = transformTobeMapped_last[0] - 0.02;
        else if (droll > 0.02) pose[0] = transformTobeMapped_last[0] + 0.02;

        if (dpitch < -0.02) pose[1] = transformTobeMapped_last[1] - 0.02;
        else if (dpitch > 0.02) pose[1] = transformTobeMapped_last[1] + 0.02;

        for (int i = 0; i < 6; ++i)
            transformTobeMapped_last[i] = pose[i];

        affine_pose = pcl::getTransformation(pose[3], pose[4], pose[5], pose[0], pose[1], pose[2]);
    }

    void calculateAdaptiveWeight(float inlier_ratio,
                                 float& pos_sigma, float& rot_sigma)
    {
        inlier_ratio_history.push_back(inlier_ratio);
        if (inlier_ratio_history.size() > history_window_size)
            inlier_ratio_history.pop_front();

        float avg_inlier = 0.0f;
        for (const auto& ratio : inlier_ratio_history)
            avg_inlier += ratio;
        avg_inlier /= inlier_ratio_history.size();

        const float stability = 1.0f - std::fabs(inlier_ratio - last_inlier_ratio);
        last_inlier_ratio = inlier_ratio;

        float confidence = globalConstraintInlierWeight * inlier_ratio + 0.3f * avg_inlier + globalConstraintStabilityWeight * stability;
        confidence = std::max(0.0f, std::min(1.0f, confidence));

        // Residuals are divided by sigma: higher confidence must yield
        // smaller noise and therefore larger information weight (1/sigma^2).
        const float uncertainty = 1.0f - confidence;
        pos_sigma = 0.1f + 0.7f * uncertainty;
        rot_sigma = 0.02f + 0.18f * uncertainty;
    }

    bool shouldAddGlobalMatchConstraint(const LOAMmapping& matcher, const char* map_label) const
    {
        if (!std::isfinite(matcher.inlier_ratio2) ||
            matcher.inlier_ratio2 < globalMatchMinInlierRatio2)
        {
            ROS_WARN_THROTTLE(1.0,
                "[GlobalMatchGate] REJECT %s constraint: inlier2=%.3f, threshold=%.3f, inlier=%.3f, regiErr=%.4f; pose not applied",
                map_label, matcher.inlier_ratio2, globalMatchMinInlierRatio2,
                matcher.inlier_ratio, matcher.regiError);
            return false;
        }
        return true;
    }

       void scan2MapOptimizationDynamicMap(pcl::PointCloud<PointType>::Ptr lidarCloudCornerLastDSnew,
                                            pcl::PointCloud<PointType>::Ptr lidarCloudSurfLastDSnew,
                                            pcl::PointCloud<PointType>::Ptr lidarCloudCornerFromMapDSnew,
                                            pcl::PointCloud<PointType>::Ptr lidarCloudSurfFromMapDSnew,
                                            const Eigen::Affine3f affine_guess_ne, bool use_mid_map=false,
                                            pcl::PointCloud<PointType>::Ptr lidarCloudCornerFromDynamicMapDSnew = nullptr,
                                            pcl::PointCloud<PointType>::Ptr lidarCloudSurfFromDynamicMapDSnew = nullptr)
    {
        // no clouds nearby
        std::lock_guard<std::mutex> lock(mtx_map_);
        if (lidarCloudCornerFromMapDSnew->empty() || lidarCloudSurfFromMapDSnew->empty())
        {

            ROS_INFO("scan2MapOptimizationDynamicMap: pose or map point is empty!, pose size = %ld, corner map size = %ld, surf map size = %ld", 
                    cloudKeyPoses3D->size(), lidarCloudCornerFromMapDSnew->size(), lidarCloudSurfFromMapDSnew->size());
            return;
        }
        
        // cout<<"corner, surf points: "<<lidarCloudCornerLastDSNum<<" "<<lidarCloudSurfLastDSNum<<endl;
        if (lidarCloudCornerLastDSnew->size() > edgeFeatureMinValidNum && lidarCloudSurfLastDSnew->size() > surfFeatureMinValidNum)
        {
            LOAMmapping::ScanMatchInput scan_match_input;
            scan_match_input.lidarCloudCornerLastDSnew = lidarCloudCornerLastDSnew;
            scan_match_input.lidarCloudSurfLastDSnew = lidarCloudSurfLastDSnew;
            scan_match_input.lidarCloudCornerFromMapDSnew = lidarCloudCornerFromMapDSnew;
            scan_match_input.lidarCloudSurfFromMapDSnew = lidarCloudSurfFromMapDSnew;
            scan_match_input.lidarCloudCornerFromDynamicMapDSnew = lidarCloudCornerFromDynamicMapDSnew;
            scan_match_input.lidarCloudSurfFromDynamicMapDSnew = lidarCloudSurfFromDynamicMapDSnew;

            LOAMmapping LM(scan_match_input, affine_guess_ne);

            if (lidarCloudCornerFromDynamicMapDSnew == nullptr || lidarCloudSurfFromDynamicMapDSnew == nullptr ||
                lidarCloudCornerFromDynamicMapDSnew->empty() || lidarCloudSurfFromDynamicMapDSnew->empty()) {
                LM.match();
            } else {
                LM.matchwithStaticAndDynamicMap();
            }

            const char* map_label = use_mid_map ? "mid" : "long";
            float pos_sigma = globalConstraintFixedPosSigma;
            float rot_sigma = globalConstraintFixedRotSigma;
            if (globalConstraintWeightMode == "adaptive") {
                calculateAdaptiveWeight(LM.inlier_ratio2, pos_sigma, rot_sigma);
            }
            const bool accept_global_match = shouldAddGlobalMatchConstraint(LM, map_label);
            
            // // for relocalization in loc mode: only needed when used in actual world
            // if (LM.inlier_ratio > 0.4 && tryReloc == true)
            // {
            //     ROS_INFO_STREAM("At time "<< cloudInfoTime - rosTimeStart <<" sec, relocalization succeeds!");
            //     relocSuccess = true;
            //     tryReloc = false;
            //     transformUpdate(); // for giving fusion pose immediate map_odom
            // }

            // if (relocSuccess == true && localizationMode == true)
            // {
            //     // for entering TMM:  
            //     //inlier_ratio2 is more sensitive to map extension, and will suffice
            //     if ( LM.inlier_ratio2 < startTemporaryMappingInlierRatioThre && temporaryMappingMode == false)
            //     {
            //         ROS_INFO_STREAM("At time "<< cloudInfoTime - rosTimeStart <<" sec, Entering temporary mapping mode due to poor mapping performace");
            //         ROS_INFO_STREAM("Inlier ratio2: "<< LM.inlier_ratio2);                        
            //         temporaryMappingMode = true; // here is the case for outdated map
            //         startTemporaryMappingIndex = temporaryCloudKeyPoses3D->size();
            //         frameTobeAbandoned = true;
            //         TMMcount++;                   
            //     }
                
            //     // more strict to exit TMM for map updating
            //     if (LM.inlier_ratio2 > exitTemporaryMappingInlierRatioThre && int(temporaryCloudKeyPoses3D->size()) > slidingWindowSize + 10 && temporaryMappingMode == true)
            //     {
            //         correctedPose = LM.affine_out;// notice: the correction cannot be simply the correction for last keyframe!
            //         affine_imu_to_map = LM.affine_out;
            //         // LM.getTransformation(transformTobeMapped); // don't change it here, need original one as odom factor
            //         goodToMergeMap = true;
            //         cout<<"Now it is okay to merge the temporary map"<<endl;
            //     }
            // }
            // only correct pose from fastlio when mapping quality is good
            // if (temporaryMappingMode == false && LM.isDegenerate == false)  // worse
            // when building a map, it is better to fuse LIO and global matching
            float transformTobeMappedDynamic[6];
            if(temporaryMappingMode){
                ROS_WARN("currnet is temporary mode!!!");
            }
            if ((temporaryMappingMode == false || localizationMode == false) && accept_global_match)
            {
                // fusion with gtsam 
                // // TicToc opt_gtsam;
                // Eigen::Affine3f affine_imu_to_map_smooth = gtsamOptimize(affine_imu_to_map,LM.affine_out, 0.5*(1-LM.inlier_ratio));
                // // cout<<"gtsam opt takes:"<<opt_gtsam.toc()<<" ms"<<endl;
                

                // // primitive fusion
                // affine_imu_to_map = LM.affine_out;
                            
                // fusion with ceres: currently only for smoothing, not for pose guess.
                // cannot update Tgl immediately because opt takes time, getting a delayed Tgl is rather forfeiting it
                //Eigen::Matrix4d affine
                if (goodToMergeMap) // reset for the frame of merging
                globalEstimator.resetOptimization(LM.affine_out.matrix().cast<double>());
                else{
                    Eigen::Affine3f constrained_pose = LM.affine_out;
                    ROS_DEBUG("[GlobalConstraintWeight] map=%s mode=%s t=%.6f inlier2=%.6f pos_sigma=%.6f rot_sigma=%.6f",
                              map_label, globalConstraintWeightMode.c_str(), cloudInfoTime,
                              LM.inlier_ratio2, pos_sigma, rot_sigma);
                    if(!use_mid_map){
                        globalEstimator.inputGlobalLocPose(cloudInfoTime, constrained_pose.matrix().cast<double>(), pos_sigma, rot_sigma);
                    }else{
                        globalEstimator.inputMidGlobalLocPose(cloudInfoTime, constrained_pose.matrix().cast<double>(), pos_sigma, rot_sigma);
                    }
                        
                }

                // === DEBUG: check dynamic global match quality vs current Tgl-based pose ===
                {
                    Eigen::Vector3f matchPos = LM.affine_out.translation();
                    Eigen::Affine3f tglPose = affine_odom_to_map * affine_imu_to_odom;
                    Eigen::Vector3f tglPos = tglPose.translation();
                    float posDiff = (matchPos - tglPos).norm();
                    ROS_INFO("[DynMatchQuality] inlier=%.3f, inlier2=%.3f, regiErr=%.4f, matchPos=(%.2f,%.2f,%.2f), tglPos=(%.2f,%.2f,%.2f), diff=%.3f m",
                             LM.inlier_ratio, LM.inlier_ratio2, LM.regiError,
                             matchPos.x(), matchPos.y(), matchPos.z(),
                             tglPos.x(), tglPos.y(), tglPos.z(), posDiff);
                    if (posDiff > 2.0) {
                        ROS_ERROR("[DynMatchQuality] LARGE diff=%.3f m! Possible bad match. inlier=%.3f, inlier2=%.3f, regiErr=%.4f",
                                  posDiff, LM.inlier_ratio, LM.inlier_ratio2, LM.regiError);
                    }
                }

                affine_imu_to_map = LM.affine_out;
                Affine3f2Trans(affine_imu_to_map,transformTobeMapped);
                ROS_INFO("dynamic pose: angle:(%f, %f, %f), pose:(%f, %f, %f)",
                                transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2],
                                transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
                // printTrans("trans: ",transformTobeMapped);
            }
     
            if(saveLog)
            {
                // for recording mapping logs
                double mappingTime = mappingTimeVec.empty()?0:mappingTimeVec.back();
                mtx.lock(); // need lock for fitnessScore
                pose_log_file<<"dynamic"<<setw(20)<<cloudInfoTime<<" "<<transformTobeMappedDynamic[0]<<" "<<transformTobeMappedDynamic[1]<<" "<<transformTobeMappedDynamic[2]<<" "
                    <<transformTobeMappedDynamic[3]<<" "<<transformTobeMappedDynamic[4]<<" "<<transformTobeMappedDynamic[5]<<" "<< LM.inlier_ratio<<" "
                    <<LM.inlier_ratio2<<" " <<LM.regiError<<" "<<temporaryMappingMode<<" "<<mappingTime<<" "<<fitnessScore<<"\n";
                mtx.unlock();
            }

            // ROS_INFO_STREAM("error: "<<regiError <<" inlier ratio: "<<  inlier_ratio);
              
        } 
        else 
        {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", lidarCloudCornerLastDSNum, lidarCloudSurfLastDSNum);
        }
    }
    void resetISAM(){
        ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        isam = new ISAM2(optParameters); // why it is okay for liosam not to add "new"?

        gtsam::NonlinearFactorGraph newGraphFactors;
        gtSAMgraph = newGraphFactors;
    }
    Eigen::Affine3f gtsamOptimize(Eigen::Affine3f odometryPose, Eigen::Affine3f globalMatchingPose, float globalError)
    {
        static int idx = 0;
        static bool inited = false;

        if (idx == 100){
            // neglect inheriting covariance for now
            inited = false;
            resetISAM();
            idx = 0;
        }
        
        if (inited == false)
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, 1e-2, 1e-1, 1e-1, 1e-1).finished()); // rad*rad, meter*meter
            gtsam::Pose3 posePrior = Affine3f2gtsamPose(odometryPose);
            gtSAMgraph.add(PriorFactor<Pose3>(idx, posePrior, priorNoise));
            initialEstimate.insert(idx, posePrior);           
            inited = true;

        }
        else
        {
            gtsam::Pose3 poseFrom = Affine3f2gtsamPose(lastOdometryPose);
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) <<1e-2, 1e-2, 1e-2, 0.1, 0.1, 0.1 ).finished());
            gtsam::Pose3 poseTo = Affine3f2gtsamPose(odometryPose);
            gtSAMgraph.add(BetweenFactor<Pose3>(idx-1,idx, poseFrom.between(poseTo), odometryNoise));

            noiseModel::Diagonal::shared_ptr corrNoise = noiseModel::Diagonal::Variances((Vector(6) << 0.1,  0.1, 0.1,
            globalError, globalError, globalError).finished()); // rad*rad, meter*meter
            gtsam::Pose3 posePrior = Affine3f2gtsamPose(globalMatchingPose);
            gtSAMgraph.add(PriorFactor<Pose3>(idx, posePrior, corrNoise));
            initialEstimate.insert(idx, posePrior);
        }

        lastOdometryPose = odometryPose;
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        isamCurrentEstimate = isam->calculateEstimate();
        
        auto latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);

        tf::Quaternion q = tf::createQuaternionFromRPY(latestEstimate.rotation().roll(),latestEstimate.rotation().pitch(),latestEstimate.rotation().yaw());
        geometry_msgs::PoseStamped odometry;
        odometry.header.stamp = timeLidarInfoStamp;
        odometry.header.frame_id = mapFrame;
        odometry.pose.position.x = latestEstimate.translation().x();
        odometry.pose.position.y = latestEstimate.translation().y();
        odometry.pose.position.z = latestEstimate.translation().z();
        odometry.pose.orientation.x = q.x();
        odometry.pose.orientation.y = q.y();
        odometry.pose.orientation.z = q.z();
        odometry.pose.orientation.w = q.w();
        global_path_gtsam.poses.push_back(odometry);
        global_path_gtsam.header.stamp = odometry.header.stamp;
        global_path_gtsam.header.frame_id = mapFrame;
        pub_global_path_gtsam.publish(global_path_gtsam);

        idx++;

        Eigen::Affine3f after_smooth = pcl::getTransformation(latestEstimate.translation().x(),latestEstimate.translation().y(),latestEstimate.translation().z(),
            latestEstimate.rotation().roll(),latestEstimate.rotation().pitch(),latestEstimate.rotation().yaw()
        );
        return after_smooth;
    }

    int saveFrame()
    {
        // indoor 1; outdoor 0; not a keyframe -1
        if (localizationMode == true && relocSuccess == false ) return -1;
        // the frame for merging should be keyframe
        if (cloudKeyPoses3D->empty() || goodToMergeMap == true)    return isIndoorJudgement();
        // allow overlapped area to display loop closures
        Eigen::Affine3f transStart;
        if (localizationMode)
        {
            if (temporaryCloudKeyPoses6D->empty() == true ) return isIndoorJudgement(); 
            transStart = pclPointToAffine3f(temporaryCloudKeyPoses6D->back());  
        }            
        else
            transStart = pclPointToAffine3f(cloudKeyPoses6D->back());

        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        float dist = sqrt(x*x + y*y + z*z);
        if (dist > surroundingkeyframeAddingDistThreshold) // for paranomic lidar angle threshold is not needed
            return isIndoorJudgement();
        // cout<<"dist: "<<dist<<endl;
        return -1;
    }

    int isIndoorJudgement()
    {
        // indoor means almost all points are confined
        int count = 0;
        int sizeP = lidarCloudSurfLastDS->points.size();
        for(int i = 0 ;i< sizeP; i++) 
        {
            PointType p = lidarCloudSurfLastDS->points[i];
            if (pointDistance(p) < 5.0) count++;
        }

        float ratio = count/(float)sizeP;
        int tmp = ratio> 0.3?1:0;
        // cout<<"ratio: " <<ratio<<" count: "<<count<<endl;
        return tmp;
    }



    void addOdomFactor()
    {
        if (cloudKeyPoses3D->empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }
        else
        {
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }


    void addGPSFactor()
    {
        if (gpsQueue.empty() || cloudKeyPoses3D->empty() || useGPS == false)
            return;
        else
        {
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
                return;
        }

        if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
            return;

        // last gps position
        static PointType lastGPSPoint;

        while (!gpsQueue.empty())
        {
            if (gpsQueue.front().header.stamp.toSec() < cloudInfoTime - 0.1)
            {
                // cout<<"message too old"<<endl;
                gpsQueue.pop_front();
            }
            else if (gpsQueue.front().header.stamp.toSec() > cloudInfoTime + 0.1)
            {
                // cout<<" message too new" << endl;
                break;
            }
            else
            {
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();
                // ground truth
                float noise_x = 1.0,noise_y = 1.0, noise_z = 1.0;
                if (thisGPS.pose.covariance[0] != 0)
                {
                    noise_x = thisGPS.pose.covariance[0];
                    noise_y = thisGPS.pose.covariance[7];
                    noise_z = thisGPS.pose.covariance[14];
                }
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z ; // for body-lidar coor. trans.
                if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f); 
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);
                aLoopIsClosed = true;
                ROS_INFO("add gps factor...");
                break;
            }
        }
    }

    void addLoopFactor()
    {
        if (loopIndexQueue.empty())
            return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            // gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            auto noiseBetween = loopNoiseQueue[i];
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    void saveTemporaryKeyframes()
    {
        //save temporary key poses
        int isIndoorJudgement = saveFrame();
        if ( isIndoorJudgement < 0 || frameTobeAbandoned) return;

        isIndoorKeyframeTMM.push_back(isIndoorJudgement);   
        int temporaryKeyPoseSize = temporaryCloudKeyPoses3D->size();
        PointType thisPose3D;
        PointTypePose thisPose6D;
        thisPose3D.x = transformTobeMapped[3];
        thisPose3D.y = transformTobeMapped[4];
        thisPose3D.z = transformTobeMapped[5];
        thisPose3D.intensity = temporaryKeyPoseSize; // this can be used as keyframe index

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = transformTobeMapped[0];
        thisPose6D.pitch = transformTobeMapped[1];
        thisPose6D.yaw   = transformTobeMapped[2];
        thisPose6D.time = cloudInfoTime;

        temporaryCloudKeyPoses3D->push_back(thisPose3D);
        temporaryCloudKeyPoses6D->push_back(thisPose6D);
        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*lidarCloudCornerLast,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*lidarCloudSurfLast,    *thisSurfKeyFrame);
        // save key frame cloud
        temporaryCornerCloudKeyFrames.push_back(thisCornerKeyFrame); // Store all keyframes globally, but search only within 50 m for local matching.
        temporarySurfCloudKeyFrames.push_back(thisSurfKeyFrame);

        ROS_INFO("[saveTemporaryKeyframes]: temporaryCloudKeyPoses3D size = %ld, temporaryCornerCloudKeyFrames = %ld", temporaryCloudKeyPoses3D->size(), temporaryCornerCloudKeyFrames.size());
        if (!temporaryMappingMode && temporaryKeyPoseSize > slidingWindowSize) // sliding-window local map
        {
            temporaryCloudKeyPoses3D->erase(temporaryCloudKeyPoses3D->begin());
            temporaryCloudKeyPoses6D->erase(temporaryCloudKeyPoses6D->begin());
            temporaryCornerCloudKeyFrames.erase(temporaryCornerCloudKeyFrames.begin());
            temporarySurfCloudKeyFrames.erase(temporarySurfCloudKeyFrames.begin());
            isIndoorKeyframeTMM.erase(isIndoorKeyframeTMM.begin());
            // reindexing: key poses and key frames are corresponding with respect to the adding sequence
            for (int i = 0 ; i< (int)temporaryCloudKeyPoses3D->size(); i++)
            {
                temporaryCloudKeyPoses3D->points[i].intensity = i;
                temporaryCloudKeyPoses6D->points[i].intensity = i;
            }
        }

    }

    void saveKeyFramesAndFactor()
    {
        ROS_INFO("[saveKeyFramesAndFactor]: saving key frames and factors");
        int indoorJudgement = saveFrame();
        if ( indoorJudgement < 0)
            return;
        
        isIndoorKeyframe.push_back(indoorJudgement);
        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // loop factor
        addLoopFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
         ROS_INFO("""iSAM2 update done, current estimate size: %d", isamCurrentEstimate.size());
        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        Pose3 latestEstimate;
        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("gtsam current estimate: ");

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as keyframe index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = cloudInfoTime;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);
        // cout<<"x cov: "<< poseCovariance(3,3)<< " y cov: "<<poseCovariance(4,4)<<endl;

        // cout<<"Before opt: "<<transformTobeMapped[3]<<endl;;
        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        affine_imu_to_map = trans2Affine3f(transformTobeMapped);
        // reset fusion
        // globalEstimator.resetOptimization(affine_imu_to_map.matrix().cast<double>());

        // cout<<"After opt: "<< transformTobeMapped[3]<<endl;
        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());       
        // if pushing the original, it changes in the vec along with lidarCloudCornerLast
        pcl::copyPointCloud(*lidarCloudCornerLast,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*lidarCloudSurfLast,    *thisSurfKeyFrame); 
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame); 
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // save path for visualization
        updatePath(thisPose6D);
        
        // if(saveRawCloud) 
        //     pcl::io::savePCDFileBinary(saveMapDirectory + "/"+ to_string(cloudKeyPoses3D->size()-1) + ".pcd", *lidarCloudRaw);

    }

    void correctPoses()
    {
        if (cloudKeyPoses3D->empty())
            return;

        if (aLoopIsClosed == true)
        {
            // clear map cache
            lidarCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = mapFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();
        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry lidarOdometryROS;
        lidarOdometryROS.header.stamp = timeLidarInfoStamp;
        lidarOdometryROS.header.frame_id = mapFrame;
        lidarOdometryROS.child_frame_id = odometryFrame;
        lidarOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        lidarOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        lidarOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        // cout<<transformTobeMapped[3]<<" "<<transformTobeMapped[4]<<" "<<endl;
        lidarOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLidarOdometryGlobal.publish(lidarOdometryROS);
        globalOdometry.push_back(lidarOdometryROS);
    }

    void publishLocalMap()
    {
        pcl::PointCloud<PointType>::Ptr cloudLocal(new pcl::PointCloud<PointType>());
        if (temporaryMappingMode == false)
        {        
            *cloudLocal += *lidarCloudSurfFromMap;
            *cloudLocal += *lidarCloudCornerFromMap; 
        }
        else
        {
            for (int i=0;i<(int)temporaryCloudKeyPoses3D->size();i++)
            {
                int idx = temporaryCloudKeyPoses3D->points[i].intensity;
                *cloudLocal += *transformPointCloud(temporarySurfCloudKeyFrames[idx],&temporaryCloudKeyPoses6D->points[i]);
                *cloudLocal += *transformPointCloud(temporaryCornerCloudKeyFrames[idx],&temporaryCloudKeyPoses6D->points[i]);
            }
        }
        publishCloud(&pubRecentKeyFrames, cloudLocal, timeLidarInfoStamp, mapFrame);

        //publish temporary keyposes for visualization
        publishCloud(&pubKeyPosesTmp,temporaryCloudKeyPoses3D,timeLidarInfoStamp, mapFrame);

        globalPath.header.stamp = timeLidarInfoStamp;
        globalPath.header.frame_id = mapFrame;
        pubPath.publish(globalPath);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roll");

    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");
    
    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);
    std::thread mappingThread{&mapOptimization::run,&MO};
    std::thread queryMapThread{&mapOptimization::queryMapThread,&MO};
    ros::AsyncSpinner odometryPublicationSpinner(1, &MO.odometry_publication_queue_);
    ros::AsyncSpinner vinsPublicationSpinner(1, &MO.vins_publication_queue_);
    odometryPublicationSpinner.start();
    vinsPublicationSpinner.start();
    
    ros::spin();
    vinsPublicationSpinner.stop();
    odometryPublicationSpinner.stop();

    loopthread.join();
    visualizeMapThread.join();
    mappingThread.join();
    queryMapThread.join();
    
    return 0;
}
