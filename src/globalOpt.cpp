#include "resource_timing.h"
#include "bounded_history.h"
/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

// adapted for global loc
// 1. add init function;
// 2. add sliding-window opt.
// 3. add outlier detection

#include "globalOpt.h"
#include "Factors.h"
#include <ros/ros.h>

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

using namespace gtsam;

// keep quaternion input to adapt ceres implementation

GlobalOptimization::GlobalOptimization(int maxNo)
{
	initGPS = false;
    newGPS = false;
    newGlobalLocPose = false;
    newMidGlobalLocPose = false;
	WGlobal_T_WLocal = Eigen::Matrix4d::Identity();
    maxFrameNum = maxNo;

    reInitialize = false;
    Tacc = Eigen::Matrix4d::Identity();
    backupTgl = Eigen::Matrix4d::Identity();
    smoothedTgl_ = Eigen::Matrix4d::Identity();
    // Start only after every field read by the worker is initialized.
    threadOpt = std::thread(&GlobalOptimization::optimize, this);

}
void GlobalOptimization::setTgl(Eigen::Matrix4d mat)
{
    std::lock_guard<std::mutex> lock(mPoseMap);
    WGlobal_T_WLocal = mat;
    backupTgl = mat;
    smoothedTgl_ = mat;  // reset smoothed Tgl when explicitly set (e.g. initialization)
    publicationCorrection_.update(mat);
}

GlobalOptimization::~GlobalOptimization()
{
    threadOpt.detach();
}

void GlobalOptimization::affine2qt(const Eigen::Matrix4d aff, Eigen::Vector3d &p, Eigen::Quaterniond &q)
{
    q = Eigen::Quaterniond(aff.block<3,3>(0,0));
    p = aff.block<3,1>(0,3);    
    // cout<<"q: "<<q.w()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<endl;
    return;
}

void GlobalOptimization::inputIMUToBodyAffine(Eigen::Affine3f &affine_imu_to_odom){
    affine_imu_to_odom_ = affine_imu_to_odom;
}

// here we update globally aligned odom pose asap
void GlobalOptimization::inputOdom(double t, Eigen::Matrix4d affine)
{
    Eigen::Vector3d OdomP;
    Eigen::Quaterniond OdomQ;
    affine2qt(affine,OdomP,OdomQ);

	mPoseMap.lock();
    vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(),
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    localPoseMap[t] = localPose;

    Eigen::Quaterniond globalQ;
    globalQ = WGlobal_T_WLocal.block<3, 3>(0, 0) * OdomQ;
    Eigen::Vector3d globalP = WGlobal_T_WLocal.block<3, 3>(0, 0) * OdomP + WGlobal_T_WLocal.block<3, 1>(0, 3);
    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalPoseMap[t] = globalPose;

    // === DEBUG: detect jump in globalOpt inputOdom ===
    {
        static Eigen::Vector3d prevGlobalP(0, 0, 0);
        static Eigen::Vector3d prevOdomP(0, 0, 0);
        static Eigen::Matrix4d prevTgl = Eigen::Matrix4d::Identity();
        static bool inputOdomInit = false;
        static double prevT = 0;
        if (inputOdomInit) {
            double dt = t - prevT;
            double globalDist = (globalP - prevGlobalP).norm();
            double odomDist = (OdomP - prevOdomP).norm();
            double globalSpeed = (dt > 1e-6) ? globalDist / dt : 0;

            Eigen::Matrix4d tglDiff = prevTgl.inverse() * WGlobal_T_WLocal;
            double tglTransChange = tglDiff.block<3,1>(0,3).norm();

            if (globalDist > 2.0) {
                ROS_ERROR("[GlobalOpt::inputOdom] JUMP! globalDist=%.3f m, odomDist=%.3f m, dt=%.4f s", globalDist, odomDist, dt);
                ROS_ERROR("[GlobalOpt::inputOdom]   globalP: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)",
                          prevGlobalP.x(), prevGlobalP.y(), prevGlobalP.z(), globalP.x(), globalP.y(), globalP.z());
                ROS_ERROR("[GlobalOpt::inputOdom]   odomP: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)",
                          prevOdomP.x(), prevOdomP.y(), prevOdomP.z(), OdomP.x(), OdomP.y(), OdomP.z());
                ROS_ERROR("[GlobalOpt::inputOdom]   Tgl trans change=%.3f", tglTransChange);
            }
            if (globalSpeed > 30.0) {
                ROS_ERROR("[GlobalOpt::inputOdom] ABNORMAL SPEED! globalSpeed=%.2f m/s", globalSpeed);
            }
            if (tglTransChange > 0.5) {
                ROS_WARN("[GlobalOpt::inputOdom] Tgl changed significantly! trans_change=%.3f m", tglTransChange);
                ROS_WARN("[GlobalOpt::inputOdom]   Tgl_trans: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f)",
                         prevTgl(0,3), prevTgl(1,3), prevTgl(2,3),
                         WGlobal_T_WLocal(0,3), WGlobal_T_WLocal(1,3), WGlobal_T_WLocal(2,3));
            }
        }
        prevGlobalP = globalP;
        prevOdomP = OdomP;
        prevTgl = WGlobal_T_WLocal;
        prevT = t;
        inputOdomInit = true;
    }

    lastP = globalP;
    lastQ = globalQ;

    // sliding-window
    if ((int)localPoseMap.size() > maxFrameNum)
    {
        // special note: erase function of map is not the same as vector
        localPoseMap.erase((localPoseMap.begin())->first);
        globalPoseMap.erase((globalPoseMap.begin())->first);
    }

    // Optimization only queries constraints at timestamps still in localPoseMap.
    // Older constraints can never be used again and otherwise grow without bound.
    if (!localPoseMap.empty())
    {
        const double oldestTime = localPoseMap.begin()->first;
        while (!globalLocPoseMap.empty() && globalLocPoseMap.begin()->first < oldestTime)
            globalLocPoseMap.erase(globalLocPoseMap.begin());
        while (!midGlobalLocPoseMap.empty() && midGlobalLocPoseMap.begin()->first < oldestTime)
            midGlobalLocPoseMap.erase(midGlobalLocPoseMap.begin());
    }
    trimPoseMapByCount(globalLocPoseMap, maxFrameNum);
    trimPoseMapByCount(midGlobalLocPoseMap, maxFrameNum);

    mPoseMap.unlock();
}

void GlobalOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

void GlobalOptimization::getGlobalAffine(Eigen::Affine3f &Tml)
{
    // need to wait for the 1st opt. completion
    Eigen::Matrix4d tmp;
    tmp.block<3,3>(0,0) = lastQ.toRotationMatrix();
    tmp.block<3,1>(0,3) = lastP;
    Tml = tmp.cast<float>();
}

void GlobalOptimization::getTgl(Eigen::Matrix4d &tgl)
{
    tgl = publicationCorrection_.read();
}

void GlobalOptimization::projectRealtimeOdom(const Eigen::Matrix4d &odometry,
                                            Eigen::Vector3d &odomP,
                                            Eigen::Quaterniond &odomQ) const
{
    publicationCorrection_.project(odometry, odomP, odomQ);
}


void GlobalOptimization::inputGlobalLocPose(double t, Eigen::Matrix4d affine, double t_error, double q_error)
{
    mPoseMap.lock();

    // Convert IMU-to-map to body-to-map
    Eigen::Matrix4d affine_imu_to_body = affine_imu_to_odom_.matrix().cast<double>();
    Eigen::Matrix4d affine_body_to_map = affine * affine_imu_to_body.inverse();

    // Print conversion result
    Eigen::Vector3d imu_pos = affine.block<3,1>(0,3);
    Eigen::Vector3d body_pos = affine_body_to_map.block<3,1>(0,3);
    // ROS_INFO("[GlobalLoc] IMU->Map: (%.3f, %.3f, %.3f) -> Body->Map: (%.3f, %.3f, %.3f)",
    //          imu_pos.x(), imu_pos.y(), imu_pos.z(), body_pos.x(), body_pos.y(), body_pos.z());

    Eigen::Vector3d locP;
    Eigen::Quaterniond locQ;
    affine2qt(affine_body_to_map, locP, locQ);
    vector<double> globalLocPose{locP.x(), locP.y(), locP.z(),
                              locQ.w(), locQ.x(), locQ.y(), locQ.z(), t_error, q_error};

    // === DEBUG: detect global loc pose jump ===
    {
        static Eigen::Vector3d prevLocP(0,0,0);
        static bool locPoseInit = false;
        if (locPoseInit) {
            double locDist = (locP - prevLocP).norm();
            if (locDist > 3.0) {
                ROS_ERROR("[GlobalOpt::inputGlobalLocPose] JUMP in global loc input! dist=%.3f m", locDist);
                ROS_ERROR("[GlobalOpt::inputGlobalLocPose]   prev: (%.3f,%.3f,%.3f) -> cur: (%.3f,%.3f,%.3f), t_err=%.3f, q_err=%.3f",
                          prevLocP.x(), prevLocP.y(), prevLocP.z(), locP.x(), locP.y(), locP.z(), t_error, q_error);
            }
        }
        prevLocP = locP;
        locPoseInit = true;
    }
    ROS_INFO("[GlobalOpt::inputGlobalLocPose] t=%.3f pos=(%.3f,%.3f,%.3f) t_err=%.3f q_err=%.3f mapSize=%d",
             t, locP.x(), locP.y(), locP.z(), t_error, q_error, (int)globalLocPoseMap.size());

    globalLocPoseMap[t] = globalLocPose;
    if (!localPoseMap.empty()) trimPoseMapBefore(globalLocPoseMap, localPoseMap.begin()->first);
    trimPoseMapByCount(globalLocPoseMap, maxFrameNum);

    if (reInitialize == true && (int)globalLocPoseMap.size() < 10)
    {
        mPoseMap.unlock();
        return;
    }
    newGlobalLocPose = true;

    mPoseMap.unlock();
}

void GlobalOptimization::inputMidGlobalLocPose(double t, Eigen::Matrix4d affine, double t_error, double q_error)
{
    mPoseMap.lock();

    // Convert IMU-to-map to body-to-map
    Eigen::Matrix4d affine_imu_to_body = affine_imu_to_odom_.matrix().cast<double>();
    Eigen::Matrix4d affine_body_to_map = affine * affine_imu_to_body.inverse();

    // Print conversion result
    Eigen::Vector3d imu_pos = affine.block<3,1>(0,3);
    Eigen::Vector3d body_pos = affine_body_to_map.block<3,1>(0,3);
    ROS_INFO("[MidGlobalLoc] IMU->Map: (%.3f, %.3f, %.3f) -> Body->Map: (%.3f, %.3f, %.3f)",
             imu_pos.x(), imu_pos.y(), imu_pos.z(), body_pos.x(), body_pos.y(), body_pos.z());

    Eigen::Vector3d locP;
    Eigen::Quaterniond locQ;
    affine2qt(affine_body_to_map, locP, locQ);
    vector<double> midGlobalLocPose{locP.x(), locP.y(), locP.z(),
                              locQ.w(), locQ.x(), locQ.y(), locQ.z(), t_error, q_error};
    midGlobalLocPoseMap[t] = midGlobalLocPose;
    if (!localPoseMap.empty()) trimPoseMapBefore(midGlobalLocPoseMap, localPoseMap.begin()->first);
    trimPoseMapByCount(midGlobalLocPoseMap, maxFrameNum);

    if (reInitialize == true && (int)midGlobalLocPoseMap.size() < 10)
    {
        mPoseMap.unlock();
        return;
    }
    newGlobalLocPose = true;

    mPoseMap.unlock();
}

void GlobalOptimization::resetOptimization(Eigen::Matrix4d Tgl)
{
    std::lock_guard<std::mutex> lock(mPoseMap);
    resetOptimizationLocked(Tgl);
}

void GlobalOptimization::resetOptimizationLocked(const Eigen::Matrix4d &Tgl)
{
    // cout<<"too much jump in Tgl change, forfeit estimate:  Tgl change "<<deltaTransGL<<endl;
    WGlobal_T_WLocal = Tgl;
    smoothedTgl_ = Tgl;  // reset smoothed Tgl on optimization reset

    globalLocPoseMap.clear();
    midGlobalLocPoseMap.clear();
    globalPoseMap.clear();
    localPoseMap.clear();
    reInitialize = true;
    publicationCorrection_.update(Tgl);

}

gtsam::Pose3 QT2gtsamPose(vector<double> qt)
{
    Eigen::Quaterniond q(qt[3],qt[4],qt[5],qt[6]);
    Eigen::Matrix3d qtMatrix = q.matrix();
    // here it returns (thetaZ,thetaY,thetaX)
    Eigen::Vector3d ypr = qtMatrix.eulerAngles(2,1,0);
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(ypr[2], ypr[1], ypr[0]), 
                                  gtsam::Point3(qt[0], qt[1], qt[2]));
}

void gtsamPose2Matrix4d(gtsam::Pose3 pose, Eigen::Matrix4d & mat)
{
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(pose.rotation().roll(),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pose.rotation().pitch(),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(pose.rotation().yaw(),Eigen::Vector3d::UnitZ())); 
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = yawAngle*pitchAngle*rollAngle;
    mat.block<3,3>(0,0) = rotation_matrix;
    mat.block<3,1>(0,3) = Eigen::Vector3d(pose.translation().x(),pose.translation().y(),pose.translation().z());

}
void gtsamPose2Vector(gtsam::Pose3 pose, vector<double> &qt)
{
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(pose.rotation().roll(),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pose.rotation().pitch(),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(pose.rotation().yaw(),Eigen::Vector3d::UnitZ())); 
    Eigen::Matrix3d rotation_matrix = (yawAngle*pitchAngle*rollAngle).matrix();
    Eigen::Quaterniond qd =  Eigen::Quaterniond(rotation_matrix);
    qt[0] = pose.translation().x();
    qt[1] = pose.translation().y();
    qt[2] = pose.translation().z();
    qt[3] = qd.w(); // are you sure?
    qt[4] = qd.x();
    qt[5] = qd.y();
    qt[6] = qd.z();
}
void GlobalOptimization::optimize()
{
    while(true)
    {
        if(newGlobalLocPose)
        {
            if (reInitialize == true) reInitialize = false;

            TicToc opt_time;
            newGlobalLocPose = false;
            // printf("global optimization using global localization and odometry\n");
            TicToc globalOptimizationTime;

            // gtsam
            NonlinearFactorGraph gtSAMgraphTM;
            Values initialEstimateTM;        
            Values isamCurrentEstimateTM;
            ISAM2Params parameters;
            parameters.relinearizeThreshold = 0.1;
            parameters.relinearizeSkip = 1;
            ISAM2 isamTM(parameters); // RAII: released on every exit, including continue.

            //add param
            mPoseMap.lock();

            int length = localPoseMap.size();
            // cout<<" pose no. before opt "<< length<<endl;
            map<double, vector<double>>::iterator iterIni,iterLIO, iterLIOnext, iterGlobalLoc, iterMidGlobalLoc;
            iterIni = globalPoseMap.begin();  //using odomTOmap value for initial guess 
            int i = 0;
            int found  = 0;
            // int outlierNO = 0;
            for (iterLIO = localPoseMap.begin(); iterLIO != localPoseMap.end(); iterLIO++, i++,iterIni++)
            {
                //vio factor
                // cout<<"i lio pose: " <<i<<endl;
                iterLIOnext = iterLIO;
                iterLIOnext++;
                if(iterLIOnext != localPoseMap.end())
                {
                    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) <<1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2 ).finished());
                    gtsam::Pose3 poseFrom = QT2gtsamPose(iterLIO->second);
                    gtsam::Pose3 poseTo   = QT2gtsamPose(iterLIOnext->second);
                    gtSAMgraphTM.add(BetweenFactor<Pose3>(i,i+1, poseFrom.between(poseTo), odometryNoise));
                }
   
                double t = iterLIO->first;
                iterGlobalLoc = globalLocPoseMap.find(t); // synchronized global loc pose
                if (iterGlobalLoc != globalLocPoseMap.end())
                {
                    gtsam::Pose3 poseGlobal = QT2gtsamPose(iterGlobalLoc->second);
                    // seeems to be overconfident
                    // noiseModel::Diagonal::shared_ptr corrNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1).finished()); // rad*rad, meter*meter
                    double tE = iterGlobalLoc->second[7];
                    double tQ = iterGlobalLoc->second[8];
                    noiseModel::Diagonal::shared_ptr corrNoise = noiseModel::Diagonal::Variances((Vector(6) << tQ*tQ,tQ*tQ,tQ*tQ,tE*tE,tE*tE,tE*tE).finished()); // rad*rad, meter*meter
                    gtSAMgraphTM.add(PriorFactor<Pose3>(i, poseGlobal, corrNoise));
                    found++;

                    // for CC
                    Eigen::Matrix4d local = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d global = Eigen::Matrix4d::Identity();
                    global.block<3, 3>(0, 0) = Eigen::Quaterniond(iterGlobalLoc->second[3], iterGlobalLoc->second[4], 
                                                                        iterGlobalLoc->second[5], iterGlobalLoc->second[6]).toRotationMatrix();
                    global.block<3, 1>(0, 3) = Eigen::Vector3d(iterGlobalLoc->second[0], iterGlobalLoc->second[1], iterGlobalLoc->second[2]);
                    local.block<3, 3>(0, 0) = Eigen::Quaterniond(iterLIO->second[3], iterLIO->second[4], 
                                                                        iterLIO->second[5], iterLIO->second[6]).toRotationMatrix();
                    local.block<3, 1>(0, 3) = Eigen::Vector3d(iterLIO->second[0], iterLIO->second[1], iterLIO->second[2]);
                    backupTgl = global*local.inverse(); // get the newest Tgl as backup
                }

                // mid-term global loc factor
                iterMidGlobalLoc = midGlobalLocPoseMap.find(t);
                if (iterMidGlobalLoc != midGlobalLocPoseMap.end())
                {
                    gtsam::Pose3 poseMidGlobal = QT2gtsamPose(iterMidGlobalLoc->second);
                    double tE = iterMidGlobalLoc->second[7];
                    double tQ = iterMidGlobalLoc->second[8];
                    noiseModel::Diagonal::shared_ptr midCorrNoise = noiseModel::Diagonal::Variances((Vector(6) << tQ*tQ,tQ*tQ,tQ*tQ,tE*tE,tE*tE,tE*tE).finished());
                    gtSAMgraphTM.add(PriorFactor<Pose3>(i, poseMidGlobal, midCorrNoise));
                    found++;
                }

                gtsam::Pose3 poseGuess = QT2gtsamPose(iterIni->second);
                initialEstimateTM.insert(i, poseGuess);
            }

            if (found == 0)
            {
                mPoseMap.unlock();
                continue;
            }

            // cout<<"found: "<<found<<endl; // why zero from the start???
            isamTM.update(gtSAMgraphTM, initialEstimateTM);
            isamTM.update();
            isamTM.update();
            gtSAMgraphTM.resize(0);
            initialEstimateTM.clear();

            isamCurrentEstimateTM = isamTM.calculateEstimate();

            // cout<<"Estimate size: "<<length<<endl;
            // update global pose
            iterIni = globalPoseMap.begin();

            Eigen::Matrix4d start;
            Eigen::Matrix4d end;
            Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity(); 
            Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
        
            for (int i = 0; i < length; i++, iterIni++)
            {
                // cout<<"i "<<i<<"length :"<<length<<endl;
                vector<double> globalPose(7,0);
                gtsamPose2Vector(isamCurrentEstimateTM.at<Pose3>(i),globalPose);
                iterIni->second = globalPose;

                double t = iterIni->first;
                WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4], 
                                                                    localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
                WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
                WGPS_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4], 
                                                                    globalPose[5], globalPose[6]).toRotationMatrix();
                WGPS_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);

                WGlobal_T_WLocal = WGPS_T_body * WVIO_T_body.inverse();

                if (i == 0 ) start = WGlobal_T_WLocal;
                if (i == length - 1 ) end = WGlobal_T_WLocal;
            }
            

            // w. consistency check: GTSAM implementation needs CC as well
            // Tgl change too much, forfeit this optimization
            Eigen::Matrix4d TglDelta = start.inverse()*end;
            double deltaTransGL = sqrt(TglDelta(0,3)*TglDelta(0,3) +  TglDelta(1,3)*TglDelta(1,3) + TglDelta(2,3)*TglDelta(2,3) );

            // === DEBUG: print optimization result ===
            ROS_INFO("[GlobalOpt::optimize] done. frames=%d, found=%d, deltaTgl=%.3f, Tgl_trans=(%.3f,%.3f,%.3f), opt_time=%.1fms",
                     length, found, deltaTransGL,
                     WGlobal_T_WLocal(0,3), WGlobal_T_WLocal(1,3), WGlobal_T_WLocal(2,3), opt_time.toc());

            // === DEBUG: print Tgl before vs after optimization ===
            {
                static Eigen::Matrix4d prevOptTgl = Eigen::Matrix4d::Identity();
                static bool optTglInit = false;
                if (optTglInit) {
                    double optTglChange = (WGlobal_T_WLocal.block<3,1>(0,3) - prevOptTgl.block<3,1>(0,3)).norm();
                    if (optTglChange > 0.3) {
                        ROS_WARN("[GlobalOpt::optimize] Tgl changed %.3f m after this optimization!", optTglChange);
                        ROS_WARN("[GlobalOpt::optimize]   before: (%.3f,%.3f,%.3f) -> after: (%.3f,%.3f,%.3f)",
                                 prevOptTgl(0,3), prevOptTgl(1,3), prevOptTgl(2,3),
                                 WGlobal_T_WLocal(0,3), WGlobal_T_WLocal(1,3), WGlobal_T_WLocal(2,3));
                    }
                }
                prevOptTgl = WGlobal_T_WLocal;
                optTglInit = true;
            }

            double deltaX = fabs(TglDelta(0,3));
            double deltaY = fabs(TglDelta(1,3));
            double deltaZ = fabs(TglDelta(2,3));
            double axisThreshold = 0.3;

            if ( deltaTransGL > 0.5 || deltaX > axisThreshold || deltaY > axisThreshold || deltaZ > axisThreshold)
            {
                ROS_ERROR("[GlobalOpt::optimize] RESET! deltaTgl=%.3f (dX=%.3f,dY=%.3f,dZ=%.3f) > threshold, resetting to backupTgl (%.3f,%.3f,%.3f)",
                          deltaTransGL, deltaX, deltaY, deltaZ, backupTgl(0,3), backupTgl(1,3), backupTgl(2,3));
                resetOptimizationLocked(backupTgl);
            }
            double opt_t = opt_time.toc();
            resource_metrics::record("backend_optimization", ros::Time::now().toSec(), opt_t);
            cout<<"optimization takes: "<<opt_t<<" ms"<<endl; // gtsam implementation usually takes less than 10 ms
            ROS_ERROR("optimization takes %f ms", opt_t);

            if(opt_t> 100) cout<<"gtsam opt. takes more than 100 ms"<<endl;

            // Commit only the final correction, after consistency checks/reset.
            publicationCorrection_.update(WGlobal_T_WLocal);
            mPoseMap.unlock();
        }
        // waiting for poses to be accumulated
        std::chrono::milliseconds dura(500);
        std::this_thread::sleep_for(dura);
    }
	return;
}

