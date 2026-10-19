#include "utility.h"

sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0) // this is important since publishing actually takes quite some time
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}


void printTrans(std::string message, vector<double> transformIn){
    if (transformIn.size() != 6) return;
    cout<<message<<transformIn[0]<<" "<<transformIn[1]<<" "
        <<transformIn[2]<<" "<<transformIn[3]<<" "<<transformIn[4]<<" "<<transformIn[5];
    return;
}
// returning c array is not easy to pull off, just forget it
void Affine3f2Trans(Eigen::Affine3f t,float transformOut[6])
{
    pcl::getTranslationAndEulerAngles(t,transformOut[3], transformOut[4], transformOut[5], transformOut[0], transformOut[1], transformOut[2]);
}

Eigen::Affine3f trans2Affine3f(float transformIn[6])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

void printTrans(std::string message, const float transformIn[]){
    ROS_INFO_STREAM(message<<transformIn[0]<<" "<<transformIn[1]<<" "
        <<transformIn[2]<<" "<<transformIn[3]<<" "<<transformIn[4]<<" "<<transformIn[5]);
    return;
}

double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}