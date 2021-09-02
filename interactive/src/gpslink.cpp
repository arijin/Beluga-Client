#include <gpslink.h>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "std_msgs/Header.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "me120_msgs/gpsPacket.h"

GPSLinkdata::GPSLinkdata()
{
}

GPSLinkdata::~GPSLinkdata()
{
}

void GPSLinkdata::setData(me120_msgs::gpsPacket PacketMsg){
    
    sensor_msgs::NavSatFix SatCoordinate_msg;
    geometry_msgs::Pose pose_msg;
    geometry_msgs::Twist velocity_msg;

    _header_msg = PacketMsg.header;
    SatCoordinate_msg = PacketMsg.SatCoordinate;
    pose_msg = PacketMsg.pose;
    velocity_msg = PacketMsg.velocity;

    _latitude = SatCoordinate_msg.latitude;
    _longitude = SatCoordinate_msg.longitude;
    _altitude = SatCoordinate_msg.altitude;

    geometry_msgs::Quaternion quat_msg;
    quat_msg = pose_msg.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quat_msg, quat);
    tf::Matrix3x3(quat).getRPY(_roll, _pitch, _heading);
    
    _Ve = velocity_msg.linear.x;
    _Vn = velocity_msg.linear.y;
    // std::cout << "Ve:" << _Ve << ", Vn:" << _Vn << std::endl;
}