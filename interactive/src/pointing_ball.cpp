#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <me120_msgs/CloudCluster.h>
#include <me120_msgs/CloudClusterArray.h>
#include <me120_msgs/gpsPacket.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>

#include <marker.h>

ros::Publisher markers_pub;

std::vector<visualization_msgs::Marker> ball_markers;

void ClickPointCb(const geometry_msgs::PointStampedConstPtr& point_msg){
  printf("x:%lf, y:%lf\n", point_msg->point.x, point_msg->point.y);
  visualization_msgs::Marker marker;
  marker = GenarateMarker(point_msg->point.x, point_msg->point.y, ball_markers.size(), 1, 0, 0, 0.6, point_msg->header);
  ball_markers.push_back(marker);
  visualization_msgs::MarkerArray markers_msg;
  markers_msg.markers = ball_markers;
  markers_pub.publish(marker);
  return;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pointing_ball");
  ros::NodeHandle nh;
  markers_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

  ros::Subscriber gps_full_data_sub = nh.subscribe("/clicked_point", 1, ClickPointCb);
  ros::spin();

  system("pause");
  return 0;
}

