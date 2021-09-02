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
#include <me120_msgs/StartInfo.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>


ros::Publisher info_pub;

int i=0;
double dx,dy;
double black_1_x, black_1_y;
double black_2_x, black_2_y;

void send(){
  me120_msgs::StartInfo info;
  info.dx = dx;
  info.dy = dy;
  info.black_1_x = black_1_x;
  info.black_1_y = black_1_y;
  info.black_2_x = black_2_x;
  info.black_2_y = black_2_y;
  info_pub.publish(info);
}

void ClickPointCb(const geometry_msgs::PointStampedConstPtr& point_msg){
  if(i==0){
    // cent_point_x -=dx  cent_point_y -=dy
    dx = point_msg->point.x;
    dy = point_msg->point.y;
    printf("dx:%lf, dy:%lf\n", dx, dy);
  }
  else if (i==1)
  {
    black_1_x = point_msg->point.x;
    black_1_y = point_msg->point.y;
    printf("b1x:%lf, b1y:%lf\n", black_1_x, black_1_y);
  }
  else if(i==2){
    black_2_x = point_msg->point.x;
    black_2_y = point_msg->point.y;
    printf("b2x:%lf, b2y:%lf\n", black_2_x, black_2_y);
    send();
    printf("sended!");
  }
  i++;
  return;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "start");
  ros::NodeHandle nh;
  info_pub = nh.advertise<me120_msgs::StartInfo>("/start_info", 1);

  ros::Subscriber point_click_sub = nh.subscribe("/clicked_point", 1, ClickPointCb);
  ros::spin();

  system("pause");
  return 0;
}

