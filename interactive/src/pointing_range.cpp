#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>
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
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <marker.h>

ros::Publisher markers_pub;
ros::Publisher polygon_pub;

bool Confirm_SIGNAL = false;
std::vector<geometry_msgs::Point32> polygon_geo_vertexes;
std::vector<cv::Point> polygon_vertexes;

void wirte_polygon(void)
{
  std::ofstream outfile("/home/arijin/Coding/catkin_ws/src/ME120/interactive/config/param.txt", std::ofstream::out);
  // outfile.setf(std::ios::fixed, std::ios::floatfield);
  // outfile.precision(12);

  for (size_t i = 0; i < polygon_geo_vertexes.size(); i++)
  {
    outfile << polygon_geo_vertexes[i].x << " " << polygon_geo_vertexes[i].y << "\n";
  }
  outfile.close();
}

int id = 0;
void ClickPointCb(const geometry_msgs::PointStampedConstPtr &point_msg)
{
  std::cout << "x:" << point_msg->point.x << ", y:" << point_msg->point.y << std::endl;
  if ((abs(point_msg->point.x) < 3 && abs(point_msg->point.y) < 3) && (!Confirm_SIGNAL))
  {
    geometry_msgs::PolygonStamped polygonMsg;
    polygonMsg.header = point_msg->header;
    polygonMsg.polygon.points = polygon_geo_vertexes;
    polygon_pub.publish(polygonMsg);
    Confirm_SIGNAL = true;
    wirte_polygon();
    std::cout << "Confirm Range!" << std::endl;
    return;
  }
  if (!Confirm_SIGNAL)
  {
    visualization_msgs::Marker marker;
    marker = GenarateVertex(point_msg->point.x, point_msg->point.y, id, point_msg->header);
    id++;
    markers_pub.publish(marker);

    geometry_msgs::Point32 geo_pt;
    geo_pt.x = point_msg->point.x;
    geo_pt.y = point_msg->point.y;
    geo_pt.z = 0;
    polygon_geo_vertexes.push_back(geo_pt);

    cv::Point pt;
    pt.x = point_msg->point.x;
    pt.y = point_msg->point.y;
    polygon_vertexes.push_back(pt);
    return;
  }
  else
  {
    cv::Point p_query(point_msg->point.x, point_msg->point.y);
    visualization_msgs::Marker marker;
    if (pointPolygonTest(polygon_vertexes, p_query, true) >= 1)
    {
      std::cout << "ball in range." << std::endl;
      marker = GenarateMarker(point_msg->point.x, point_msg->point.y, id, 1, 1, 0, 0.7, point_msg->header);
    }
    else
    {
      std::cout << "ball off range." << std::endl;
      marker = GenarateMarker(point_msg->point.x, point_msg->point.y, id, 0.1, 0.1, 0.1, 0.35, point_msg->header);
    }
    id++;
    markers_pub.publish(marker);
  }
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "range");
  ros::NodeHandle nh;
  markers_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("/visualization_polygon", 1);

  ros::Subscriber gps_full_data_sub = nh.subscribe("/clicked_point", 1, ClickPointCb);
  std::cout << "start to pain the range:" << std::endl;
  ros::spin();

  system("pause");
  return 0;
}
