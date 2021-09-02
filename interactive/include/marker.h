#include <vector>

#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

visualization_msgs::Marker GenarateMarker(double x, double y,int id,float r, float g, float b, float a, std_msgs::Header in_header);
visualization_msgs::Marker GenarateVertex(double x, double y,int id, std_msgs::Header in_header);