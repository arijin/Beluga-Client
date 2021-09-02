#ifndef _CLUSTERMEMBER_H_
#define _CLUSTERMEMBER_H_
#include <limits>
#include <cmath>
#include <chrono>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_rviz_plugins/PictogramArray.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cluster.h>

struct ClusterInfo{
  pcl::PointXYZ cent_point_;
  float length_, width_, height_;
  double orientation_angle_;
};

class ClusterMember{
public:
  int attend_times;
  int absent_times;
  bool selected;
  ClusterInfo NewestInfo;
  std::vector<ClusterInfo> ClusterInfo_track;
  
  ClusterMember();
  virtual ~ClusterMember();
  void UpdateInfoTrack(Cluster cluster);
  void UpdateNewestInfo(ClusterInfo newinfo);
};

#endif