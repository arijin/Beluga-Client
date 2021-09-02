#ifndef _CLUSTER_H_
#define _CLUSTER_H_
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

#include "me120_msgs/CloudCluster.h"

class Cluster
{
public:
  pcl::PointXYZ min_point_;
  pcl::PointXYZ max_point_;
  pcl::PointXYZ cent_point_;
  float length_, width_, height_;
  double orientation_angle_;
  std::string label_;

  bool valid_cluster_;

  Cluster();
  virtual ~Cluster();
  void SetData(me120_msgs::CloudCluster in_cluster_message);
  jsk_recognition_msgs::BoundingBox GetBoundingBox(void);
};

typedef boost::shared_ptr<Cluster> ClusterPtr;

#endif