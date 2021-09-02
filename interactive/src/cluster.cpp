#include "cluster.h"

Cluster::Cluster()
{
  valid_cluster_ = true;
}

Cluster::~Cluster()
{
  // TODO Auto-generated destructor stub
}

void Cluster::SetData(me120_msgs::CloudCluster in_cluster_message){
  min_point_.x = in_cluster_message.min_point.point.x;
  min_point_.y = in_cluster_message.min_point.point.y;
  min_point_.z = in_cluster_message.min_point.point.z;

  max_point_.x = in_cluster_message.max_point.point.x;
  max_point_.y = in_cluster_message.max_point.point.y;
  max_point_.z = in_cluster_message.max_point.point.z;

  cent_point_.x = in_cluster_message.bounding_box.pose.position.x;
  cent_point_.y = in_cluster_message.bounding_box.pose.position.y;
  cent_point_.z = in_cluster_message.bounding_box.pose.position.z;


  length_ = in_cluster_message.bounding_box.dimensions.y;
  width_ = in_cluster_message.bounding_box.dimensions.x;
  height_ = in_cluster_message.bounding_box.dimensions.z;

  geometry_msgs::Quaternion quat_msg;
  quat_msg = in_cluster_message.bounding_box.pose.orientation;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(quat_msg, quat);
  double rx,ry,rz;
  tf::Matrix3x3(quat).getRPY(rx, ry, rz);
  orientation_angle_ = rz;  //  / 3.1416 * 180;

  label_ = in_cluster_message.label;

}

jsk_recognition_msgs::BoundingBox Cluster::GetBoundingBox(void){
  jsk_recognition_msgs::BoundingBox bounding_box_;
  bounding_box_.pose.position.x = cent_point_.x;
  bounding_box_.pose.position.y = cent_point_.y;
  bounding_box_.pose.position.z = cent_point_.z;
  bounding_box_.dimensions.x = width_;
  bounding_box_.dimensions.y = length_;
  bounding_box_.dimensions.z = height_;
  tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, orientation_angle_);
  tf::quaternionTFToMsg(quat, bounding_box_.pose.orientation);
  return bounding_box_;
}

