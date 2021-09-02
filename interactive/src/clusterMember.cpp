#include "clusterMember.h"

ClusterMember::ClusterMember(){
  attend_times = 0;
  absent_times = 0;
  selected = false;
  ClusterInfo_track.clear();
}

ClusterMember::~ClusterMember(){;}

void ClusterMember::UpdateNewestInfo(ClusterInfo newinfo){
  NewestInfo = newinfo;
}

void ClusterMember::UpdateInfoTrack(Cluster cluster){
  ClusterInfo newinfo;
  newinfo.width_ = cluster.width_;
  newinfo.length_ = cluster.length_;
  newinfo.height_ = cluster.height_;
  newinfo.cent_point_ = cluster.cent_point_;
  newinfo.orientation_angle_ = cluster.orientation_angle_;
  ClusterInfo_track.push_back(newinfo);
  this->UpdateNewestInfo(newinfo);
  absent_times = 0;
}


