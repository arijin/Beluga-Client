/*

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the usv_gazebo_dynamics_plugin package,
known as this Package.

This Package free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This Package s distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this package.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>

#include <ignition/math/Pose3.hh>

#include "usv_gazebo_plugins/usv_gazebo_dynamics_plugin.hh"

#define GRAVITY 9.815

using namespace asv;
using namespace gazebo;

//////////////////////////////////////////////////
UsvDynamicsPlugin::UsvDynamicsPlugin() {}

//////////////////////////////////////////////////
double UsvDynamicsPlugin::SdfParamDouble(sdf::ElementPtr _sdfPtr,
                                         const std::string &_paramName,
                                         const double _defaultVal) const {
  if (!_sdfPtr->HasElement(_paramName)) {
    ROS_INFO_STREAM("Parameter <" << _paramName
                                  << "> not found: "
                                     "Using default value of <"
                                  << _defaultVal << ">.");
    return _defaultVal;
  }

  double val = _sdfPtr->Get<double>(_paramName);
  ROS_DEBUG_STREAM("Parameter found - setting <" << _paramName << "> to <"
                                                 << val << ">.");
  return val;
}

//////////////////////////////////////////////////
void UsvDynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  ROS_DEBUG("Loading usv_gazebo_dynamics_plugin");
  this->world = _model->GetWorld();

  // Get parameters from SDF
  std::string linkName;
  if (!_sdf->HasElement("bodyName") ||
      !_sdf->GetElement("bodyName")->GetValue()) {
    this->link = _model->GetLink();
    linkName = this->link->GetName();
    ROS_INFO_STREAM("Did not find SDF parameter bodyName");
  } else {
    linkName = _sdf->GetElement("bodyName")->Get<std::string>();
    this->link = _model->GetLink(linkName);

    ROS_DEBUG_STREAM("Found SDF parameter bodyName as <" << linkName << ">");
  }
  if (!this->link) {
    ROS_FATAL("usv_gazebo_dynamics_plugin error: bodyName: %s does not exist\n",
              linkName.c_str());
    return;
  } else {
    ROS_DEBUG_STREAM("USV Dynamics Model Link Name = " << linkName);
  }

  this->waterLevel = this->SdfParamDouble(_sdf, "waterLevel", 0.5);
  this->waterDensity = this->SdfParamDouble(_sdf, "waterDensity", 997.7735);
  this->paramXdotU = this->SdfParamDouble(_sdf, "xDotU", 5);
  this->paramYdotV = this->SdfParamDouble(_sdf, "yDotV", 5);
  this->paramNdotR = this->SdfParamDouble(_sdf, "nDotR", 1);
  this->paramXu = this->SdfParamDouble(_sdf, "xU", 20);
  this->paramXuu = this->SdfParamDouble(_sdf, "xUU", 0);
  this->paramYv = this->SdfParamDouble(_sdf, "yV", 20);
  this->paramYvv = this->SdfParamDouble(_sdf, "yVV", 0);
  this->paramZw = this->SdfParamDouble(_sdf, "zW", 20);
  this->paramKp = this->SdfParamDouble(_sdf, "kP", 20);
  this->paramMq = this->SdfParamDouble(_sdf, "mQ", 20);
  this->paramNr = this->SdfParamDouble(_sdf, "nR", 20);
  this->paramNrr = this->SdfParamDouble(_sdf, "nRR", 0);
  this->paramHullRadius = this->SdfParamDouble(_sdf, "hullRadius", 0.213);
  this->paramBoatWidth = this->SdfParamDouble(_sdf, "boatWidth", 1.0);
  this->paramBoatLength = this->SdfParamDouble(_sdf, "boatLength", 1.35);

  if (!_sdf->HasElement("length_n")) {
    int defaultVal = 2;
    ROS_INFO_STREAM(
        "Parameter <length_n> not found: "
        "Using default value of <"
        << defaultVal << ">.");
    this->paramLengthN = defaultVal;
  } else {
    this->paramLengthN = _sdf->GetElement("length_n")->Get<int>();
  }

  //  Wave model
  // 海浪模型
  if (_sdf->HasElement("wave_model")) {
    this->waveModelName = _sdf->Get<std::string>("wave_model");
  }
  this->waveParams = nullptr;

// Get inertia and mass of vessel
// 船的惯性系数和质量
#if GAZEBO_MAJOR_VERSION >= 8
  const ignition::math::Vector3d kInertia =
      this->link->GetInertial()->PrincipalMoments();
  const double kMass = this->link->GetInertial()->Mass();
#else
  const ignition::math::Vector3d kInertia =
      this->link->GetInertial()->GetPrincipalMoments().Ign();
  const double kMass = this->link->GetInertial()->GetMass();
#endif

  // Report some of the pertinent parameters for verification
  // 报告一些相关参数以供验证
  ROS_DEBUG("USV Dynamics Parameters: From URDF XACRO model definition");
  ROS_DEBUG_STREAM("Vessel Mass (rigid-body): " << kMass);
  ROS_DEBUG_STREAM("Vessel Inertia Vector (rigid-body): X:"
                   << kInertia[0] << " Y:" << kInertia[1]
                   << " Z:" << kInertia[2]);

// Initialize time and odometry position
// 初始化时间和里程表位置
#if GAZEBO_MAJOR_VERSION >= 8
  this->prevUpdateTime = this->world->SimTime();
#else
  this->prevUpdateTime = this->world->GetSimTime();
#endif

  // Listen to the update event broadcastes every physics iteration.
  // 收听更新事件广播每个物理迭代。
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&UsvDynamicsPlugin::Update, this));

  // Initialize Added Mass Matrix
  // 初始化附加质量矩阵
  this->Ma = Eigen::MatrixXd(6, 6);
  this->Ma << this->paramXdotU, 0, 0, 0, 0, 0, 0, this->paramYdotV, 0, 0, 0, 0,
      0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0,
      this->paramNdotR;
}

double UsvDynamicsPlugin::CircleSegment(double R, double h) {
  return R * R * acos((R - h) / R) - (R - h) * sqrt(2 * R * h - h * h);
}

//////////////////////////////////////////////////
void UsvDynamicsPlugin::Update() {
  // If we haven't yet, retrieve the wave parameters from ocean model plugin.
  if (waveParams == nullptr) {
    gzmsg << "usv_gazebo_dynamics_plugin: waveParams is null. "
          << " Trying to get wave parameters from ocean model" << std::endl;
    this->waveParams =
        WavefieldModelPlugin::GetWaveParams(this->world, this->waveModelName);
  }

#if GAZEBO_MAJOR_VERSION >= 8
  const common::Time kTimeNow = this->world->SimTime();
#else
  const common::Time kTimeNow = this->world->GetSimTime();
#endif
  double dt = (kTimeNow - this->prevUpdateTime).Double();
  this->prevUpdateTime = kTimeNow;

// Get Pose/Orientation from Gazebo (if no state subscriber is active)
//从gazebo获取姿势/方向（如果没有state subscriberw处于活动状态）
#if GAZEBO_MAJOR_VERSION >= 8
  const ignition::math::Pose3d kPose = this->link->WorldPose();
#else
  const ignition::math::Pose3d kPose = this->link->GetWorldPose().Ign();
#endif
  const ignition::math::Vector3d kEuler = kPose.Rot().Euler();

// Get body-centered linear and angular rates
// 得到以船体为中心的线性和角速率
#if GAZEBO_MAJOR_VERSION >= 8
  const ignition::math::Vector3d kVelLinearBody =
      this->link->RelativeLinearVel();
#else
  const ignition::math::Vector3d kVelLinearBody =
      this->link->GetRelativeLinearVel().Ign();
#endif
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Vel linear: " << kVelLinearBody);

#if GAZEBO_MAJOR_VERSION >= 8
  const ignition::math::Vector3d kVelAngularBody =
      this->link->RelativeAngularVel();
#else
  const ignition::math::Vector3d kVelAngularBody =
      this->link->GetRelativeAngularVel().Ign();
#endif
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Vel angular: " << kVelAngularBody);

  // Estimate the linear and angular accelerations.
  // Note the the GetRelativeLinearAccel() and AngularAccel() functions
  // appear to be unreliable
  const ignition::math::Vector3d kAccelLinearBody =
      (kVelLinearBody - this->prevLinVel) / dt;
  this->prevLinVel = kVelLinearBody;
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Accel linear: " << kAccelLinearBody);
  const ignition::math::Vector3d kAccelAngularBody =
      (kVelAngularBody - this->prevAngVel) / dt;
  this->prevAngVel = kVelAngularBody;
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Accel angular: " << kAccelAngularBody);

  // Create state and derivative of state (accelerations)
  // 创造状态和状态导数（加速度）
  Eigen::VectorXd stateDot = Eigen::VectorXd(6);
  Eigen::VectorXd state = Eigen::VectorXd(6);
  Eigen::MatrixXd Cmat = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Dmat = Eigen::MatrixXd::Zero(6, 6);

  stateDot << kAccelLinearBody.X(), kAccelLinearBody.Y(), kAccelLinearBody.Z(),
      kAccelAngularBody.X(), kAccelAngularBody.Y(), kAccelAngularBody.Z();

  state << kVelLinearBody.X(), kVelLinearBody.Y(), kVelLinearBody.Z(),
      kVelAngularBody.X(), kVelAngularBody.Y(), kVelAngularBody.Z();

  // Added Mass
  // 添加质量
  const Eigen::VectorXd kAmassVec = -1.0 * this->Ma * stateDot;  // Ma质量矩阵
  ROS_DEBUG_STREAM_THROTTLE(1.0, "stateDot: \n" << stateDot);
  ROS_DEBUG_STREAM_THROTTLE(1.0, "amassVec :\n" << kAmassVec);

  // Coriolis - added mass components
  // 科氏力（旋转引发的）
  Cmat(0, 5) = this->paramYdotV * kVelLinearBody.Y();
  Cmat(1, 5) = this->paramXdotU * kVelLinearBody.X();
  Cmat(5, 0) = this->paramYdotV * kVelLinearBody.Y();
  Cmat(5, 1) = this->paramXdotU * kVelLinearBody.X();

  // Drag
  // 阻力(6个方向)
  Dmat(0, 0) = this->paramXu + this->paramXuu * std::abs(kVelLinearBody.X());
  Dmat(1, 1) = this->paramYv + this->paramYvv * std::abs(kVelLinearBody.Y());
  Dmat(2, 2) = this->paramZw;
  Dmat(3, 3) = this->paramKp;
  Dmat(4, 4) = this->paramMq;
  Dmat(5, 5) = this->paramNr + this->paramNrr * std::abs(kVelAngularBody.Z());
  ROS_DEBUG_STREAM_THROTTLE(1.0, "Dmat :\n" << Dmat);
  const Eigen::VectorXd kDvec = -1.0 * Dmat * state;
  ROS_DEBUG_STREAM_THROTTLE(1.0, "Dvec :\n" << kDvec);
  /*--------------------------------------------------浮力、坐标-------------------------------------------------*/

  // Vehicle frame transform
  // 航器坐标转换
  tf2::Quaternion vq = tf2::Quaternion();
  tf2::Matrix3x3 m;
  m.setEulerYPR(kEuler.Z(), kEuler.Y(), kEuler.X());
  m.getRotation(vq);
  tf2::Transform xformV = tf2::Transform(vq);

  // Sum all forces - in body frame
  // body坐标系合力
  const Eigen::VectorXd kForceSum = kAmassVec + kDvec;

  // Forces in fixed frame
  // 固定坐标系合力
  ROS_DEBUG_STREAM_THROTTLE(1.0, "forceSum :\n" << kForceSum);

  // Add dynamic forces/torques to link at CG
  // 在CG中添加 动力/力矩
  this->link->AddRelativeForce(
      ignition::math::Vector3d(kForceSum(0), kForceSum(1), kForceSum(2)));
  this->link->AddRelativeTorque(
      ignition::math::Vector3d(kForceSum(3), kForceSum(4), kForceSum(5)));
  // Loop over boat grid points
  // Grid point location in boat frame - might be able to precalculate these?
  // 绕船网格点
  // 船框架中的网格点位置-可能可以预先计算这些？
  tf2::Vector3 bpnt(0, 0, 0);  // 船舶坐标系中的网格点位置
  // Grid point location in world frame
  tf2::Vector3 bpntW(0, 0, 0);  // 世界坐标系中的网格点位置
  // For each hull
  for (int ii = 0; ii < 2; ii++) {
    // Grid point in boat frame
    bpnt.setY((ii * 2.0 - 1.0) * this->paramBoatWidth / 2.0);
    // For each length segment
    for (int jj = 1; jj <= this->paramLengthN; jj++) {
      bpnt.setX(((jj - 0.5) / (static_cast<float>(this->paramLengthN)) - 0.5) *
                this->paramBoatLength);

      // Transform from vessel to water/world frame
      bpntW = xformV * bpnt;

      // Debug
      ROS_DEBUG_STREAM_THROTTLE(1.0, "[" << ii << "," << jj << "] grid points"
                                         << bpnt.x() << "," << bpnt.y() << ","
                                         << bpnt.z());
      ROS_DEBUG_STREAM_THROTTLE(1.0, "v frame euler " << kEuler);
      ROS_DEBUG_STREAM_THROTTLE(1.0, "in water frame" << bpntW.x() << ","
                                                      << bpntW.y() << ","
                                                      << bpntW.z());

      // Vertical location of boat grid point in world frame
      // 船舶网格点在世界坐标系中的垂直位置
      const float kDdz = kPose.Pos().Z() + bpntW.z();
      ROS_DEBUG_STREAM("Z, pose: " << kPose.Pos().Z() << ", bpnt: " << bpntW.z()
                                   << ", dd: " << kDdz);

      // Find vertical displacement of wave field
      // World location of grid point
      //求波场的垂直位移
      //网格点的世界位置
      ignition::math::Vector3d X;
      X.X() = kPose.Pos().X() + bpntW.x();
      X.Y() = kPose.Pos().Y() + bpntW.y();

      // Compute the depth at the grid point.
      double simTime = kTimeNow.Double();
      // double depth = WavefieldSampler::ComputeDepthDirectly(
      //  *waveParams, X, simTime);
      double depth = WavefieldSampler::ComputeDepthSimply(
          *waveParams, X, simTime);  // 波浪相对海平面高度

      // Vertical wave displacement.
      // 垂直波位移
      double dz = depth + X.Z();

      // Total z location of boat grid point relative to water surface
      // 船的吃水深度，deltaZ为船底到水面的距离（正的）
      double deltaZ = (this->waterLevel + dz) - kDdz;
      deltaZ = std::max(deltaZ, 0.0);  // enforce only upward buoy force
      deltaZ = std::min(deltaZ, this->paramHullRadius);
      // Buoyancy force at grid point
      // 浮力
      const float kBuoyForce = CircleSegment(this->paramHullRadius, deltaZ) *
                               this->paramBoatLength /
                               (static_cast<float>(this->paramLengthN)) *
                               GRAVITY * this->waterDensity * 1.0;
      ROS_DEBUG_STREAM("buoyForce: " << kBuoyForce);

      // Apply force at grid point
      // From web, Appears that position is in the link frame
      // and force is in world frame
      // 从web向网格点施力，显示位置在链接框架中，力在世界框架中 
      this->link->AddForceAtRelativePosition(
          ignition::math::Vector3d(0, 0, kBuoyForce),
          ignition::math::Vector3d(bpnt.x(), bpnt.y(), bpnt.z()));
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(UsvDynamicsPlugin);
