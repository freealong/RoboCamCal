/*
 * Copyright (c) XYZ Robotics Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Author: yongqi <Frank.lee@xyzrobotics.ai>, 2019/11/7
 */
#ifndef ROBOCAMCAL_SRC_ROBOTBRIDGE_HPP_
#define ROBOCAMCAL_SRC_ROBOTBRIDGE_HPP_

#include <exception>
#include <Eigen/Eigen>
#include <ros/node_handle.h>
#include <ros/service_client.h>

#include "robocamcal/RobotPose.h"

namespace Robocamcal {

class RobotBridge {
 public:
  RobotBridge(std::string source) {
    from_ros_service_ = source.find('.') == std::string::npos;
    // @TODO: support reading pose from file
    if (!from_ros_service_) {
      throw std::invalid_argument("reading pose from file not supported");
    }
    rs_client_ = nh_.serviceClient<robocamcal::RobotPose>(source);
  }

  bool GetPose(Eigen::Isometry3d &pose) {
    robocamcal::RobotPose robot_pose;
    if (rs_client_.call(robot_pose)) {
      auto res = robot_pose.response;
      pose.setIdentity();
      Eigen::Quaterniond quat(res.qx, res.qy, res.qz, res.q0);
      pose *= quat;
      pose.translation() = Eigen::Vector3d(res.x / 1000., res.y / 1000., res.z / 1000.);
      return true;
    }
    return false;
  }

 private:
  bool from_ros_service_;
  ros::NodeHandle nh_;
  ros::ServiceClient rs_client_;
};

}

#endif //ROBOCAMCAL_SRC_ROBOTBRIDGE_HPP_
