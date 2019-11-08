/*
 * Copyright (c) XYZ Robotics Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Author: yongqi <Frank.lee@xyzrobotics.ai>, 2019/10/23
 */
#ifndef ROBOCAMCAL_SRC__HPP_
#define ROBOCAMCAL_SRC__HPP_

#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <cv_bridge/cv_bridge.h>

#include "robocamcal/CameraData.h"

namespace Robocamcal {

namespace ImageType {
const std::string RGB = "rgb";
const std::string Depth = "depth";
}

class CameraBridge {
 public:
  CameraBridge(const std::string &source) {
    from_ros_service_ = source.find('.') == std::string::npos;
    if (from_ros_service_) {
      auto pos = source.find(':');
      auto service_name = source.substr(0, pos);
      req_.camera_id = source.substr(pos+1, source.length());
      std::cout << "input source: camera " << req_.camera_id << " from ROS "
                << service_name << std::endl;
      rs_client_ = nh_.serviceClient<robocamcal::CameraData>(service_name);
    } else {
      cap_.open(source);
      std::cout << "input source: " << source << std::endl;
    }
  }

  bool Update() {
    if (from_ros_service_) {
      req_.cmd = "update";
      if (rs_client_.call(req_, res_)) {
        return res_.success;
      }
      return false;
    } else {
      return cap_.grab();
    }
  }

  bool FetchFrame(cv::Mat &frame, const std::string &type = ImageType::RGB) {
    if (from_ros_service_) {
      req_.cmd = "get_" + type;
      if (rs_client_.call(req_, res_) && res_.success) {
        auto cv_ptr = cv_bridge::toCvCopy(res_.image);
        frame = cv_ptr->image;
        return res_.success;
      }
      return false;
    } else {
      cap_.retrieve(frame);
      return true;
    }
  }

 private:
  bool from_ros_service_;
  cv::VideoCapture cap_;
  ros::NodeHandle nh_;
  ros::ServiceClient rs_client_;
  robocamcal::CameraDataRequest req_;
  robocamcal::CameraDataResponse res_;
};

static bool grab_frame(const std::unique_ptr<CameraBridge> &bridge, cv::Mat &frame) {
  if (bridge->Update()) {
    return bridge->FetchFrame(frame);
  }
  return false;
}

}

#endif //ROBOCAMCAL_SRC_HPP_
