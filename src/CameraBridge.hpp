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
  CameraBridge(const std::string &service_name) {
    rs_client_ = nh_.serviceClient<robocamcal::CameraData>(service_name);
  }

  bool Update(const std::string &camera_id) {
    req_.camera_id = camera_id;
    req_.cmd = "update";
    if (rs_client_.call(req_, res_)) {
      return res_.success;
    }
    return false;
  }

  bool FetchFrame(const std::string &camera_id, const std::string &type, cv::Mat &frame) {
    req_.camera_id = camera_id;
    req_.cmd = "get_" + type;
    if (rs_client_.call(req_, res_) && res_.success) {
      auto cv_ptr = cv_bridge::toCvCopy(res_.image);
      frame = cv_ptr->image;
      return res_.success;
    }
    return false;
  }

 private:
  ros::NodeHandle nh_;
  ros::ServiceClient rs_client_;
  robocamcal::CameraDataRequest req_;
  robocamcal::CameraDataResponse res_;
};

}

#endif //ROBOCAMCAL_SRC_HPP_
