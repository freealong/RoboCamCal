/*
 * Copyright (c) XYZ Robotics Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Author: yongqi <Frank.lee@xyzrobotics.ai>, 2019/10/24
 */

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "CameraBridge.hpp"

using namespace Robocamcal;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "camera_bridge_test SERVICE_NAME CAMERA_ID" << std::endl;
    return -1;
  }
  ros::init(argc, argv, "camera_bridge_test");
  std::string camera_id(argv[2]);
  CameraBridge bridge(argv[1]);

  while (true) {
    cv::Mat color;
    if (bridge.Update(camera_id) && bridge.FetchFrame(camera_id, ImageType::RGB, color)) {
      cv::imshow("color", color);
      auto key = cv::waitKey(30);
      if (key == 'q')
        break;
    }
  }

  return 0;
}