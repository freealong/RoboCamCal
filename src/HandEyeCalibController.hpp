/*
 * Copyright (c) XYZ Robotics Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Author: yongqi <Frank.lee@xyzrobotics.ai>, 2019/11/7
 */
#ifndef ROBOCAMCAL_SRC_HANDEYECALIBCONTROLLER_HPP_
#define ROBOCAMCAL_SRC_HANDEYECALIBCONTROLLER_HPP_

#include "CalibController.hpp"
#include "RobotBridge.hpp"

namespace Robocamcal {

class HandEyeCalibController : public CalibController {
 public:
  HandEyeCalibController(std::shared_ptr<HandEyeCalibData> data,
                         std::shared_ptr<HandEyeCalibResults> results,
                         std::shared_ptr<BoardDetector> detector,
                         std::shared_ptr<RobotBridge> robot_bridge);

  virtual void FeedData();

  virtual void DropData();

  virtual bool Calibrate();

 private:
  std::shared_ptr<HandEyeCalibData> data_;
  std::shared_ptr<HandEyeCalibResults> results_;
  std::shared_ptr<BoardDetector> detector_;
  std::shared_ptr<RobotBridge> robot_bridge_;
};

}

#endif //ROBOCAMCAL_SRC_HANDEYECALIBCONTROLLER_HPP_
