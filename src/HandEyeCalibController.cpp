/*
 * Copyright (c) XYZ Robotics Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Author: yongqi <Frank.lee@xyzrobotics.ai>, 2019/11/7
 */

#include <visp/vpCalibration.h>

#include "HandEyeCalibController.hpp"
#include "ConversionUtils.hpp"

namespace Robocamcal {

HandEyeCalibController::HandEyeCalibController(std::shared_ptr<HandEyeCalibData> data,
                                               std::shared_ptr<HandEyeCalibResults> results,
                                               std::shared_ptr<BoardDetector> detector,
                                               std::shared_ptr<RobotBridge> robot_bridge) :
    data_(data), results_(results), detector_(detector), robot_bridge_(robot_bridge) {
}

void HandEyeCalibController::FeedData() {
  Eigen::Isometry3d robot_pose;
  if (!robot_bridge_->GetPose(robot_pose)) {
    std::cerr << "Feed data failed, get robot pose failed" << std::endl;
    return;
  }
  cv::Vec3d rvec, tvec;
  if (!detector_->CalculateBoardPose(results_->intrs.camera_matrix, results_->intrs.dist_coeffs,
      rvec, tvec)) {
    std::cerr << "Feed data failed, calculate board pose failed" << std::endl;
    return;
  }
  data_->robot_pose.emplace_back(robot_pose);
  Eigen::Isometry3d board_pose;
  cv::cv2eigen(build_transfromation(rvec, tvec), board_pose.matrix());
  data_->board_pose.emplace_back(board_pose);
  std::cout << "Feed data successfully, current data size: " << data_->size() << std::endl;
}

void HandEyeCalibController::DropData() {
  if (data_->size() == 0)
    std::cerr << "Drop data failed, current data size already equal to zero" << std::endl;
  else {
    data_->robot_pose.pop_back();
    data_->board_pose.pop_back();
    std::cout << "Drop data successfully, current data size: " << data_->size() << std::endl;
  }
}

bool HandEyeCalibController::Calibrate() {
  auto data_size = data_->size();
  if (data_size < 5) {
    std::cerr << "Calibration failed, too few data to run calibration,"
                 " please press s to feed more data" << std::endl;
    return (results_->valid = false);
  }
  std::vector<vpHomogeneousMatrix> A, B;
  for (const auto &x : data_->board_pose) {
    vpHomogeneousMatrix a;
    eigen2vp(x.matrix(), a);
    A.emplace_back(a);
  }
  for (const auto &x : data_->robot_pose) {
    vpHomogeneousMatrix b;
    eigen2vp(x.matrix(), b);
    B.emplace_back(results_->eye_in_hand ? b : b.inverse());
  }
  vpHomogeneousMatrix X;
  vpCalibration::calibrationTsai(A, B, X);
  vp2eigen(X, results_->x.matrix());
  return (results_->valid = true);
}

}
