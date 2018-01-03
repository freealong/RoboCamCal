//
// Created by yongqi on 18-1-3.
//

#include "CalibrationController.hpp"
#include "CalibrationCommon.hpp"

namespace Robocamcal {

CameraCalibController::CameraCalibController(std::shared_ptr<CameraCalibData> data,
                                             std::shared_ptr<CameraCalibResults> results) :
    data_(data), results_(results) {
}

void CameraCalibController::FeedData(const BoardDetector &detector) {
  auto image_points = detector.GetImagePoints();
  auto object_points = detector.GetObjectPoints();
  if (image_points.size() > 0 && object_points.size() > 0) {
    data_->image_points.emplace_back(detector.GetImagePoints());
    data_->object_points.emplace_back(detector.GetObjectPoints());
    std::cout << "Feed data successfully, current data size: " << data_->image_points.size() << std::endl;
  }
  else
    std::cerr << "Feed data failed, no board detected" << std::endl;
}

void CameraCalibController::DropData() {
  if (data_->image_points.empty())
    std::cerr << "Drop data failed, current data size already equal to zero" << std::endl;
  else {
    data_->image_points.pop_back();
    data_->object_points.pop_back();
    std::cout << "Drop data successfully, current data size: " << data_->image_points.size() << std::endl;
  }
}

bool CameraCalibController::Calibrate() {
  auto data_size = data_->image_points.size();
  if (data_size < 5) {
    std::cerr << "Calibration failed, too few data to run calibration, please press s to feed more data" << std::endl;
    return false;
  }
  if (results_->image_size.width <= 0 || results_->image_size.height <= 0)
    throw std::invalid_argument("Invalid image size");
  std::cout << "Begin to calibration with " << data_size << " data......" << std::endl;
  results_->rsm_error = cv::calibrateCamera(data_->object_points, data_->image_points,
                                           results_->image_size, results_->camera_matrix,
                                           results_->dist_coeffs, cv::noArray(), cv::noArray(),
                                           cv::noArray(), cv::noArray(), cv::noArray());
  results_->valid = true;
  return true;
}

}
