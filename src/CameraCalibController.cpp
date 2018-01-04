//
// Created by yongqi on 18-1-4.
//

#include "CameraCalibController.hpp"

namespace Robocamcal {

CameraCalibController::CameraCalibController(std::shared_ptr<CameraCalibData> data,
                                             std::shared_ptr<CameraCalibResults> results,
                                             std::shared_ptr<BoardDetector> detector) :
    data_(data), results_(results), detector_(detector) {
}

void CameraCalibController::FeedData() {
  auto image_points = detector_->GetImagePoints();
  auto object_points = detector_->GetObjectPoints();
  if (image_points.size() > 0 && object_points.size() > 0) {
    data_->image_points.emplace_back(image_points);
    data_->object_points.emplace_back(object_points);
    std::cout << "Feed data successfully, current data size: " << data_->size() << std::endl;
  }
  else
    std::cerr << "Feed data failed, no board detected" << std::endl;
}

void CameraCalibController::DropData() {
  if (data_->size() == 0)
    std::cerr << "Drop data failed, current data size already equal to zero" << std::endl;
  else {
    data_->image_points.pop_back();
    data_->object_points.pop_back();
    std::cout << "Drop data successfully, current data size: " << data_->size() << std::endl;
  }
}

bool CameraCalibController::Calibrate() {
  auto data_size = data_->size();
  if (data_size < 5) {
    std::cerr << "Calibration failed, too few data to run calibration, please press s to feed more data" << std::endl;
    return (results_->valid = false);
  }
  if (results_->image_size.width <= 0 || results_->image_size.height <= 0)
    throw std::invalid_argument("Invalid image size");
  std::cout << "Begin to run camera calibration with " << data_size << " data......" << std::endl;
  results_->rsm_error = cv::calibrateCamera(data_->object_points, data_->image_points,
                                            results_->image_size, results_->camera_matrix,
                                            results_->dist_coeffs, cv::noArray(), cv::noArray(),
                                            cv::noArray(), cv::noArray(), cv::noArray());
  return (results_->valid = true);
}

}