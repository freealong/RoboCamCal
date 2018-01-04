//
// Created by yongqi on 18-1-4.
//

#include "StereoCalibController.hpp"

namespace Robocamcal {

StereoCalibController::StereoCalibController(std::shared_ptr<StereoCalibData> data,
                                             std::shared_ptr<StereoCalibResults> results,
                                             std::shared_ptr<BoardDetector> left_detector,
                                             std::shared_ptr<BoardDetector> right_detector) :
    data_(data), results_(results), left_detector_(left_detector), right_detector_(right_detector ){
}

void StereoCalibController::FeedData() {
  auto left_image_points = left_detector_->GetImagePoints();
  auto left_object_points = left_detector_->GetObjectPoints();
  auto right_image_points = right_detector_->GetImagePoints();
  auto right_object_points = right_detector_->GetObjectPoints();
  bool valid_points = left_image_points.size() > 0 && left_object_points.size() > 0 &&
      right_image_points.size() > 0 && right_object_points.size() > 0;
  if (valid_points) {
    data_->left_data.image_points.emplace_back(left_image_points);
    data_->left_data.object_points.emplace_back(left_object_points);
    data_->right_data.image_points.emplace_back(right_image_points);
    data_->right_data.object_points.emplace_back(right_object_points);
    std::cout << "Feed data successfully, current data size: " << data_->size() << std::endl;
  }
  else
    std::cerr << "Feed data failed, board detected failed on left or right frame" << std::endl;
}

void StereoCalibController::DropData() {
  if (data_->size() == 0)
    std::cerr << "Drop data failed, current data size already equal to zero" << std::endl;
  else {
    data_->left_data.image_points.pop_back();
    data_->left_data.object_points.pop_back();
    data_->right_data.image_points.pop_back();
    data_->right_data.object_points.pop_back();
    std::cout << "Drop data successfully, current data size: " << data_->size() << std::endl;
  }
}

bool StereoCalibController::Calibrate() {
  if (data_->size() < 5) {
    std::cerr << "Calibration failed, too few data to run calibration, please press s to feed more data" << std::endl;
    return (results_->valid = false);
  }
  if (!results_->left_res.valid || !results_->right_res.valid)
    throw std::invalid_argument("Invalid camera calibration results");
  std::cout << "Begin to run stereo calibration with " << data_->size() << " data......" << std::endl;
  results_->rsm_error = cv::stereoCalibrate(data_->left_data.object_points,
                                            data_->left_data.image_points,
                                            data_->right_data.image_points,
                                            results_->left_res.camera_matrix,
                                            results_->left_res.dist_coeffs,
                                            results_->right_res.camera_matrix,
                                            results_->right_res.dist_coeffs,
                                            results_->left_res.image_size,
                                            results_->R, results_->T,
                                            results_->E, results_->F);
  cv::stereoRectify(results_->left_res.camera_matrix, results_->left_res.dist_coeffs,
                    results_->right_res.camera_matrix, results_->right_res.dist_coeffs,
                    results_->left_res.image_size,
                    results_->R, results_->T,
                    results_->R1, results_->R2,
                    results_->P1, results_->P2, results_->Q);
  return (results_->valid = true);
}

}
