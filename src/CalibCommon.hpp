//
// Created by yongqi on 18-1-3.
//

#ifndef ROBOCAMCAL_CALIBCOMMON_HPP
#define ROBOCAMCAL_CALIBCOMMON_HPP

#include <opencv2/opencv.hpp>

namespace Robocamcal {

// supported calibration type
enum class CalibrationType {CameraCalibration, StereoCalibration, HandeyeCalibration};

// supported calibration board type
enum class BoardType {ChessBoard, AcirclesGrid, ArucoBoard};

// camera calibration Results
struct CameraCalibResults {
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  cv::Size image_size;
  double rsm_error;
  bool valid;

  CameraCalibResults() : image_size(0, 0), valid(false) {}
  void Print();
  void Save(std::string s);
};

// camera calibration data
struct CameraCalibData {
  std::vector<std::vector<cv::Point2f>> image_points;
  std::vector<std::vector<cv::Point3f>> object_points;
  size_t size() {
    return image_points.size();
  }
};

}

#endif //ROBOCAMCAL_CALIBCOMMON_HPP
