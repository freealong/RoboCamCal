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
enum class BoardType {ChessBoard, AcirclesGrid, CharucoBoard, GridBoard};

// camera calibration results
struct CameraCalibResults {
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  cv::Size image_size;
  double rsm_error;
  bool valid;

  cv::Mat map1, map2;

  CameraCalibResults() : image_size(0, 0), valid(false) {}
  void print();
  void write(std::string s);
  bool read(std::string s);
};

// camera calibration data
struct CameraCalibData {
  std::vector<std::vector<cv::Point2f>> image_points;
  std::vector<std::vector<cv::Point3f>> object_points;
  size_t size() {
    return image_points.size();
  }
};

// stereo calibration results
struct StereoCalibResults {
  CameraCalibResults left_res, right_res;
  cv::Mat R, T;
  cv::Mat E, F;
  cv::Mat R1, P1, R2, P2;
  cv::Mat Q;
  double rsm_error;
  bool valid;

  StereoCalibResults() : valid(false) {}
  void print();
  void write(std::string s);
  bool read(std::string s);
};

// stereo calibration data
struct StereoCalibData {
  CameraCalibData left_data;
  CameraCalibData right_data;
  size_t size() {
    return left_data.size();
  }
};

}

#endif //ROBOCAMCAL_CALIBCOMMON_HPP
