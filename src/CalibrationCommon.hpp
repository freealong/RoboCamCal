//
// Created by yongqi on 18-1-3.
//

#ifndef ROBOCAMCAL_CALIBRATIONCOMMON_HPP
#define ROBOCAMCAL_CALIBRATIONCOMMON_HPP

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
  void Print() {
    if (!valid) {
      std::cerr << "Results print failed. Please press r to run calibration first." << std::endl;
      return;
    }
    std::cout << "Calibration Result:" << std::endl;
    std::cout << "fx: " << camera_matrix.at<double>(0, 0) << " "
              << "cx: " << camera_matrix.at<double>(0, 2) << std::endl;
    std::cout << "fy: " << camera_matrix.at<double>(1, 1) << " "
              << "cy: " << camera_matrix.at<double>(1, 2) << std::endl;
    std::cout << "dist coeffs: " << dist_coeffs << std::endl;
    std::cout << "rsm error: " << rsm_error << std::endl;
    std::cout << std::endl;
  }
  void Save(std::string s) {
    if (!valid) {
      std::cerr << "Results save failed. Please press r to run calibration first." << std::endl;
      return;
    }
    cv::FileStorage fs(s, cv::FileStorage::WRITE);
    fs << "width" << image_size.width;
    fs << "height" << image_size.height;
    fs << "K" << camera_matrix;
    fs << "D" << dist_coeffs;
    fs << "rms" << rsm_error;
    std::cout << "Save calibration results to " << s << " successfully" << std::endl;
  }
};

// camera calibration data
struct CameraCalibData {
  std::vector<std::vector<cv::Point2f>> image_points;
  std::vector<std::vector<cv::Point3f>> object_points;
};

inline void grab_frame(cv::VideoCapture &capture, cv::Mat &frame) {
  if (capture.grab())
    capture.retrieve(frame);
  else
    std::cerr << "Grab frame failed, already reach the end of the Frames" << std::endl;
}

}

#endif //ROBOCAMCAL_CALIBRATIONCOMMON_HPP
