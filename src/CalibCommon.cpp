//
// Created by yongqi on 18-1-3.
//

#include "CalibCommon.hpp"

namespace Robocamcal {

void CameraCalibResults::Print() {
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

void CameraCalibResults::Save(std::string s) {
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

}
