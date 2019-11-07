//
// Created by yongqi on 18-1-3.
//

#include "CalibCommon.hpp"

namespace Robocamcal {

void CameraCalibResults::print() {
  if (!valid) {
    std::cerr << "Results print failed. Please press r to run calibration first." << std::endl;
    return;
  }
  std::cout << "Camera Calibration Result:" << std::endl;
  std::cout << "fx: " << camera_matrix.at<double>(0, 0) << " "
            << "cx: " << camera_matrix.at<double>(0, 2) << std::endl;
  std::cout << "fy: " << camera_matrix.at<double>(1, 1) << " "
            << "cy: " << camera_matrix.at<double>(1, 2) << std::endl;
  std::cout << "dist coeffs: " << dist_coeffs << std::endl;
  std::cout << "rsm error: " << rsm_error << std::endl;
  std::cout << std::endl;
}

bool CameraCalibResults::write(std::string s) {
  if (!valid) {
    std::cerr << "Results save failed. Please press r to run calibration first." << std::endl;
    return false;
  }
  cv::FileStorage fs(s, cv::FileStorage::WRITE);
  fs << "width" << image_size.width;
  fs << "height" << image_size.height;
  fs << "K" << camera_matrix;
  fs << "D" << dist_coeffs;
  fs << "rms" << rsm_error;
  std::cout << "write calibration results to " << s << " successfully" << std::endl;
  return true;
}

bool CameraCalibResults::read(std::string s) {
  cv::FileStorage fs(s, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Read file failed: " << s << std::endl;
    return (valid = false);
  }
  fs["width"] >> image_size.width;
  fs["height"] >> image_size.height;
  fs["K"] >> camera_matrix;
  fs["D"] >> dist_coeffs;
  fs["rms"] >> rsm_error;
  std::cout << "read camera params from " << s << " successfully" << std::endl;
  return (valid = true);
}

void StereoCalibResults::print() {
  if (!valid) {
    std::cerr << "Results print failed. Please press r to run calibration first." << std::endl;
    return;
  }
  std::cout << "Stereo Calibration Result:" << std::endl;
  std::cout << "R: " << R << std::endl;
  std::cout << "T: " << T << std::endl;
  std::cout << "E: " << E << std::endl;
  std::cout << "F: " << F << std::endl;
  std::cout << "R1: " << R1 << std::endl;
  std::cout << "R2: " << R2 << std::endl;
  std::cout << "P1: " << P1 << std::endl;
  std::cout << "P2: " << P2 << std::endl;
  std::cout << "Q: " << Q << std::endl;
  std::cout << "rsm error: " << rsm_error << std::endl;
  std::cout << std::endl;
}

bool StereoCalibResults::write(std::string s) {
  if (!valid) {
    std::cerr << "Results save failed. Please press r to run calibration first." << std::endl;
    return false;
  }
  cv::FileStorage fs(s, cv::FileStorage::WRITE);
  fs << "R" << R << "T" << T;
  fs << "E" << E << "F" << F;
  fs << "R1" << R1 << "R2" << R2;
  fs << "P1" << P1 << "P2" << P2;
  fs << "Q" << Q;
  fs << "rms" << rsm_error;
  std::cout << "write calibration results to " << s << " successfully" << std::endl;
  return true;
}

bool StereoCalibResults::read(std::string s) {
  cv::FileStorage fs(s, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Read file failed: " << s << std::endl;
    return (valid = false);
  }
  fs["R"] >> R;
  fs["T"] >> T;
  fs["E"] >> E;
  fs["F"] >> F;
  fs["R1"] >> R1;
  fs["R2"] >> R2;
  fs["P1"] >> P1;
  fs["P2"] >> P2;
  fs["Q"] >> Q;
  fs["rms"] >> rsm_error;
  std::cout << "read stereo params from " << s << " successfully" << std::endl;
  return (valid = true);
}

void HandEyeCalibResults::print() {
  if (!valid) {
    std::cerr << "Results print failed. Please press r to run calibration first." << std::endl;
    return;
  }
  std::cout << "HandEye Calibration Result:" << std::endl;
  std::cout << x.matrix() << std::endl;
  std::cout << std::endl;
}

bool HandEyeCalibResults::read(std::string s) {
  std::cerr << "not implement" << std::endl;
  return false;
}

bool HandEyeCalibResults::write(std::string s) {
  std::cerr << "not implement" << std::endl;
  return false;
}

}
