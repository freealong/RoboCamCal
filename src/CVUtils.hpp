//
// Created by yongqi on 17-12-12.
//

#ifndef ROBOCAMCAL_CVUTILS_HPP
#define ROBOCAMCAL_CVUTILS_HPP

#include <opencv2/opencv.hpp>

namespace Robocamcal {
namespace Utils {

/**
 * Read camMatrix and distCoeffs from yml file
 * @param filename
 * @param camMatrix
 * @param distCoeffs
 * @return
 */
static bool read_camera_params(std::string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["r_intrin"] >> camMatrix;
  fs["r_coeffs"] >> distCoeffs;
  return true;
}

template<typename T>
static void mat2pose(const cv::Mat &t, T &x, T &y, T &z, T &roll, T &pitch, T &yaw) {
  x = t.at<double>(0, 3);
  y = t.at<double>(1, 3);
  z = t.at<double>(2, 3);
  roll = atan2(t.at<double>(2, 1), t.at<double>(2, 2));
  pitch = atan2(-t.at<double>(2, 0),
                sqrt(t.at<double>(2, 1) * t.at<double>(2, 1) + t.at<double>(2, 2) * t.at<double>(2, 2)));
  yaw = atan2(t.at<double>(1, 0), t.at<double>(0, 0));
}

inline void grab_frame(cv::VideoCapture &capture, cv::Mat &frame) {
  if (capture.grab())
    capture.retrieve(frame);
  else
    std::cerr << "Grab frame failed, already reach the end of the Frames" << std::endl;
}

void draw_lines(cv::Mat &frame, bool vertical, int step = 20) {
  if (vertical) {
    for (int x = step; x < frame.cols; x += step)
      cv::line(frame, {0, x}, {frame.rows, x}, {0, 255, 0});
  }
  else {
    for (int y = step; y < frame.rows; y += step)
      cv::line(frame, {y, 0}, {y, frame.cols}, {0, 255, 0});
  }
}

}
}
#endif //ROBOCAMCAL_CVUTILS_HPP
