//
// Created by yongqi on 17-12-12.
//

#ifndef ROBOCAMCAL_CVVPCONVERSION_HPP
#define ROBOCAMCAL_CVVPCONVERSION_HPP

#include <visp/vpHomogeneousMatrix.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace Robocamcal {
namespace Utils {

/**
 * Convert vpHomogeneousMatrix to CV 4*4 double Mat
 * @param t
 * @return
 */
static cv::Mat Convert2CV(const vpHomogeneousMatrix &t) {
  cv::Mat m(t.getRows(), t.getCols(), CV_64F);
  for (int y = 0; y < m.rows; ++y)
    for (int x = 0; x < m.cols; ++x)
      m.at<double>(y, x) = t[y][x];
  return m;
}

/**
 * Convert opencv's pose(rvec, tvec) to Eigen::Isometry
 * @tparam T
 * @param rvec
 * @param tvec
 * @return
 */
template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> Convert2Eigen(const cv::Vec<T, 3> &rvec,
    const cv::Vec<T, 3> &tvec) {
  Eigen::Transform<T, 3, Eigen::Isometry> tf;
  // @TODO:
  return tf;
}

/**
 * Convert visp's vpHomogeneousMatrix to Eigen::Isometry
 * @tparam T
 * @param t
 * @return
 */
template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> Convert2Eigen(const vpArray2D<T> &t) {
  Eigen::Transform<T, 3, Eigen::Isometry> tf;
  // @TODO:
  return tf;
}

/**
 * Convert Eigen::Isometry to visp's vpHomogeneousMatrix
 * @tparam T
 * @param tf
 * @return
 */
template <typename T>
vpHomogeneousMatrix Convert2Visp(const Eigen::Transform<T, 3, Eigen::Isometry> &tf) {
  vpHomogeneousMatrix vp_tf;
  // @TODO:
  return vp_tf;
}

}
}
#endif //ROBOCAMCAL_CVVPCONVERSION_HPP
