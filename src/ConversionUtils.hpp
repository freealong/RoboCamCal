//
// Created by yongqi on 17-12-12.
//

#ifndef ROBOCAMCAL_CVVPCONVERSION_HPP
#define ROBOCAMCAL_CVVPCONVERSION_HPP

#include <visp/vpHomogeneousMatrix.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

namespace Robocamcal {

/**
 * Convert opencv's pose(rvec, tvec) to cv::Mat
 * @tparam T
 * @param rvec
 * @param tvec
 * @return
 */
template <typename T>
cv::Mat build_transfromation(const cv::Vec<T, 3> &rvec,
                 const cv::Vec<T, 3> &tvec) {
  cv::Mat mat = cv::Mat::eye(4, 4, cv::traits::Type<T>::value);
  cv::Mat rmat;
  Rodrigues(rvec, rmat);
  rmat.copyTo(mat(cv::Rect(0, 0, 3, 3)));
  mat.at<T>(0, 3) = tvec(0);
  mat.at<T>(1, 3) = tvec(1);
  mat.at<T>(2, 3) = tvec(2);
  return mat;
}

/**
 * Convert vpArray2D to cv::Mat
 * @param t
 * @return
 */
template <typename T>
void vp2cv(const vpArray2D<T> &t, cv::Mat &m) {
  m = cv::Mat(t.getRows(), t.getCols(), cv::traits::Type<T>::value);
  for (int y = 0; y < m.rows; ++y)
    for (int x = 0; x < m.cols; ++x)
      m.at<T>(y, x) = t[y][x];
}

/**
 * Convert visp's vpArray2D to Eigen::Isometry
 * @tparam T
 * @param t
 * @return
 */
template <typename T, int ROW, int COL>
void vp2eigen(const vpArray2D<T> &t, Eigen::Matrix<T, ROW, COL> &m) {
  assert(t.getRows() == ROW && t.getCols() == COL);
  for (auto r = 0; r < t.getRows(); ++r)
    for (auto c = 0; c < t.getCols(); ++c)
      m(r, c) = t[r][c];
}

/**
 * Convert Eigen::Matrix to visp's vpArray2D
 * @tparam T
 * @param tf
 * @return
 */
template <typename T, int ROW, int COL>
void eigen2vp(const Eigen::Matrix<T, ROW, COL> &m, vpArray2D<T> &t) {
  t.resize(ROW, COL);
  for (auto r = 0; r < t.getRows(); ++r)
    for (auto c = 0; c < t.getCols(); ++c)
      t[r][c] = m(r, c);
}

}
#endif //ROBOCAMCAL_CVVPCONVERSION_HPP
