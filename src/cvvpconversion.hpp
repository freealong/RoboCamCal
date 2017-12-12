//
// Created by yongqi on 17-12-12.
//

#ifndef ROBOCAMCAL_CVVPCONVERSION_HPP
#define ROBOCAMCAL_CVVPCONVERSION_HPP

#include <visp/vpHomogeneousMatrix.h>
#include <opencv2/opencv.hpp>

namespace Robocamcal {
namespace Utils {

/**
 * Convert vpHomogeneousMatrix to CV 4*4 double Mat
 * @param t
 * @return
 */
static cv::Mat vpHM2cvMat(const vpHomogeneousMatrix &t) {
  cv::Mat m(t.getRows(), t.getCols(), CV_64F);
  for (int y = 0; y < m.rows; ++y)
    for (int x = 0; x < m.cols; ++x)
      m.at<double>(y, x) = t[y][x];
  return m;
}

}
}
#endif //ROBOCAMCAL_CVVPCONVERSION_HPP
