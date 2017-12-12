//
// Created by yongqi on 17-12-12.
//

#ifndef ROBOCAMCAL_MATHUTILS_HPP
#define ROBOCAMCAL_MATHUTILS_HPP

#include <cmath>

namespace Robocamcal {
namespace Utils {

/**
 * make angle between [-pi, pi)
 * @param x input angle in radius
 * @return normalized angle
 */
inline double normalize_angle(double x) {
  x = fmod(x + M_PI, 2 * M_PI);
  return x < 0 ? x += 2 * M_PI : x - M_PI;
}

/**
 * radian to degree
 * @tparam T
 * @param x
 * @return
 */
template<typename T>
inline T rad2deg(T x) {
  return x / M_PI * 180.;
}

/**
 * degree to radian
 * @tparam T
 * @param x
 * @return
 */
template<typename T>
inline T deg2rad(T x) {
  return x * M_PI / 180.;
}

}
}
#endif //ROBOCAMCAL_MATHUTILS_HPP
