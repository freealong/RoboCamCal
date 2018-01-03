//
// Created by yongqi on 17-12-12.
//

#ifndef ROBOCAMCAL_VISPUTILS_HPP
#define ROBOCAMCAL_VISPUTILS_HPP

#include <visp/vpHomogeneousMatrix.h>

namespace Robocamcal {
namespace Utils {

/**
 * Convert pose(in Rz(yaw)Ry(pitch)Rx(roll) order) into transformation matrix
 * @tparam T
 * @param x
 * @param y
 * @param z
 * @param roll
 * @param pitch
 * @param yaw
 * @return
 */
template <typename T>
vpHomogeneousMatrix XYZRPY2vpHM(T x, T y, T z, T roll, T pitch, T yaw) {
  vpHomogeneousMatrix t;
  T A = cos(yaw),  B = sin(yaw),  C = cos(pitch), D = sin (pitch),
      E = cos(roll), F = sin(roll), DE = D*E,       DF = D*F;

  t[0][0] = A*C;  t[0][1] = A*DF - B*E;  t[0][2] = B*F + A*DE;  t[0][3] = x;
  t[1][0] = B*C;  t[1][1] = A*E + B*DF;  t[1][2] = B*DE - A*F;  t[1][3] = y;
  t[2][0] = -D;   t[2][1] = C*F;         t[2][2] = C*E;         t[2][3] = z;
  t[3][0] = 0;    t[3][1] = 0;           t[3][2] = 0;           t[3][3] = 1;
  return t;
}

/**
 * Convert transformation matrix into pose
 * @tparam T
 * @param matrix
 * @param x
 * @param y
 * @param z
 * @param roll
 * @param pitch
 * @param yaw
 */
template<typename T>
void matrix2pose(const vpHomogeneousMatrix &t,
                 T &x, T &y, T &z,
                 T &roll, T &pitch, T &yaw) {
  x = t[0][3];
  y = t[1][3];
  z = t[2][3];
  roll = atan2(t[2][1], t[2][2]);
  pitch = atan2(-t[2][0], sqrt(t[2][1] * t[2][1] + t[2][2] * t[2][2]));
  yaw = atan2(t[1][0], t[0][0]);
}

static std::istream& operator>>(std::istream &in, vpHomogeneousMatrix &t) {
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 4; ++x) {
      in >> t[y][x];
      if (!in) { // input broken
        std::cerr << "read vpHomogeneousMatrix failed, set vpHomogeneousMatrix to eye" << std::endl;
        t.eye();
        return in;
      }
    }
  return in;
}

}
}
#endif //ROBOCAMCAL_VISPUTILS_HPP
