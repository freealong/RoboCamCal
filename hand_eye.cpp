//
// Created by yongqi on 17-12-11.
//

#include <iostream>
#include <fstream>
#include <gflags/gflags.h>
#include <visp/vpCalibration.h>
#include "visputils.hpp"
#include "mathutils.hpp"
#include "cvvpconversion.hpp"

DEFINE_string(cMo_filename, "cMo.txt", "name of the file which stores camera pose");
DEFINE_string(rMe_filename, "rMe.txt", "name of the file which stores robot pose");
DEFINE_string(output_filename, "hand_eye.yml", "name of the file which stores calibration results");
DEFINE_bool(eye_in_hand, false, "camera fixed on the robot end effector");

using namespace std;
using namespace Robocamcal;
using namespace Utils;

/**
 * load data from file to vector of vpHomogeneousMatrix
 * @param filename
 * @param vec_m
 * @return success
 */
bool prepare_data_from_file(string filename, vector<vpHomogeneousMatrix> &vec_m) {
  ifstream fs(filename);
  if (!fs.is_open()) {
    cerr << "open " << filename << " failed" << endl;
    return false;
  }
  int type, num;
  fs >> type >> num;
  for (int i = 0; i < num; ++i) {
    if (type == 0) { // 4*4 matrix
      vpHomogeneousMatrix m;
      fs >> m;
      vec_m.emplace_back(m);
    }
    else if (type == 1) {
      double x, y, z, roll, pitch, yaw;
      fs >> x >> y >> z >> roll >> pitch >> yaw;
      // @ATTENTION: unit matters
//      vec_m.emplace_back(Utils::XYZRPY2vpHM(x, y, z, roll, pitch, yaw));
      vec_m.emplace_back(Utils::XYZRPY2vpHM(x * 0.001, y * 0.001, z * 0.001,
                                            deg2rad(roll), deg2rad(pitch), deg2rad(yaw)));
    }
    else {
      cerr << "Unrecognized type: " << type << endl;
      return false;
    }
  }
  cout << "Load " << num << " matrixs from " << filename << endl;
  return true;
}

int main(int argc, char ** argv) {
  // prepare data
  vector<vpHomogeneousMatrix> vec_cMo, vec_rMe;
  if (!prepare_data_from_file(FLAGS_cMo_filename, vec_cMo)) {
    cerr << "prepare data from " << FLAGS_cMo_filename << " failed" << endl;
    return -1;
  }
  if (!prepare_data_from_file(FLAGS_rMe_filename, vec_rMe)) {
    cerr << "prepare data from " << FLAGS_rMe_filename << " failed" << endl;
    return -1;
  }
  // run hand-eye calibration
  cv::FileStorage fs(FLAGS_output_filename, cv::FileStorage::WRITE);
  if (FLAGS_eye_in_hand) {
    vpHomogeneousMatrix eMc;
    vpCalibration::calibrationTsai(vec_cMo, vec_rMe, eMc);
    cout << "Output: eye in hand calibration result: eMc estimated:\n" << eMc << endl;
    fs << "eMc" << vpHM2cvMat(eMc);
    cout << "save eMc to " << FLAGS_output_filename << endl;
  }
  else {
    for (auto &m : vec_rMe)
      m = m.inverse();
    vpHomogeneousMatrix rMc;
    vpCalibration::calibrationTsai(vec_cMo, vec_rMe, rMc);
    cout << "Output: eye to hand calibration result: rMc estimated:\n" << rMc << endl;
    fs << "rMc" << vpHM2cvMat(rMc);
    cout << "save rMc to " << FLAGS_output_filename << endl;
  }
  return 0;
}