/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <fstream>
#include "CVUtils.hpp"

using namespace std;
using namespace cv;

namespace {
const char *about = "Basic marker detection";
const char *keys =
    "{d        |              | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
                                           "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5,"
                                           "DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8,"
                                           "DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11,"
                                           "DICT_7X7_50=12, DICT_7X7_100=13, DICT_7X7_250=14,"
                                           "DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
    "{v        |              | Input from video file, if ommited, input comes from camera }"
    "{ci       | 0            | Camera id if input doesnt come from video (-v) }"
    "{c        |              | Camera intrinsic parameters. Needed for camera pose }"
    "{l        | 0.1          | Marker side lenght (in meters). Needed for correct scale in camera pose }"
    "{dp       |              | File of marker detector parameters }"
    "{r        |              | show rejected candidates too }"
    "{s        |              | write camera pose to cMo.txt }"
    "{in       | hand_eye.yml | hand eye calibration results}";
}

/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
  fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
  fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
  fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
  fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
  fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
  fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
  fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
  fs["minDistanceToBorder"] >> params->minDistanceToBorder;
  fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
  fs["doCornerRefinement"] >> params->doCornerRefinement;
  fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
  fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
  fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
  fs["markerBorderBits"] >> params->markerBorderBits;
  fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
  fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
  fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
  fs["minOtsuStdDev"] >> params->minOtsuStdDev;
  fs["errorCorrectionRate"] >> params->errorCorrectionRate;
  return true;
}

/**
 */
int main(int argc, char *argv[]) {
  CommandLineParser parser(argc, argv, keys);
  parser.about(about);

  if (argc < 2) {
    parser.printMessage();
    return 0;
  }

  int dictionaryId = parser.get<int>("d");
  bool showRejected = parser.has("r");
  bool estimatePose = parser.has("c");
  float markerLength = parser.get<float>("l");

  Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
  if (parser.has("dp")) {
    bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
    if (!readOk) {
      cerr << "Invalid detector parameters file" << endl;
      return 0;
    }
  }
  detectorParams->doCornerRefinement = true; // do corner refinement in markers

  // open cMo file to save camera pose
  bool saveCameraPose = parser.has("s");
  ofstream cMo_file;
  if (saveCameraPose) {
    cMo_file.open("cMo.txt");
    if (!cMo_file.is_open()) {
      cout << "open save file failed" << endl;
      return -1;
    }

  }

  // get rMc matrix
  string hand_eye_filename = parser.get<string>("in");
  FileStorage fs(hand_eye_filename, FileStorage::READ);
  Mat rMc;
  fs["rMc"] >> rMc;

  int camId = parser.get<int>("ci");
  String video;
  if (parser.has("v")) {
    video = parser.get<String>("v");
  }

  if (!parser.check()) {
    parser.printErrors();
    return 0;
  }

  Ptr<aruco::Dictionary> dictionary =
      aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

  Mat camMatrix, distCoeffs;
  if (estimatePose) {
    bool readOk = Robocamcal::Utils::read_camera_params(parser.get<string>("c"), camMatrix, distCoeffs);
    if (!readOk) {
      cerr << "Invalid camera file" << endl;
      return 0;
    }
  }

  VideoCapture inputVideo;
  int waitTime;
  if (!video.empty()) {
    inputVideo.open(video);
    waitTime = 0;
  } else {
    inputVideo.open(camId);
    waitTime = 10;
    for (int i = 0; i < 30; ++i) {
      inputVideo.grab();
    }
  }

  double totalTime = 0;
  int totalIterations = 0;

  cout << "begin to loop" << endl;
  while (inputVideo.grab()) {
    Mat image, imageCopy;
    inputVideo.retrieve(image);

    double tick = (double) getTickCount();

    vector<int> ids;
    vector<vector<Point2f> > corners, rejected;
    vector<Vec3d> rvecs, tvecs;

    // detect markers and estimate pose
    aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
    if (estimatePose && ids.size() > 0)
      aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
                                       tvecs);

//    double currentTime = ((double) getTickCount() - tick) / getTickFrequency();
//    totalTime += currentTime;
//    totalIterations++;
//    if (totalIterations % 30 == 0) {
//      cout << "Detection Time = " << currentTime * 1000 << " ms "
//           << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
//    }

    // draw results
    image.copyTo(imageCopy);
    if (ids.size() > 0) {
      aruco::drawDetectedMarkers(imageCopy, corners, ids);

      if (estimatePose) {
        for (unsigned int i = 0; i < ids.size(); i++) {
          aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                          markerLength * 0.5f);
        }
      }
    }

    if (showRejected && rejected.size() > 0)
      aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

    imshow("out", imageCopy);
    char key = (char) waitKey(waitTime);
    if (key == 27) break;

    // save tf to file
    bool validPose;
    Vec3d rvec, tvec;
    if (ids.size() == 1) {
      rvec = rvecs[0];
      tvec = tvecs[0];
      validPose = true;
    }
    else {
      validPose = false;
    }
    Mat rmat;
    Rodrigues(rvec, rmat);
    Mat cMo = Mat::eye(4, 4, CV_64F);
    rmat.copyTo(cMo(Rect(0, 0, 3, 3)));
    cMo.at<double>(0, 3) = tvec(0);
    cMo.at<double>(1, 3) = tvec(1);
    cMo.at<double>(2, 3) = tvec(2);
    if (validPose && key == 115) {
      cout << cMo << endl;
      if (saveCameraPose) {
        for (int y = 0; y < cMo.rows; ++y) {
          cMo_file << cMo.at<double>(y, 0) << " " << cMo.at<double>(y, 1) << " "
                   << cMo.at<double>(y, 2) << " " << cMo.at<double>(y, 3) << endl;
        }
        cout << "save tf ok" << endl;
      }
    }

    // calculate pose based on rMc
    if (validPose && key == 116) {
      Mat rMo = rMc * cMo;
      double x, y, z, roll, pitch, yaw;
      Robocamcal::Utils::mat2pose(rMo, x, y, z, roll, pitch, yaw);
      cout << "target pose in robot coordinate:" << endl;
      cout << "X: " << x * 1000 << ", Y: " << y * 1000 << ", Z: " << z * 1000
           << ", r: " << roll * 180 / M_PI << ", p: " << pitch * 180 / M_PI << ", y: " << yaw * 180 / M_PI << endl;
    }
  }

  return 0;
}
