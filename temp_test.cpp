//
// Created by yongqi on 17-12-12.
//

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "VispUtils.hpp"

using namespace std;
using namespace Robocamcal;

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
    case CV_8U: r = "8U";
      break;
    case CV_8S: r = "8S";
      break;
    case CV_16U: r = "16U";
      break;
    case CV_16S: r = "16S";
      break;
    case CV_32S: r = "32S";
      break;
    case CV_32F: r = "32F";
      break;
    case CV_64F: r = "64F";
      break;
    default: r = "User";
      break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}

int main(int argc, char ** argv) {
  cv::Mat img = cv::imread(argv[1]);
  cv::Mat gray = img.clone();
//  cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
  cv::Mat pyr;
  pyrDown(gray, pyr, cv::Size(img.cols / 2, img.rows / 2));
  pyrUp(pyr, gray, gray.size());


  cv::namedWindow("img", 1);
  cv::Mat points;
  if (cv::findCirclesGrid(gray, cv::Size(4,11), points,cv::CALIB_CB_ASYMMETRIC_GRID)) {
    cout << "found" << endl;
    cv::drawChessboardCorners(img, cv::Size(4, 11), points, true);
  }
  cv::imshow("img", img);
  cv::waitKey();
  return 0;
}