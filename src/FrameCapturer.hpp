//
// Created by yongqi on 18-1-3.
//

#ifndef ROBOCAMCAL_FRAMECAPTURER_HPP
#define ROBOCAMCAL_FRAMECAPTURER_HPP

#include <opencv2/opencv.hpp>

namespace Robocamcal {

class FrameCapturer {
 public:
  bool Open(std::string filename) {
    cv::VideoCapture capture;
    if (!capture.open(filename))
      return false;
    captures_.clear();
    captures_.emplace_back(capture);
    return true;
  }

  bool Open(int id) {
    cv::VideoCapture capture;
    if (!capture.open(id))
      return false;
    captures_.clear();
    captures_.emplace_back(capture);
    return true;
  }

  bool Open(std::vector<std::string> filenames);

  bool Open(std::vector<int> ids);

  std::vector<cv::Mat> GetFrames();

 private:
  std::vector<cv::VideoCapture> captures_;
};

}

#endif //ROBOCAMCAL_FRAMECAPTURER_HPP
