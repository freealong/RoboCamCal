//
// Created by yongqi on 18-1-3.
//

#include "FrameCapturer.hpp"

namespace Robocamcal {

bool FrameCapturer::Open(std::vector<std::string> filenames) {
  for (auto filename : filenames) {
    cv::VideoCapture capture;
    if (!capture.open(filename))
      return false;
    captures_.emplace_back(capture);
  }
  return true;
}

bool FrameCapturer::Open(std::vector<int> ids) {
  for (auto id : ids) {
    cv::VideoCapture capture;
    if (!capture.open(id))
      return false;
    captures_.emplace_back(capture);
  }
  return true;
}

std::vector<cv::Mat> FrameCapturer::GetFrames() {
  std::vector<cv::Mat> frames;
  for (auto &capture : captures_) {
    if (capture.grab()) {
      cv::Mat frame;
      capture.retrieve(frame);
      frames.emplace_back(frame);
    }
    else {
      std::cerr << "Get frames failed, already reach the end of the Frames" << std::endl;
      frames.clear();
      break;
    }
  }
  return frames;
}

}
