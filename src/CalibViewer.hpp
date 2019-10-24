//
// Created by yongqi on 18-1-3.
//

#ifndef ROBOCAMCAL_CALIBVIEWER_HPP
#define ROBOCAMCAL_CALIBVIEWER_HPP

#include <opencv2/opencv.hpp>

namespace Robocamcal {

enum class CalibrationStatus {
  Finished,
  Calibration,
  SaveCurrentData,
  DropLastData,
  SwitchUndistort,
  SwitchVisualisation,
  NextFrame,
  WriteResult,
  SwitchPlayMode,
  None
};

class CalibViewer {
 public:
  CalibrationStatus Update();

  void AddImage(std::string name, cv::Mat &image) {
    names_.emplace_back(name);
    images_.emplace_back(image);
  }

  static void PrintHelp();

 private:
  int delay_ = 10;
  CalibrationStatus status_;
  std::vector<std::string> names_;
  std::vector<cv::Mat> images_;
};

}

#endif //ROBOCAMCAL_CALIBVIEWER_HPP
