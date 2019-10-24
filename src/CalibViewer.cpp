//
// Created by yongqi on 18-1-3.
//

#include "CalibViewer.hpp"

namespace Robocamcal {

CalibrationStatus CalibViewer::Update() {
  for (int i = 0; i < names_.size(); ++i) {
    cv::imshow(names_[i], images_[i]);
  }
  int key = cv::waitKey(delay_);
  if (key == 27 || key == 'q' || key == 'Q') // escape and q
    status_ = CalibrationStatus::Finished;
  else if (key == 'r' || key == 'R')
    status_ = CalibrationStatus::Calibration;
  else if (key == 's' || key == 'S')
    status_ = CalibrationStatus::SaveCurrentData;
  else if (key == 'd' || key == 'D')
    status_ = CalibrationStatus::DropLastData;
  else if (key == 'u' || key == 'U')
    status_ = CalibrationStatus::SwitchUndistort;
  else if (key == 'v' || key == 'V')
    status_ = CalibrationStatus::SwitchVisualisation;
  else if (key == 'n' || key == 'N')
    status_ = CalibrationStatus::NextFrame;
  else if (key == 'w' || key == 'W')
    status_ = CalibrationStatus::WriteResult;
  else if (key == 'p' || key == 'P')
    status_ = CalibrationStatus::SwitchPlayMode;
  else
    status_ = CalibrationStatus::None;

  // self control
  if (key == 32) // space
//    delay_ = delay_ == 0 ? 10 : 0;
    delay_ = 10;
  else if (key == 'h' || key == 'H')
    PrintHelp();

  names_.clear();
  images_.clear();
  return status_;
}

void CalibViewer::PrintHelp() {
  std::cout << "Press h to show this message\n"
            << "ESC or q to quit\n"
            << "r to run calibration\n"
            << "s to save current frame data\n"
            << "d to drop last saved data\n"
            << "u to switch show undistort/distort image\n"
            << "v to switch mask visualisation\n"
            << "n to load next frame\n"
            << "w to write calibration results to file\n"
            << "p to switch play mode: auto grab next frame or not\n"
//            << "space to pause/continue\n"
            << std::endl;
}

}