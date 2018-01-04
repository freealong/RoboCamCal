//
// Created by yongqi on 18-1-4.
//

#ifndef ROBOCAMCAL_CAMERACALIBCONTROLLER_HPP
#define ROBOCAMCAL_CAMERACALIBCONTROLLER_HPP

#include "CalibController.hpp"

namespace Robocamcal {

class CameraCalibController : public CalibController {
 public:
  CameraCalibController(std::shared_ptr<CameraCalibData> data, std::shared_ptr<CameraCalibResults> results);

  virtual void FeedData(const BoardDetector &detector);

  virtual void DropData();

  virtual bool Calibrate();

 private:
  std::shared_ptr<CameraCalibData> data_;
  std::shared_ptr<CameraCalibResults> results_;
};

}

#endif //ROBOCAMCAL_CAMERACALIBCONTROLLER_HPP
