//
// Created by yongqi on 18-1-3.
//

#ifndef ROBOCAMCAL_CALIBRATIONCONTROLLER_HPP
#define ROBOCAMCAL_CALIBRATIONCONTROLLER_HPP

#include <memory>
#include "CalibrationCommon.hpp"
#include "BoardDetector.hpp"

namespace Robocamcal {

class CalibrationController {
 public:
  /**
   * Feed a new data form current frame
   * @param detector
   */
  virtual void FeedData(const BoardDetector &detector) = 0;

  /**
   * Drop last feed data
   */
  virtual void DropData() = 0;

  /**
   * Run calibration
   * @return if calibrated successfully
   */
  virtual bool Calibrate() = 0;
};

class CameraCalibController : public CalibrationController {
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

#endif //ROBOCAMCAL_CALIBRATIONCONTROLLER_HPP
