//
// Created by yongqi on 18-1-3.
//

#ifndef ROBOCAMCAL_CALIBRATIONCONTROLLER_HPP
#define ROBOCAMCAL_CALIBRATIONCONTROLLER_HPP

#include <memory>
#include "CalibCommon.hpp"
#include "BoardDetector.hpp"

namespace Robocamcal {

class CalibController {
 public:
  /**
   * Feed a new data form current frame
   */
  virtual void FeedData() = 0;

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

}

#endif //ROBOCAMCAL_CALIBRATIONCONTROLLER_HPP
