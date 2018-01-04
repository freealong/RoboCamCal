//
// Created by yongqi on 18-1-4.
//

#ifndef ROBOCAMCAL_STEREOCALIBCONTROLLER_HPP
#define ROBOCAMCAL_STEREOCALIBCONTROLLER_HPP

#include "CalibController.hpp"
#include "CalibCommon.hpp"

namespace Robocamcal {

class StereoCalibController : public CalibController {
 public:
  StereoCalibController(std::shared_ptr<StereoCalibData> data,
                        std::shared_ptr<StereoCalibResults> results,
                        std::shared_ptr<BoardDetector> left_detector,
                        std::shared_ptr<BoardDetector> right_detector);

  virtual void FeedData();

  virtual void DropData();

  /**
   * Before run Calibration, should set the camera params of both cameras in results_
   * @return
   */
  virtual bool Calibrate();

 private:
  std::shared_ptr<StereoCalibData> data_;
  std::shared_ptr<StereoCalibResults> results_;
  std::shared_ptr<BoardDetector> left_detector_, right_detector_;
};

}

#endif //ROBOCAMCAL_STEREOCALIBCONTROLLER_HPP
