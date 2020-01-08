//
// Created by yongqi on 18-1-3.
//

#include <ros/ros.h>

#include "cxxopts.hpp"
#include "HandEyeCalibController.hpp"
#include "CalibViewer.hpp"
#include "CameraBridge.hpp"
#include "RobotBridge.hpp"

using namespace Robocamcal;

int main(int argc, char **argv) try {
  ros::init(argc, argv, "hand_eye_calibration");
  // parser command line args
  cxxopts::Options options("hand_eye_calibration", "hand eye calibration.");
  options.add_options()
      ("h,hand_input", "hand input source, eg: robot_ros_service_name",
       cxxopts::value<std::string>());
  options.add_options()
      ("e,eye_input", "eye input source, eg: camera.mp4,intrinsic_file.yml"
                       " image%03d.jpg,intrinsic_file.yml"
                       " ros_camera_service_name:camera_id,intrinsic_file.yml",
       cxxopts::value<std::vector<std::string>>());
  options.add_options()
      ("E,eye_in_hand", "eye in hand",
       cxxopts::value<bool>());
  options.add_options()
      ("b,board", "board cfg file", cxxopts::value<std::string>());
  options.add_options()
      ("o,output", "output file", cxxopts::value<std::string>()->default_value("output.yml"));
  auto args = options.parse(argc, argv);
  std::string board_cfg = args["board"].as<std::string>();
  std::string output_file = args["output"].as<std::string>();
  std::string hand_input = args["hand_input"].as<std::string>();
  std::vector<std::string> eye_input_list = args["eye_input"].as<std::vector<std::string>>();
  std::unique_ptr<CameraBridge> camera_bridge(new CameraBridge(eye_input_list[0]));
  std::string camera_cfg = eye_input_list[eye_input_list.size()-1];
  std::shared_ptr<RobotBridge> robot_bridge(new RobotBridge(hand_input));
  // init BoardDetector
  std::shared_ptr<BoardDetector> detector(new BoardDetector(board_cfg));
  // init CalibController
  CalibrationStatus status = CalibrationStatus::None;
  std::shared_ptr<HandEyeCalibData> data(new HandEyeCalibData);
  std::shared_ptr<HandEyeCalibResults> results(new HandEyeCalibResults);
  std::shared_ptr<CalibController> controller(new HandEyeCalibController(
      data, results, detector, robot_bridge));
  // init CalibViewer
  CalibViewer viewer;
  CalibViewer::PrintHelp();
  // set camera params
  results->intrs.read(camera_cfg);
  results->eye_in_hand = args["eye_in_hand"].as<bool>();
  // begin to loop
  cv::Mat frame, show_frame;
  cv::Vec3d rvec, tvec;
  bool auto_grab = false;
  grab_frame(camera_bridge, frame);
  while (status != CalibrationStatus::Finished) {
    // detect board
    show_frame = frame.clone();
    if (detector->Detect(frame)) {
      detector->DrawImagePoints(show_frame);
    }
    if (detector->CalculateBoardPose(results->intrs.camera_matrix, results->intrs.dist_coeffs,
        rvec, tvec)) {
      detector->DrawBoardPose(show_frame, results->intrs.camera_matrix, results->intrs.dist_coeffs,
          rvec, tvec, 0.1);
    }
    // show image
    viewer.AddImage("Image", show_frame);
    status = viewer.Update();
    // process keyboard event
    switch (status) {
      case CalibrationStatus::Calibration :
        controller->Calibrate();
        results->print();
        break;
      case CalibrationStatus::SaveCurrentData :
        controller->FeedData();
        break;
      case CalibrationStatus::DropLastData :
        controller->DropData();
        break;
      case CalibrationStatus::SwitchUndistort :
        // not support
        break;
      case CalibrationStatus::SwitchVisualisation :
        // not support
        break;
      case CalibrationStatus::SwitchPlayMode:
        auto_grab = !auto_grab;
      case CalibrationStatus::NextFrame :
        grab_frame(camera_bridge, frame);
        break;
      case CalibrationStatus::WriteResult :results->write(output_file);
      default:break;
    }
    // always get next frames if input source is from camera or video
    if (auto_grab)
      grab_frame(camera_bridge, frame);
  }
  return 0;
}
catch (std::exception &e) {
  std::cerr << e.what() << std::endl;
}
