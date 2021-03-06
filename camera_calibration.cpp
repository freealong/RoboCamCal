//
// Created by yongqi on 18-1-3.
//

#include <ros/ros.h>

#include "cxxopts.hpp"
#include "CameraCalibController.hpp"
#include "CalibViewer.hpp"
#include "CameraBridge.hpp"

using namespace Robocamcal;

int main(int argc, char **argv) try {
  ros::init(argc, argv, "camera_calibration");
  // parser command line args
  cxxopts::Options options("camera_calibration", "camera calibration.");
  options.add_options()
      ("i,input", "input source, eg: camera.mp4, image%03d.jpg, ros_camera_service_name:camera_id",
          cxxopts::value<std::string>());
  options.add_options()
      ("b,board", "board cfg file", cxxopts::value<std::string>());
  options.add_options()
      ("o,output", "output file", cxxopts::value<std::string>()->default_value("output.yml"));
  auto args = options.parse(argc, argv);
  std::string board_cfg = args["board"].as<std::string>();
  std::string output_file = args["output"].as<std::string>();
  std::string input_source = args["input"].as<std::string>();
  std::unique_ptr<CameraBridge> camera_bridge(new CameraBridge(input_source));

  // init BoardDetector
  std::shared_ptr<BoardDetector> detector(new BoardDetector(board_cfg));
  // init CalibController
  CalibrationStatus status = CalibrationStatus::None;
  std::shared_ptr<CameraCalibData> data(new CameraCalibData);
  std::shared_ptr<CameraCalibResults> results(new CameraCalibResults);
  std::shared_ptr<CalibController> controller(new CameraCalibController(data, results, detector));
  // init CalibViewer
  CalibViewer viewer;
  CalibViewer::PrintHelp();
  // begin to loop
  cv::Mat frame, show_frame;
  bool undistort_view = false;
  bool auto_grab = false;
  grab_frame(camera_bridge, frame);
  while (status != CalibrationStatus::Finished) {
    // detect board
    detector->Detect(frame);
    show_frame = frame.clone();
    detector->DrawImagePoints(show_frame);
    // show image
    if (undistort_view && results->valid)
      cv::remap(show_frame, show_frame, results->map1, results->map2, cv::INTER_LINEAR);
    viewer.AddImage("Image", show_frame);
    status = viewer.Update();
    // process keyboard event
    switch (status) {
      case CalibrationStatus::Calibration :
        // set image size before calibration
        results->image_size.height = frame.rows;
        results->image_size.width = frame.cols;
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
        undistort_view = !undistort_view;
        break;
      case CalibrationStatus::SwitchVisualisation :
        // @TODO:
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