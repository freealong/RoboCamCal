//
// Created by yongqi on 18-1-3.
//

#include <ros/ros.h>

#include "cxxopts.hpp"
#include "StereoCalibController.hpp"
#include "CalibViewer.hpp"
#include "CameraBridge.hpp"

using namespace Robocamcal;

void draw_lines(cv::Mat &frame, bool vertical, int step = 20) {
  if (vertical) {
    for (int x = step; x < frame.cols; x += step)
      cv::line(frame, {0, x}, {frame.rows, x}, {0, 255, 0});
  }
  else {
    for (int y = step; y < frame.rows; y += step)
      cv::line(frame, {y, 0}, {y, frame.cols}, {0, 255, 0});
  }
}

int main(int argc, char **argv) try {
  ros::init(argc, argv, "stereo_calibration");
  // parser command line args
  cxxopts::Options options("stereo_camera_calibration", "stereo camera calibration.");
  options.add_options()
      ("li,left_input", "left input source, eg: camera.mp4,intrinsic_file.yml"
                        " image%03d.jpg,intrinsic_file.yml"
                        " ros_camera_service_name:camera_id,intrinsic_file.yml",
       cxxopts::value<std::vector<std::string>>());
  options.add_options()
      ("ri,right_input", "right input source, eg: camera.mp4,intrinsic_file.yml"
                         " image%03d.jpg,intrinsic_file.yml"
                         " ros_camera_service_name:camera_id,intrinsic_file.yml",
       cxxopts::value<std::vector<std::string>>());
  options.add_options()
      ("b,board", "board cfg file", cxxopts::value<std::string>());
  options.add_options()
      ("o,output", "output file", cxxopts::value<std::string>()->default_value("output.yml"));
  auto args = options.parse(argc, argv);
  std::string board_cfg = args["board"].as<std::string>();
  std::string output_file = args["output"].as<std::string>();
  std::vector<std::string> left_input_list = args["left_input"].as<std::vector<std::string>>();
  std::vector<std::string> right_input_list = args["right_input"].as<std::vector<std::string>>();
  std::unique_ptr<CameraBridge> lbridge, rbridge;
  if (left_input_list.size() == 2) {
    lbridge = std::unique_ptr<CameraBridge>(new CameraBridge(left_input_list[0]));
  } else {
    std::cerr << "invalid left input source" << std::endl;
    return -1;
  }
  if (right_input_list.size() == 2) {
    rbridge = std::unique_ptr<CameraBridge>(new CameraBridge(right_input_list[0]));
  } else {
    std::cerr << "invalid right input source" << std::endl;
    return -1;
  }
  std::string left_camera_cfg = left_input_list[left_input_list.size()-1];
  std::string right_camera_cfg = right_input_list[right_input_list.size()-1];
  // init BoardDetector
  std::shared_ptr<BoardDetector> left_detector(new BoardDetector(board_cfg));
  std::shared_ptr<BoardDetector> right_detector(new BoardDetector(board_cfg));
  // init CalibController
  CalibrationStatus status = CalibrationStatus::None;
  std::shared_ptr<StereoCalibData> data(new StereoCalibData);
  std::shared_ptr<StereoCalibResults> results(new StereoCalibResults);
  std::shared_ptr<CalibController> controller(new StereoCalibController(
      data, results, left_detector, right_detector));
  // init CalibViewer
  CalibViewer viewer;
  CalibViewer::PrintHelp();
  // set left and right cameras params
  results->left_res.read(left_camera_cfg);
  results->right_res.read(right_camera_cfg);
  if (results->left_res.image_size != results->right_res.image_size) {
    std::cerr << "left image size not same with the right image size" << std::endl;
    return 0;
  }
  // begin to loop
  cv::Mat left_frame, left_show_frame;
  cv::Mat right_frame, right_show_frame;
  bool rectify_view = false;
  bool auto_grab = false;
  grab_frame(lbridge, left_frame);
  grab_frame(rbridge, right_frame);
  while (status != CalibrationStatus::Finished) {
    // detect board
    left_detector->Detect(left_frame);
    right_detector->Detect(right_frame);
    left_show_frame = left_frame.clone();
    right_show_frame = right_frame.clone();
    left_detector->DrawImagePoints(left_show_frame);
    right_detector->DrawImagePoints(right_show_frame);
    // show image
    if (rectify_view && results->valid) {
      cv::remap(left_show_frame, left_show_frame, results->left_res.map1,
                results->left_res.map2, cv::INTER_LINEAR);
      cv::remap(right_show_frame, right_show_frame, results->right_res.map1,
                results->right_res.map2, cv::INTER_LINEAR);
      bool vertical = results->P2.at<double>(1, 3) > results->P2.at<double>(0, 3);
      draw_lines(left_show_frame, vertical);
      draw_lines(right_show_frame, vertical);
    }
    viewer.AddImage("Left Image", left_show_frame);
    viewer.AddImage("Right Image", right_show_frame);
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
        rectify_view = !rectify_view;
        break;
      case CalibrationStatus::SwitchVisualisation :
        // @TODO:
        break;
      case CalibrationStatus::SwitchPlayMode:
        auto_grab = !auto_grab;
      case CalibrationStatus::NextFrame :
        grab_frame(lbridge, left_frame);
        grab_frame(rbridge, right_frame);
        break;
      case CalibrationStatus::WriteResult :results->write(output_file);
      default:break;
    }
    if (auto_grab) {
      grab_frame(lbridge, left_frame);
      grab_frame(rbridge, right_frame);
    }
  }
  return 0;
}
catch (std::exception &e) {
  std::cerr << e.what() << std::endl;
}