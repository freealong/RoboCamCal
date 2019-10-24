//
// Created by yongqi on 18-1-3.
//

#include <ros/ros.h>

#include "cxxopts.hpp"
#include "CVUtils.hpp"
#include "StereoCalibController.hpp"
#include "CalibViewer.hpp"
#include "CameraBridge.hpp"

using namespace Robocamcal;
using namespace Robocamcal::Utils;

std::unique_ptr<CameraBridge> g_lbridge, g_rbridge;
std::string g_lcamera_id, g_rcamera_id;
cv::VideoCapture g_lcap, g_rcap;

bool grab_lframe(cv::Mat &lframe) {
  if (g_lbridge) {
    return g_lbridge->Update(g_lcamera_id) &&
        g_lbridge->FetchFrame(g_lcamera_id, ImageType::RGB, lframe);
  } else if (g_lcap.isOpened()) {
    if (g_lcap.grab()) {
      g_lcap.retrieve(lframe);
      return true;
    } else {
      return false;
    }
  }
  return false;
}

bool grab_rframe(cv::Mat &rframe) {
  if (g_rbridge) {
    return g_rbridge->Update(g_rcamera_id) &&
        g_rbridge->FetchFrame(g_rcamera_id, ImageType::RGB, rframe);
  } else if (g_rcap.isOpened()) {
    if (g_rcap.grab()) {
      g_rcap.retrieve(rframe);
      return true;
    } else {
      return false;
    }
  }
  return false;
}

int main(int argc, char **argv) try {
  // parser command line args
  cxxopts::Options options("stereo_camera_calibration", "stereo camera calibration.");
  options.add_options()
      ("li,left_input", "left input source, eg: camera.mp4,intrinsic_file.yml"
                        " image%03d.jpg,intrinsic_file.yml"
                        " ros_camera_service_name,camera_id,intrinsic_file.yml",
       cxxopts::value<std::vector<std::string>>());
  options.add_options()
      ("ri,right_input", "right input source, eg: camera.mp4,intrinsic_file.yml"
                         " image%03d.jpg,intrinsic_file.yml"
                         " ros_camera_service_name,camera_id,intrinsic_file.yml",
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
  if (left_input_list.size() > 2) {
    ros::init(argc, argv, "left_camera_calibration");
    g_lcamera_id = left_input_list[1];
    g_lbridge = std::unique_ptr<CameraBridge>(new CameraBridge(left_input_list[0]));
    std::cout << "left input source: " << g_lcamera_id << " in service "
              << left_input_list[0] << std::endl;
  } else if (left_input_list.size() > 1) {
    g_lcap.open(left_input_list[0]);
  } else {
    std::cerr << "invalid left input source" << std::endl;
    return -1;
  }
  if (right_input_list.size() > 2) {
    ros::init(argc, argv, "right_camera_calibration");
    g_rcamera_id = right_input_list[1];
    g_rbridge = std::unique_ptr<CameraBridge>(new CameraBridge(right_input_list[0]));
    std::cout << "right input source: " << g_rcamera_id << " in service "
              << right_input_list[0] << std::endl;
  } else if (right_input_list.size() > 1) {
    g_rcap.open(right_input_list[0]);
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
  grab_lframe(left_frame);
  grab_rframe(right_frame);
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
        grab_lframe(left_frame);
        grab_rframe(right_frame);
        break;
      case CalibrationStatus::WriteResult :results->write(output_file);
      default:break;
    }
    if (auto_grab) {
      grab_lframe(left_frame);
      grab_rframe(right_frame);
    }
  }
  return 0;
}
catch (std::exception &e) {
  std::cerr << e.what() << std::endl;
}