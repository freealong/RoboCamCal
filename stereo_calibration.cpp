//
// Created by yongqi on 18-1-3.
//

#include "CVUtils.hpp"
#include "StereoCalibController.hpp"
#include "CalibViewer.hpp"

using namespace Robocamcal;
using namespace Robocamcal::Utils;

const std::string keys =
    "{is     | 0         | Input source, 0 from camera, 1 from video file, 2 from pictures}"
    "{lv     |           | Left Video file or picture list}"
    "{rv     |           | Right Video file or picture list}"
    "{lci    | 0         | Default left camera id}"
    "{rci    | 1         | Default right camera id}"
    "{lcf    |           | Left camera config file}"
    "{rcf    |           | Right camera config file}"
    "{bc     |           | Board config yml file}"
    "{o      |           | Output file name}"
    "{help   |           | print help}";

// params paser
struct CmdParameters {
  int input_source;
  std::string left_video_filename, right_video_filename;
  int left_camera_id, right_camera_id;
  std::string left_camera_cfg, right_camera_cfg;
  std::string board_cfg;
  std::string output_filename;
  CmdParameters(const cv::CommandLineParser &parser) {
    input_source = parser.get<int>("is");
    if (input_source == 0) {
      left_camera_id = parser.get<int>("lci");
      right_camera_id = parser.get<int>("rci");
    }
    else {
      left_video_filename = parser.get<std::string>("lv");
      right_video_filename = parser.get<std::string>("rv");
    }
    left_camera_cfg = parser.get<std::string>("lcf");
    right_camera_cfg = parser.get<std::string>("rcf");
    board_cfg = parser.get<std::string>("bc");
    output_filename = parser.get<std::string>("o");
  }
};

int main(int argc, char **argv) try {
  // parser command line args
  cv::CommandLineParser parser(argc, argv, keys);
  if (parser.has("help")) {
    parser.printMessage();
    return 0;
  }
  CmdParameters parameters(parser);
  // init FrameCapturer
  cv::VideoCapture left_capture, right_capture;
  if (parameters.input_source == 0) {
    left_capture.open(parameters.left_camera_id);
    right_capture.open(parameters.right_camera_id);
  }
  else {
    left_capture.open(parameters.left_video_filename);
    right_capture.open(parameters.right_video_filename);
  }
  // init BoardDetector
  std::shared_ptr<BoardDetector> left_detector(new BoardDetector(parameters.board_cfg));
  std::shared_ptr<BoardDetector> right_detector(new BoardDetector(parameters.board_cfg));
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
  results->left_res.read(parameters.left_camera_cfg);
  results->right_res.read(parameters.right_camera_cfg);
  if (results->left_res.image_size != results->right_res.image_size) {
    std::cerr << "left image size not same with the right image size" << std::endl;
    return 0;
  }
  // begin to loop
  cv::Mat left_frame, left_show_frame;
  cv::Mat right_frame, right_show_frame;
  bool rectify_view = false;
  grab_frame(left_capture, left_frame);
  grab_frame(right_capture, right_frame);
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
        if (parameters.input_source == 2) {
          grab_frame(left_capture, left_frame);
          grab_frame(right_capture, right_frame);
        }
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
      case CalibrationStatus::NextFrame :
        // only works when input source is from pictures
        if (parameters.input_source == 2) {
          grab_frame(left_capture, left_frame);
          grab_frame(right_capture, right_frame);
        }
        break;
      case CalibrationStatus::WriteResult :results->write(parameters.output_filename);
      default:break;
    }
    // always get next frames if input source is from camera or video
    if (parameters.input_source < 2) {
      grab_frame(left_capture, left_frame);
      grab_frame(right_capture, right_frame);
    }
  }
  return 0;
}
catch (std::exception &e) {
  std::cerr << e.what() << std::endl;
}