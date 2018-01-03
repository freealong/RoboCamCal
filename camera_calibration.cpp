//
// Created by yongqi on 18-1-3.
//

#include "CalibrationController.hpp"
#include "FrameCapturer.hpp"
#include "CalibrationViewer.hpp"

using namespace Robocamcal;

const std::string keys =
    "{c      | false     | Input from camera}"
    "{v      |           | Video file}"
    "{ci     | 0         | Default camera id}"
    "{bc     |           | Board config yml file}"
    "{o      |           | Output file name}"
    "{help   |           | Print help}";

// params paser
struct CmdParameters {
  bool camera;
  std::string video_filename;
  int camera_id;
  std::string board_cfg;
  std::string output_filename;
  CmdParameters(const cv::CommandLineParser &parser) {
    camera = parser.get<bool>("c");
    if (camera)
      camera_id = parser.get<int>("ci");
    else
      video_filename = parser.get<std::string>("v");
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
  FrameCapturer capturer;
  if (parameters.camera)
    capturer.Open(parameters.camera_id);
  else
    capturer.Open(parameters.video_filename);
  // init BoardDetector
  BoardDetector detector(parameters.board_cfg);
  // init CalibrationController
  CalibrationStatus status = CalibrationStatus::None;
  std::shared_ptr<CameraCalibData> data(new CameraCalibData);
  std::shared_ptr<CameraCalibResults> results(new CameraCalibResults);
  std::shared_ptr<CalibrationController> controller(new CameraCalibController(data, results));
  // init CalibrationViewer
  CalibrationViewer viewer;
  CalibrationViewer::PrintHelp();
  // begin to loop
  auto frames = capturer.GetFrames();
  cv::Mat frame, show_frame;
  while (status != CalibrationStatus::Finished) {
    // detect board
    if (!frames.empty()) {
      frame = frames[0];
      show_frame = frame.clone();
    }
    detector.Detect(frame);
    detector.DrawImagePoints(show_frame);
    // show image
    viewer.AddImage("Image", show_frame);
    status = viewer.Update();
    // process keyboard event
    switch (status) {
      case CalibrationStatus::Calibration :
        results->image_size.height = frame.rows;
        results->image_size.width = frame.cols;
        controller->Calibrate();
        results->Print();
        break;
      case CalibrationStatus::SaveCurrentData :
        controller->FeedData(detector);
        frames = capturer.GetFrames();
        break;
      case CalibrationStatus::DropLastData :
        controller->DropData();
        break;
      case CalibrationStatus::SwitchUndistort :
        // @TODO:
        break;
      case CalibrationStatus::SwitchVisualisation :
        // @TODO:
        break;
      case CalibrationStatus::NextFrame :
        frames = capturer.GetFrames();
        break;
      case CalibrationStatus::WriteResult :
        results->Save(parameters.output_filename);
      default:break;
    }
    if (parameters.camera)
      frames = capturer.GetFrames();
  }
  return 0;
}
catch (std::exception &e) {
  std::cerr << e.what() << std::endl;
}