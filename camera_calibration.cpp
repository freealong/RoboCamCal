//
// Created by yongqi on 18-1-3.
//

#include "CVUtils.hpp"
#include "CameraCalibController.hpp"
#include "CalibViewer.hpp"

using namespace Robocamcal;
using namespace Robocamcal::Utils;

const std::string keys =
    "{is     | 0         | Input source, 0 from camera, 1 from video file, 2 from pictures}"
    "{v      |           | Video file or picture list}"
    "{ci     | 0         | Default camera id}"
    "{bc     |           | Board config yml file}"
    "{o      |           | Output file name}"
    "{help   |           | print help}";

// params paser
struct CmdParameters {
  int input_source;
  std::string video_filename;
  int camera_id;
  std::string board_cfg;
  std::string output_filename;
  CmdParameters(const cv::CommandLineParser &parser) {
    input_source = parser.get<int>("is");
    if (input_source == 0)
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
  cv::VideoCapture capture;
  if (parameters.input_source == 0)
    capture.open(parameters.camera_id);
  else
    capture.open(parameters.video_filename);
  // init BoardDetector
  std::shared_ptr<BoardDetector> detector(new BoardDetector(parameters.board_cfg));
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
  grab_frame(capture, frame);
  while (status != CalibrationStatus::Finished) {
    // detect board
    detector->Detect(frame);
    show_frame = frame.clone();
    detector->DrawImagePoints(show_frame);
    // show image
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
        if (parameters.input_source == 2)
          grab_frame(capture, frame);
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
        // only works when input source is from pictures
        if (parameters.input_source == 2)
          grab_frame(capture, frame);
        break;
      case CalibrationStatus::WriteResult :results->write(parameters.output_filename);
      default:break;
    }
    // always get next frames if input source is from camera or video
    if (parameters.input_source < 2)
      grab_frame(capture, frame);
  }
  return 0;
}
catch (std::exception &e) {
  std::cerr << e.what() << std::endl;
}