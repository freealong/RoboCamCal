//
// Created by yongqi on 18-1-3.
//

#include "BoardDetector.hpp"

namespace Robocamcal {

BoardDetector::BoardDetector(const std::string &cfg_file) : board_found_(false) {
  cv::FileStorage fs(cfg_file, cv::FileStorage::READ);
  // get board type
  std::string board_type;
  fs["board_type"] >> board_type;
  if (board_type == "ChessBoard")
    board_type_ = BoardType::ChessBoard;
  else if (board_type == "AcircleBoard")
    board_type_ = BoardType::AcirclesGrid;
  else if (board_type == "ArucoBoard")
    board_type_ = BoardType::ArucoBoard;
  else
    throw std::invalid_argument("Unknown Board Type");
  // get board size and length
  fs["board_width"] >> board_size_.width;
  fs["board_height"] >> board_size_.height;
  fs["board_length"] >> board_len_;
  // checkout board params
  if (board_size_.width <= 0)
    throw std::invalid_argument("Invalid board width: " + std::to_string(board_size_.width));
  if (board_size_.height <= 0)
    throw std::invalid_argument("Invalid board height: " + std::to_string(board_size_.height));
  if (board_len_ <= 0)
    throw std::invalid_argument("Invalid board length: " + std::to_string(board_len_));
}

bool BoardDetector::Detect(const cv::Mat &frame) {
  // reset result
  board_found_ = false;
  image_points_.clear();
  object_points_.clear();
  // detect
  switch (board_type_) {
    case BoardType::ChessBoard:
      board_found_ = DetectChessBoard(frame);
      break;
    case BoardType::AcirclesGrid:
      board_found_ = DetectAcircleGrid(frame);
      break;
    case BoardType ::ArucoBoard:
      board_found_ = DetectArucoBoard(frame);
      break;
  }
  return board_found_;
}

void BoardDetector::DrawImagePoints(cv::Mat &frame) const {
  if (!board_found_)
    return;
  switch (board_type_) {
    case BoardType::ChessBoard:
      cv::drawChessboardCorners(frame, board_size_, cv::Mat(image_points_), board_found_);
      break;
    case  BoardType::AcirclesGrid:
      cv::drawChessboardCorners(frame, board_size_, cv::Mat(image_points_), board_found_);
      break;
    case BoardType::ArucoBoard:
      // @TODO
      break;
  }
}

bool BoardDetector::DetectChessBoard(const cv::Mat &frame) {
  if (cv::findChessboardCorners(frame, board_size_, image_points_)) {
    // sub pixel image points
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(gray, image_points_, cv::Size(11,11), cv::Size(-1,-1),
                     cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1));
    // generate object points
    for( int i = 0; i < board_size_.height; ++i )
      for( int j = 0; j < board_size_.width; ++j )
        object_points_.emplace_back(cv::Point3f(j*board_len_, i*board_len_, 0));
    return true;
  }
  else return false;
}

bool BoardDetector::DetectAcircleGrid(const cv::Mat &frame) {
  if (cv::findCirclesGrid(frame, board_size_, image_points_, cv::CALIB_CB_ASYMMETRIC_GRID)) {
    for( int i = 0; i < board_size_.height; i++ )
      for( int j = 0; j < board_size_.width; j++ )
        object_points_.emplace_back(cv::Point3f((2*j + i % 2)*board_len_, i*board_len_, 0));
    return true;
  }
  else return false;
}

bool BoardDetector::DetectArucoBoard(const cv::Mat &frame) {
  // @TODO
  return false;
}
}