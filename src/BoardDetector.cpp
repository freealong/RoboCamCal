//
// Created by yongqi on 18-1-3.
//

#include "BoardDetector.hpp"
#include <memory>
#include <opencv2/aruco/charuco.hpp>

namespace Robocamcal {

BoardDetector::BoardDetector(const std::string &cfg_file) : board_found_(false) {
  cv::FileStorage fs(cfg_file, cv::FileStorage::READ);
  // get board info
  std::string board_type;
  fs["board_type"] >> board_type;
  fs["board_cols"] >> board_size_.width;
  fs["board_rows"] >> board_size_.height;
  fs["square_length"] >> board_len_;
  if (board_type == "ChessBoard") {
    board_type_ = BoardType::ChessBoard;
  }
  else if (board_type == "AcircleBoard") {
    board_type_ = BoardType::AcirclesGrid;
  }
  else if (board_type == "CharucoBoard") {
    board_type_ = BoardType::CharucoBoard;
  }
  else if (board_type == "GridBoard") {
    board_type_ = BoardType::GridBoard;
  }
  else {
    throw std::invalid_argument("Unknown Board Type");
  }
  // get aruco info if using aruco
  if (board_type_ == BoardType::CharucoBoard or board_type_ == BoardType::GridBoard) {
    fs["marker_length"] >> marker_len_;
    int dictionary_id;
    fs["dictionary"] >> dictionary_id;
    dictionary_ = cv::aruco::Dictionary::get(dictionary_id);
    if (board_type_ == BoardType::CharucoBoard) {
      charuco_board_ = cv::aruco::CharucoBoard::create(board_size_.width,
          board_size_.height, board_len_, marker_len_, dictionary_);
    } else {
      board_ = cv::aruco::GridBoard::create(board_size_.width,
          board_size_.height, marker_len_, board_len_, dictionary_);
    }
  }
  // generate object points
  for( int i = 0; i < board_size_.height; ++i ) {
    for( int j = 0; j < board_size_.width; ++j ) {
      auto w = board_type_ != BoardType::AcirclesGrid ? j : 2 * j + i % 2;
      object_points_.emplace_back(cv::Point3f(w*board_len_, i*board_len_, 0));
    }
  }
}

bool BoardDetector::Detect(const cv::Mat &frame) {
  // reset result
  board_found_ = false;
  image_points_.clear();
  marker_ids_.clear();
  marker_corners_.clear();
  charuco_ids_.clear();
  // detect
  switch (board_type_) {
    case BoardType::ChessBoard:
      board_found_ = DetectChessBoard(frame);
      break;
    case BoardType::AcirclesGrid:
      board_found_ = DetectAcircleGrid(frame);
      break;
    case BoardType ::CharucoBoard:
      board_found_ = DetectCharucoBoard(frame);
      break;
    case BoardType ::GridBoard:
      board_found_ = DetectGridBoard(frame);
      break;
  }
  return board_found_;
}

bool BoardDetector::CalculateBoardPose(const cv::Mat &camera_matrix,
                                       const cv::Mat &dist_coeffs,
                                       cv::Vec3d &rotation,
                                       cv::Vec3d &translation) const {
  if (board_type_ <= BoardType::AcirclesGrid) {
    return cv::solvePnPRansac(object_points_, image_points_, camera_matrix,
        dist_coeffs, rotation, translation);
  } else if (board_type_ == BoardType::CharucoBoard) {
    return cv::aruco::estimatePoseCharucoBoard(image_points_, charuco_ids_,
        charuco_board_, camera_matrix, dist_coeffs, rotation, translation);
  } else if (board_type_ == BoardType::GridBoard) {
    return cv::aruco::estimatePoseBoard(marker_corners_, marker_ids_, board_,
        camera_matrix, dist_coeffs, rotation, translation);
  } else {
    return false;
  }
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
    case BoardType::CharucoBoard:
      cv::aruco::drawDetectedCornersCharuco(frame, image_points_, charuco_ids_);
      break;
    case BoardType::GridBoard:
      cv::aruco::drawDetectedMarkers(frame, marker_corners_, marker_ids_);
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
    return true;
  }
  return false;
}

bool BoardDetector::DetectAcircleGrid(const cv::Mat &frame) {
  return cv::findCirclesGrid(frame, board_size_, image_points_, cv::CALIB_CB_ASYMMETRIC_GRID);
}

bool BoardDetector::DetectCharucoBoard(const cv::Mat &frame) {
  cv::aruco::detectMarkers(frame, dictionary_, marker_corners_, marker_ids_);
  if (!marker_ids_.empty()) {
    cv::aruco::interpolateCornersCharuco(marker_corners_, marker_ids_, frame,
        charuco_board_, image_points_, charuco_ids_);
    return !charuco_ids_.empty();
  }
  return false;
}

bool BoardDetector::DetectGridBoard(const cv::Mat &frame) {
  cv::aruco::detectMarkers(frame, dictionary_, marker_corners_, marker_ids_);
  return !marker_ids_.empty();
}

}