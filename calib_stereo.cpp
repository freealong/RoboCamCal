#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

const std::string keys =
    "{n     |            | Number of checkerboard images}"
    "{lc    |            | Left camera calibration}"
    "{rc    |            | Right camera calibration}"
    "{d     |            | Image Dir}"
    "{lp    |            | Left image prefix}"
    "{rp    |            | Right image prefix}"
    "{o     | stereo.yml | Output stereo calibration filename}";

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > imagePoints1, imagePoints2;
vector< Point2f > corners1, corners2;
vector< vector< Point2f > > left_img_points, right_img_points;

Mat img1, img2, gray1, gray2;

void load_image_points(int board_width, int board_height, int num_imgs, float square_size,
                      string img_dir, string leftimg_filename, string rightimg_filename) {

  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;

  for (int i = 1; i <= num_imgs; i++) {
    string left_img = img_dir + "/" + leftimg_filename + to_string(i) + ".png";
    string right_img = img_dir + "/" + rightimg_filename + to_string(i) + ".png";
    img1 = imread(left_img, CV_LOAD_IMAGE_COLOR);
    img2 = imread(right_img, CV_LOAD_IMAGE_COLOR);
    cvtColor(img1, gray1, CV_BGR2GRAY);
    cvtColor(img2, gray2, CV_BGR2GRAY);

    bool found1 = false, found2 = false;

    found1 = cv::findCirclesGrid(img1, board_size, corners1, cv::CALIB_CB_ASYMMETRIC_GRID);
    found2 = cv::findCirclesGrid(img2, board_size, corners2, cv::CALIB_CB_ASYMMETRIC_GRID);
//    found1 = cv::findChessboardCorners(img1, board_size, corners1,
//                                       CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
//    found2 = cv::findChessboardCorners(img2, board_size, corners2,
//                                       CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if (found1)
    {
      cv::drawChessboardCorners(gray1, board_size, cv::Mat(corners1), found1);
//      cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
//                       cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
//      cv::drawChessboardCorners(gray1, board_size, corners1, found1);
    }
    if (found2)
    {
      cv::drawChessboardCorners(gray2, board_size, cv::Mat(corners2), found2);
//      cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
//                       cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
//      cv::drawChessboardCorners(gray2, board_size, corners2, found2);
    }

    vector< Point3f > obj;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

    if (found1 && found2) {
      cout << i << ". Found corners!" << endl;
      imagePoints1.push_back(corners1);
      imagePoints2.push_back(corners2);
      object_points.push_back(obj);
    }
  }
  for (int i = 0; i < imagePoints1.size(); i++) {
    vector< Point2f > v1, v2;
    for (int j = 0; j < imagePoints1[i].size(); j++) {
      v1.push_back(Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
      v2.push_back(Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
    }
    left_img_points.push_back(v1);
    right_img_points.push_back(v2);
  }
}

int main(int argc, char const *argv[])
{
  CommandLineParser parser(argc, argv, keys);

  if (argc < 6) {
    parser.printMessage();
    return 0;
  }

  string leftcalib_file = parser.get<string>("lc");
  string rightcalib_file = parser.get<string>("rc");
  string img_dir = parser.get<string>("d");
  string leftimg_prefix = parser.get<string>("lp");
  string rightimg_prefix = parser.get<string>("rp");
  string out_file = parser.get<string>("o");
  int num_imgs = parser.get<int>("n");

  FileStorage fsl(leftcalib_file, FileStorage::READ);
  FileStorage fsr(rightcalib_file, FileStorage::READ);

  load_image_points(4, 11, num_imgs, 20, img_dir, leftimg_prefix, rightimg_prefix);

  printf("Starting Calibration\n");
  Mat K1, K2, R, F, E;
  Vec3d T;
  Mat D1, D2;
  fsl["r_intrin"] >> K1;
  fsr["r_intrin"] >> K2;
  fsl["r_coeffs"] >> D1;
  fsr["r_coeffs"] >> D2;
  int flag = 0;
  flag |= CV_CALIB_FIX_INTRINSIC;
  
  cout << "Read intrinsics" << endl;
  
  stereoCalibrate(object_points, left_img_points, right_img_points, K1, D1, K2, D2, img1.size(), R, T, E, F);

  cv::FileStorage fs1(out_file, cv::FileStorage::WRITE);
  fs1 << "K1" << K1;
  fs1 << "K2" << K2;
  fs1 << "D1" << D1;
  fs1 << "D2" << D2;
  fs1 << "R" << R;
  fs1 << "T" << T;
  fs1 << "E" << E;
  fs1 << "F" << F;
  
  printf("Done Calibration\n");

  printf("Starting Rectification\n");

  cv::Mat R1, R2, P1, P2, Q;
  stereoRectify(K1, D1, K2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);

  fs1 << "R1" << R1;
  fs1 << "R2" << R2;
  fs1 << "P1" << P1;
  fs1 << "P2" << P2;
  fs1 << "Q" << Q;

  printf("Done Rectification\n");

  return 0;
}
