# RoboCamCal(Robot Camera Calibration)
This project implements some calibrations including hand-eye calibration, camera calibration, stereo camera calibration;

## Dependence
1. OpenCV(>=3.0.0)
2. Visp
3. ROS

## How to build
```bash
cd PROJECT_DIR
mkdir build && cd build
cmake .. && make -j4
```

### Supported Calibration Types
* [x] Intrinsic Calibration
* [x] Stereo Calibration
* [x] Hand-Eye Calibration
### Supported Calibration Boards
* [x] Chess Board
* [x] Acircles Grid Board
* [x] Charuco Board
* [x] Aruco Grid Board
### Supported Input sources
* [x] image files
* [x] video sequence
* [x] camera

To handle different kinds of camera, We use a camera bridge to get images
from a ros service. The ros service should handle these cmd:
1. update: capture and update all frames
2. get_rgb: provide color image
3. get_camera_info[optional, for hand-eye calibration]: provide camera info like intrinsics, image size...

## Usage
* camera calibration
```bash
# input from a camera called 2018-11-025-LC3 provided by xyz_camera_service
./camera_calibration --input=xyz_camera_service:2018-11-025-LC3 -b ../config/board.yml
# input from image list: color001.jpg, color002.jpg ...
./camera_calibration --input=color%03d.jpg -b ../config/board.yml
# input from video file: video.mp4 and save output to output.yml
./camera_calibration --input=video.mp4 -b ../config/board.yml -o output.yml
```
* stereo calibration
```bash
./stereo_calibration --left_input=xyz_camera_service:1111111 \
--right_input=xyz_camera_service:222222 -b ../config/board.yml -o output.yml
```

* hand eye calibration
```bash
./handeye_calibration --hand_input=robot1_GetCartesian \
--eye_input=xyz_camera_service:2018-11-025-LC3 -b ../config/board.yml \
--eye_in_hand=true
```
### License [MIT](LICENSE.md)
