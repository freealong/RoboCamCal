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
* [ ] Hand-Eye Calibration
### Supported Calibration Boards
* [x] Chess Board
* [x] Acircles Grid Board
* [x] Charuco Board
* [x] Aruco Grid Board
### Supported Cameras
To handle different kinds of camera, We use a camera bridge to get images
from a ros service. The ros service should handle these cmd:
1. update: capture and update all frames
2. get_rgb: provide color image
3. get_camera_info[optional, for hand-eye calibration]: provide camera info like intrinsics, image size...

### License [MIT](LICENSE.md)