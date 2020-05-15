# orbslam2 with LK optical flow
 This project is modified from orbslam2. All dependencies are consistent with orbslam2, but only support RGBD camera now

This project：

  1、improve the speed of orbslam2
  
  2、use optical flow to track the 3D points of the previous key frame, and non-key frames use PnPransac (the projection of the tracked 3D point on the current frame) to calculate the camera pose
  
# how to install:
If you have installed all the dependencies of orbslam2, modify "run.sh" based on your directory name.

Then run:

chmod +x build.sh

chmod +x run.sh

./build.sh

./run.sh
 
# Extract ORB Features:
![image](https://github.com/suffeeen/orbslam2-with-LK-optical-flow/blob/master/result_pics/Screenshot%20from%202020-01-15%2021-15-24.png?raw=true)
# Track ORB Features:
![image](https://github.com/suffeeen/orbslam2-with-LK-optical-flow/blob/master/result_pics/Screenshot%20from%202020-01-15%2021-15-25.png?raw=true)
