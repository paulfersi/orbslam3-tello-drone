# orbslam3-tello-drone

Orbslam3 on a Ryze Tello drone using ROS2 

The driver I used to use the drone with ros can be found here: https://github.com/clydemcqueen/tello_ros.git

The drone sends the frame to the computer that process it running ORB_SLAM in monocular mode.

You have to install **asio**(asyncronous IO library) by running:

```
sudo apt install libasio-dev
```