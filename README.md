# orbslam3-tello-drone 

## (WIP)

Orbslam3 on a Dji Tello drone using ROS2 

<p align="center">
  <img src="/docs/nodes_structure_drone.png" width="700">
</p>

The drone sends the frame to the computer that process it running ORB_SLAM in monocular mode(or monocular-inertial or monocular-D-intertial)

You have to install **asio**(asyncronous IO library) by running:

```
sudo apt install libasio-dev
```

#### To run the GUI

the telemetry gui is made using tkinter and can be launched once the driver is up by typing:

```
ros2 run drone_info_gui drone_info_gui
```



### For ORBSLAM3

#### Install Pangolin

```shell
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin

# Install dependencies (as described above, or your preferred method)
./scripts/install_prerequisites.sh recommended

# Configure and build
cmake -B build
cmake --build build

cmake --build build -t pypangolin_pip_install

``` 

#### Install Eigen3

sudo apt install libeigen3-dev


BE CAREFUL when you build to the number of cores dedicated to the build(in the build.sh file of orbslam next to "make -j" command). The build could freeze the pc due to this

I got issues with the c++ version. I had to change from the c++11 to 14 with the command: 

```bash
sed -i 's/++11/++14/g' CMakeLists.txt
```

### Run monocular mode

```bash
# in the first terminal
ros2 launch tello_driver teleop_launch.py

# second terminal

source install/setup.sh

ros2 launch orbslam3_odometry orbslam_odometry_launch.py

```

Note: if the Pangolin visualizer says "waiting for image" it may not be detecting enough features in the image.

## Others

- to view imu data in rviz2:
```sudo apt install ros-foxy-imu-tools```

Don't forget to set Fixed Frame as "base_link"

for octomap

- sudo apt install ros-foxy-octomap
- sudo apt install ros-foxy-octomap-msgs
- sudo apt install ros-foxy-pcl-ros

### Sources 

https://github.com/clydemcqueen/tello_ros.git 

https://github.com/Il-castor/orbslam3-odometry
