# orbslam3-tello-drone

Orbslam3 on a Ryze Tello drone using ROS2 

The driver I used to use the drone with ros can be found here: https://github.com/clydemcqueen/tello_ros.git

The drone sends the frame to the computer that process it running ORB_SLAM in monocular mode.

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

````bash
sed -i 's/++11/++14/g' CMakeLists.txt
```
## Run monocular mode

ros2 run orbslam3 mono orbslam_ros2/vocabulary/ORBvoc.txt orbslam3_ros2/config/monocular/TUM1.yaml





