# DRE-SLAM 
## Dynamic RGB-D Encoder SLAM for Differential-Drive Robot
**Authors**: [Dongsheng Yang](https://github.com/ydsf16), [Shusheng Bi](http://ir.lib.buaa.edu.cn/Scholar/ScholarCard/5784), [Wei Wang](http://ir.lib.buaa.edu.cn/Scholar/ScholarCard/5800), Chang Yuan, Wei Wang, Xianyu Qi, and [Yueri Cai](http://ir.lib.buaa.edu.cn/Scholar/ScholarCard/5785)

**DRE-SLAM** is developed for a differential-drive robot that runs in dynamic indoor scenarios. It takes the information of an RGB-D camera and two wheel-encoders as inputs. The outputs are the 2D pose of the robot and a static background OctoMap.

**Video**:  [Youtube](https://youtu.be/3A5wpWgrHTI) [Dropbox](https://www.dropbox.com/s/uvqyb3mo6tj4pf2/DRE-SLAM-20190111-v3.mp4?dl=0) [Pan.Baidu](https://pan.baidu.com/s/1vVakfXZJziU12-vqw7Go1Q)

<a href="https://youtu.be/3A5wpWgrHTI" target="_blank"><img src="http://img.youtube.com/vi/3A5wpWgrHTI/0.jpg" 
alt="DRE-SLAM" width="320" height="240"/></a>

**Paper**: ***DRE-SLAM: Dynamic RGB-D Encoder SLAM for Differential-Drive Robot***, Dongsheng Yang, Shusheng Bi, Wei Wang, Chang Yuan, Wei Wang, Xianyu Qi, and Yueri Cai. (Remote Sensing, 2019) [PDF](https://www.mdpi.com/2072-4292/11/4/380/pdf), [WEB](https://www.mdpi.com/2072-4292/11/4/380)

#  Prerequisites
### 1. **Ubuntu 16.04**

### 2. **[ROS Kinetic](http://wiki.ros.org)**
Follow the instructions in: <http://wiki.ros.org/kinetic/Installation/Ubuntu> 
### 3. **ROS pacakges**
```
sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-tf ros-kinetic-message-filters ros-kinetic-image-transport ros-kinetic-octomap ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server ros-kinetic-pcl-ros ros-kinetic-pcl-msgs ros-kinetic-pcl-conversions ros-kinetic-geometry-msgs
```

### 4. [OpenCV 4.0](https://opencv.org/opencv-4-0-0.html)
We use the YOLOv3 implemented in OpenCV 4.0.

Follow the instructions in: <https://opencv.org/opencv-4-0-0.html> 

### 5. Ceres
Follow the instructions in: <http://www.ceres-solver.org/installation.html>

# Build DRE-SLAM
### 1. Clone the repository
```
cd ~/catkin_ws/src
git clone https://github.com/ydsf16/dre_slam.git
```

### 2. Build DBow2
```
cd dre_slam/third_party/DBoW2
mkdir build
cd build
cmake ..
make -j4
```

### 3. Build Sophus
```
cd ../../Sophus
mkdir build
cd build
cmake ..
make -j4
```

### 4. Build object detector
```
cd ../../../object_detector
mkdir build
cd build
cmake ..
make -j4
```

### 5. Download the YOLOv3 model
```
cd ../../config
mkdir yolov3
cd yolov3
wget https://pjreddie.com/media/files/yolov3.weights
wget https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg?raw=true -O ./yolov3.cfg
wget https://github.com/pjreddie/darknet/blob/master/data/coco.names?raw=true -O ./coco.names
```

### 6. Catkin_make
```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

# Example

## Dataset
We collected several data sequences in our lab using our Redbot robot. The dataset is available at [Pan.Baidu](https://pan.baidu.com/s/1freJVLeIE525xHUZY01HmQ) or [Dropbox](https://www.dropbox.com/sh/f7fsx8s9k9oya3r/AACBoOUlPNo7inOVceHD5gy_a?dl=0).

## Run
### 1. Open a terminal and launch dre_slam
```
roslaunch dre_slam comparative_test.launch
```

### 2. Open a terminal and play one rosbag

```
rosbag play <bag_name>.bag
```

# Run on your own robot
**You need to do three things:**

1. Calibrate the intrinsic parameter of the camera, the robot odometry parameter, and the rigid transformation from the camera to the robot.

2. Prepare a parameter configuration file, refer to the ***config*** folder.

3. Prepare a launch file, refer to the ***launch*** folder.

# Contact us
For any issues, please feel free to contact **[Dongsheng Yang](https://github.com/ydsf16)**: <ydsf16@buaa.edu.cn>
