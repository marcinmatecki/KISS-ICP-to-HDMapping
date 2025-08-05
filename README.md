# kiss-icp-converter


## Intended use 

This small toolset allows to integrate SLAM solution provided by [kiss-icp](https://github.com/PRBonn/kiss-icp) with [HDMapping](https://github.com/MapsHD/HDMapping).
This repository contains ROS 2 workspace that :
  - submodule to tested revision of Kiss ICP
  - a converter that listens to topics advertised from odometry node and save data in format compatible with HDMapping.

## Building

Clone the repo
```shell
mkdir -p /test_ws/src
cd /test_ws/src
git clone https://github.com/marcinmatecki/kiss-icp-to-hdmapping.git --recursive
cd ..
colcon build
```

## Usage - data SLAM:

Prepare recorded bag with estimated odometry:

In first terminal record bag:
```shell
ros2 bag record /kiss/local_map /kiss/odometry
```

and start odometry:
```shell 
cd /test_ws/
source ./install/setup.sh # adjust to used shell
ros2 launch kiss_icp odometry.launch.py bagfile:=<path_to_rosbag> topic:=<topic_name>
```

## Usage - conversion:

```shell
cd /test_ws/
source ./install/setup.sh # adjust to used shell
ros2 run kiss-icp-to-hdmapping listener <recorded_bag> <output_dir>
```

## Example:

Download the dataset from [GitHub - ConSLAM](https://github.com/mac137/ConSLAM) or 
directly from this [Google Drive link](https://drive.google.com/drive/folders/1TNDcmwLG_P1kWPz3aawCm9ts85kUTvnU). 
Then, download **sequence2**.

## Convert(If it's a ROS1 .bag file):

```shell
rosbags-convert --src {your downloaded bag} --dst {desired destination for the converted bag}
```

## Record the bag file:

```shell
cd {your directory for the bag}
ros2 bag record /kiss/local_map /kiss/odometry -o {your directory for the recorded bag}
and start odometry:
```

## Kiss Launch:

```shell
cd /test_ws/
source ./install/setup.sh # adjust to used shell
ros2 launch kiss_icp odometry.launch.py bagfile:={path_to_bag_file} topic:=pp_points/synced2rgb
```

## During the record (if you want to stop recording earlier) / after finishing the bag:

```shell
In the terminal where the ros record is, interrupt the recording by CTRL+C
Do it also in ros launch terminal by CTRL+C.
```

## Usage - Conversion (ROS bag to HDMapping, after recording stops):

```shell
cd /test_ws/
source ./install/setup.sh # adjust to used shell
ros2 run kiss-icp-to-hdmapping listener <recorded_bag> <output_dir>
```