[![ROS2 Humble](https://github.com/cvidkal/DeltaVINS/actions/workflows/ros2-ci.yaml/badge.svg)](https://github.com/cvidkal/DeltaVINS/actions/workflows/ros2-ci.yaml)
# DeltaVINS
## An imprementation of SRIF(Square root information filter) with MSCKF
This project can be compiled and run on both Linux and macOS. It also supports both ROS and non-ROS environments. For detailed information on supported environments, please see the table.
|           | Linux      | macOS    |
|-----------|------------|----------|
| non-ROS   | ✔️          |          |
| ROS2      | ✔️          | ✔️        |

# Build
## Build without ROS (only tested on Linux)
Conan is used to manage some third-party libraries.
```
# make build folder
mkdir build
cd build

# make sure to install conan 1.x
pip install conan==1.66.0

# install dependencies
sudo apt install libegl1-mesa-dev libgl-dev cmake gcc g++ pkg-config libva-dev libvdpau-dev libx11-xcb-dev libfontenc-dev libice-dev libsm-dev libxaw7-dev libxcomposite-dev libxcursor-dev libxdamage-dev libxext-dev libxfixes-dev libxi-dev libxinerama-dev libxkbfile-dev libxmu-dev libxmuu-dev libxpm-dev libxrandr-dev libxrender-dev libxres-dev libxss-dev libxt-dev libxtst-dev libxv-dev libxxf86vm-dev libxcb-glx0-dev libxcb-render0-dev libxcb-render-util0-dev libxcb-xkb-dev libxcb-icccm4-dev libxcb-image0-dev libxcb-keysyms1-dev libxcb-randr0-dev libxcb-shape0-dev libxcb-sync-dev libxcb-xfixes0-dev libxcb-xinerama0-dev libxcb-dri3-dev uuid-dev libxcb-cursor-dev libxcb-dri2-0-dev libxcb-dri3-dev libxcb-present-dev libxcb-composite0-dev libxcb-ewmh-dev libxcb-res0-dev libxcb-util-dev libxcb-util0-dev libgtk2.0-dev libglew-dev

# install dependency
conan install .. --build=missing
# if you get error for missing pangolin,try following in project home folder
cd 3rdParty/pangolin
conan create . --build=missing

# run in build folder
conan build ..
```

## Build with ROS2 (support both Linux and macOS)
Note: For macOS, [Robostack](https://robostack.github.io/index.html) is highly recommended for a hassle-free ROS2 installation. However, if you prefer, you can also install ROS2 directly on macOS using the native methods.
```
# make workspace
mkdir -p ~/delta_ws/src
cd ~/delta_ws/src
git clone https://github.com/cvidkal/DeltaVINS
cd ~/delta_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
bash install/setup.bash
```

# Run
A `Config.yaml` is used to set required parameters for running the system. Change `CameraCalibrationPath` to your own path to the `calibrations.yaml`.

## Run without ROS
```
cd build
./DeltaVINSTest -c path_to_Config.yaml
```

## Run with ROS2
Open three terminals, then
```
# in first terminal
ros2 run delta_vins DeltaVINSTest path_to_Config.yaml

# in second terminal
rviz2 -d path_to_delta_vins.rviz

# in third terminal
ros2 bag play path_to_data.db3
```

## Convert ROS1 data bag to ROS2
```
pip3 install rosbags>=0.9.11
rosbags-convert --src V1_01_easy.bag --dst <ros2_bag_folder>
```
See [this page](https://docs.openvins.com/dev-ros1-to-ros2.html) for more details.
