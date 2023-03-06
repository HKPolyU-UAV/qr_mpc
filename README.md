# qr_mpc
This work implements a NMPC controller for a quadcopter with [ROS](https://www.ros.org/).

## Prerequisites
* Python 3.7
* ROS ([ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) recommended)
* [QGroundControl](http://qgroundcontrol.com/)
* [MAVROS](http://wiki.ros.org/mavros)
* [Acados](https://docs.acados.org/installation/index.html)

## Getting started
Install python dependencies
```
python3 -m pip install pip
pip3 install numpy matplotlib scipy future-fstrings casadi>=3.5.1 setuptools
sudo apt-get install python3.7-tk
```
Download and install the PX4 (1.11.0)
```
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot/
git checkout 71db090
git submodule sync --recursive
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
sudo apt upgrade libignition-math2 #(libignition-math4 for noetic)
```
Create a catkin workspace and clone this repository to catkin src folder (ex. ~/catkin_ws/src)
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
cd src
git clone https://github.com/HKPolyU-UAV/qr_mpc.git
```
Install acados_template Python packages
```
pip install -e <acados_root>/interfaces/acados_template
```
Add the path to the compiled shared libraries (Hint: you can add these lines to your ```.bashrc``` by ```sudo gedit ~/.bashrc```)
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"<acados_root>/lib"
export ACADOS_SOURCE_DIR="<acados_root>"
```
Modify <acados_root> path (18th and 19th lines) in CMakeLists
```
cd ~/catkin_ws/src/qr_mpc
gedit CMakeLists.txt
```
Generate C code for NMPC controller
```
cd scripts
python generate_c_code.py
```
Compile
```
cd ~/catkin_ws/
catkin_make
```

## Running
Start PX4 SITL
```
cd ~/PX4-Autopilot
make px4_sitl_default gazebo
```
Run MAVROS
```
cd ~/catkin_ws
roslaunch qr_mpc PX4_gazebo.launch
```
Open QGroundControl to takoff, and set flight mode to "offboard"

Run NMPC controller (there are three generated trajectories in this repo: circle, lemniscate and waypoints, circle is used as example)
```
roslaunch qr_mpc circle.launch
```
