# DORA

![logo](misc/logo.png)

DORA (Daycare Organising Robotic Assistant) 
is an autonomous daycare tidying robot, 
capable of navigating a daycare play area in order to identify and pick up toys, 
and sort them into piles. 

## Install
Clone repo and install 
[requirements.txt](https://github.com/JacobInwald/DORA/blob/main/requirements.txt).
```bash
git clone https://github.com/JacobInwald/DORA.git  # clone
cd DORA
pip install -r requirements.txt  # install
```
ROS2 is required for this project.
To install ROS2, refer [here](https://docs.ros.org/en/humble/Installation.html).


## Build
Source ROS2 and build project before running anything.
```bash
source /opt/ros/<distro>/setup.bash
colcon build
source install/setup.bash
```
Alternatively just run `source setup.bash`


## Launch
To launch DORA, run
```bash
ros2 launch launch/dora.launch.py
```


## Remote Display
To display camera stream on another computer, 
source ROS2 and build required package
```bash
source /opt/ros/<distro>/setup.bash
colcon build --packages-select remote
source install/setup.bash
```
then launch the display node
```bash
ros2 launch launch/display.launch.py
```
