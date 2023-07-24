# Body Design and Gait Generation of <br>Chair-Type Asymmetrical Tripedal Robot
https://github.com/shin0805/Chair-TypeAsymmetricalTripedalRobot/assets/85533177/e9bb9f02-83b9-44ec-9171-5d3cb4048a59



## Instration
- ROS

You need to have ROS installed on your system. If you haven't installed ROS yet, follow the steps below:
1. Visit the official ROS website: http://www.ros.org/
2. Choose the appropriate ROS distribution for your operating system (e.g., Melodic, Noetic).
3. Follow the installation instructions provided on the ROS website for your specific OS.

And also install the following
```shell
sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino
sudo apt-get install ros-${ROS_DISTRO}-rosserial
```

- Python
```shell
pip install rospkg
pip install numpy
pip install onnxruntime
```
## Usage
- Run the launch file:
```shell
cd launch
roslaunch drive.launch
```

- Execute the pose planning program in a separate terminal:
```shell
cd src
./rl_walk.py # or any other relevant program (e.g., connect_walk.py, connect_walk.py, rl_walk.py, rl_stand.py, rl_walk_and_stand.py)
```

## Description
- `embedded`: Programs to be flashed onto Arduino Nano Every.
- `launch`: ROS launch files.
- `mjcf`: MuJoCo XML File and STL files.
- `models`: Pre-trained models saved in ONNX format.
- `src`: Gate Generating programs, including five types  
connect_walk.py, connect_walk.py, rl_walk.py, rl_stand.py, rl_walk_and_stand.py.
- `video`: Demonstration videos.
