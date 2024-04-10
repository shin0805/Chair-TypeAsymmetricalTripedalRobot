# Body Design and Gait Generation of <br>Chair-Type Asymmetrical Tripedal <br>Low-rigidity Robot
- [website](https://shin0805.github.io/chair-type-tripedal-robot/)
- [Paper](https://arxiv.org/abs/2404.05932)
- [YouTube](https://youtu.be/-f8LDlhmdBg)

https://github.com/shin0805/Chair-TypeAsymmetricalTripedalRobot/assets/85533177/6861e9ec-b4c3-4dad-bf00-435fc8d69de6

## Installation
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
