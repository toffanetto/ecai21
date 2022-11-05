colcon build --packages-select reactive_control

source /usr/share/gazebo/setup.sh 

source ./install/setup.zsh

ros2 launch reactive_control reactive_control.py

