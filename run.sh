colcon build --packages-select potential_field_control

source /usr/share/gazebo/setup.sh 

source ./install/setup.zsh

ros2 launch potential_field_control potential_field_control.py

