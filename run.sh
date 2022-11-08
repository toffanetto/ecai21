colcon build --packages-select tangent_bug_control

source /usr/share/gazebo/setup.sh 

source ./install/setup.zsh

ros2 launch tangent_bug_control tangent_bug_control.py

