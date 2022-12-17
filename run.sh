colcon build --packages-select feedback_linearization_control

source /usr/share/gazebo/setup.sh 

source ./install/setup.zsh

ros2 launch feedback_linearization_control feedback_linearization_control.py

