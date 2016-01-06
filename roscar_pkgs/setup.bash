DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

. /opt/ros/jade/setup.bash
export ROS_PACKAGE_PATH=${DIR}:$ROS_PACKAGE_PATH
