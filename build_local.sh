
echo "###############################################################################"
echo " RUNNING COLCON BUILD "
echo " NOTE: This will take 10-20 minutes for the first build, 1-2 minutes for subsequent build operations. "
echo "###############################################################################"

BASE_DIR=`pwd`
ROS_APP_DIR=$BASE_DIR/simulation_ws
ROS_ROBOT_DIR=$BASE_DIR/robot_ws

cd $ROS_APP_DIR
rosws update
./install_additional_deps.sh
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-clean-cache --cmake-clean-first
source install/setup.sh

cd $ROS_ROBOT_DIR
rosws update
./install_additional_deps.sh
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-clean-cache --cmake-clean-first
source install/setup.sh