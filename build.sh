
echo "###############################################################################"
echo " CLOUD SUMMIT WORKSHOP: RUNNING COLCON BUILD AND BUNDLE (Open this shell script for command reference) "
echo " NOTE: This will take 10-20 minutes for the first build/bundle, 1-2 minutes for subsequent build/bundle operations. "
echo "###############################################################################"

BASE_DIR=`pwd`
ROS_APP_DIR=$BASE_DIR/simulation_ws
ROS_ROBOT_DIR=$BASE_DIR/robot_ws

cd $ROS_APP_DIR
# rosws update
rosdep install --from-paths src --ignore-src -r -y
colcon build  
colcon bundle

cd $ROS_ROBOT_DIR
# rosws update
rosdep install --from-paths src --ignore-src -r -y
colcon build  
colcon bundle