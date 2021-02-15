BASE_DIR=`pwd`
ROS_APP_DIR=$BASE_DIR/simulation_ws
ROS_ROBOT_DIR=$BASE_DIR/robot_ws

cd $ROS_ROBOT_DIR
source install/setup.sh
cd ..

cd $ROS_APP_DIR
source install/setup.sh
export START_X=10.5
export START_Y=10.5
export START_YAW=0.0
export MODEL_NAME=/
export SIM_TIMEOUT_SECONDS=100
export START_TIME_SECONDS=5
roslaunch rs_tests straight_line_test.launch gui:=true