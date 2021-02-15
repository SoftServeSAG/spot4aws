BASE_DIR=`pwd`
ROS_APP_DIR=$BASE_DIR/simulation_ws
ROS_ROBOT_DIR=$BASE_DIR/robot_ws

cd $ROS_ROBOT_DIR
source install/setup.sh
cd ..

cd $ROS_APP_DIR
source install/setup.sh
export START_X=0.5
export START_Y=0.0
export START_YAW=0.0
export MODEL_NAME=/
export SIM_TIMEOUT_SECONDS=1500
export START_TIME_SECONDS=5
export GOAL_POSITION_TOLERANCE=1.0
export GOAL_ORIENTATION_TOLERANCE=1.0
export GOAL_POSITION_X=-1.6
export GOAL_POSITION_Y=1.5
export GOAL_ORIENTATION_YAW=3.1

roslaunch rs_tests localization_ab_test.launch gui:=true rviz:=true