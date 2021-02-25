BASE_DIR=`pwd`
ROS_APP_DIR=$BASE_DIR/simulation_ws
ROS_ROBOT_DIR=$BASE_DIR/robot_ws

export START_X=0.5
export START_Y=0.0
export START_YAW=0.0
export MODEL_NAME=/
export SIM_TIMEOUT_SECONDS=1500
export START_TIME_SECONDS=5
export ENVIRONMENT=LOCAL

cd $ROS_ROBOT_DIR
source install/setup.sh
roslaunch rs_robot_tests localization_kidnapped_test.launch rviz:=true
cd ..

cd $ROS_APP_DIR
source install/setup.sh
roslaunch rs_tests localization_kidnapped_test.launch gui:=true