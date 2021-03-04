#!/bin/bash

echo "###############################################################################"
echo "ROS environment setup starting.."
echo "###############################################################################"

BASE_DIR=`pwd`
ROS_APP_DIR=$BASE_DIR/simulation_ws
ROS_ROBOT_DIR=$BASE_DIR/robot_ws


# Wait if apt is running. 
while :
do
    count=`ps -ef | grep apt.systemd.daily | grep lock_is_held | grep -v grep | wc -l`
    if [ $count = 0 ]; then
        break
    else
        echo "System update is running.. Wait until the completion"
        sleep 10
    fi
done

sudo apt-get update
source /opt/ros/$ROS_DISTRO/setup.sh

#Update source list
cd $BASE_DIR
# Setup custom rosdep dependencies
CUSTOM_DEP_SOURCE_LIST_LOCATION=/etc/ros/rosdep/sources.list.d/21-customdependencies.list
CUSTOM_DEP_FILE=$BASE_DIR/setup/custom_dependencies.yaml

if [ -f "$CUSTOM_DEP_SOURCE_LIST_LOCATION" ]; then
    echo "rosdep file already exists. Skipping"
else
    sudo touch $CUSTOM_DEP_SOURCE_LIST_LOCATION
    if grep -Fxq "yaml file://$CUSTOM_DEP_FILE" $CUSTOM_DEP_SOURCE_LIST_LOCATION
    then
        echo "dependency file already setup"
    else
        echo "source list not setup"
        echo "yaml file://$CUSTOM_DEP_FILE" | sudo tee -a $CUSTOM_DEP_SOURCE_LIST_LOCATION
    fi
fi

 echo "###############################################################################"
 echo " DOWNLOADING, INSTALLING DEPENDENCIES"
 echo " This section will take 3-4 minutes to a minute. "
 echo "###############################################################################"

 cd $ROS_APP_DIR
 rosws update
 ./install_additional_deps.sh
 rosdep update
 rosdep install --from-paths src --ignore-src -r -y

 cd $ROS_ROBOT_DIR
 rosws update
 ./install_additional_deps.sh
 rosdep update
 rosdep install --from-paths src --ignore-src -r -y

 echo "###############################################################################"
 echo " RUNNING INITIAL COLCON BUILD/BUNDLE "
 echo " This section will take a minute or two. "
 echo "###############################################################################"

 echo " RUNNING INITIAL COLCON BUILD/BUNDLE FOR SIMULATION APPLICATION "
 cd $ROS_APP_DIR
 colcon build
 colcon bundle

 echo " RUNNING INITIAL COLCON BUILD/BUNDLE FOR ROBOT APPLICATION "
 cd $ROS_ROBOT_DIR
 colcon build
 colcon bundle