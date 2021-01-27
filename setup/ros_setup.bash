#!/bin/bash

echo "###############################################################################"
echo " CLOUD SUMMIT WORKSHOP: INSTALL STEP 1: SETTING UP CUSTOM ROSDEP "
echo " This section will take about 30 seconds to a minute. "
echo "###############################################################################"

BASE_DIR=`pwd`
APP_DIR=$BASE_DIR/simulation_ws

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

# echo "###############################################################################"
# echo " CLOUD SUMMIT WORKSHOP: INSTALL STEP 2: DOWNLOADING, INSTALLING DEPENDENCIES"
# echo " This section will take 3-4 minutes to a minute. "
# echo "###############################################################################"

# cd $APP_DIR
# rosws update
# rosdep update
# rosdep install --from-paths src --ignore-src -r -y

# echo "###############################################################################"
# echo " CLOUD SUMMIT WORKSHOP: INSTALL STEP 3: RUNNING INITIAL COLCON BUILD/BUNDLE "
# echo " This section will take a minute or two. "
# echo "###############################################################################"

# colcon build
# colcon bundle