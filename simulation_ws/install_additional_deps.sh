echo "###############################################################################"
echo " Clone Spot configuration package "
echo "###############################################################################"

cd src/deps

if [ -d "spot_config" ] 
then
    echo "Directory spot_config exists." 
else
    mkdir spot_config
    cd spot_config
    git init
    git remote add robots https://github.com/chvmp/robots.git
    git fetch robots
    git checkout robots/master -- configs/spot_config
    cd ..
fi


echo "###############################################################################"
echo " Clone Spot description package "
echo "###############################################################################"


if [ -d "spot_description" ] 
then
    echo "Directory spot_description exists." 
else
    mkdir spot_description
    cd spot_description
    git init
    git remote add spot_ros https://github.com/chvmp/spot_ros.git
    git fetch spot_ros
    git checkout spot_ros/gazebo -- spot_description
    cd ..
fi


echo "###############################################################################"
echo " Clone HQ office world "
echo "###############################################################################"


if [ -d "rs_gazebo" ] 
then
    echo "Directory rs_gazebo exists." 
else
    mkdir rs_gazebo
    cd rs_gazebo
    git init
    git remote add spot_simulation https://github.com/SoftServeSAG/spot_simulation.git
    git fetch spot_simulation
    git checkout spot_simulation/spot_control -- rs_gazebo
fi

