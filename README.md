# spot4aws
The repository contains packages for simulating Spot in AWS Robomaker or on a  local machine.

The IAM user needs read and write permissions to several services, including provisioning resources with AWS CloudFormation, AWS RoboMaker, Amazon S3, AWS Cloud9, Amazon CloudWatch, Amazon VPC, AWS Lambda and AWS Identity and Access Management. 
Please make sure you have these permissions in your account before proceeding further.

## Initial ROS setup
1. Download this repository 
2. Go to downloaded folder and run: `./setup/ros_setup.bash`
It downloads all needed ROS dependencies and builds all ROS packages

## Cloudformation setup    
Run : `./setup.bash`. It deploys the cloudformation stack to setup the AWS resources (IAM roles, VPC, etc)

# Usage
## Local Execution
Set the appropriate environement variables required for the application.

````
export MODEL_NAME=/ # model name of robot in Gazebo
export START_X=0.5 # start location of robot
export START_Y=0.0
export START_YAW=0.0
export ENVIRONMENT=LOCAL
````

In order to see the application running on your local machine, run the following command.
````
source simulation_ws/install/setup.bash
roslaunch rs_config gazebo.launch
````
To launch a navigation stack run in other terminal` tab:
````
source robot_ws/install/setup.bash
roslaunch rs_navigation navigate.launch rviz:=true
````
## Cloud Execution

### Run Single Robot
To run single Spot, you need to run following command:
````
/run.bash launch_single.json
````

### Run Fleet of Spots
To run fleet on AWS Robomaker
````
/run.bash launch_two_robots.json
````

### Typical config to launch fleet 

```json
{
    "robots": [
        {
            "name": "robot1",
            "environmentVariables": {
                "START_X": "2",
                "START_Y": "10",
                "START_YAW": "3.143",
                "USE_CUSTOM_MOVE_OBJECT_GAZEBO_PLUGIN":"false",
                "ROS_AWS_REGION": "us-west-2"
            },
            "robot_app":{
                "packageName": "rs_config",
                "launchFile": "navigate.launch",
                "environmentVariables": {
                    "ADD_PARAM":"1" 
                }
            },
            "simulation_app": {
                "packageName": "robot_fleet",
                "launchFile": "robot_fleet_rosbridge.launch",
                "environmentVariables": {
                    "ADD_PARAM":"1"
                }
            }
        }
    ],
    "server": {
        "name": "SERVER",
        "environmentVariables": {
            "START_X": "0.5",
            "START_Y": "0",
            "START_YAW": "0",
            "USE_CUSTOM_MOVE_OBJECT_GAZEBO_PLUGIN":"false",
            "ROS_AWS_REGION": "us-west-2"
        },
        "robot_app":{
            "packageName": "rs_config",
            "launchFile": "navigate.launch",
            "environmentVariables": {
                "ADD_PARAM":"1"
            }
        },
        "simulation_app": {
            "packageName": "robot_fleet",
            "launchFile": "robot_fleet_rosbridge.launch",
            "environmentVariables": {
                "ADD_PARAM":"1"
            }
        }
      }
  }
```

## Cleanup

To delete the sample application use the AWS CLI.

For Fleet application
```
aws cloudformation delete-stack --stack-name spottestboro
```