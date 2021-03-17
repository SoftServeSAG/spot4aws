# Scenario-based Tests with AWS RoboMaker for Boston Dynamics' Spot
The repository contains packages for launching scenario-based tests for Spot in AWS Robomaker.

The simulation package for Spot is [champ](https://github.com/chvmp/champ) project which is an open source development framework for controlling quadrupedal robots. 

The IAM user needs read and write permissions to several services, including provisioning resources with AWS CloudFormation, AWS RoboMaker, Amazon S3, AWS Cloud9, Amazon CloudWatch, Amazon VPC, AWS Lambda and AWS Identity and Access Management. 
Please make sure you have these permissions in your account before proceeding further.

## Requirements
- ROS Melodic
- Colcon 
- Gazebo9 
- Colcon bundle 
- boto3 
- awscli 

## Setup
1. Create a development environment in AWS RoboMaker.
2. Download this repository to the created environment.
3. Give to the main scrips execute permission:
````shell
   chmod +x /aws_ros_spot_test/setup.sh
   chmod +x /aws_ros_spot_test/setup/ros_setup.sh
   chmod +x /aws_ros_spot_test/run.sh
````
4. In the terminal of your environment run the setup file:
````shell
   ./aws_ros_spot_test/setup.sh
 ````
   That performs the next steps:
   - Downloads and installs all needed ROS dependencies.
   - Runs initial colcon build and colcon bundle operations.
   - Deploys the cloudformation stack to setup the AWS resources (IAM roles, VPC, etc).

# Run test

1. Configure scenarios in .json files.

In order to launch test, the scenarios should be configured. Directory **test_launch_json** consists of the predefined 
scenarios for tests available in package [aws_ros_tests](https://github.com/SoftServeSAG/aws_ros_tests),
which includes the test types as: slippage and motor saturation tests, planning tests, behaviour tests, localization tests.

For example, the .json file to configure coverage test given below.The coverage test shows the efficiency of the robot's ability to cover the floor of the environment. 
Here, the test-related parameter is **ROBOT_COVERAGE_TEST_COVERAGE_GOAL** which determines the to coverage goal. You can set the different goal values in different scenarios.

```json
{
   "scenarios":{
      "Scenario1":{
         "simEnvironmentVariables":{
            "MODEL_NAME":"/",
            "START_X":"0",
            "START_Y":"0",
            "START_Z":"0.0",
            "START_YAW":"0",
            "ROBOT_COVERAGE_TEST_COVERAGE_GOAL":"80"
         },
         "robotEnvironmentVariables":{
            "MODEL_NAME":"/",
            "START_X":"0",
            "START_Y":"0",
            "START_Z":"0.0",
            "START_YAW":"0"
         }
      },
      "Scenario2":{
         "simEnvironmentVariables":{
            "MODEL_NAME":"/",
            "START_X":"0",
            "START_Y":"0",
            "START_Z":"0.0",
            "START_YAW":"0",
            "ROBOT_COVERAGE_TEST_COVERAGE_GOAL":"10"
         },
         "robotEnvironmentVariables":{
            "MODEL_NAME":"/",
            "START_X":"0",
            "START_Y":"0",
            "START_Z":"0.0",
            "START_YAW":"0"
         }
      }
   },
   "simulations":[
      {
         "scenarios":[
            "Scenario1", 
            "Scenario2"
         ],
         "params":{
            "failureBehavior": "Fail",
			   "maxJobDurationInSeconds": 600,
            "simulationApplications":[
               {
                  "applicationVersion": "$LATEST",
                  "launchConfig":{
                     "launchFile":"coverage_test.launch",
                     "packageName":"rs_tests"
                  }
               }
            ],
            "robotApplications":[
               {
                  "applicationVersion": "$LATEST",
                  "launchConfig":{
                     "launchFile":"coverage_test.launch",
                     "packageName":"rs_robot_tests"
                  }
               }
            ],
            "vpcConfig": {
				   "assignPublicIp": true
            }
         }
      }
   ]
}
```
2. Run test

In order to run test, execute the following command:
```
cd ~/environment/aws_ros_tests
./run.sh test_launch_json/<json_file_name>.json
```
For example, run the coverage test:
```
cd ~/environment/aws_ros_tests
./run.sh test_launch_json/coverage_test.json.json
```
This command launches two Simulation Jobs picking parameters determined in .json file for two scenarios. 
When the jobs complete, you will see the test results tagged to the simulation jobs.

## Cleanup

To delete the sample application use the AWS CLI.
```
aws cloudformation delete-stack --stack-name spottest
```