#!/bin/bash

# This script will create the AWS infrastructure required to launch the demo, as well as future multi-robot apps.
BASE_DIR=`pwd`
ROS_APP_DIR=$BASE_DIR/simulation_ws
ROS_ROBOT_DIR=$BASE_DIR/robot_ws
LAUNCHER_APP_DIR=$BASE_DIR/setup/fleetLauncherApp
STACK_NAME=spottestboro
AWS_PROFILE=default
CURRENT_STACK=.current-aws-stack
S3_OUTPUT_APP_KEY=spottestdemo/bundle/output.tar
S3_OUTPUT_ROBOT_KEY=spottestdemorobot/bundle/output.tar

sudo pip3 install awscli boto3 --upgrade
$BASE_DIR/setup/ros_setup.bash

STACK_TRUE=$(aws cloudformation describe-stacks --stack-name $STACK_NAME --query 'Stacks[0].StackName' --profile $AWS_PROFILE --output text)

# Setup AWS resources for the application
if [ ! $STACK_TRUE ]
then
  # Deploy base stack (NOTE: This will NOT deploy the SAM-based Lambda function. To do that, follow the instructions in the README.)
  aws cloudformation deploy --template-file $LAUNCHER_APP_DIR/base_template.yml --stack-name $STACK_NAME --capabilities CAPABILITY_NAMED_IAM --parameter-overrides SimulationApplicationS3Key=$S3_OUTPUT_APP_KEY RobotApplicationS3Key=$S3_OUTPUT_ROBOT_KEY --profile $AWS_PROFILE
  aws cloudformation wait stack-create-complete --stack-name $STACK_NAME --profile $AWS_PROFILE && echo "stackname=$STACK_NAME" > .current-aws-stack
else
    # Deploy base stack (NOTE: This will NOT deploy the SAM-based Lambda function. To do that, follow the instructions in the README.)
  aws cloudformation deploy --template-file $LAUNCHER_APP_DIR/base_template.yml --stack-name $STACK_NAME --capabilities CAPABILITY_NAMED_IAM --parameter-overrides SimulationApplicationS3Key=$S3_OUTPUT_APP_KEY RobotApplicationS3Key=$S3_OUTPUT_ROBOT_KEY --profile $AWS_PROFILE
  aws cloudformation wait stack-update-complete --stack-name $STACK_NAME --profile $AWS_PROFILE && echo "stackname=$STACK_NAME" > .current-aws-stack

fi