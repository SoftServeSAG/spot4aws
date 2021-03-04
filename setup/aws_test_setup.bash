
#!/bin/bash

# This script will create the AWS infrastructure required to launch the demo, as well as future multi-robot apps.
BASE_DIR=`pwd`
ROS_APP_DIR=$BASE_DIR/simulation_ws
ROS_ROBOT_DIR=$BASE_DIR/robot_ws
LAUNCHER_APP_DIR=$BASE_DIR/setup
STACK_NAME=spottestboro
AWS_PROFILE=default
CURRENT_STACK=.current-aws-stack
S3_OUTPUT_APP_KEY=spottestdemo/bundle/output.tar
S3_OUTPUT_ROBOT_KEY=spottestdemorobot/bundle/output.tar


export AWS_DEFAULT_REGION=us-east-2

# sudo pip3 install boto3==1.14.28 > /dev/null

# Setup AWS resources for the application
STACK_TRUE=$(aws cloudformation describe-stacks --stack-name $STACK_NAME --query 'Stacks[0].StackName' --profile $AWS_PROFILE --output text)

# Setup AWS resources for the application
if [ ! $STACK_TRUE ]
then
  # Deploy base stack (NOTE: This will NOT deploy the SAM-based Lambda function. To do that, follow the instructions in the README.)
  aws cloudformation deploy --template-file $LAUNCHER_APP_DIR/fleetLauncherApp/base_template.yml --stack-name $STACK_NAME --capabilities CAPABILITY_NAMED_IAM --parameter-overrides SimulationApplicationS3Key=$S3_OUTPUT_APP_KEY RobotApplicationS3Key=$S3_OUTPUT_ROBOT_KEY --profile $AWS_PROFILE
  aws cloudformation wait stack-create-complete --stack-name $STACK_NAME --profile $AWS_PROFILE && echo "stackname=$STACK_NAME" > .current-aws-stack
else
    # Deploy base stack (NOTE: This will NOT deploy the SAM-based Lambda function. To do that, follow the instructions in the README.)
  aws cloudformation deploy --template-file $LAUNCHER_APP_DIR/fleetLauncherApp/base_template.yml --stack-name $STACK_NAME --capabilities CAPABILITY_NAMED_IAM --parameter-overrides SimulationApplicationS3Key=$S3_OUTPUT_APP_KEY RobotApplicationS3Key=$S3_OUTPUT_ROBOT_KEY --profile $AWS_PROFILE
  aws cloudformation wait stack-update-complete --stack-name $STACK_NAME --profile $AWS_PROFILE && echo "stackname=$STACK_NAME" > .current-aws-stack

fi

echo "Upload the simulation application bundle."
s3Bucket=$(aws cloudformation describe-stacks --stack-name $STACK_NAME --query "Stacks[0].Outputs[?OutputKey=='RoboMakerS3Bucket'].OutputValue" --profile $AWS_PROFILE --output text)

if [ ! $s3Bucket==None ] || [ -f "$ROS_APP_DIR/bundle/output.tar" ]
then
  echo "Uploading to S3. Note: We did not run colcon build or colcon bundle. Using current output.tar."
  aws s3 cp $ROS_APP_DIR/bundle/output.tar s3://$s3Bucket/$S3_OUTPUT_APP_KEY --profile $AWS_PROFILE
else
  fail "Bundle could not be uploaded. Please ensure \"$ROS_APP_DIR/bundle/output.tar\" and the S3 bucket \"$s3Bucket\" exist."
fi

echo " Upload the robot application bundle "
if [ ! $s3Bucket==None ] || [ -f "$ROS_ROBOT_DIR/bundle/output.tar" ]
then
  echo "Uploading to S3. Note: We did not run colcon build or colcon bundle. Using current output.tar."
  aws s3 cp $ROS_ROBOT_DIR/bundle/output.tar s3://$s3Bucket/$S3_OUTPUT_ROBOT_KEY --profile $AWS_PROFILE
else
  fail "Bundle could not be uploaded. Please ensure \"$ROS_ROBOT_DIR/bundle/output.tar\" and the S3 bucket \"$s3Bucket\" exist."
fi

read -t 5 -p "Press any key to launch the sample simulation, or Ctrl+c within 5 seconds to exit." some_key

if [ "$#" -eq  "0" ]
 then
   python3 $LAUNCHER_APP_DIR/testLauncherApp/app.py $STACK_NAME 
else
   python3 $LAUNCHER_APP_DIR/testLauncherApp/app.py $STACK_NAME $BASE_DIR/$1
fi
 
