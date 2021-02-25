BASE_DIR=`pwd`
ROS_APP_DIR=$BASE_DIR/simulation_ws
ROS_ROBOT_DIR=$BASE_DIR/robot_ws
LAUNCHER_APP_DIR=$BASE_DIR/setup/fleetLauncherApp
STACK_NAME=spottestboro
AWS_PROFILE=default
CURRENT_STACK=.current-aws-stack
S3_OUTPUT_APP_KEY=spottestdemo/bundle/output.tar
S3_OUTPUT_ROBOT_KEY=spottestdemorobot/bundle/output.tar

echo "###############################################################################"
echo " CLOUD SUMMIT WORKSHOP: RUNNING SIMULATION JOBS WITH JSON CONFIGURATION "
echo " NOTE: This will take 1-2 minutes for a single robot simulation, 3-4 minutes for multiple robots "
echo " Once this is running, open the RoboMaker console to watch for a change from Pending to Running "
echo "###############################################################################"

function fail {
    printf '%s\n' "$1" >&2  ## Send message to stderr. Exclude >&2 if you don't want it that way.
    exit "${2-1}"  ## Return a code specified by $2 or 1 by default.
}

if [ $# -eq 0 ]
  then
    fail "Can not launch application. You need to supply the path to your JSON configuration file. eg: run.sh launch_single.json"
fi

running=$(aws robomaker list-simulation-jobs --profile $AWS_PROFILE --filters name="status",values="Running")
empty="\"simulationJobSummaries\": []"

if [[ ! $running == *"$empty"* ]]
then
   fail "There are still simulations running. Please click on the robomaker console and cancel the running simulations before continuing."
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


read -t 15 -p "Press any key to launch the sample simulation, or Ctrl-C to exit." some_key
python3 $LAUNCHER_APP_DIR/fleetLauncherLambda/app.py $STACK_NAME $1

