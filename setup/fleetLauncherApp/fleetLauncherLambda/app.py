import json
import botocore
import boto3
from botocore.exceptions import WaiterError
from botocore.waiter import WaiterModel
from botocore.waiter import create_waiter_with_client
import datetime
import os
import sys
from copy import deepcopy
DEFAULT_ROSBRIDGE_SERVER_PORT = 9090
DEFAULT_IS_PUBLIC = False
DEFAULT_MAX_DURATION = 3600
DEFAULT_STREAM_UI = True
SIM_SERVER_PORT_MAPPINGS = [
  {
    "applicationPort": DEFAULT_ROSBRIDGE_SERVER_PORT,
    "enableOnPublicIp": DEFAULT_IS_PUBLIC,
    "jobPort": DEFAULT_ROSBRIDGE_SERVER_PORT
  }
]
sim_job_params = {
      "maxJobDurationInSeconds": (DEFAULT_MAX_DURATION if "MAX_JOB_DURATION" not in os.environ else os.getenv("MAX_JOB_DURATION")),
      "iamRole": os.getenv("IAM_ROLE"),
      "failureBehavior": "Fail",
      "simulationApplications": [],
      "robotApplications": [],
      "vpcConfig": {
        "securityGroups": [ os.getenv('SECURITY_GROUP') ] if "SECURITY_GROUP" in os.environ else [],
        "subnets": [ os.getenv('SUBNET_1'), os.getenv('SUBNET_2') ] if ("SUBNET_1" in os.environ and "SUBNET_2" in os.environ) else []
      },
      "outputLocation": {
        "s3Bucket": os.getenv("S3_BUCKET"),
        "s3Prefix": "logs" 
      },
      "loggingConfig": {
        "recordAllRosTopics": True
      }
}
app_arn = os.getenv('SIM_APPLICATION_ARN')
robot_arn = os.getenv('ROBOT_APPLICATION_ARN')
waiter_config = {
  'version': 2,
  'waiters': {
    'SimJobCreated': {
      'operation': 'DescribeSimulationJob',
      'delay': 5,
      'maxAttempts': 70,
      'acceptors': [
          { 'matcher': 'path', 'expected': 'Pending', 'argument': 'status', 'state': 'retry' },
          { 'matcher': 'path', 'expected': 'Running', 'argument': 'status',  'state': 'success' },
          { 'matcher': 'path', 'expected': 'Terminated', 'argument': 'status',  'state': 'failure' },
          { 'matcher': 'path', 'expected': 'Completed', 'argument': 'status',  'state': 'failure' },
          { 'matcher': 'path', 'expected': 'Failed', 'argument': 'status',  'state': 'failure' }
        ]
    }
  }
}
robomaker = boto3.client('robomaker')
def create_application_config(input_params, is_server, server_ip): 
  
  if (is_server):
      port_mappings = SIM_SERVER_PORT_MAPPINGS
  else:
      port_mappings = []
  to_set_params = {
    "application": app_arn,
    "applicationVersion": "$LATEST",
    "launchConfig": {
      "environmentVariables": input_params['simulation_app']['environmentVariables'],
      "launchFile": input_params['simulation_app']['launchFile'],
      "packageName": input_params['simulation_app']['packageName'],
      "portForwardingConfig" : { 'portMappings': port_mappings },
      "streamUI": DEFAULT_STREAM_UI
    }
  }
  
  for key in input_params['environmentVariables']:
    to_set_params["launchConfig"]["environmentVariables"].update({key : input_params['environmentVariables'][key]})

  to_set_params['launchConfig']['environmentVariables']['ROBOT_NAME'] = input_params['name']
  if (is_server):
    to_set_params['launchConfig']['environmentVariables']['ROSBRIDGE_STATE'] = "SERVER"
    to_set_params['launchConfig']['environmentVariables']['ROSBRIDGE_IP'] = "localhost"
  elif (server_ip):
    to_set_params['launchConfig']['environmentVariables']['ROSBRIDGE_STATE'] =  "CLIENT"
    to_set_params['launchConfig']['environmentVariables']['ROSBRIDGE_IP'] = server_ip
  return to_set_params
  
def create_robot_config(input_params): 
  
  to_set_params = {
    "application": robot_arn,
    "applicationVersion": "$LATEST",
    "launchConfig": {
      "environmentVariables": input_params["robot_app"]['environmentVariables'],
      "launchFile": input_params["robot_app"]['launchFile'],
      "packageName": input_params["robot_app"]['packageName'],
      # "portForwardingConfig" : { 'portMappings': port_mappings },
      "streamUI": DEFAULT_STREAM_UI
    }
  }
  for key in input_params['environmentVariables']:
    to_set_params["launchConfig"]["environmentVariables"].update({key : input_params['environmentVariables'][key]})
  return to_set_params

  
def lambda_handler(event, context):
  
  # Set base parameters for each simulation job. Order from Lambda event, then environment variables.
  if 'simulationJobParams' in event:
    if 'vpcConfig' in event['simulationJobParams']:
      sim_job_params['vpcConfig'] = sim_job_params['vpcConfig']
    
    if 'iamRole' in event['simulationJobParams']:
      sim_job_params['iamRole'] = sim_job_params['iamRole']
    
    if 'outputLocation' in event['simulationJobParams']:
      sim_job_params['outputLocation'] = sim_job_params['outputLocation']
    
  if 'simulationApplicationArn' in event:
    app_arn = event['simulationApplicationArn']
  if 'robotApplicationArn' in event:
    robot_arn = event['robotApplicationArn']
  if 'serverIP' in event:
    private_ip = event['serverIP']
  else:
    # Launch Server.
    server_app_params = create_application_config(event['server'], True, 'localhost')
    robot_app_params = create_robot_config(event['server'])
    server_job_params = deepcopy(sim_job_params)
    server_job_params['simulationApplications'].append(server_app_params)
    server_job_params['robotApplications'].append(robot_app_params)
    server_job_response = robomaker.create_simulation_job(
      iamRole=server_job_params["iamRole"],
      maxJobDurationInSeconds=server_job_params["maxJobDurationInSeconds"],
      simulationApplications=[server_app_params],
      robotApplications=[robot_app_params],
      vpcConfig=server_job_params["vpcConfig"],
      loggingConfig=server_job_params["loggingConfig"],
      outputLocation=server_job_params["outputLocation"]
    )
    # Wait for server to be available.
    waiter_name = 'SimJobCreated'
    waiter_model = WaiterModel(waiter_config)
    custom_waiter = create_waiter_with_client(waiter_name, waiter_model, robomaker)
    custom_waiter.wait(job=server_job_response['arn'])
    desc_result = robomaker.describe_simulation_job( job = server_job_response['arn'] )
    private_ip = desc_result['networkInterface']['privateIpAddress']
  # skip simulation batch job creation of we don't have other robots in
  if len(event['robots']) == 0:
    return {
    'statusCode': 200,
    'body': json.dumps({
            'simulation_job': server_job_response['arn']
        })
    }
  # Launch multiple robot batches.
  batch_job_requests = []
  client_app_params = {}
  client_robot_params = {}
  client_job_params = {}
  for robot in event['robots']:
    client_app_params[robot['name']] = create_application_config(robot, False, private_ip)
    client_robot_params[robot['name']] = create_robot_config(robot)
    client_job_params[robot['name']] = deepcopy(sim_job_params)
    client_job_params[robot['name']]['simulationApplications'].append(client_app_params[robot['name']])
    client_job_params[robot['name']]['robotApplications'].append(client_robot_params[robot['name']])
    batch_job_requests.append(client_job_params[robot['name']])
  response = robomaker.start_simulation_job_batch(
        batchPolicy={
          'timeoutInSeconds': DEFAULT_MAX_DURATION,
          'maxConcurrency': len(event['robots'])
          }, 
        createSimulationJobRequests=batch_job_requests, 
        #tags = {
        #  'launcher': 'multi_robot_fleet'
        #}
        )
  return {
    'statusCode': 200,
    'body': json.dumps({
            'simulation_job': server_job_response['arn'],
            'simulation_batch': response['arn']
        })
  } 
# To run locally on your machine based on CFN stack outputs.
if __name__ == "__main__":
  sim_job_path = ".simulation_jobs"
  # check if we have run jobs
  try:
    with open(sim_job_path) as json_file:
      robomaker = boto3.client('robomaker')
      json_str = json.load(json_file)
      jobs = json.loads(json_str)
      try:
        print("Cancel simulation job")
        print(jobs['simulation_job'])
        if jobs['simulation_job'] is not None:
          robomaker.cancel_simulation_job(job=jobs['simulation_job'])
      except Exception as e:
        print(e)    
      
      try:
        print("Cancel simulation batch job")
        print(jobs['simulation_batch'] )
        if jobs['simulation_batch'] is not None:
          robomaker.cancel_simulation_job_batch(batch=jobs['simulation_batch'])
      except Exception as e:
        print(e)    
  except Exception as e:
    print(e)
  if sys.argv[2]:
    print("Using config file {}".format(sys.argv[2]))
    event_path = sys.argv[2]
  else:
    print("Using default config file")
    # Get Sample Event
    event_path = "%s/event.json" % sys.path[0]
  with open(event_path) as f:
    event = json.load(f)
  
  if (len(sys.argv)>1):
    cfn = boto3.client('cloudformation')
    response = cfn.describe_stacks(
        StackName=sys.argv[1]
    )
    if len(response['Stacks'])>0:
      if len(response['Stacks'][0]['Outputs'])>0:
        for output in response['Stacks'][0]['Outputs']:
          if output['OutputKey'] == 'PublicSubnet1' or output['OutputKey'] == 'PublicSubnet2':
            sim_job_params['vpcConfig']['subnets'].append(output['OutputValue'])
          elif output['OutputKey'] == 'DefaultSecurityGroupID':
            sim_job_params['vpcConfig']['securityGroups'].append(output['OutputValue'])
          elif output['OutputKey'] == 'SimulationRole':
            sim_job_params['iamRole'] = output['OutputValue']
          elif output['OutputKey'] == 'RoboMakerS3Bucket':
            sim_job_params['outputLocation']['s3Bucket'] = output['OutputValue']
          elif output['OutputKey'] == 'SimulationApplicationARN':
            app_arn = output['OutputValue']
          elif output['OutputKey'] == 'RobotApplicationARN':
            robot_arn = output['OutputValue']
      else:
        print("Cloudformation stack did not any outputs. Using event.json values or environment variables for AWS infrastructure configuration.")
    else:
      print("Cloudformation stack not found. Using event.json values or environment variables for AWS infrastructure configuration.")
  else:
    print("No cloudformation defined. Using event.json values or environment variables for AWS infrastructure configuration.")
  
  # If we want to send data to s3 we need to assign public IP for VNC config
  sim_job_params["vpcConfig"]["assignPublicIp"] = True
    
  print("Starting handler")
  res = lambda_handler(event, {})
  print("Simulations launched. Check out the AWS console to connect to the fleet simulation.")
  print("Created jobs:")
  print(res['body'])
  
  
  with open(sim_job_path, 'w') as outfile:
    json.dump(res['body'], outfile)