import json
import time
import uuid
import os
import boto3
from copy import deepcopy
import sys

client = boto3.client('robomaker')

DEFAULT_MAX_DURATION = 3600

app_arn = os.getenv('SIM_APPLICATION_ARN')
robot_arn = os.getenv('ROBOT_APPLICATION_ARN')

def lambda_handler(event, context):
    '''
        In this sample lambda function, multiple simulation jobs will be run based on a common configuration.
        Below is a sample event that you could use to invoke the lambda function and launch a set of simulations.
        
        Input Event:
        {
            'codePipelineID': String | The ID of the CodePipeline job
            'scenarios': {
                '': {
                    'robotEnvironmentVariables': {}
                    'simEnvironmentVariables': {}
                }
            },
            'simulations': [{
                'scenarios': ['<SCENARIO_NAME>']
                'params': CreateSimulationJobParams
            }]
        }
        Output Event:
        { 
            isDone: Boolean | If the batch simulation describe call returns complete.
            batchSimJobArn: String | The ARN of the simulation batch.
            status: String | InProgress, Success or Failed for downstream processing.
            codePipelineJobId: String | The ID of the active CodePipeline job.
        }
    '''

    output = {
        'isDone': False,
        'batchSimJobArn': None
    }
    
    jobs = []
    
    # This will loop through each defined simulation job and inject the environment variables for the scenarios associated.
    # If parameters are not set in the input event JSON (Robot, Sim ARNs, Subnets, etc), it will use the defaults created from CloudFormation
    for simulation in event['simulations']:
            
        print('Preparing simulation %s...' % json.dumps(simulation))

        if 'S3_BUCKET' in os.environ and not simulation.get('params', {}).get('outputLocation', {}).get('s3Bucket', {}):
            if not "outputLocation" in simulation['params']:
                simulation['params']['outputLocation'] = {}
            simulation['params']['outputLocation']['s3Bucket'] = os.getenv('S3_BUCKET')
 
        if 'IAM_ROLE' in os.environ and not "iamRole" in simulation['params']:
            simulation['params']['iamRole'] = os.getenv('IAM_ROLE')
                    
        if 'vpcConfig' in simulation['params']:
            if 'SECURITY_GROUP' in os.environ and os.getenv('SECURITY_GROUP') and "securityGroups" in simulation['params']['vpcConfig']:
                simulation['params']['vpcConfig']['securityGroups'].append(os.getenv('SECURITY_GROUP'))
            if not 'subnets' in simulation['params']['vpcConfig']:
                simulation['params']['vpcConfig']['subnets'] = []
            if 'SUBNET_1' in os.environ and os.getenv('SUBNET_1') not in simulation['params']['vpcConfig']['subnets']:
                simulation['params']['vpcConfig']['subnets'].append(os.getenv('SUBNET_1'))
            if 'SUBNET_2' in os.environ and os.getenv('SUBNET_2') not in simulation['params']['vpcConfig']['subnets']:
                simulation['params']['vpcConfig']['subnets'].append(os.getenv('SUBNET_2'))
    
        simulation['params']['vpcConfig']['assignPublicIp'] = True
        
        for x, scenario in enumerate(simulation['scenarios']):
            
            if scenario in event['scenarios'].keys():
                
                _sim_params = deepcopy(simulation['params'])
                
                print('Scenario %s found...' % scenario)
        
                _sim_params['tags'] = { 'Scenario': scenario }
                y, z = 0, 0
        
                    
                for z, simApp in enumerate(_sim_params['simulationApplications']):
                    _sim_params['simulationApplications'][z]['launchConfig']['environmentVariables'] = event['scenarios'][scenario]['simEnvironmentVariables']
                    if 'SIMULATION_APP_ARN' in os.environ and not 'application' in _sim_params['simulationApplications'][z]:
                        _sim_params['simulationApplications'][z]['application'] = os.getenv('SIMULATION_APP_ARN')
                for z, simApp in enumerate(_sim_params['robotApplications']):
                    _sim_params['robotApplications'][z]['launchConfig']['environmentVariables'] = event['scenarios'][scenario]['robotEnvironmentVariables']
                   
                    if 'ROBOT_APP_ARN' in os.environ and not 'application' in _sim_params['robotApplications'][z]:
                        _sim_params['robotApplications'][z]['application'] = os.getenv('ROBOT_APP_ARN')
                
                print('Adding following job: ' + json.dumps(_sim_params))
                
                jobs.append(_sim_params)
                
            else:
                raise Exception('Scenario %s does not exist.' % scenario)
    print(jobs)        
    response = client.start_simulation_job_batch(
            batchPolicy={
                'timeoutInSeconds': 800,
                'maxConcurrency': 4
            }, 
            createSimulationJobRequests=jobs, 
            tags = {
                'launcher': 'tests',
        })

    output['batchSimJobArn'] = response['arn']
        
    if not output['batchSimJobArn']:
        raise Exception('Error launching batch simulation jobs. Check your scenarios JSON document.')
        
    return output
    
    # To run locally on your machine based on CFN stack outputs.
if __name__ == "__main__":

#   sim_job_path = ".simulation_jobs"

#   # check if we have run jobs
#   try:
#     with open(sim_job_path) as json_file:
#       robomaker = boto3.client('robomaker')
#       json_str = json.load(json_file)
#       jobs = json.loads(json_str)

#       try:
#         print("Cancel simulation job")
#         print(jobs['simulation_job'])
#         if jobs['simulation_job'] is not None:
#           robomaker.cancel_simulation_job(job=jobs['simulation_job'])
#       except Exception as e:
#         print(e)    
      
#       try:
#         print("Cancel simulation batch job")
#         print(jobs['simulation_batch'] )
#         if jobs['simulation_batch'] is not None:
#           robomaker.cancel_simulation_job_batch(batch=jobs['simulation_batch'])
#       except Exception as e:
#         print(e)    
#   except Exception as e:
#     print(e)

  if len(sys.argv) > 2:
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
          if output['OutputKey'] == 'PublicSubnet1':
            os.environ["SUBNET_1"] = output['OutputValue']
          if output['OutputKey'] == 'PublicSubnet2':
            os.environ["SUBNET_2"] = output['OutputValue']
          elif output['OutputKey'] == 'DefaultSecurityGroupID':
            os.environ["SECURITY_GROUP"] = output['OutputValue']
          elif output['OutputKey'] == 'SimulationRole':
            os.environ["IAM_ROLE"] = output['OutputValue']
          elif output['OutputKey'] == 'RoboMakerS3Bucket':
            os.environ["S3_BUCKET"] = output['OutputValue']
          elif output['OutputKey'] == 'SimulationApplicationARN':
            os.environ["SIMULATION_APP_ARN"] = output['OutputValue']
            app_arn = output['OutputValue']
          elif output['OutputKey'] == 'RobotApplicationARN':
            os.environ["ROBOT_APP_ARN"] = output['OutputValue']
            robot_arn = output['OutputValue']
      else:
        print("Cloudformation stack did not any outputs. Using event.json values or environment variables for AWS infrastructure configuration.")
    else:
      print("Cloudformation stack not found. Using event.json values or environment variables for AWS infrastructure configuration.")
  else:
    print("No cloudformation defined. Using event.json values or environment variables for AWS infrastructure configuration.")

    
  print("Starting handler")
  res = lambda_handler(event, {})
  print("Simulations launched. Check out the AWS console to connect to the fleet simulation.")
  print("Created jobs:")
  print(res['body'])
  
  
#   with open(sim_job_path, 'w') as outfile:
#     json.dump(res['body'], outfile)