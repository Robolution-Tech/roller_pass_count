"""
File created by Bowen Xie, 2021
bowen.x@robolution.ca
Copyright (c) Robolution Technologies Inc. and its affiliates.
"""
import json
import logging
import datetime
import time
import os
import configparser
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from sensor_msgs.msg import NavSatFix
from script.helpers import *

# Global settings
config_base_path = "config"
project_config = os.path.join(config_base_path, "project_config.ini")


class AWSConnector():
    def __init__(self):
        if os.path.exists(project_config):
            raise FileNotFoundError("No project config file is given!")
        self.key_base_path = os.path.realpath(__file__)
        self.parse_variables(config_file)
        self.set_logger()
        self.aws_connect()
        self.setup_ros()
        self.companyId = None
        self.is_aws_connection_on = False
        self.gps_msg = {
            'companyId': self.companyId,
            'projectId': self.projectId,
            'timeStamp': None,
            'gpsData':{
                'gpsDeviceId': None,
                'gpsLat': None,
                'gpsLng': None
            }
        }
        

        #request for all equipment locations, with in the time (startTime, endTime)
        self.location_sharing_request_msg = {
            'companyId': self.companyId,
            'projectId': self.projectId,
            'startTime': None,
            'endTime': None
        }

        self.location_sharing_topic = 'equipment_location_data_topic'
        self.location_sharing_sub(self.location_sharing_topic)

    def setup_ros(self):
        rospy.Subscriber(self.gps_sub, NavSatFix, self.gps_sub_callback)
        rospy.Publisher("/")

    def gps_sub_callback(self, data):
        lat = str(data.latitude)
        lon = str(data.longitude)
        if self.is_aws_connection_on:
            gpsDatetime = datetime.datetime.now().isoformat()
            self.gps_msg_update(gpsDatetime, 1, lon, lat)

    gps_publisher.gps_publish()

    @threaded
    def check_aws_connection(self):
        while True:
            # TO-DO
            time.sleep(self.aws_reconect_timer)

    def set_logger(self):
        logging.basicConfig(filename= 'log/'+str(datetime.datetime.now())+'.log', filemode='w', format='%(name)s - %(levelname)s - %(message)s',level=logging.DEBUG)
        self.logger = logging.getLogger(self.clientId)
        self.logger.info(self.clientId)
        
    def parse_variables(self,config_file):
        parser = configparser.ConfigParser()
        parser.read(config_file)

        # AWS config
        self.clientId = parser['AWS_CONFIG']['clientId']
        self.endpoint = parser['AWS_CONFIG']['endpoint']
        self.certificate_path = os.path.join(self.key_base_path, parser['AWS_CONFIG']['certificate_path'])
        self.rootCA_path = os.path.join(self.key_base_path, parser['AWS_CONFIG']['rootCA_path'])
        self.privateKey_path = os.path.join(self.key_base_path, parser['AWS_CONFIG']['privateKey_path'])
        self.aws_reconect_timer = parser['AWS_CONFIG']['aws_reconect_timer']

        # project config
        self.companyId = parser['PROJECT_CONFIG']['companyId']
        self.projectId = parser['PROJECT_CONFIG']['projectId']

        # ros config
        self.gps_sub = parser['ROS_CONFIG']['gpsSubTopic']
        self.gps_pub = parser['ROS_CONFIG']['gpsPubTopic']

         

    def aws_connect(self):
        self.myMQTTClient = AWSIoTMQTTClient(self.clientId)
        # For TLS mutual authentication
        self.myMQTTClient.configureEndpoint(self.endpoint, 8883)
        self.myMQTTClient.configureCredentials(self.rootCA_path, self.privateKey_path, self.certificate_path) #Set path for Root CA and provisioning claim credentials

        #print(self.endpoint)
        #print(self.rootCA_path)
        #print(self.privateKey_path)
        #print(self.certificate_path)

        self.myMQTTClient.configureAutoReconnectBackoffTime(1,32,20)
        self.myMQTTClient.configureOfflinePublishQueueing(-1)
        self.myMQTTClient.configureDrainingFrequency(2)
        self.myMQTTClient.configureConnectDisconnectTimeout(10)
        self.myMQTTClient.configureMQTTOperationTimeout(5)

        self.logger.info("Device Connecting...")
        self.myMQTTClient.connect()

    def gps_msg_update(self,gpsDatTm, gpsDeviceId, gpsLat, gpsLng):
        self.gps_msg['timeStamp'] = gpsDatTm
        self.gps_msg['gpsData']['gpsDeviceId'] = gpsDeviceId
        self.gps_msg['gpsData']['gpsLat'] = gpsLat
        self.gps_msg['gpsData']['gpsLng'] = gpsLng

    def gps_publish(self, data = 0):
        self.logger.info("GPS Publishing...")
        messageJson = json.dumps(self.gps_msg)
        self.myMQTTClient.publish("gps_topic", messageJson, 0)

    def aws_disconnect(self):
        self.logger.info("Device Disconnecting...")
        self.myMQTTClient.disconnect()


    def location_sharing_request(self,startTime=None,endTime=None):
        self.logger.info("location sharing request...")
        self.location_sharing_request_msg['startTime'] = startTime
        self.location_sharing_request_msg['endTime'] = endTime
        messageJson = json.dumps(self.location_sharing_request_msg)
        self.myMQTTClient.publish("equipment_sharing_request_topic",messageJson,0)


    def location_sharing_callback(self,client,userdata,message):
        self.logger.info("location sharing data recived:")
        print(message.payload)
        return(message.payload)

    def location_sharing_sub(self, topic):
        self.logger.info("location sharing subscribe...")
        self.myMQTTClient.subscribe(topic, 1, self.location_sharing_callback)
        time.sleep(2)


if __name__=="__main__":
    config_file = 'config.ini'
    gps_publisher = AWSConnector(config_file)
    gpsDatetime = datetime.datetime.now().isoformat()
    print(gpsDatetime)
    gps_publisher.gps_msg_update(gpsDatetime,1,10.1,10.1)


    gps_publisher.gps_publish()
    gps_publisher.location_sharing_request()

    while True:
        #print('waiting .. .. ')
        time.sleep(2)

    gps_publisher.aws_disconnect()