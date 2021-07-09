#!/usr/bin/env python
import rospy
import os
import json
import time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool





pub = rospy.Publisher('ublox_gps_raw', NavSatFix, queue_size=10)
pub_state = rospy.Publisher('ublox_gps_ready', Bool, queue_size=10)
rospy.init_node('ublox_gps_node')


#setPAth = 'set' + '4'
#print('saving to ', setPAth)

lat_dec = 53.312098 
lon_dec = -113.580626 

inc = 0.0005


while not rospy.is_shutdown():

    try:
    
        navfix = NavSatFix()
        navfix.header.stamp = rospy.get_rostime()
        navfix.header.frame_id = "ublox_gps_frame"

        lat_dec += inc 
        lon_dec += inc 
        print('%.6f , %.6f , '%(lat_dec,  lon_dec) + str(navfix.header.stamp))
        
        navfix.latitude  = lat_dec
        navfix.longitude = lon_dec
        pub.publish(navfix)
        
        gps_state = Bool()
        gps_state.data = True
        pub_state.publish(gps_state)
        
        #count = count+1
        
        time.sleep(1)
                
        '''    else:
                print('gps initilizing')
                
                gps_state.data = False
                pub_state.publish(gps_state)
                time.sleep(2)'''
                
                
    except Exception as e: 
        print(e)
        print("Keyboard Interrupt")
        break
        
        

