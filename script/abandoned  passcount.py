#import plotly.graph_objects as go
#import pandas as pd
#import csv
#import glob
import json
import os
import numpy as np
from numpy.core.fromnumeric import partition
from numpy.lib.shape_base import split
from bresenham import bresenham
import math
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import NavSatFix
import cv2 
#from draw_icon import rotate_image, add_icon
import time
from std_msgs.msg import Bool
from update_map import gen_map


class gps2grid():

    def __init__(self):
        
        t1 = time.time()
        
        #self.sub = rospy.Subscriber("/ublox_gps_raw",NavSatFix,self.callback)
        
        # wait until the gps module is ready 
        gps_state = False
        while gps_state == False:
            msg = rospy.wait_for_message("/ublox_gps_ready" , Bool)
            state = msg.data
            print('Checking gps state. Received: {}'.format(state))
            if state == True:
                gps_state = True
            else:
                gps_state = False
        
        # get the inital lat-lon 
        msg = rospy.wait_for_message("/ublox_gps_raw" , NavSatFix)
        initial_lat = msg.latitude
        initial_lon = msg.longitude
        
        # get the first set of maps 
        zoom_level, img_resolution = 17, 1024
        corners, diffs = gen_map(initial_lon, initial_lat, zoom_level, img_resolution, 'map1.jpg')
        print(corners)
        print(diffs)
        diffs[0] *= 4
        diffs[1] *= 4
        gen_map(initial_lon+diffs[0]*1, initial_lat, zoom_level, img_resolution, 'map2.jpg')
        gen_map(initial_lon+diffs[0]*2, initial_lat, zoom_level, img_resolution, 'map3.jpg')
        gen_map(initial_lon+diffs[0]*0, initial_lat-diffs[1]*1, zoom_level, img_resolution, 'map4.jpg')
        gen_map(initial_lon+diffs[0]*1, initial_lat-diffs[1]*1, zoom_level, img_resolution, 'map5.jpg')
        # main loop conuter 
        self.cb_counter = 0
        
        self.ll_last = [0,0]
        self.ll= [0,0]
        
        self.mchn_w = 6 
        self.mchn_l = 6 
        
        self.cell_size = 5

        self.click_pt_1 = [53.315735, -113.588066 ] #[53.313459, -113.580967 ]
        self.click_pt_2 = [53.308094, -113.573614 ] # [53.309228, -113.575307 ]

        self.deg_near_last = 0

        '''

        self.road_area_layer = 0
        self.pass_count_layer = 1
        self.lat_int_layer = 2
        self.lon_int_layer = 3

        self.grid = 0

        self.lats = []
        self.lons = []
        
        t2 = time.time()
        print('\ntime from start to b4 setup grids : {}'.format(t2-t1))

        self.setup_grids()
        
        t3 = time.time()
        print('\ntime to setup grids : {}'.format(t3-t2))

        # setup base map
        self.zoom_level_near = 18
        self.zoom_level_far = 15.5

        mapinfofile = open('18.json','r')
        map_info = json.load(mapinfofile)
        self.corner_1_ll = map_info['top_left_corner']
        self.corner_3_ll = map_info['bottom_right_corner']
        
        
        
        # self.center_lon, self.center_lat = -113.577480 , 53.311479
        
        self.center_lon = (self.corner_1_ll[0] + self.corner_3_ll[0]) / 2.0 
        self.center_lat = (self.corner_1_ll[1] + self.corner_3_ll[1]) / 2.0 

        self.base_map = cv2.imread("img_18_stacked_satellite-v9.jpg")
        hsv = cv2.cvtColor(self.base_map, cv2.COLOR_BGR2HSV)
        hsv[:,:,1] = hsv[:,:,1] * 0.5
        self.base_map = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


        self.street_map = cv2.imread("img_18_stacked_streets-v11.jpg")

        self.h, self.w, _ = self.base_map.shape

        self.lon_per_pix = abs( self.corner_3_ll[0] - self.corner_1_ll[0] ) *1000000 / self.w
        self.lat_per_pix = abs( self.corner_3_ll[1] - self.corner_1_ll[1] ) *1000000 / self.h

        print('lat_per_pix  ', self.lat_per_pix)
        print('lon_per_pix  ', self.lon_per_pix)


        earth_radius = 6378137.0 

        earth_circumference = 2 * earth_radius * np.pi
        self.meter_per_lat = earth_circumference / 360
        self.meter_per_lon = earth_circumference * np.cos(self.center_lat * np.pi / 180) / 360

        self.resolution_near = (earth_radius * 2 * math.pi / self.h) * math.cos(self.center_lat * math.pi / 180) / (2 ** self.zoom_level_near) # meter / pixel 
        print("resolution_near:")
        print(self.resolution_near)

        # self.resolution_far = (earth_radius * 2 * math.pi / self.h) * math.cos(self.center_lat * math.pi / 180) / (2 ** self.zoom_level_far) # meter / pixel 
        # print("resolution_far:")
        # print(self.resolution_far)

        self.color_list = [(0,0,255), (255,0,0), (0,255,255), (0,255,0), (100,100,255)]


        self.out = self.base_map.copy()
        self.out_street = self.street_map.copy()


        self.icon = cv2.resize(cv2.imread("roller_icon.png", -1), (40,35) )
        self.last_xy = [0, 0]

        self.window_detection_name = "Pass count "

        self.satelite_size_bar_name = 'satelite_size'
        self.satelite_size_low = 500
        self.satelite_size_lowest = 100
        self.satelite_size_high = 1500
        self.satelite_view_size = 500
        self.satelite_view_size_actual = (512,512)
                

        cv2.namedWindow(self.window_detection_name)
        cv2.createTrackbar(self.satelite_size_bar_name , self.window_detection_name , self.satelite_size_low, self.satelite_size_high, self.on_low_H_thresh_trackbar)
        
        t4 = time.time()
        
        print('\ntime for rest steps in init : {}'.format(t4-t3))
        '''




def main():
    
    rospy.init_node('passnode', anonymous=True)
    
    obc = gps2grid()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()








