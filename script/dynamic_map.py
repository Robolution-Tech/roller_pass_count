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
from std_msgs.msg import Bool, String, Int32MultiArray
from update_map import gen_map
from os import listdir
from os.path import isfile, join

class gps2grid():

    def __init__(self):
    
        # # load origin latitude and lonitude values
        # origin_f = open('lat_lon_origin.json', 'r')
        # f_content = json.load(origin_f)
        # self.kLat_origin, self.kLon_origin = f_content["lat"], f_content["lon"]
        # print('\norigin ')
        # print(self.kLat_origin, self.kLon_origin)

        # load global tile info
        all_tile_info_file = open('global_tile_coord.json','r')
        all_tile_info_file_content = json.load(all_tile_info_file)
        all_tile_info_file.close()

        self.all_tile_center_lats = all_tile_info_file_content["lat_centers_list"]
        self.all_tile_center_lons = all_tile_info_file_content["lon_centers_list"]

        t1 = time.time()
        
        self.sub = rospy.Subscriber("/ublox_gps_raw", NavSatFix, self.callback)
        self.new_map_request = rospy.Publisher("/map_download_request", String, queue_size=100)
        print('\nGPS subscriber started')
        
        # wait until the gps module is ready
        #print('\ncheck if GPS module is outputting') 
        gps_state = False
        while gps_state == False:
            msg = rospy.wait_for_message("/ublox_gps_ready" , Bool)
            state = msg.data
            print('\nChecking gps state. Received: {}'.format(state))
            if state == True:
                gps_state = True
            else:
                gps_state = False
        
        # get the inital lat-lon 
        print('\ngetting the first gps reading ')
        msg = rospy.wait_for_message("/ublox_gps_raw" , NavSatFix)
        self.initial_lat = msg.latitude
        self.initial_lon = msg.longitude

        print( self.LatLonToTileIndex(self.initial_lat, self.initial_lon) )

        self.requested_tile_xy = []

        self.current_center_tile_index = [0,0]
        
        
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

    def callback(self, data):
        lat = data.latitude 
        lon = data.longitude 

        new_ll_tile_index = self.LatLonToTileIndex(lat, lon)
        if new_ll_tile_index == self.current_center_tile_index:
            print("use same tile")
        else:
            print("need to update map image")
            index_x = new_ll_tile_index[1]
            index_y = new_ll_tile_index[0]
            # check the 8 tiles around this center tile 
            
            file_names_list = [f for f in listdir('maps/') if isfile(join('maps/', f))]
            def is_this_file_exist(filename):
                return filename in file_names_list
            
            imgs_to_download = []
            # imgs_to_download_xy = []
            imgs_to_download_xy_string = ""
            imgs_to_download_center_latlon = []

            x_start = index_x - 1
            y_start = index_y - 1 

            for i in range(3):
                for j in range(3):
                    x = x_start+i 
                    y = y_start+j 
                    img_name = '{}_{}.jpg'.format( x , y )
                    if img_name in file_names_list:
                        print( img_name + ' exist, no need to download')
                    else: 
                        # print('need to download ' + img_name)
                        imgs_to_download.append(img_name)

                        # imgs_to_download_xy.append( int(x) )
                        # imgs_to_download_xy.append( int(y) )
                        imgs_to_download_xy_string += str(int(x)) + ' ' 
                        imgs_to_download_xy_string += str(int(y)) + ' ' 

                        imgs_to_download_center_latlon.append( self.all_tile_center_lons[x] ) 
                        imgs_to_download_center_latlon.append( self.all_tile_center_lats[y] ) 
                        

            if len(imgs_to_download) != 0:
                print('\nneed to download these: ', imgs_to_download )

                # now publish these info to other node that downloads the images 
                self.new_map_request.publish(String(imgs_to_download_xy_string + ';' + str(imgs_to_download_center_latlon)))





        
    def LatLonToTileIndex(self, lat, lon):
        # check the range of input, give warning if not right 
        if lat > 65 or lat < 45 :
            print('\n*********************************')
            print('\n*********************************')
            print('check input latitude value range ')
            print('\n*********************************')
            print('\n*********************************')

        if lon > -106 or lon < -125 :
            print('\n*********************************')
            print('\n*********************************')
            print('check input longitude value range ')
            print('\n*********************************')
            print('\n*********************************')

        # find the tile where this latlon locate at 
        lat_diffs_from_all_tiles = np.abs( np.array( self.all_tile_center_lats) - lat )
        lon_diffs_from_all_tiles = np.abs( np.array( self.all_tile_center_lons) - lon ) 
        lat_index = np.argmin(lat_diffs_from_all_tiles)
        lon_index = np.argmin(lon_diffs_from_all_tiles)

        return [lat_index, lon_index]








    #@staticmethod
    def Assemble9Maps(self, request_lat, request_lon ):

        earth_radius = 6378137.0
        earth_circumference = 2 * earth_radius * np.pi

        # assume 1:1 aspect ratio
        meter_per_lat = earth_circumference / 360
        meter_per_lon = earth_circumference * np.cos(request_lat * np.pi / 180) / 360

        # resolution: meters/pixel
        resolution = (earth_radius * 2 * math.pi / self.img_resolution) * math.cos(request_lat * math.pi / 180) / (2 ** self.zoom_level)

        # Now calculating the corner coordinates (lon, lat):
        diff_meters = self.img_resolution / 2 * resolution
        diff_in_lon = diff_meters / meter_per_lon
        diff_in_lat = diff_meters / meter_per_lat
        
        # Now assemble the cooridnates: (lon, lat)
        upper_left = (lon - diff_in_lon, lat + diffs_in_lat)
        upper_right = (lon + diff_in_lon, lat + diff_in_lat)
        bottom_left = (lon - diff_in_lon, lat - diff_in_lat)
        bottom_right = (lon + diff_in_lon, lat - diff_in_lat)
        
        corner_coords = [upper_left, upper_right, bottom_left, bottom_right]
        
        #center_tile_lon_indix = (request_lon - self.kLon_origin ) /  
        
        #map[key] = map[key] + 1 if key in map else 1 
        #return map    




def main():
    
    rospy.init_node('passnode', anonymous=True)
    
    obc = gps2grid()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()








