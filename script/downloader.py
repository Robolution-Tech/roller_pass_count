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
from os import listdir
from os.path import isfile, join
import threading
from helpers import *


class MapDownloader:
    def __init__(self, tile_file="global_tile_coord.json"):
        if not os.path.exists(tile_file):
            print("No global tile coordinates file found, calculating now ...")
            # TO-DO
        else:
            with open(tile_file) as f:
                self.global_tile_content = json.np.load(f)
        self.all_tile_center_lats = self.global_tile_content["lat_centers_list"]
        self.all_tile_center_lons = self.global_tile_content["lon_centers_list"] 
        self.gps_sub = rospy.Subscriber("/ublox_gps_raw", NavSatFix, self.download_callback)
        self.threads_process = []    
        self.download_queue = set() 
        self.img_resolution = 1024
        self.zoom_level = 17
        self.sleep_time = 0.01

    def download_callback(self, data):
        lat = data.latitude
        lon = data.longitude
        x_index, y_index = self.lat_lon_to_tile_index(lat, lon)
        self.update_download_list(y_index, x_index)
    
    @threaded
    def request_map(self, x_indx, y_indx): 
        print("Now downloading x: {}, y: {} tile".format(x_indx, y_indx))
        center_lon = self.all_tile_center_lons[x_indx]
        center_lat = self.all_tile_center_lats[y_indx]
        img_name = os.path.join("map", "{}_{}.jpg".format(x_indx, y_indx))
        gen_map(center_lon, center_lat, self.zoom_level, self.img_resolution, img_name)
        self.download_queue.remove((x_indx, y_indx))


    def lat_lon_to_tile_index(self, lat, lon): 
        lat_diffs_from_all_tiles = np.abs( np.array( self.all_tile_center_lats) - lat )
        lon_diffs_from_all_tiles = np.abs( np.array( self.all_tile_center_lons) - lon ) 
        lat_index = np.argmin(lat_diffs_from_all_tiles)
        lon_index = np.argmin(lon_diffs_from_all_tiles)
        return (lon_index, lat_index)

    @timer        
    def update_download_list(self, x_index, y_index):
        download_list = set()
        x_check_list = range(x_index-1, x_index+2)
        y_check_list = range(y_index-1, y_index+2)
        for x in x_check_list:
            for y in y_check_list:
                if not os.path.exists(os.path.join("map", "{}_{}.jpg".format(x, y))):
                    download_list.add((x, y))
        self.download_queue = self.download_queue.union(download_list)
    
    def main(self):
        while True:
            for indices in self.download_queue:
                self.threads_process.append(self.request_map(indices[0], indices[1]))
            for th in self.threads_process:
                th.join()
            self.threads_process.clear()
            time.sleep(self.sleep_time)
