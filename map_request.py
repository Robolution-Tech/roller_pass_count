import numpy as np
import cv2
import math
import requests, shutil
import configparser
import os
from collections import defaultdict
import re


class PassCounter:
    def __init__(self, config_file) -> None:
        self.parsing_variables(config_file)        
        self.earth_circumference = 2 * self.earth_radius * np.pi
        self.meter_per_lat = self.earth_circumference / 360
        self.hashable_map = defaultdict(dict)
        

    def parsing_variables(self, config_file):
        self.parser = configparser.ConfigParser()
        self.parser.read(config_file)
        self.gps_sub_name = self.parser['ROS_SETTINGS']['GPS_subscriber']
        self.earth_radius = float(self.parser['EARTH_FACTS']['earth_radius'])
        self.api_key = self.parser['API_SETTINGS']['api_key']
        self.img_resolution = self.parser['MAP_SETTINGS']['image_resolution']
        self.zoom_level_low = self.parser['MAP_SETTINGS']['zoom_level_low']
        self.zoom_level_high = self.parser['MAP_SETTINGS']['zoom_level_high']
        self.grid_resolution = self.parser['MAP_SETTINGS']['grid_resolution']
        self.pass_color = [self.to_tuple(self.parser['MAP_SETTINGS']['pass_1_color']), 
                           self.to_tuple(self.parser['MAP_SETTINGS']['pass_2_color']),
                           self.to_tuple(self.parser['MAP_SETTINGS']['pass_3_color']),
                           self.to_tuple(self.parser['MAP_SETTINGS']['pass_4_color'])]
    
    def convert_lon_lat_to_map(self, data):
        lon = self.lon_to_dec(data.longitude * (-1))
        lat = self.lat_to_dec(data.latitude)
        query_key = (lon * self.grid_resolution, lat * self.grid_resolution)
        self.hashable_map = self.update_hashtable(self.hashable_map, query_key)
            
    def draw_on_map(self):
        for key1, key2, value in self.recursive_items(self.hashable_map):
            lon, lat = key1 / float(self.grid_resolution), key2 / float(self.grid_resolution)
            
    def get_resolution(self, lat, zoom_level): 
        return (self.earth_radius * 2 * math.pi / self.img_resolution) * math.cos(lat * math.pi / 180) / (2 ** zoom_level)
    
    def request_map(self, zoom_level, lon, lat): 
        lon_lat_command = "{},{},{},0,0".format(lon, lat, zoom_level)
        command = "https://api.mapbox.com/styles/v1/mapbox/satellite-streets-v11/static/{}/{}x{}?access_token={}".format(lon_lat_command, 
                   self.img_resolution,self.img_resolution, self.api_key)
        response = requests.get(command, stream=True)
        print(response)
        # img_name = os.path.join("map", "{}_{}_{}.jpg".format(lon, lat, zoom_level)) 
        img_name = "map0.jpg" 
        with open(img_name, 'wb') as out_file:
            shutil.copyfileobj(response.raw, out_file)
        # # Now calculating the corner coordinates (lon, lat):
        # diff_meters = self.img_resolution / 2 * self.get_resolution(lat, zoom_level)
        # meter_per_lon = self.earth_circumference * np.cos(lat * np.pi / 180) / 360
        # diff_in_lon = diff_meters / meter_per_lon
        # diff_in_lat = diff_meters / self.meter_per_lat
        # # Now assemble the cooridnates: (lon, lat)
        # upper_left = (lon - diff_in_lon, lat + diff_in_lat)
        # upper_right = (lon + diff_in_lon, lat + diff_in_lat)
        # bottom_left = (lon - diff_in_lon, lat - diff_in_lat)
        # bottom_right = (lon + diff_in_lon, lat - diff_in_lat)
        # corner_coords = [upper_left, upper_right, bottom_left, bottom_right]
        # return cv2.imread(img_name), corner_coords
    
    @staticmethod
    def update_hashtable(map, key):
        map[key] = map[key] + 1 if key in map else 1 
        return map    
    
    @staticmethod
    def recursive_items(dictionary):
        for key1, dd in dictionary.items():
            for key2, value in dd.items():
                yield (key1, key2, value)    
    
    @staticmethod
    def to_tuple(parser_string):
        return tuple(int(v) for v in re.findall("[0-9]+", parser_string))

    @staticmethod
    def lat_to_dec(lat):
        raw = str(lat * 100.)
        lat_deg = int(raw[0:2])
        lat_min = float(raw[2:]) / 60.
        return lat_deg + lat_min

    @staticmethod
    def lon_to_dec(lon):
        raw  = str(lon * 100.)
        lat_deg = int(raw[1:4])
        lat_min = float(raw[4:])/60.0
        lat_dec = lat_deg + lat_min
        lat_dec *= -1.0 
        return lat_dec

if __name__ == "__main__":
    counter = PassCounter('config.ini')
    lat = PassCounter.lat_to_dec(51.48053)
    lon = PassCounter.lon_to_dec(-113.25482)
    counter.request_map(17,-113.25482,51.48053)
