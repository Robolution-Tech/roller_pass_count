import numpy as np
import cv2
import math
import requests, shutil
import configparser
import os

class PassCounter:
    def __init__(self, config_file) -> None:
        self.parsingVariables(config_file)        
        self.earth_circumference = 2 * self.earth_radius * np.pi
        self.meter_per_lat = self.earth_circumference / 360

    def parsingVariables(self, config_file):
        self.parser = configparser.ConfigParser()
        self.parser.read(config_file)
        self.earth_radius = float(self.parser['EARTH_FACTS']['earth_radius'])
        self.api = self.parser['API_SETTINGS']['api_key']
        self.img_resolution = self.parser['MAP_SETTINGS']['image_resolution']
        self.zoom_level_low = self.parser['MAP_SETTINGS']['zoom_level_low']
        self.zoom_level_high = self.parser['MAP_SETTINGS']['zoom_level_high']
        
    def getResolution(self, lat, zoom_level): 
        return (self.earth_radius * 2 * math.pi / self.img_resolution) * math.cos(lat * math.pi / 180) / (2 ** zoom_level)
    
    def requestMap(self, zoom_level, lon, lat): 
        lon_lat_command = "{},{},{},0,0".format(lon, lat, zoom_level)
        command = "https://api.mapbox.com/styles/v1/mapbox/satellite-streets-v11/static/{}/{}x{}?access_token={}".format(lon_lat_command, 
                   self.img_resolution, self.api_key)
        response = requests.get(command, stream=True)
        img_name = os.path.join("map", "{}_{}_{}.jpg".format(lon, lat, zoom_level)) 
        with open(img_name, 'wb') as out_file:
            shutil.copyfileobj(response.raw, out_file)
        # Now calculating the corner coordinates (lon, lat):
        diff_meters = self.img_resolution / 2 * self.getResolution(lat, zoom_level)
        meter_per_lon = self.earth_circumference * np.cos(lat * np.pi / 180) / 360
        diff_in_lon = diff_meters / meter_per_lon
        diff_in_lat = diff_meters / self.meter_per_lat
        # Now assemble the cooridnates: (lon, lat)
        upper_left = (lon - diff_in_lon, lat + diff_in_lat)
        upper_right = (lon + diff_in_lon, lat + diff_in_lat)
        bottom_left = (lon - diff_in_lon, lat - diff_in_lat)
        bottom_right = (lon + diff_in_lon, lat - diff_in_lat)
        corner_coords = [upper_left, upper_right, bottom_left, bottom_right]
        return cv2.imread(img_name), corner_coords

if __name__ == "__main__":
    counter = PassCounter()

