
import numpy as np
import cv2
import math
import requests, shutil

# config:
earth_radius = 6378137.0
earth_circumference = 2 * earth_radius * np.pi
api_key = 'pk.eyJ1IjoieGJ3MTI2NiIsImEiOiJja3BqOHI1cTAxajNoMnZwM2s3ejJ6NjY1In0.teLIW7gG945RkmgJDWWJWQ'

# (lon, lat) is the center coordinate of the location
def gen_map(lon, lat, zoom_level, img_resolution, img_name):
    '''# assume 1:1 aspect ratio
    meter_per_lat = earth_circumference / 360
    meter_per_lon = earth_circumference * np.cos(lat * np.pi / 180) / 360
    # resolution: meters/pixel
    resolution = (earth_radius * 2 * math.pi / img_resolution) * math.cos(lat * math.pi / 180) / (2 ** zoom_level)'''
    lon_lat_command = "{},{},{},0,0".format(lon, lat, zoom_level)
    command = "https://api.mapbox.com/styles/v1/mapbox/satellite-streets-v11/static/{}/{}x{}?access_token={}".format(lon_lat_command, img_resolution, img_resolution, api_key)
    response = requests.get(command, stream=True)
    with open(img_name, 'wb') as out_file:
        shutil.copyfileobj(response.raw, out_file)
    #print(response.headers)

    '''# Now calculating the corner coordinates (lon, lat):
    diff_meters = img_resolution / 2 * resolution
    diff_in_lon = diff_meters / meter_per_lon
    diff_in_lat = diff_meters / meter_per_lat
    
    # Now assemble the cooridnates: (lon, lat)
    upper_left = (lon - diff_in_lon, lat + diff_in_lat)
    upper_right = (lon + diff_in_lon, lat + diff_in_lat)
    bottom_left = (lon - diff_in_lon, lat - diff_in_lat)
    bottom_right = (lon + diff_in_lon, lat - diff_in_lat)
    
    corner_coords = [upper_left, upper_right, bottom_left, bottom_right]
    
    return ( corner_coords , [ diff_in_lon, diff_in_lat, diff_meters ] )'''
    
    
    
def compute_lonlat_info(lon, lat, zoom_level, img_resolution):
    # assume 1:1 aspect ratio
    meter_per_lat = earth_circumference / 360
    meter_per_lon = earth_circumference * np.cos(lat * np.pi / 180) / 360
    # resolution: meters/pixel
    resolution = (earth_radius * 2 * math.pi / img_resolution) * math.cos(lat * math.pi / 180) / (2 ** zoom_level)
    
    # Now calculating the corner coordinates (lon, lat):
    diff_meters = img_resolution / 2 * resolution
    diff_in_lon = diff_meters / meter_per_lon
    diff_in_lat = diff_meters / meter_per_lat
    
    # Now assemble the cooridnates: (lon, lat)
    upper_left = (lon - diff_in_lon, lat + diff_in_lat)
    upper_right = (lon + diff_in_lon, lat + diff_in_lat)
    bottom_left = (lon - diff_in_lon, lat - diff_in_lat)
    bottom_right = (lon + diff_in_lon, lat - diff_in_lat)
    
    corner_coords = [upper_left, upper_right, bottom_left, bottom_right]

    ranges_convered_in_this_image = [ diff_in_lon, diff_in_lat, diff_meters ]
    
    return ( corner_coords ,  ranges_convered_in_this_image)
    
    
