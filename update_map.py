
import numpy as np
import cv2
import math
import requests, shutil


EARTHRADIUS = 6378137.0
MINLATITUDE = -85.05112878
MAXLATITUDE = 85.05112878
MINLONGITUDE = -180
MAXLONGITUDE = 180
EARTH_CIRCUMFERENCE = 2*math.pi*EARTHRADIUS
MAP_SIZE = {
    1: 512,
    2: 1024,
    3: 2048,
    4: 4096,
    5: 8192,
    6: 16384,
    7: 32768,
    8: 65536,
    9: 131072,
    10: 262144,
    11: 524288,
    12: 1048576,
    13: 2097152,
    14: 4194304,
    15: 8388608,
    16: 16777216,
    17: 33554432,
    18: 67108864,
    19: 134217728,
    20: 268435456,
    21: 536870912,
    22: 1073741824,
    23: 2147483648
}
class TileSystem():

    def clip(self, n,minValue,maxValue):
        return min(max(n,minValue),maxValue)

    def map_size(self, levelofDetail):
        return 256 * 2 ** levelofDetail

    def ground_resolution(self, latitude, levelofDetail):
        latitude = self.clip(latitude,MINLATITUDE,MAXLATITUDE)
        return math.cos(latitude*math.pi/180)* EARTH_CIRCUMFERENCE/ self.map_size(levelofDetail)

    def map_scale(self,latitude,levelofDetail,screenDpi):
        return self.ground_resolution(latitude,levelofDetail)*screenDpi/0.0254

    def latlong_to_pixelxy(self,latitude,longitude,levelofDetail):
        latitude = self.clip(latitude,MINLATITUDE,MAXLATITUDE)
        longitude = self.clip(longitude,MINLONGITUDE,MAXLONGITUDE)

        x = (longitude +180)/360
        sinLatitude = math.sin(latitude*math.pi / 180)
        y = 0.5 - math.log((1+sinLatitude) / (1 - sinLatitude)) / (4*math.pi)
        mapSize = MAP_SIZE[levelofDetail]
        pixelX = int(self.clip(x*mapSize + 0.5, 0 , mapSize -1))
        pixelY = int(self.clip(y*mapSize + 0.5, 0 , mapSize -1))
        return pixelX,pixelY
    
    def pixelxy_to_latlong(self,pixelX,pixelY,levelofDetail):
        mapSize = MAP_SIZE[levelofDetail]
        x = (self.clip(pixelX, 0 ,mapSize - 1) / mapSize) - 0.5
        # y = 0.5 - (self.clip(pixelY, 0, mapSize - 1) / mapSize) 
        y = 0.5 - (self.clip(pixelY, 0, mapSize - 1) / mapSize) 
        latitude = 90 - 360 * math.atan(math.exp(-y*2*math.pi)) / math.pi
        longitude = 360 * x

        return latitude,longitude

    def pixelxy_to_tilexy(self,pixelX, pixelY):
        tileX = math.floor(pixelX /256)
        tileY = math.floor(pixelY / 256)
        # tileX = pixelX /256
        # tileY = pixelY / 256
        return tileX, tileY

    def tilexy_to_pixelxy(self,tileX,tileY):
        pixelX = tileX * 256 
        pixelY = tileY * 256
        return pixelX, pixelY

latitude = 51.48053
longitude = -113.25482
levelofDetail = 17
tiles = TileSystem()
print("map size is",MAP_SIZE[levelofDetail])
print("gound resolution is ",tiles.ground_resolution(latitude, levelofDetail), "meter/pixel")

pixelX, pixelY = tiles.latlong_to_pixelxy(latitude,longitude,levelofDetail)
print("pixelx, pixelY",pixelX, pixelY)
print(tiles.pixelxy_to_tilexy(pixelX,pixelY))

la,lon = tiles.pixelxy_to_latlong(pixelX,pixelY,levelofDetail)




# (lon, lat) is the center coordinate of the location
def gen_map(lon, lat, zoom_level, img_resolution, img_name="map0.jpg"):
    # assume 1:1 aspect ratio
    meter_per_lat = earth_circumference / 360
    meter_per_lon = earth_circumference * np.cos(lat * np.pi / 180) / 360
    # resolution: meters/pixel
    resolution = (earth_radius * 2 * math.pi / img_resolution) * math.cos(lat * math.pi / 180) / (2 ** zoom_level)
    lon_lat_command = "{},{},{},0,0".format(lon, lat, zoom_level)
    command = "https://api.mapbox.com/styles/v1/mapbox/satellite-streets-v11/static/{}/{}x{}?access_token={}".format(lon_lat_command, img_resolution, img_resolution, api_key)
    response = requests.get(command, stream=True)
    with open(img_name, 'wb') as out_file:
        shutil.copyfileobj(response.raw, out_file)

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
    
    

    return ( corner_coords , [ diff_in_lon, diff_in_lat ] )


# # config:
# earth_radius = 6378137.0
# earth_circumference = 2 * earth_radius * np.pi
# api_key = 'pk.eyJ1IjoieGJ3MTI2NiIsImEiOiJja3BqOHI1cTAxajNoMnZwM2s3ejJ6NjY1In0.teLIW7gG945RkmgJDWWJWQ'

# gen_map(-113.25482,51.48053,17,512)

    
    
    
    
    
    
    
