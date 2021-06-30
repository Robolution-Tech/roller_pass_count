import functools
import time
import numpy as np
import requests, shutil
import threading

# config:
earth_radius = 6378137.0
earth_circumference = 2 * earth_radius * np.pi
api_key = 'pk.eyJ1IjoieGJ3MTI2NiIsImEiOiJja3BqOHI1cTAxajNoMnZwM2s3ejJ6NjY1In0.teLIW7gG945RkmgJDWWJWQ'


def timer(func):
	@functools.wraps(func)
	def wrapper_timer(*args, **kwargs):
		start_time = time.time()
		value = func(*args, **kwargs)
		end_time = time.time()
		run_time = end_time - start_time
		print("Finished {} in {} secs".format(func.__name__, run_time))
		return value
	return wrapper_timer 

# (lon, lat) is the center coordinate of the location
def gen_map(lon, lat, zoom_level, img_resolution, img_name):
    lon_lat_command = "{},{},{},0,0".format(lon, lat, zoom_level)
    command = "https://api.mapbox.com/styles/v1/mapbox/satellite-streets-v11/static/{}/{}x{}?access_token={}".format(lon_lat_command, img_resolution, img_resolution, api_key)
    response = requests.get(command, stream=True)
    with open(img_name, 'wb') as out_file:
        shutil.copyfileobj(response.raw, out_file)

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = threading.Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread
    return wrapper
