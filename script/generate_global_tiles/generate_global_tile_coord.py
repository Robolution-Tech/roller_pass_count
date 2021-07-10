
import json
from update_map import gen_map, compute_lonlat_info
import numpy as np 

class gen_tiles_coord():
    def __init__(self):
        # load origin latitude and lonitude values
        origin_f = open('../lat_lon_global_config.json', 'r')
        f_content = json.load(origin_f)
        self.kLat_origin_center, self.kLon_origin_center = f_content["lat_origin_tile_center"], f_content["lon_origin_tile_center"]
        self.kMax_lat, self.kMax_lon  = f_content["max_lat"], f_content["max_lon"]
        self.zoom_level = f_content["zoom_level"]
        self.image_resolution = f_content["image_resolution"]
        print('\nOrigin json info')
        print(self.kLat_origin_center, self.kLon_origin_center, self.zoom_level, self.image_resolution)

        # self.kLat_boundary_origin -= res[1][1]*2
        # self.kLon_boundary_origin -= res[1][0]*2

        self.lat_centers_list = []
        self.lon_centers_list = []

        self.lat_center = self.kLat_origin_center
        self.lon_center = self.kLon_origin_center

        self.lat_corners_list = []
        self.lon_corners_list = []

        # self.lat_centers_list.append( self.lat_center )
        # self.lon_centers_list.append( self.lon_center )


        ii = 0
        while self.lat_center < self.kMax_lat:
            res = compute_lonlat_info(self.lon_center, self.lat_center, self.zoom_level, self.image_resolution)
            lat_increment = res[1][1]*4
            self.lat_center += lat_increment
            self.lat_centers_list.append( self.lat_center )
            ii+=1
            print('{} , {:.6f}, {:.6f}'.format(ii, lat_increment , self.lat_center) )

            lat_range = [ self.lat_center - lat_increment/2 , self.lat_center + lat_increment/2 ]
            self.lat_corners_list.append(lat_range)


        ii = 0
        while self.lon_center < self.kMax_lon:
            res = compute_lonlat_info(self.lon_center, self.lat_center, self.zoom_level, self.image_resolution)
            lon_increment = res[1][0]*4
            self.lon_center += lon_increment
            self.lon_centers_list.append( self.lon_center )
            ii+=1
            print('{} , {:.6f}, {:.6f}'.format(ii, lon_increment , self.lon_center) )

            lon_range = [ self.lon_center - lon_increment/2 , self.lon_center + lon_increment/2 ]
            self.lon_corners_list.append(lon_range)

        print(len(self.lat_centers_list))
        print(len(self.lat_corners_list))
        print(len(self.lon_centers_list))
        print(len(self.lon_corners_list))

        print( self.lat_centers_list[:3] )
        print( self.lat_corners_list[:3] )

        output_content = {"lat_length":len(self.lat_centers_list), "lon_length":len(self.lon_centers_list), "lat_centers_list":self.lat_centers_list, "lon_centers_list":self.lon_centers_list}
        output_content["lat_tile_range"] = self.lat_corners_list
        output_content["lon_tile_range"] = self.lon_corners_list
        global_tile_coord_file = open('../global_tile_coord.json', 'w')
        json.dump(output_content, global_tile_coord_file, indent=2)
        global_tile_coord_file.close()

        istart = 1298
        js = 1325
        for i in range(istart, istart+3):
            for j in range( js, js+ 3):
                gen_map(self.lon_centers_list[j], self.lat_centers_list[i], self.zoom_level, self.image_resolution, '{}_{}.jpg'.format(j,i))


gen_tiles_coord()





