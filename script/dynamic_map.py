import json, cv2, time, os, math

from numpy.core.fromnumeric import resize

from os import listdir
from os.path import isfile, join

import numpy as np
from numpy.lib import load

from bresenham import bresenham

import rospy
from rospy.names import remap_name
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String, Int32MultiArray

from tools.draw_icon import add_icon


class gps2grid():

    def __init__(self):

        # load config 
        general_config_file = open('lat_lon_global_config.json', 'r')
        general_config = json.load(general_config_file)
        general_config_file.close()

        self.image_resolution = general_config['image_resolution']

        # load global tile info
        all_tile_info_file = open('global_tile_coord.json','r')
        all_tile_info_file_content = json.load(all_tile_info_file)
        all_tile_info_file.close()

        self.all_tile_center_lats = all_tile_info_file_content["lat_centers_list"]
        self.all_tile_center_lons = all_tile_info_file_content["lon_centers_list"]

        self.all_tile_corner_lats = all_tile_info_file_content["lat_tile_range"]
        self.all_tile_corner_lons = all_tile_info_file_content["lon_tile_range"]

        # load project config
        proj_config_file = open('project_config.json')
        proj_config = json.load(proj_config_file)
        proj_config_file.close()

        self.number_of_devices = proj_config['number_of_devices']
        device_id_list = proj_config['dev_id_list']
        self.my_id = proj_config['self_id']

        self.other_dev_latlon = {}
        # self.other_pass_progress = []
        self.other_pass_progress = {}
        self.other_dev_uv = []
        for i in range(self.number_of_devices ):
            self.other_dev_latlon[ device_id_list[i] ] = [ ]
            self.other_pass_progress[ device_id_list[i] ] = 0
            # self.other_pass_progress.append(0)
            self.other_dev_uv.append( [80,80] )

        print('device_id_list')
        print(device_id_list)

        print('my_id')
        print(self.my_id)
        
        print('other_dev_latlon')
        print(self.other_dev_latlon)

        print('other_pass_progress')
        print(self.other_pass_progress)

        self.my_uv = [0,0]

        
        

        t1 = time.time()
        
        self.sub  = rospy.Subscriber("/ublox_gps_raw", NavSatFix, self.callback )
        self.sub2 = rospy.Subscriber("/gps_locations", String,    self.callback2)
        print('\nGPS subscriber started')
        
        # wait until the gps module is ready
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
        # self.initial_lat = msg.latitude
        # self.initial_lon = msg.longitude

        # print( self.LatLonToTileIndex(self.initial_lat, self.initial_lon) )

        self.current_center_tile_index = [0,0]
        self.top_left_latlon = [0.0, 0.0]

        self.current_lat_range = None
        self.current_lon_range = None

        self.lat_pix_ratio = 0.0
        self.lon_pix_ratio = 0.0

        self.amount_of_tile_for_map = 3

        self.map  = None
        self.grid = None
        self.grid_other = None
        self.other_grid_ready = True

        self.last_latlon = [0.0, 0.0 ]
        self.last_last_latlon = [0.0, 0.0 ]
        self.latlon_record = []

        # self.

        self.num_passes = 5
        # self.num_passes_gropus = np.zeros((2, self.num_passes)).astype(np.int8)
        # self.num_passes_gropus[0] = [1,2,3,4,5,6,7,8,9,10]
        # self.num_passes_gropus[1] = [1,2,2,3,3,4,4,5,5,5]
        
        self.color_list = [ (0,0,255), (3,233,252), (0,255,0), (255,0,0), (255,100,100) ]

        self.pass_width = 4

        self.roller_orientation = 0.0
        self.roller_orientation_last = 0.0

        self.chech_small_movement = False 
        self.small_move_critiria = 3

        self.cb1_updating_grid = False
        self.cb2_updating_grid = False
        self.cb1_need_grid = True
        
        # main loop conuter 
        self.cb_counter = 0
        
        
        self.window_detection_name = "Pass Count"
        self.satelite_size_bar_name = 'view range'
        self.satelite_size_low = 100
        self.satelite_size_lowest = 100
        self.satelite_size_high = 1500


        self.actual_visible_size_v = 360
        self.actual_visible_size_u = 600

        self.crop_ratio = float(self.actual_visible_size_v) / self.actual_visible_size_u

        self.view_cropping_size_u = 350
        self.view_cropping_size_v = int(self.view_cropping_size_u * self.crop_ratio)
        
        # load icon images for  
        self.icon = cv2.resize(cv2.imread("ui_materials/myself_roller.png", -1), (40,35) )
        self.icon_other = cv2.resize(cv2.imread("ui_materials/other_roller.png", -1), (40,35) )

        # legend base image
        self.legend_base = cv2.resize( cv2.imread( 'ui_materials/passcount_lenged.png' ), ( 200, self.actual_visible_size_v ) )
        self.complete_legend()

        cv2.namedWindow(self.window_detection_name)
        cv2.createTrackbar(self.satelite_size_bar_name , self.window_detection_name , self.satelite_size_low, self.satelite_size_high, self.on_low_H_thresh_trackbar)

    # ==========================================================
    # cv ui slide bar
    # ==========================================================
    def on_low_H_thresh_trackbar(self, val):
        if val < self.satelite_size_lowest:
            val = self.satelite_size_lowest
        self.view_cropping_size_u = int(val)
        self.view_cropping_size_v = int(self.view_cropping_size_u * self.crop_ratio)
        low_H = val
        low_H = min(self.satelite_size_high-1, low_H)
        cv2.setTrackbarPos(self.satelite_size_bar_name, self.window_detection_name, low_H)
        

    # ==========================================================
    # put the icons and colors code onto the legend base image 
    # ==========================================================
    def complete_legend(self):
        v_spacing = 42
        bar_height = 30
        bar_width = 95
        bar_u_start = 30
        bar_v_start = 15
        for i in range( len(self.color_list) ):
            cv2.rectangle(self.legend_base, ( bar_u_start  , bar_v_start + v_spacing*i ) , ( bar_u_start+bar_width , bar_v_start + v_spacing*i + bar_height ) , self.color_list[i], -1)
        
        roller_size = ( 100, 50 )
        u1 = 30
        v1 = 230
        roller_spacing = 62

        self.legend_base[ v1:v1+roller_size[1], u1:u1+roller_size[0] ] = cv2.resize( cv2.imread('ui_materials/me_solid_bg.png'), roller_size )
        self.legend_base[ v1+roller_spacing:v1+roller_size[1]+roller_spacing, u1:u1+roller_size[0] ] = cv2.resize( cv2.imread('ui_materials/other_solid_bg.png'), roller_size )




    # ==========================================================
    # rostopic callback 2
    # ==========================================================
    def callback2(self, data):
        string_raw = data.data
        list_raw = json.loads(string_raw)
        # print(list_raw)

        # update the dict/list storing the latlon from other devices
        for i in range(len(list_raw)): 
            device_id = list_raw[i]['gpsData']['gpsDeviceId'] 
            # if device_id != self.my_id: 
            lat = list_raw[i]['gpsData']['gpsLat'] 
            lon = list_raw[i]['gpsData']['gpsLng'] 

            # device_id += 1

            self.other_dev_latlon[device_id].append([lat,lon])
            
            # print('')
        print("\nthis is callback  2")
        # print("\nthis is callback  2  \n  other device gps lists length ")
        # print( len(self.other_dev_latlon[1]) , len(self.other_dev_latlon[2]) )

        # print('other_dev_latlon')
        # print(self.other_dev_latlon)


        # update the grid
        if self.other_grid_ready:
            # update pass count on grid 
            # print('\n ****************\n  callback 2 , now updating grid2 \n **************** \n')
            for i in range(self.number_of_devices):
                # #if len( self.other_dev_latlon[i+1] ) > 2:
                # print('\n ****************\n  callback 2 , updating grid2  device {} \n **************** \n'.format(i+1))
                # print( 'pass count progress {}'.format( self.other_pass_progress ) )
                # print( 'len( self.other_dev_latlon[i+1] )   {}'.format( len( self.other_dev_latlon[i+1] ) ) )
                if i != 0 and i != self.my_id:
                    # while self.other_pass_progress[i] < len( self.other_dev_latlon[i+1] )-1:
                    while self.other_pass_progress[i] <= len( self.other_dev_latlon[i] )-1:    
                        # get the lat lon , lat lon last 
                        # print( '\n ****************\n  device {} \n{} \n{}  \n**************** \n'.format(i+1, self.other_pass_progress[i], len(self.other_dev_latlon[i+1]) ) )
                        # print( self.other_dev_latlon[i+1][self.other_pass_progress[i] +1 ] )

                        # lat           = self.other_dev_latlon[i+1][ self.other_pass_progress[i] +1 ][0]
                        # lon           = self.other_dev_latlon[i+1][ self.other_pass_progress[i] +1 ][1]
                        # last_lat      = self.other_dev_latlon[i+1][ self.other_pass_progress[i] +0 ][0]
                        # last_lon      = self.other_dev_latlon[i+1][ self.other_pass_progress[i] +0 ][1]
                        # last_last_lat = self.other_dev_latlon[i+1][ self.other_pass_progress[i] -1 ][0]
                        # last_last_lon = self.other_dev_latlon[i+1][ self.other_pass_progress[i] -1 ][1]

                        lat           = self.other_dev_latlon[i][ self.other_pass_progress[i]  ][0]
                        lon           = self.other_dev_latlon[i][ self.other_pass_progress[i]  ][1]
                        last_lat      = self.other_dev_latlon[i][ self.other_pass_progress[i] -1 ][0]
                        last_lon      = self.other_dev_latlon[i][ self.other_pass_progress[i] -1 ][1]
                        last_last_lat = self.other_dev_latlon[i][ self.other_pass_progress[i] -2 ][0]
                        last_last_lon = self.other_dev_latlon[i][ self.other_pass_progress[i] -2 ][1]

                        # get the uv 
                        machine_u, machine_v                     = self.latlonToImgpixel( lat= lat,            lon= lon )
                        last_machine_u, last_machine_v           = self.latlonToImgpixel( lat= last_lat,       lon= last_lon )
                        last_last_machine_u, last_last_machine_v = self.latlonToImgpixel( lat= last_last_lat , lon= last_last_lon )

                        self.other_dev_uv[i+1] = [ machine_u, machine_v ]

                        # update grid
                        self.update_grid( machine_u, machine_v, last_machine_u, last_machine_v, last_last_machine_u,  last_last_machine_v, 'other' )

                        self.other_pass_progress[i] += 1
            


    # ==========================================================
    # rostopic callback 1
    # ==========================================================
    def callback(self, data):
        lat = data.latitude 
        lon = data.longitude 
        print('\ncallback {}'.format(self.cb_counter))

        t1 = time.time()

        new_ll_tile_index = self.LatLonToTileIndex(lat, lon)
        # print( (lat,lon),new_ll_tile_index ,self.current_center_tile_index)
        
        if new_ll_tile_index == self.current_center_tile_index:
            print("use same tile")
        else:
            print("need to update map image")
            index_x = new_ll_tile_index[0]
            index_y = new_ll_tile_index[1]

            # check the 9 tiles around this center tile 
            file_names_list = [f for f in listdir('../data/maps/') if isfile(join('../data/maps/', f))] 

            tile_missing = []

            x_start = index_x - 1
            y_start = index_y - 1 

            for i in range(3):
                for j in range(3):
                    x = x_start+i 
                    y = y_start+j 
                    img_name = '{}_{}.jpg'.format( x , y )
                    if img_name in file_names_list:
                        print( img_name + ' exist ')
                    else: 
                        tile_missing.append(img_name)

            if len(tile_missing) != 0:
                print('\n\n**==**==**==**==**==**==**==**==**')
                print('Not enough map tiles saved; these tiles are missing: ', tile_missing )
            elif len(tile_missing) == 0:
                print('all tiles ready')
                if self.cb_counter != 0:
                    self.DisassembleAndSave()
                self.map, self.grid = self.AssembleTiles( index_x , index_y , self.amount_of_tile_for_map, self.amount_of_tile_for_map)
                self.grid_other = np.zeros_like( self.grid ,dtype=np.uint8)
                self.other_grid_ready = True

        # the pixel location of machine itself, in the current map image 
        machine_u, machine_v = self.latlonToImgpixel(lat= lat, lon=lon)

        self.my_uv = [ machine_u, machine_v ]

        if self.cb_counter == 0:
            self.latlon_record.append([lat,lon])
            self.latlon_record.append([lat,lon])
            self.latlon_record.append([lat,lon])
            # self.last_latlon = [ lat, lon ]
            # self.last_last_latlon = [ lat, lon ]
        else:
            self.latlon_record.append([lat,lon])

        # if self.cb_counter == 1: 
        #     self.last_last_latlon = self.last_latlon

        self.last_last_latlon = self.latlon_record[ self.cb_counter ]
        self.last_latlon      = self.latlon_record[ self.cb_counter + 1 ]

        
        # crop the ROI
        hf_crop_u = int( self.view_cropping_size_u / 2 )
        hf_crop_v = int( self.view_cropping_size_v / 2 )
        roi_range = ( machine_v-hf_crop_v, machine_v+hf_crop_v, machine_u- hf_crop_u, machine_u+hf_crop_u ) # top, bottom, left, right 
        # print('roi_range')    
        # print(roi_range)

        last_machine_u, last_machine_v = self.latlonToImgpixel(lat= self.last_latlon[0], lon= self.last_latlon[1])
        last_last_machine_u, last_last_machine_v = self.latlonToImgpixel(lat= self.last_last_latlon[0], lon= self.last_last_latlon[1])

        # chech if the location change is significant enough 
        # if it is too small, then it is considered as noise, and this movment is not draw into grid, then exit this callback function 
        if self.chech_small_movement:
            if ( abs( machine_u - last_machine_u) + abs( machine_v - last_machine_v) ) < self.small_move_critiria:
                print('move is too small')
                self.cb_counter += 1
                return 

        while self.cb2_updating_grid == True:
            self.cb1_need_grid = True
            
        self.cb1_updating_grid = True
        self.update_grid( machine_u, machine_v, last_machine_u, last_machine_v, last_last_machine_u,  last_last_machine_v , 'my')
        self.cb1_updating_grid = False

        self.last_latlon = [lat, lon] 

        self.update_img_view(roi_range, new_ll_tile_index)

        self.cb_counter += 1


    # ==========================================================
    # update self.grid 
    # ==========================================================            
    def update_grid(self, machine_u,machine_v, last_machine_u, last_machine_v, last_last_machine_u,  last_last_machine_v , which_grid ):

        t1 = time.time()

        this_pass_grid, this_small_array_top, this_small_array_left, this_sa_size_v, this_sa_size_u, this_corners = self.UVToRecCorners(new_u= machine_u, new_v= machine_v, last_u= last_machine_u, last_v= last_machine_v)
        
        # compute the heading angle difference of vehicle, from the previous second to now  
        # 'heading' and 'direction' mean the same thing in this code & comment 
        vector_1 =  [ machine_u-last_machine_u, machine_v - last_machine_v ]
        vector_2 = [ last_machine_u - last_last_machine_u, last_machine_v - last_last_machine_v ]

        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        direction_change = np.arccos(dot_product) * 180.0 / np.pi

        #print('direction_change : ', direction_change )
        
        
        # if the roller have the same movement direction, from the previous second to now 
        # the the overlap between the 2 area, the previous one and the new one, needs to be substracted 
        if abs(direction_change) < 90.0:
            last_pass_grid, last_small_array_top, last_small_array_left, last_sa_size_v, last_sa_size_u, last_corners = self.UVToRecCorners(new_u=last_machine_u, new_v=last_machine_v,last_u=last_last_machine_u,last_v=last_last_machine_v)
            all_corners = np.zeros((8,2))
            all_corners[0:4, 0] = this_corners[:,0]
            all_corners[0:4, 1] = this_corners[:,1]
            all_corners[4: , 0] = last_corners[:,0]
            all_corners[4: , 1] = last_corners[:,1]
            combd_array_top  = min(all_corners[:,1])
            combd_array_left = min(all_corners[:,0])
            combd_array_size_u = int( max(all_corners[:,0]) - combd_array_left ) +1
            combd_array_size_v = int( max(all_corners[:,1]) - combd_array_top  ) +1

            # print('all corners ')
            # print(all_corners)
            # print( min(all_corners[:,0]), max(all_corners[:,0]) )
            # print( min(all_corners[:,1]), max(all_corners[:,1]) )
            
            # print('combd_array_size   ')
            # print( combd_array_size_v, combd_array_size_u )

            combd_array = np.zeros((combd_array_size_v, combd_array_size_u)).astype(np.uint8)
            
            this_pass_grid_top_in_combd  = int( this_small_array_top  - combd_array_top   )
            this_pass_grid_left_in_combd = int( this_small_array_left - combd_array_left  )

            last_pass_grid_top_in_combd  = int( last_small_array_top  - combd_array_top )
            last_pass_grid_left_in_combd = int( last_small_array_left - combd_array_left  )

            combd_array[ this_pass_grid_top_in_combd : this_pass_grid_top_in_combd + this_sa_size_v, this_pass_grid_left_in_combd : this_pass_grid_left_in_combd + this_sa_size_u ] +=  this_pass_grid 

            combd_array[ last_pass_grid_top_in_combd : last_pass_grid_top_in_combd + last_sa_size_v, last_pass_grid_left_in_combd : last_pass_grid_left_in_combd + last_sa_size_u ] -= last_pass_grid

            combd_array[ combd_array[:,:] == 255 ] = 0

            this_pass_grid = combd_array
            this_small_array_left = int( combd_array_left )
            this_small_array_top  = int( combd_array_top  )
            this_sa_size_v = combd_array_size_v
            this_sa_size_u = combd_array_size_u

            # print(combd_array)

        # update the entire 3x3 grid 
        if which_grid == 'my':
            self.grid[ this_small_array_top:this_small_array_top+this_sa_size_v, this_small_array_left:this_small_array_left+this_sa_size_u ] += this_pass_grid
            # print(' updated my grid  ')
        elif which_grid == 'other':
            self.grid_other[ this_small_array_top:this_small_array_top+this_sa_size_v, this_small_array_left:this_small_array_left+this_sa_size_u ] += this_pass_grid
            # print(' updated other grid  ')
        else: 
            print('check grid argument input ')

        t2  = time.time()
        # print( 'time in update grid {} : {} ms'.format( which_grid , int((t2-t1)*1000 ) ))



    # ==========================================================
    # update the image to be viewed 
    # ==========================================================
    def update_img_view(self, roi_range, new_ll_tile_index):
        t1  = time.time()

        composed_grid = self.grid_other  + self.grid
        
        # crop the grid and map for visulization 
        grid_roi = composed_grid[ roi_range[0]:roi_range[1], roi_range[2]:roi_range[3]]
        # print(grid_roi.shape)

        map_roi = self.map[ roi_range[0]:roi_range[1], roi_range[2]:roi_range[3]] 
        # print(map_roi.shape)

        # assign colors based on the pass count value 
        for i in range(1, self.num_passes):
            ci = i -1
            map_roi[ grid_roi[:,:] == i ] = self.color_list[ci]

        # for the grid with pass count larger than max value, assign the highest color to it
        map_roi[ grid_roi[:,:] >= self.num_passes ] = self.color_list[self.num_passes-1]

        # add other roller icon at location of gps 
        other_uv = []
        for i in range(self.number_of_devices):
            #if len(self.other_dev_latlon[i]) != 0:
            u_from_center = self.other_dev_uv[i][0] - self.my_uv[0]
            v_from_center = self.other_dev_uv[i][1] - self.my_uv[1]
            
            the_u = int(map_roi.shape[1]/2) + u_from_center
            the_v = int(map_roi.shape[0]/2) + v_from_center
            print('id: {}  u:{}   v:{}'.format( i, the_u , the_v  ))
            # if abs(the_u) < map_roi.shape[1]/2 or abs(the_v) < map_roi.shape[0]/2:
                # add_icon(self.icon_other, self.roller_orientation , ( the_u, the_v), roi_resized)
                # cv2.circle( map_roi, ( the_u, the_v), 5, (255,0,0), -1 )
            other_uv.append( [the_u, the_v] )
        



        # resize the cropped image
        roi_resized = cv2.resize( map_roi, ( self.actual_visible_size_u, self.actual_visible_size_v ) )
        # print( 'map_roi shape' )
        # print(map_roi.shape)
        # print( 'roi_resized shape' )
        # print(roi_resized.shape)
        # print( 'other_uv' )
        # print(other_uv)
        
        
        # get ratio from map_roi to resized 
        resized_other_uv = []
        for i in range(len(other_uv)):
            resized_other_uv.append([ int((float(roi_resized.shape[1]) / map_roi.shape[1] ) * other_uv[i][0]) , int( (float(roi_resized.shape[0]) / map_roi.shape[0] ) * other_uv[i][1] ) ] )
        # print( 'resized_other_uv' )
        # print(resized_other_uv)

        # add my roller icon at location of gps 
        add_icon(self.icon, self.roller_orientation , (int(self.actual_visible_size_u/2), int(self.actual_visible_size_v/2) ), roi_resized)
        print('add my icon')
        for i in range(len(other_uv)):
            if i != self.my_id:
                add_icon(self.icon_other, self.roller_orientation , ( resized_other_uv[i][0], resized_other_uv[i][1] ), roi_resized)
                print('add device icon {}'.format(i))
            # cv2.circle( roi_resized, ( resized_other_uv[i][0], resized_other_uv[i][1] ), 5, (255,0,0), -1 )

        # add lengend 
        showimg = np.hstack((roi_resized, self.legend_base))

        # update the tile index being used in the current view 
        self.current_center_tile_index = new_ll_tile_index

        cv2.imwrite( '../data/save/{}.jpg'.format(self.cb_counter) , showimg )

        t2  = time.time()
        print( 'time in update_img_view: {} ms'.format( int((t2-t1)*1000 ) ))

        cv2.imshow( self.window_detection_name, showimg )
        # cv2.imshow( 'grid', cv2.resize( self.grid * 30, (800,800) ) )
        cv2.waitKey(30)

        


    # ==========================================================
    # a tool for resizing img arrays
    # ==========================================================
    def resize_img(self, the_array, ratio):
        """resize an img array

        Args:
            the_array (np.array): [description]
            ratio (float): [description]
        """

        img_w_old = the_array.shape[1]
        img_h_old = the_array.shape[0]
        img_w_new = int( img_w_old * ratio )
        img_h_new = int( img_h_old * ratio ) 
        newarray = cv2.resize( the_array, ( img_w_new, img_h_new) )

        return newarray
    
    # ==========================================================
    # a tool, input pixel uv, return the grid cells covered in this step 
    # ==========================================================
    def UVToRecCorners(self, new_u, new_v, last_u, last_v ):
        hfw = self.pass_width / 2
        du = new_u - last_u
        dv = new_v - last_v
        orientation = math.atan2(dv,du) * -1
        self.roller_orientation = orientation * 180.0 / np.pi

        print( ' computed angle: {} ; last angle: {} '.format( self.roller_orientation , self.roller_orientation_last ) )

        # if abs(self.roller_orientation - self.roller_orientation_last ) > 80:
        # if abs(du) + abs(dv) < 4:
        #     self.roller_orientation = self.roller_orientation_last

        # self.roller_orientation_last = self.roller_orientation

        corners = np.zeros((4,2)).astype(np.int)

        def mc(d):
            return math.cos(d)
        def ms(d):
            return math.sin(d)

        corners[0,0] = new_u + hfw * ms(orientation) 
        corners[0,1] = new_v + hfw * mc(orientation) 
        corners[1,0] = new_u - hfw * ms(orientation) 
        corners[1,1] = new_v - hfw * mc(orientation) 

        corners[3,0] = last_u + hfw * ms(orientation) 
        corners[3,1] = last_v + hfw * mc(orientation) 
        corners[2,0] = last_u - hfw * ms(orientation) 
        corners[2,1] = last_v - hfw * mc(orientation)  

        '''print('compute rec corners')
        print(new_u, new_v, last_u, last_v)
        print(orientation * 180.0 / np.pi)
        print(corners)'''

        local_boundary_top = min(corners[:,1]) 
        local_boundary_left = min(corners[:,0]) 

        localized_corners = np.zeros_like(corners) 
        localized_corners[:,0] = corners[:,0] - local_boundary_left 
        localized_corners[:,1] = corners[:,1] - local_boundary_top 

        size_v = max(localized_corners[:,1]) +1 
        size_u = max(localized_corners[:,0]) +1 

        local_array = np.zeros((size_v,size_u)).astype(np.uint8) 

        local_array_cp = local_array.copy()
        for i in range(len(localized_corners)):
            local_array_cp[ localized_corners[i,1], localized_corners[i,0] ] = 1

        cv2.imwrite('../data/tmpgrids/{}_v0.png'.format(self.cb_counter), self.resize_img( local_array_cp*180, 5) )

        # print( '\nlocal_array size: \n{}, \ncorners: \n{}\n'.format( local_array.shape, localized_corners  ) )

        ordered_corners = np.zeros((4,2))
        ordered_corners[0] = localized_corners[np.argmin(corners[:,1])]
        ordered_corners[1] = localized_corners[np.argmax(corners[:,0])]
        ordered_corners[2] = localized_corners[np.argmax(corners[:,1])]
        ordered_corners[3] = localized_corners[np.argmin(corners[:,0])]
        # print(ordered_corners)

        # print(corners)
        # print(localized_corners)
        lines = []
        lines.append(self.BresenLine(ordered_corners[3][1],ordered_corners[3][0], ordered_corners[0][1], ordered_corners[0][0]))
        lines.append(self.BresenLine(ordered_corners[0][1],ordered_corners[0][0], ordered_corners[1][1], ordered_corners[1][0]))
        lines.append(self.BresenLine(ordered_corners[1][1],ordered_corners[1][0], ordered_corners[2][1], ordered_corners[2][0]))
        lines.append(self.BresenLine(ordered_corners[2][1],ordered_corners[2][0], ordered_corners[3][1], ordered_corners[3][0]))

        clean_lines = []

        for i in range(4):
            line = lines[i]
            for j in range(len(line)):
                v = line[j][0]
                u = line[j][1]

                local_array[v,u] = 1

        cv2.imwrite('../data/tmpgrids/{}_v1.png'.format(self.cb_counter), self.resize_img( local_array*180, 5) )


        for j in range(size_v):
            saw_01 = 0
            saw_10 = size_u-1
            left_count = False
            for i in range(0, size_u):
                if local_array[j,i] == 1 and left_count == False:
                    saw_01 = i
                    left_count = True
                
            for k in range( 0 , size_u ):
                i = size_u - k -1 
                if local_array[j,i] == 1:
                    saw_10 = i
                    break

            local_array[j, saw_01:saw_10] = [1]*(saw_10 - saw_01)


        array_corner_list = [ [size_v-1,size_u-1], [0,0], [0,size_u-1], [size_v-1, 0] ]
        if [ localized_corners[0,1],localized_corners[0,0] ] in array_corner_list:
            if [ localized_corners[1,1],localized_corners[1,0] ] in array_corner_list:
                if [ localized_corners[2,1],localized_corners[2,0] ] in array_corner_list:
                    if [ localized_corners[3,1],localized_corners[3,0] ] in array_corner_list:
                        local_array[:,:] = 1

        cv2.imwrite('../data/tmpgrids/{}_v2.png'.format(self.cb_counter), self.resize_img( local_array*180, 5) )

        return (local_array , local_boundary_top, local_boundary_left, size_v, size_u , corners)


    # ==========================================================
    # a tool for get the grid cells connecting 2 pixel locations 
    # ==========================================================
    def BresenLine(self, v1, u1, v2, u2):
        return list(bresenham(int(v1), int(u1), int(v2), int(u2) ) )
                


    # =============================================================
    # a tool for computing which tile the input lat-lon locates at 
    # =============================================================
    def LatLonToTileIndex(self, lat, lon):
        lat_diffs_from_all_tiles = np.abs( np.array( self.all_tile_center_lats) - lat )
        lon_diffs_from_all_tiles = np.abs( np.array( self.all_tile_center_lons) - lon ) 
        lat_index = np.argmin(lat_diffs_from_all_tiles)
        lon_index = np.argmin(lon_diffs_from_all_tiles)
        return [lon_index, lat_index]



    # ==========================================================
    # grab the map tiles and grid tiles following the inputs
    # ==========================================================
    def AssembleTiles(self, center_tile_x, center_tile_y, x_amount, y_amount ):
        if x_amount % 2 != 1 or y_amount % 2 != 1:
            print('tile assembly input size wrong ')

        print('assembling the tiles into map')

        check_each_image = False

        x_start = int( center_tile_x - (x_amount-1)/2 )
        y_start = int( center_tile_y + (y_amount-1)/2 )

        complete_image = np.zeros( ( x_amount * self.image_resolution , y_amount * self.image_resolution, 3) ).astype(np.uint8)
        sz = self.image_resolution  # this is only to use a shorter name for the image resolution in the below lines 

        complete_grid  = np.zeros_like(complete_image)[:,:,0]

        for i in range(x_amount):
            x = x_start+i
            for j in range(y_amount):
                y = y_start-j 
                img_name = '../data/maps/' + '{}_{}.jpg'.format( x , y )
                one_tile = cv2.imread( img_name )
                npy_name = '../data/maps/' + '{}_{}.npy'.format( x , y )
                one_grid = np.load(npy_name)
                if check_each_image :
                    cv2.imshow( "map", cv2.resize( one_tile , (640,640) ) )
                    cv2.waitKey(30)
                    print( j*sz, (j+1)*sz, i*sz, (i+1)*sz )

                complete_image[ j*sz:(j+1)*sz, i*sz:(i+1)*sz ] = ( one_tile * 0.6 ).astype(np.uint8)
                complete_grid[j*sz:(j+1)*sz, i*sz:(i+1)*sz] = one_grid

        lats = self.all_tile_corner_lats[y_start]
        lons = self.all_tile_corner_lons[x_start]

        print(lats, lons)

        self.top_left_latlon = [ max(lats) , min(lons) ]

        xhf , yhf = (x_amount-1)/2, (y_amount-1)/2

        top_left_tile_x_index = center_tile_x - xhf
        top_left_tile_y_index = center_tile_y + yhf

        bottom_right_tile_x_index = center_tile_x + xhf
        bottom_right_tile_y_index = center_tile_y - yhf

        self.current_lat_range = [ min( self.all_tile_corner_lats[bottom_right_tile_y_index] )  , max( self.all_tile_corner_lats[top_left_tile_y_index] )  ]
        self.current_lon_range = [ min( self.all_tile_corner_lons[top_left_tile_x_index] )  , max( self.all_tile_corner_lons[bottom_right_tile_x_index] )  ]

        self.lat_pix_ratio = ( self.current_lat_range[1] - self.current_lat_range[0] ) / (self.image_resolution * y_amount )
        self.lon_pix_ratio = ( self.current_lon_range[1] - self.current_lon_range[0] ) / (self.image_resolution * x_amount )

        return complete_image , complete_grid

    # ==========================================================
    # save the current grids into npy files
    # ==========================================================
    def DisassembleAndSave(self):
        center_tile_x = self.current_center_tile_index[0]
        center_tile_y = self.current_center_tile_index[1]
        print('dis-assembling the current map and grid')
        print(self.current_center_tile_index)

        hf = (self.amount_of_tile_for_map-1)/2

        left_top_tile_center_x = int(center_tile_x - hf )
        left_top_tile_center_y = int(center_tile_y + hf )  

        composed_grid = self.grid + self.grid_other

        for i in range( self.amount_of_tile_for_map ):        #left_top_tile_center_x, left_top_tile_center_x + self.amount_of_tile_for_map):
            for j in range( self.amount_of_tile_for_map ):       #left_bottom_tile_center_y, left_bottom_tile_center_y + self.amount_of_tile_for_map):
                one_grid = composed_grid[ j*self.image_resolution : (j+1)*self.image_resolution, i*self.image_resolution: (i+1)*self.image_resolution ]
                this_tile_x = left_top_tile_center_x + i
                this_tile_y = left_top_tile_center_y - j
                npy_name =  '../data/maps/{}_{}.npy'.format( this_tile_x, this_tile_y ) 
                np.save( npy_name  , one_grid )


    # ============================================================================
    # a tool for computing the pixel index of a lat-lon, in the current grid/map
    # ============================================================================
    def latlonToImgpixel(self, lat, lon ):
        if lat > self.current_lat_range[1] or lat < self.current_lat_range[0]:
            print(' this latitude is not in current map image ')
        if lon > self.current_lon_range[1] or lon < self.current_lon_range[0]:
            print(' this longitude is not in current map image ')

        v = int( (lat - self.current_lat_range[1])*-1 / self.lat_pix_ratio )
        u = int( (lon - self.current_lon_range[0]) / self.lon_pix_ratio )

        return (u,v)

        




def main():
    
    rospy.init_node('passcountnode', anonymous=True)
    
    obc = gps2grid()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()








