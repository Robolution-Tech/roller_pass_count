#import plotly.graph_objects as go
#import pandas as pd
#import csv
#import glob
import json
import os
import numpy as np
from numpy.core.fromnumeric import partition
from numpy.core.numeric import allclose
from numpy.lib import load
from numpy.lib.shape_base import split
from bresenham import bresenham
import math
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import NavSatFix
import cv2 
from draw_icon import rotate_image, add_icon
import time
from std_msgs.msg import Bool, String, Int32MultiArray
from update_map import gen_map
from os import listdir
from os.path import isfile, join

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

        t1 = time.time()
        
        self.sub = rospy.Subscriber("/ublox_gps_raw", NavSatFix, self.callback)
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
        self.initial_lat = msg.latitude
        self.initial_lon = msg.longitude

        print( self.LatLonToTileIndex(self.initial_lat, self.initial_lon) )

        self.requested_tile_xy = []

        self.current_center_tile_index = [0,0]
        self.top_left_latlon = [0.0, 0.0]

        self.current_lat_range = None
        self.current_lon_range = None

        self.lat_pix_ratio = 0.0
        self.lon_pix_ratio = 0.0

        self.amount_of_tile_for_map = 3

        self.map  = None
        self.grid = None

        self.last_latlon = [0.0, 0.0 ]
        self.last_last_latlon = [0.0, 0.0 ]
        self.latlon_record = []

        self.num_passes = 5
        # self.num_passes_gropus = np.zeros((2, self.num_passes)).astype(np.int8)
        # self.num_passes_gropus[0] = [1,2,3,4,5,6,7,8,9,10]
        # self.num_passes_gropus[1] = [1,2,2,3,3,4,4,5,5,5]
        
        self.color_list = [ (0,0,255), (3,233,252), (0,255,0), (255,0,0), (255,100,100) ]

        self.pass_width = 4

        self.roller_orientation = 0.0

        self.chech_small_movement = True 
        self.small_move_critiria = 3
        
        # main loop conuter 
        self.cb_counter = 0
        
        # self.mchn_w = 6 
        # self.mchn_l = 6 
        
        # self.cell_size = 5

        # self.click_pt_1 = [53.315735, -113.588066 ] #[53.313459, -113.580967 ]
        # self.click_pt_2 = [53.308094, -113.573614 ] # [53.309228, -113.575307 ]

        # self.deg_near_last = 0
        
        self.window_detection_name = "Pass Count"
        self.satelite_size_bar_name = 'view range'
        self.satelite_size_low = 100
        self.satelite_size_lowest = 100
        self.satelite_size_high = 1500


        self.actual_visible_size_v = 360
        self.actual_visible_size_u = 600

        self.crop_ratio = float(self.actual_visible_size_v) / self.actual_visible_size_u
        # print('---------------------------------------')
        # print(self.crop_ratio)
        self.view_cropping_size_u = 150
        self.view_cropping_size_v = int(self.view_cropping_size_u * self.crop_ratio)
        

        self.icon = cv2.resize(cv2.imread("myself_roller.png", -1), (40,35) )
        self.icon_other = cv2.resize(cv2.imread("other_roller.png", -1), (40,35) )

        

        self.legend_base = cv2.resize( cv2.imread( 'passcount_lenged.png' ), ( 200, self.actual_visible_size_v ) )
        self.complete_legend()


        cv2.namedWindow(self.window_detection_name)
        cv2.createTrackbar(self.satelite_size_bar_name , self.window_detection_name , self.satelite_size_low, self.satelite_size_high, self.on_low_H_thresh_trackbar)

        # start_picture = cv2.imread( 'load.png' )
        # cv2.imshow( self.window_detection_name, cv2.resize( start_picture, ( self.actual_visible_size_u, self.actual_visible_size_v ) ) )
        # cv2.waitKey(30)


    def on_low_H_thresh_trackbar(self, val):
        if val < self.satelite_size_lowest:
            val = self.satelite_size_lowest
        self.view_cropping_size_u = int(val)
        self.view_cropping_size_v = int(self.view_cropping_size_u * self.crop_ratio)
        low_H = val
        low_H = min(self.satelite_size_high-1, low_H)
        cv2.setTrackbarPos(self.satelite_size_bar_name, self.window_detection_name, low_H)
        


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

        

        self.legend_base[ v1:v1+roller_size[1], u1:u1+roller_size[0] ] = cv2.resize( cv2.imread('me_solid_bg.png'), roller_size )

        self.legend_base[ v1+roller_spacing:v1+roller_size[1]+roller_spacing, u1:u1+roller_size[0] ] = cv2.resize( cv2.imread('other_solid_bg.png'), roller_size )

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
            
            file_names_list = [f for f in listdir('maps/') if isfile(join('maps/', f))] 

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

        # the pixel location of machine itself, in the current map image 
        machine_u, machine_v = self.latlonToImgpixel(lat= lat, lon=lon)

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
        hf_crop_u = self.view_cropping_size_u / 2
        hf_crop_v = self.view_cropping_size_v / 2
        # print('u=={}==={}=='.format(self.view_cropping_size_u, hf_crop_u))
        # print('v=={}==={}=='.format(self.view_cropping_size_v, hf_crop_v))
        roi_range = ( machine_v-hf_crop_v, machine_v+hf_crop_v, machine_u- hf_crop_u, machine_u+hf_crop_u )
        print('roi_range')    
        print(roi_range)

        last_machine_u, last_machine_v = self.latlonToImgpixel(lat= self.last_latlon[0], lon= self.last_latlon[1])
        last_last_machine_u, last_last_machine_v = self.latlonToImgpixel(lat= self.last_last_latlon[0], lon= self.last_last_latlon[1])

        if self.chech_small_movement:
            if ( abs( machine_u - last_machine_u) + abs( machine_v - last_machine_v) ) < self.small_move_critiria:
                print('move is too small')
                self.cb_counter += 1
                return 
            

        this_pass_grid, this_small_array_top, this_small_array_left, this_sa_size_v, this_sa_size_u, this_corners = self.UVToRecCorners(new_u= machine_u, new_v= machine_v, last_u= last_machine_u, last_v= last_machine_v)
        
        vector_1 =  [ machine_u-last_machine_u, machine_v - last_machine_v ]
        vector_2 = [ last_machine_u - last_last_machine_u, last_machine_v - last_last_machine_v ]

        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        direction_change = np.arccos(dot_product) * 180.0 / np.pi

        print('direction_change : ', direction_change )
        
        

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

            combd_array[combd_array[:,:]==255] = 0

            this_pass_grid = combd_array
            this_small_array_left = int( combd_array_left )
            this_small_array_top  = int( combd_array_top  )
            this_sa_size_v = combd_array_size_v
            this_sa_size_u = combd_array_size_u

            print(combd_array)

            

        # update the entire 3x3 grid 
        self.grid[ this_small_array_top:this_small_array_top+this_sa_size_v, this_small_array_left:this_small_array_left+this_sa_size_u ] += this_pass_grid

        self.last_latlon = [lat, lon] 

        # crop the grid and map for visulization 
        grid_roi = self.grid[ roi_range[0]:roi_range[1], roi_range[2]:roi_range[3]]
        # print(grid_roi.shape)

        map_roi = self.map[ roi_range[0]:roi_range[1], roi_range[2]:roi_range[3]] 
        # print(map_roi.shape)

        # assign colors based on the pass count value 
        for i in range(1, self.num_passes):
            '''if i == 1:
                ci = 0
            elif i == 2:
                ci = 1
            elif i == 3 or i == 4:
                ci = 2
            elif i == 5 or i> 5:
                ci = 2'''
            ci = i -1
                
            map_roi[ grid_roi[:,:] == i ] = self.color_list[ci]

        # for the grid with pass count larger than max value, assign the highest color to it
        map_roi[ grid_roi[:,:] >= self.num_passes ] = self.color_list[self.num_passes-1]

        # resize the cropped imaget 
        roi_resized = cv2.resize( map_roi, ( self.actual_visible_size_u, self.actual_visible_size_v ) )
        
        # add roller icon at location of gps 
        add_icon(self.icon, self.roller_orientation , (int(self.actual_visible_size_u/2), int(self.actual_visible_size_v/2) ), roi_resized)

        # add lengend 
        showimg = np.hstack((roi_resized, self.legend_base))

        # update the tile index being used in the current view 
        self.current_center_tile_index = new_ll_tile_index

        t2  = time.time()
        
        cv2.imwrite( 'save/{}.jpg'.format(self.cb_counter) , showimg )

        print( 'time in callback: {} ms'.format( int((t2-t1)*1000 ) ))

        cv2.imshow( self.window_detection_name, showimg )
        # cv2.imshow( 'grid', cv2.resize( self.grid * 30, (800,800) ) )
        cv2.waitKey(30)

        self.cb_counter += 1


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
    

    def UVToRecCorners(self, new_u, new_v, last_u, last_v ):
        hfw = self.pass_width / 2
        du = new_u - last_u
        dv = new_v - last_v
        orientation = math.atan2(dv,du) * -1
        self.roller_orientation = orientation * 180.0 / np.pi

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

        cv2.imwrite('tmpgrids/{}_v0.png'.format(self.cb_counter), self.resize_img( local_array_cp*180, 5) )

        print( '\nlocal_array size: \n{}, \ncorners: \n{}\n'.format( local_array.shape, localized_corners  ) )

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

        cv2.imwrite('tmpgrids/{}_v1.png'.format(self.cb_counter), self.resize_img( local_array*180, 5) )


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

        cv2.imwrite('tmpgrids/{}_v2.png'.format(self.cb_counter), self.resize_img( local_array*180, 5) )

        return (local_array , local_boundary_top, local_boundary_left, size_v, size_u , corners)


    def BresenLine(self, v1, u1, v2, u2):
        return list(bresenham(int(v1), int(u1), int(v2), int(u2) ) )
                
    def find_points(self,  corners):
        # im = img.copy()
        im = np.zeros_like( self.grid )
        cv2.polylines(im, [corners], True, 200)
        ret, thresh = cv2.threshold(im, 10, 255, 0)
        aout = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        conts = aout[0]
        #print(conts)
        return conts



        
    def LatLonToTileIndex(self, lat, lon):
        # check the range of input, give warning if not right 
        # if lat > 65 or lat < 45 :
        #     print('\n*********************************')
        #     print('check input latitude value range ')
        #     print('\n*********************************')
        # if lon > -106 or lon < -125 :
        #     print('\n*********************************')
        #     print('check input longitude value range ')
        #     print('\n*********************************')
        # find the tile where this latlon locate at 
        lat_diffs_from_all_tiles = np.abs( np.array( self.all_tile_center_lats) - lat )
        lon_diffs_from_all_tiles = np.abs( np.array( self.all_tile_center_lons) - lon ) 
        lat_index = np.argmin(lat_diffs_from_all_tiles)
        lon_index = np.argmin(lon_diffs_from_all_tiles)

        return [lon_index, lat_index]



    
    def AssembleTiles(self, center_tile_x, center_tile_y, x_amount, y_amount ):
        if x_amount % 2 != 1 or y_amount % 2 != 1:
            print('tile assembly input size wrong ')

        print('assembling the tiles into map')

        check_each_image = False

        x_start = center_tile_x - (x_amount-1)/2
        y_start = center_tile_y + (y_amount-1)/2 

        complete_image = np.zeros( ( x_amount * self.image_resolution , y_amount * self.image_resolution, 3) ).astype(np.uint8)
        sz = self.image_resolution  # this is only to use a shorter name for the image resolution in the below lines 

        complete_grid  = np.zeros_like(complete_image)[:,:,0]

        for i in range(x_amount):
            x = x_start+i
            for j in range(y_amount):
                y = y_start-j 
                img_name = 'maps/' + '{}_{}.jpg'.format( x , y )
                one_tile = cv2.imread( img_name )
                npy_name = 'maps/' + '{}_{}.npy'.format( x , y )
                one_grid = np.load(npy_name)
                if check_each_image :
                    cv2.imshow( "map", cv2.resize( one_tile , (640,640) ) )
                    cv2.waitKey(30)
                    print( j*sz, (j+1)*sz, i*sz, (i+1)*sz )

                complete_image[ j*sz:(j+1)*sz, i*sz:(i+1)*sz ] = one_tile
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


    def DisassembleAndSave(self):
        center_tile_x = self.current_center_tile_index[0]
        center_tile_y = self.current_center_tile_index[1]
        print('dis-assembling the current map and grid')
        print(self.current_center_tile_index)

        hf = (self.amount_of_tile_for_map-1)/2

        left_top_tile_center_x = center_tile_x - hf
        left_top_tile_center_y = center_tile_y + hf 

        for i in range( self.amount_of_tile_for_map ):        #left_top_tile_center_x, left_top_tile_center_x + self.amount_of_tile_for_map):
            for j in range( self.amount_of_tile_for_map ):       #left_bottom_tile_center_y, left_bottom_tile_center_y + self.amount_of_tile_for_map):
                one_grid = self.grid[ j*self.image_resolution : (j+1)*self.image_resolution, i*self.image_resolution: (i+1)*self.image_resolution ]
                this_tile_x = left_top_tile_center_x + i
                this_tile_y = left_top_tile_center_y - j
                npy_name =  'maps/{}_{}.npy'.format( this_tile_x, this_tile_y ) 
                np.save( npy_name  , one_grid )



    def latlonToImgpixel(self, lat, lon ):
        if lat > self.current_lat_range[1] or lat < self.current_lat_range[0]:
            print(' this latitude is not in current map image ')
        if lon > self.current_lon_range[1] or lon < self.current_lon_range[0]:
            print(' this longitude is not in current map image ')

        v = int( (lat - self.current_lat_range[1])*-1 / self.lat_pix_ratio )
        u = int( (lon - self.current_lon_range[0]) / self.lon_pix_ratio )

        return (u,v)

        




def main():
    
    rospy.init_node('passnode', anonymous=True)
    
    obc = gps2grid()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()








