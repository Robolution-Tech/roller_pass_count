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
from draw_icon import rotate_image, add_icon


class gps2grid():

    def __init__(self):
        self.sub = rospy.Subscriber("/ublox_gps_raw",NavSatFix,self.callback)
        self.cb_counter = 0
        self.ll_last = [0,0]
        self.ll= [0,0]
        self.mchn_w = 6 
        self.mchn_l = 6 
        self.cell_size = 5

        self.click_pt_1 = [53.312079, -113.582580 ]
        self.click_pt_2 = [53.308733, -113.576996 ]

        self.deg_near_last = 0

        

        self.road_area_layer = 0
        self.pass_count_layer = 1
        self.lat_int_layer = 2
        self.lon_int_layer = 3

        self.grid = 0

        self.lats = []
        self.lons = []

        self.setup_grids()

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


    def on_low_H_thresh_trackbar(self, val):
        if val < self.satelite_size_lowest:
            val = self.satelite_size_lowest
        self.satelite_view_size = int(val)
        low_H = val
        low_H = min(self.satelite_size_high-1, low_H)
        cv2.setTrackbarPos(self.satelite_size_bar_name, self.window_detection_name, low_H)
        

    
    def find_current_xy(self, ll, norf):
        if norf == "near":
            res = self.resolution_near
        # else:
            # res = self.resolution_far
        assert len(ll) == 2
        dmeter_lat = (ll[0] - self.center_lat) * self.meter_per_lat
        dmeter_lon = (ll[1] - self.center_lon) * self.meter_per_lon
        x = int(dmeter_lon / res + self.w//2)
        y = int(dmeter_lat / res + self.h//2)
        y = self.h - y
        return (x, y)
        

    def callback(self, navfix):
        lat =  navfix.latitude  
        lon =  navfix.longitude*-1

        lat = self.lat_dmd2dd(lat)
        lon = self.lon_dmd2dd(lon)


        self.ll = [lat,lon]
        print('\ncallback')
            
        if self.cb_counter == 0:
            self.ll_last = self.ll
            self.cb_counter += 1
            
        if self.cb_counter != 0:
    
            line = self.draw_line( self.ll, self.ll_last )
            self.ll_last = self.ll
    
            line.remove(line[0])
    
            line = self.add_machine_pose(line)


    
            for gds in line:
                self.grid[ gds[1] , gds[0] , self.pass_count_layer ] += 1

        
            # plt.imshow(self.grid)
            # plt.show()

            point_list = self.grid.copy()
            # point_list[:,:,2] = point_list[:,:,2] * -1.0 + 2*self.center_lat * 1000000.
            '''for i in range(1, 5):
                lat = point_list[point_list[:,:,1]==i][:,2] / 1000000.
                lon = point_list[point_list[:,:,1]==i][:,3] / 1000000.
                dLat = (lat - self.center_lat)
                dLon = (lon - self.center_lon)
                dmeter_lat = dLat * self.meter_per_lat 
                dmeter_lon = dLon * self.meter_per_lon 
                x = (dmeter_lon / self.resolution_near + self.w//2).astype(np.int32).tolist()
                y = (dmeter_lat / self.resolution_near + self.h//2).astype(np.int32).tolist()

                print(i, x,y)

                # x_far = (dmeter_lon / self.resolution_far + self.w//2).astype(np.int32).tolist()
                # y_far = (dmeter_lat / self.resolution_far + self.h//2).astype(np.int32).tolist()
                assert len(x) == len(y)
                for j in range(len(x)):
                    center = (x[j], y[j])
                    # center_far = (x_far[j], y_far[j])
                    cv2.circle(self.out, center, 1, self.color_list[i-1], -1)
                    # cv2.circle(self.out_far, center_far, 1, self.color_list[i-1], -1)
            '''


            for i in range(1,5):
                lat = point_list[point_list[:,:,1]==i][:,2] 
                lon = point_list[point_list[:,:,1]==i][:,3] 
                lat_to_corner_1 = int(self.corner_1_ll[1]*1000000) - lat
                lon_to_corner_1 = lon - int(self.corner_1_ll[0]*1000000)
                pix_h = lat_to_corner_1  / self.lat_per_pix
                pix_v = lon_to_corner_1  / self.lon_per_pix
                pix_h = np.array( pix_h, dtype='int')
                pix_v = np.array( pix_v, dtype='int')
                '''print('')
                #print(self.corner_1_ll[0],  self.corner_1_ll[1] )
                print(i)
                print('')
                print(lat[:10],lon[:10])
                print('')  
                print(lat_to_corner_1[:10],lon_to_corner_1[:10])  
                print('')  
                print(pix_h[:10],pix_v[:10])  
                #print('')  
                #print(pix_h.shape   ,   pix_v.shape  )'''
                for j in range(pix_h.shape[0]):
                    cv2.circle(self.out, (  pix_v[j], pix_h[j]  ) , 3, self.color_list[i-1], -1)
                    cv2.circle(self.out_street, (  pix_v[j], pix_h[j]  ) , 3, self.color_list[i-1], -1)

            curr_car_pix_hv =(  int((self.corner_1_ll[1]*1000000-self.ll[0]*1000000)/ self.lat_per_pix) , int((self.ll[1]*1000000-self.corner_1_ll[0]*1000000)/ self.lon_per_pix) )
            # print('\ncurr_car_pix_hv ')  
            # print(curr_car_pix_hv)

            curr_car_xy_near = ( curr_car_pix_hv[1], curr_car_pix_hv[0] )

            #curr_car_xy_near = self.find_current_xy(self.ll, "near")
            # curr_car_xy_far = self.find_current_xy(self.ll, "far")

            if (curr_car_xy_near[0] - self.last_xy[0]) != 0.0:
                print("aaaaa")
                deg_near = math.atan((curr_car_xy_near[1] - self.last_xy[1]) / (curr_car_xy_near[0] - self.last_xy[0]))
                deg_near = deg_near * 180 / np.pi * -1
            else:
                print("bbbbb")
                deg_near = self.deg_near_last 
            self.deg_near_last = deg_near

            print('deg_near = {}'.format(deg_near) )

            # print('\n  self.ll  ')  
            # print(self.ll)

            

            temp_near = self.out.copy()
            temp_street = self.out_street.copy()

            cv2.circle(temp_street, (curr_car_pix_hv[1], curr_car_pix_hv[0]) , 12, (0,255,0), -1)

            #print(curr_car_pix_hv)
            #print(curr_car_xy_near)
            

            add_icon(self.icon, deg_near, (curr_car_pix_hv[1], curr_car_pix_hv[0]), temp_near) #near

            if self.cb_counter != 0:
                self.last_xy = curr_car_xy_near


            # cv2.imwrite('pass_count_images/'+str(self.cb_counter)+'.jpg', self.out)
            
            # savefile = open('pass_count_images/'+str(self.cb_counter)+'.json', 'w')
            # json.dump(center, savefile)
            # savefile.close()

            splittter = np.zeros((512,20,3)) + 254
            splittter = np.array( splittter , dtype='uint8' )



            # crop ROI

            #satelite_view_size = 512
            svshf = self.satelite_view_size/2
            
            satelite_view_center_pix_v = curr_car_pix_hv[0]
            satelite_view_center_pix_u = curr_car_pix_hv[1]
            
            satelite_view_top = satelite_view_center_pix_v - svshf
            satelite_view_bottom = satelite_view_center_pix_v + svshf
            satelite_view_left = satelite_view_center_pix_u - svshf
            satelite_view_right = satelite_view_center_pix_u + svshf

            # print('ROI ')
            # print(satelite_view_bottom, satelite_view_top, satelite_view_left, satelite_view_right)

            satelite_view_img  = temp_near[  satelite_view_top:satelite_view_bottom, satelite_view_left:satelite_view_right,:  ]

            far_view_ratio = 1.8
            far_view_panning = int(svshf * far_view_ratio)

            street_view_img  = temp_street[  satelite_view_top-far_view_panning:satelite_view_bottom+far_view_panning, satelite_view_left-far_view_panning:satelite_view_right+far_view_panning,:  ]

            fk1 = cv2.resize( satelite_view_img, self.satelite_view_size_actual)
            fk2 = cv2.resize( street_view_img,   self.satelite_view_size_actual)

            temp = np.hstack((fk1, splittter, fk2))
            
            cv2.imwrite('pass_count_images/'+str(self.cb_counter-1)+'.jpg', temp)
            self.cb_counter += 1

            

            cv2.imshow(self.window_detection_name, temp)
            # cv2.imshow("b", temp_far)
            cv2.waitKey(10)
            
        

            



            showmat = 0

            if showmat == 1:
                plt.imshow( self.grid[:,:,1].T ,origin='lower' )
                plt.axis('equal')
                plt.ylabel('some numbers')
                plt.pause(0.001)
                plt.update()

        
                



    
    def lat_dmd2dd(self, lat):
        raw_string  = str(lat*100.0)
        #print(raw_string)
        
        lat_deg = int(raw_string[0:2])
        
        lat_min = float(raw_string[2:])/60.0
        
        #print( raw_string[0:2] , raw_string[2:]   )
        lat_dec = lat_deg + lat_min
        
        #print(lat_dec)
        return(lat_dec)


    def lon_dmd2dd(self, lon):
        raw_string  = str(lon*100.0)
        #print(raw_string)
        
        lat_deg = int(raw_string[1:4])
        
        lat_min = float(raw_string[4:])/60.0
        
        #print( raw_string[0:4] , raw_string[4:]   )
        lat_dec = lat_deg + lat_min
        lat_dec *= -1.0 
        #print(lat_dec)
        return(lat_dec)




    
    # input: 
    #   new: the newest gps location
    #   last: the previous gps location
    # return:
    #   a list containing the grid cells covered by the straight line connectting new and last 
    def draw_line( self,  new, last ):
        # new:  [ xx.x, xx.x ]
        # last: [ xx.x, xx.x ]
        # bresenham(x0, y0, x1, y1)
        new_grid_loc  = self.get_point_grid_loc(new)
        last_grid_loc = self.get_point_grid_loc(last)
        
        #print( 'in function :  draw_line ' )
        #print(new  ,last  )
        #print(new_grid_loc  ,last_grid_loc  )

        # print(last_grid_loc[0], last_grid_loc[1], new_grid_loc[0], new_grid_loc[1] )

        pixs = list(bresenham(int(last_grid_loc[0]), int(last_grid_loc[1]), int(new_grid_loc[0]), int(new_grid_loc[1]) ) )
        #print(pixs)
        return pixs
    

    # for an input: point ( 53.06215, -113.325164 )
    # return the grid index where this point locate at 
    def get_point_grid_loc( self,  pt ):
        #global lats, lons, cell_size 
        pt_lat = int( pt[0]*1000000 )
        pt_lon = int( pt[1]*1000000 )
        #print(self.lats) 
        lat_index = math.floor( (pt_lat - self.lats[0]) / self.cell_size )
        lon_index = math.floor( (pt_lon - self.lons[0]) / self.cell_size )
        #lat_index = lats.index(pt_lat)
        #lon_index = lons.index(pt_lon)
        return [lat_index, lon_index]


    # input: a list containing the grid cells covered by the straight line connectting new and last
    # return: the thickened line with considering the width of machine  
    def add_machine_pose(self, line_pixels):
        mchn_w, mchn_l = self.mchn_w, self.mchn_l
        gridy_l = int(mchn_w / (self.cell_size/10.0) )
        gridy_w = int(mchn_l / (self.cell_size/10.0) )
        #print( gridy_w, gridy_l )
        
        if gridy_w % 2 == 0:
            gridy_w += 1
        gridy_w_hf = int((gridy_w-1)/2)
        
        if gridy_l % 2 == 0:
            gridy_l += 1
        gridy_l_hf = int((gridy_l-1)/2)
        
        enlarged = [] # list to store the grids of the thick line
        '''for i in line_pixels:
            enlarged.append(i)
            print(i)
            ws = range( i[0]-gridy_w_hf, i[0]+gridy_w_hf )
            ls = range( i[1]-gridy_l_hf, i[1]+gridy_l_hf )
            for wi in ws:
                for li in ls:
                    enlarged.append( (wi,li) )'''
                    
        if self.cb_counter  < 155:
            for i in line_pixels:
                enlarged.append(i)
                #print(i)
                ws = range( i[0]-gridy_w_hf, i[0]+gridy_w_hf )
                ls = range( i[1]-gridy_l_hf, i[1]+gridy_l_hf )
                for li in ls:
                    #for li in ls:
                    enlarged.append( (i[0],li) )
                    
        if self.cb_counter  > 155:
            for i in line_pixels:
                enlarged.append(i)
                #print(i)
                ws = range( i[0]-gridy_w_hf, i[0]+gridy_w_hf )
                ls = range( i[1]-gridy_l_hf, i[1]+gridy_l_hf )
                for wi in ws:
                    #for li in ls:
                    enlarged.append( (wi,i[1]) )
                    
        enlarged = list(dict.fromkeys(enlarged))# remove duplicated item in the list
        return enlarged




    def setup_grids(self):
        # get working area info before start
        click_pt_1 = self.click_pt_1
        click_pt_2 = self.click_pt_2

        bottom_left_corner = [ min(click_pt_1[0],click_pt_2[0]), min(click_pt_1[1],click_pt_2[1]) ]
        top_right_corner   = [ max(click_pt_1[0],click_pt_2[0]), max(click_pt_1[1],click_pt_2[1]) ]

        bottom_left_corner[0] = int( bottom_left_corner[0] * 1000000 )
        bottom_left_corner[1] = int( bottom_left_corner[1] * 1000000 )

        top_right_corner[0]   = int( top_right_corner[0] * 1000000 )
        top_right_corner[1]   = int( top_right_corner[1] * 1000000 )


        # set the size for each cell in the grid
        # e.g.  5  means  each cell has length of 0.5 meter 
        # smaller size -> larger array, finer grid
        

        # the latitude bins
        self.lats = range( bottom_left_corner[0], top_right_corner[0], self.cell_size )
        # the longitude bins
        self.lons = range( bottom_left_corner[1], top_right_corner[1], self.cell_size )

        #print(lats[0])
        #print(lats[1])
        #print(lats[2])

        #print(len(lons))


        # build the array to store the information of the grid
        # the third number is the number of layers 
        # each layer contains one type of information 
        self.grid = np.empty( [ len(self.lons), len(self.lats), 4]  )


        print('grid size : ' + str( self.grid.shape) )
        print('layer content: ')
        print(str(self.road_area_layer) + ' = road_area_layer')
        print(str(self.pass_count_layer) + ' = pass_count_layer')
        print(str(self.lat_int_layer) + ' = lat_int_layer')
        print(str(self.lon_int_layer) + ' = lon_int_layer')


        for i in range(len(self.lats)):
            aaaaaaa = self.lats[i]
            self.grid[ : , i ,  self.lat_int_layer] = aaaaaaa
            
        for i in range(len(self.lons)):
            aaaaaaa = self.lons[i]
            self.grid[ i , : ,  self.lon_int_layer] = aaaaaaa

        


def main():
    
    rospy.init_node('gpssub1', anonymous=True)
    
    obc = gps2grid()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()



