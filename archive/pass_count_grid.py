import plotly.graph_objects as go
import pandas as pd
import csv
import glob
import json
import os
import numpy as np
from bresenham import bresenham

import matplotlib.pyplot as plt




# get working area info before start
click_pt_1 = [53.312079, -113.581980 ]
click_pt_2 = [53.309733, -113.576696 ]

bottom_left_corner = [ min(click_pt_1[0],click_pt_2[0]), min(click_pt_1[1],click_pt_2[1]) ]
top_right_corner   = [ max(click_pt_1[0],click_pt_2[0]), max(click_pt_1[1],click_pt_2[1]) ]

bottom_left_corner[0] = int( bottom_left_corner[0] * 100000 ) * 10
bottom_left_corner[1] = int( bottom_left_corner[1] * 100000 ) * 10

top_right_corner[0]   = int( top_right_corner[0] * 100000 ) * 10
top_right_corner[1]   = int( top_right_corner[1] * 100000 ) * 10


# set the size for each cell in the grid
# e.g.  5  means  each cell has length of 0.5 meter 
# smaller size -> larger array, finer grid
cell_size = 5

# the latitude bins
lats = range( bottom_left_corner[0], top_right_corner[0], cell_size )
# the longitude bins
lons = range( bottom_left_corner[1], top_right_corner[1], cell_size )


#print(lats)

#print(len(lons))


# build the array to store the information of the grid
# the third number is the number of layers 
# each layer contains one type of information 
grid = np.empty( [ len(lons), len(lats), 6]  )

# the meaning of each layer in the array
road_area_layer = 0
pass_count_layer = 1

print(grid.shape)



# machine dimensions  # meter
mchn_w = 4  # width
mchn_l = 8  # length 






# for an input: point ( 53.06215, -113.325164 )
# return the grid index where this point locate at 
def get_point_grid_loc( pt ):
    global lats, lons
    pt_lat = int( pt[0]*100000 ) *10
    pt_lon = int( pt[1]*100000 ) *10
    lat_index = lats.index(pt_lat)
    lon_index = lons.index(pt_lon)
    return [lat_index, lon_index]
    

# input: 
#   new: the newest gps location
#   last: the previous gps location
# return:
#   a list containing the grid cells covered by the straight line connectting new and last 
def draw_line( new, last ):
    # new:  [ xx.x, xx.x ]
    # last: [ xx.x, xx.x ]
    # bresenham(x0, y0, x1, y1)
    new_grid_loc  = get_point_grid_loc(new)
    last_grid_loc = get_point_grid_loc(last)
    
    #print(new_grid_loc  ,last_grid_loc  )
    
    pixs = list(bresenham(last_grid_loc[0], last_grid_loc[1], new_grid_loc[0], new_grid_loc[1] ) )
    #print(pixs)
    return pixs
 
    
# input: a list containing the grid cells covered by the straight line connectting new and last
# return: the thickened line with considering the width of machine  
def add_machine_pose(line_pixels):
    global mchn_w, mchn_l
    gridy_l = int(mchn_w / (cell_size/10.0) )
    gridy_w = int(mchn_l / (cell_size/10.0) )
    #print( gridy_w, gridy_l )
    
    if gridy_w % 2 == 0:
        gridy_w += 1
    gridy_w_hf = (gridy_w-1)/2
    
    if gridy_l % 2 == 0:
        gridy_l += 1
    gridy_l_hf = (gridy_l-1)/2
    
    enlarged = [] # list to store the grids of the thick line
    '''for i in line_pixels:
        enlarged.append(i)
        print(i)
        ws = range( i[0]-gridy_w_hf, i[0]+gridy_w_hf )
        ls = range( i[1]-gridy_l_hf, i[1]+gridy_l_hf )
        for wi in ws:
            for li in ls:
                enlarged.append( (wi,li) )'''
    for i in line_pixels:
        enlarged.append(i)
        #print(i)
        ws = range( i[0]-gridy_w_hf, i[0]+gridy_w_hf )
        ls = range( i[1]-gridy_l_hf, i[1]+gridy_l_hf )
        for li in ls:
            #for li in ls:
            enlarged.append( (i[0],li) )
                
    enlarged = list(dict.fromkeys(enlarged))# remove duplicated item in the list
    return enlarged
    





# get the newest gps location
# in this code, it is reading from json files
def get_gps( i, path):
    filepath = path + '/' + str(i) + '.json'
    fff = json.load( open( filepath, 'r' )  )
    return fff


# number of main loop  /  number of gps locaiton measurements 
numb = 200 

# folder for storing the json files, 
json_folder_path = 'gpsss' 

for i in range(numb): 

    #print('\n------' + str(i) + '------')
    
    ll = get_gps(i,json_folder_path) 
    
    #print('json get ' + str(ll[0]) )
    
    if i == 0:
        ll_last = ll 
        continue
        
    line = draw_line( ll, ll_last )
    ll_last = ll
    
    line.remove(line[0])
    
    line = add_machine_pose(line)
    
    for gds in line:
        grid[ gds[1] , gds[0] , pass_count_layer ] += 1
    



plt.imshow( grid[:,:,1].T ,origin='lower' )
plt.axis('equal')
plt.ylabel('some numbers')
plt.show()




