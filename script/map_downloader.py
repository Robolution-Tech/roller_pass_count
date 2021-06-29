
import rospy
import numpy as np
from std_msgs.msg import String
from update_map import gen_map



def callback(data):
    string_raw = data.data
    split_category = string_raw.split(';')
    tile_indeces  = split_category[0]
    tile_center_ll = split_category[1]

    tile_indeces_slpit = tile_indeces.split(' ')
    tile_center_ll_split = tile_center_ll.split(' ')
    number_of_images = len(tile_indeces_slpit) / 2
    img_names_list = []
    for i in range(number_of_images):
        img_name =  'maps/' + tile_indeces_slpit[i*2] + '_' + tile_indeces_slpit[2*i+1] + '.jpg' 
        center_lon = tile_center_ll_split[i*2][:-2]
        center_lat = tile_center_ll_split[i*2+1][:-2]
        gen_map(center_lon, center_lat, 17, 1024, img_name=img_name)
        print(img_name, center_lon, center_lat)
    

    print('tile_indeces_slpit ')
    print(tile_indeces_slpit)
    print('tile_center_ll ')
    print(tile_center_ll)

def listener():
    rospy.init_node('map_request_listener', anonymous=True)
    rospy.Subscriber("map_download_request", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()























