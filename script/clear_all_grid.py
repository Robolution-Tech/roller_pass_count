
import numpy as np
from os import listdir
from os.path import isfile, join


# get all npy file names
all_names = listdir('maps/')

npy_name_list = []
for i in range(len(all_names)):
    if 'npy' in all_names[i]:
        npy_name_list.append(all_names[i])

print(npy_name_list)

empty_array = np.zeros((1024,1024))
for i in range(len(npy_name_list)):
    np.save('maps/'+npy_name_list[i], empty_array)
    print("{} / {}".format(i, len(npy_name_list)))




