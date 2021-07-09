import numpy as np

num_passes = 5
color_list = [[0, 0, 255], [3, 233, 252], [0, 255, 0], [255, 0, 0], [255, 100, 100]]
img = np.array() # img
points = np.array()
# Approach 1:
for i in range(1, num_passes+1):
    img[points[:,:] == i] = color_list[i-1]

# Approach 2:

