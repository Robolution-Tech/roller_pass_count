import math
from time import time
import cv2
import numpy as np


def rotate_image(image, angle):
    """
    Rotates an OpenCV 2 / NumPy image about it's centre by the given angle
    (in degrees). The returned image will be large enough to hold the entire
    new image, with a black background
    """

    # Get the image size
    # No that's not an error - NumPy stores image matricies backwards
    image_size = (image.shape[1], image.shape[0])
    image_center = tuple(np.array(image_size) / 2)

    # Convert the OpenCV 3x2 rotation matrix to 3x3
    rot_mat = np.vstack(
        [cv2.getRotationMatrix2D(image_center, angle, 1.0), [0, 0, 1]]
    )

    rot_mat_notranslate = np.matrix(rot_mat[0:2, 0:2])

    # Shorthand for below calcs
    image_w2 = image_size[0] * 0.5
    image_h2 = image_size[1] * 0.5

    # Obtain the rotated coordinates of the image corners
    rotated_coords = [
        (np.array([-image_w2,  image_h2]) * rot_mat_notranslate).A[0],
        (np.array([ image_w2,  image_h2]) * rot_mat_notranslate).A[0],
        (np.array([-image_w2, -image_h2]) * rot_mat_notranslate).A[0],
        (np.array([ image_w2, -image_h2]) * rot_mat_notranslate).A[0]
    ]

    # Find the size of the new image
    x_coords = [pt[0] for pt in rotated_coords]
    x_pos = [x for x in x_coords if x > 0]
    x_neg = [x for x in x_coords if x < 0]

    y_coords = [pt[1] for pt in rotated_coords]
    y_pos = [y for y in y_coords if y > 0]
    y_neg = [y for y in y_coords if y < 0]

    right_bound = max(x_pos)
    left_bound = min(x_neg)
    top_bound = max(y_pos)
    bot_bound = min(y_neg)

    new_w = int(abs(right_bound - left_bound))
    new_h = int(abs(top_bound - bot_bound))

    # We require a translation matrix to keep the image centred
    trans_mat = np.matrix([
        [1, 0, int(new_w * 0.5 - image_w2)],
        [0, 1, int(new_h * 0.5 - image_h2)],
        [0, 0, 1]
    ])

    # Compute the tranform for the combined rotation and translation
    affine_mat = (np.matrix(trans_mat) * np.matrix(rot_mat))[0:2, :]

    # Apply the transform
    result = cv2.warpAffine(
        image,
        affine_mat,
        (new_w, new_h),
        flags=cv2.INTER_LINEAR
    )

    return result


def add_icon(icon, angle, loc, bg_img):
    assert icon is not None
    icon_dst = rotate_image(icon, angle)
    x_off = int(loc[0] - icon_dst.shape[1] / 2)
    y_off = int(loc[1] - icon_dst.shape[0] / 2)
    
    y1, y2 = y_off, y_off + icon_dst.shape[0]
    x1, x2 = x_off, x_off + icon_dst.shape[1]

    bg_w = bg_img.shape[1]
    bg_h = bg_img.shape[0]

    alpha_s = icon_dst[:, :, 3] / 255.0
    alpha_l = 1.0 - alpha_s
    panning = 100

    bg_tmp_w = bg_w + panning*2
    bg_tmp_h = bg_h + panning*2
    bg_tmp_d = bg_img.shape[2]

    
    
    for c in range(0, 3):
        
        

        if x1 >=0 and x2 <= bg_w and y1 >= 0 and y2 <= bg_h:
            # print('add icon c {}, case 1 '.format(c))
            bg_img[y1:y2, x1:x2, c] = ( alpha_s * icon_dst[:, :, c] + alpha_l * bg_img[y1:y2, x1:x2, c] )
        elif not ( x2 < 0 or x1 > bg_w or y1 > bg_h or y2 < 0 ):
            
            # print('add icon c {}, case 2 '.format(c))
            # print('tmp size {} {} {}'.format( bg_tmp_h, bg_tmp_w, bg_tmp_d  ))
            
            t1 = time()
            bg_tmp = np.zeros(( bg_tmp_h, bg_tmp_w, bg_tmp_d)).astype(np.uint8)
            t2 = time()
            bg_tmp[panning:panning+bg_h, panning:panning+bg_w] = bg_img 
            new_x1 = x1 + panning
            new_x2 = x2 + panning
            new_y1 = y1 + panning
            new_y2 = y2 + panning
            bg_tmp[new_y1:new_y2, new_x1:new_x2, c] = ( alpha_s * icon_dst[:, :, c] + alpha_l * bg_tmp[new_y1:new_y2, new_x1:new_x2, c] )
            bg_img = bg_tmp[panning:panning+bg_h, panning:panning+bg_w]
            t3 = time()
            # print('time in case 2, {} {} ms'.format( int( (t2-t1)*1000 ), int( (t3-t2)*1000 ) ) )

        # bg_img[y1:y2, x1:x2, c] = ( alpha_s * icon_dst[:, :, c] + alpha_l * bg_img[y1:y2, x1:x2, c] )

    return bg_img

