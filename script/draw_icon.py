import math
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

    # print(loc,x_off,y_off, y1,y2, x1,x2)

    alpha_s = icon_dst[:, :, 3] / 255.0
    alpha_l = 1.0 - alpha_s
    for c in range(0, 3):
        bg_img[y1:y2, x1:x2, c] = (alpha_s * icon_dst[:, :, c] + alpha_l * bg_img[y1:y2, x1:x2, c])
    return bg_img

# img = cv2.imread("img_17.png", -1)
# # img_a = cv2.cvtColor(img, cv2.COLOR_BGRA2RGBA)
# icon = cv2.imread("roller_icon.png", -1)
# height, width = (icon.shape[0] , icon.shape[1])
# center = (width/2,height/2)
# size = (width, height)
# angle = 45.0
# scale = 0.5

# rotation_matrix = cv2.getRotationMatrix2D(center, angle, scale)
# # icon_dst = cv2.warpAffine(icon, rotation_matrix, size,
# #                          flags=cv2.INTER_LINEAR,
# #                          borderMode=cv2.BORDER_TRANSPARENT)
# icon_dst = rotate_image(icon, 45)

# cv2.imshow("b", icon_dst)
# x_offset = int((img.shape[1] - icon_dst.shape[1])/2)
# y_offset =  int((img.shape[0] - icon_dst.shape[0])/2)

# y1, y2 = y_offset, y_offset +icon_dst.shape[0]
# x1, x2 = x_offset, x_offset + icon_dst.shape[1]

# alpha_s = icon_dst[:, :, 3] / 255.0
# alpha_l = 1.0 - alpha_s

# for c in range(0, 3):
#     img[y1:y2, x1:x2, c] = (alpha_s * icon_dst[:, :, c] + alpha_l * img[y1:y2, x1:x2, c])

# cv2.imshow("a", img)
# cv2.waitKey(0)


