import numpy as np
import cv2


img = np.zeros((400, 400)).astype(np.uint8)
corners = np.array([(100, 100), (300, 150), (200, 300), (50, 200)])

def find_points(img, corners, count):
        im = img.copy()
        cv2.polylines(im, [corners], True, 200)
        ret, thresh = cv2.threshold(im, 10, 255, 0)
        conts, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print(conts)
        cnt = cv2.drawContours(im, conts, -1, 200, 3) 
        return im

cv2.imshow('img', img)
im = find_points(img, corners, 1)
cv2.imshow('after', im)
cv2.waitKey(0)
