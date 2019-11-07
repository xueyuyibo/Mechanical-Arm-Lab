# import cv2
# import numpy as np
# import freenect

# depth_frame = freenect.sync_get_depth()[0]

# np.clip(depth_frame,0,2**10 - 1,depth_frame)
# depth_frame >>= 2
# depth_frame = depth_frame.astype(np.uint8)

# print depth_frame
# cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
# cv2.imshow('window', depth_frame)

# while True:
#     ch = 0xFF & cv2.waitKey(10)
#     if ch == 0x1B:
#             break
# cv2.destroyAllWindows()

#-------color meter---
# import sys
# import cv2
# import numpy as np

# font = cv2.FONT_HERSHEY_SIMPLEX

# def mouse_callback(event,x,y,flags,param):
#     r = img[y][x][2]
#     g = img[y][x][1]
#     b = img[y][x][0]
#     h = hsv[y][x][0]
#     s = hsv[y][x][1]
#     v = hsv[y][x][2]
#     output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
#     output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
#     tmp = img.copy()
#     cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (0,0,0))
#     cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (0,0,0))
#     cv2.imshow('window', tmp)
#     if event == cv2.EVENT_LBUTTONDOWN:
#         print "bgr: (%d, %d, %d) \nhsv: (%d, %d, %d)" % (b,g,r,h,s,v)

# if len(sys.argv) == 2:
#     print "Opening " + str(sys.argv[1])
#     img = cv2.imread(sys.argv[1])
#     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#     # lower = np.array([110, 50, 50])
#     # upper = np.array([130, 255, 255])
#     lower_yellow = np.array([20, 100, 200])
#     upper_yellow = np.array([24, 250, 255])    
#     mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
#  #----------------- select range 
#     cv2.namedWindow("window",1)
#     cv2.imshow('window', img)
#     cv2.setMouseCallback("window",mouse_callback)

#     while True:
#         ch = 0xFF & cv2.waitKey(10)
#         if ch == 27:
#             break
#     cv2.destroyAllWindows()    

# else:
#     print "Expected filename as argument"

# #----------------- test selection
# # kernel = np.ones((4,4),np.uint8)
# # # erosion = cv2.erode(master, kernel, iterations = 1)
# # # dilation = cv2.dilate(master,kernel,iterations = 1)
# # closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
# # # cv2.namedWindow("window",1)
# # cv2.imshow('img', img)
# # cv2.imshow('mask', mask)
# # cv2.imshow('closing', closing)
# # while True:
# #     ch = 0xFF & cv2.waitKey(10)
# #     if ch == 27:
# #         break
# # cv2.destroyAllWindows()    


import cv2
import freenect

rgb_frame = freenect.sync_get_video()[0]
rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

print rgb_frame.shape
cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
cv2.imshow('window', rgb_frame)
while True:
    ch = 0xFF & cv2.waitKey(10)
    if ch == 0x1B:
            break
cv2.destroyAllWindows()