import cv2
import freenect
import numpy as np

font = cv2.FONT_HERSHEY_SIMPLEX

def mouse_callback(event,x,y,flags,param):
    r = img[y][x][2]
    g = img[y][x][1]
    b = img[y][x][0]
    h = hsv[y][x][0]
    s = hsv[y][x][1]
    v = hsv[y][x][2]
    output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
    output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
    tmp = img.copy()
    cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (0,0,0))
    cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (0,0,0))
    cv2.imshow('window', tmp)
    if event == cv2.EVENT_LBUTTONDOWN:
    	# print "hsv: (%d, %d, %d)" % (h,s,v)
        print "bgr: (%d, %d, %d) \nhsv: (%d, %d, %d)" % (b,g,r,h,s,v)


rgb_frame = freenect.sync_get_video()[0]
img = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
# print rgb_frame
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
cv2.imshow('window', img)

cv2.setMouseCallback("window",mouse_callback)

while True:
    ch = 0xFF & cv2.waitKey(10)
    if ch == 0x1B:
            break
cv2.destroyAllWindows()

# yellow first layer middle hsv
# h: 30, s: 177, v: 254

# yellow second layer middle hsv
# h: 30, s: 178, v: 254

# yellow third layer middle hsv
# h: 30, s: 165, v: 254

# blue first layer middle hsv
# h: 113, s: 156, v: 165

# blue second layer middle hsv
# h: 114, s: 156, v: 177

# blue third layer middle hsv
# h: 114, s: 131, v: 175



#####---------------range, this can be done only with h value, can relax the bound
# orange, h: 7, [0, 10]
# yellow, h: 30 [20, 35]
# green, h: 57, [50, 80]
# blue, h: 113 [110, 120]
#####-----------if h > 125, need additional info to decide the color, use r value
# pink, h: 168, [165, 172], r: [190,255]
# red, h: 174, [173, 180], r: [110, 189]
###-------problems
# purple, h: 144 [136, 145], r: [80, 109]
# black, h: 135 [150, 170], r: [0, 79]