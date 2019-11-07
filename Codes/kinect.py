import cv2
from numpy.linalg import inv
from PyQt4.QtGui import QImage
import freenect
import math
import time
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(threshold=np.inf)

class Kinect():
    def __init__(self):
        self.currentVideoFrame = np.array([])
        self.currentDepthFrame = np.array([])
        if(freenect.sync_get_depth() == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True
        
        # mouse clicks & calibration variables
        
        self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])

        ######### hard code value 
        # self.depth2rgb_affine[0][0] = 9.21093609e-01
        # self.depth2rgb_affine[0][1] = 5.74762797e-04
        # self.depth2rgb_affine[0][2] = 1.00493484e+01
        # self.depth2rgb_affine[1][0] = 7.21723597e-04
        # self.depth2rgb_affine[1][1] = 9.18735922e-01
        # self.depth2rgb_affine[1][2] = 4.92139662e+01
        ########### this needs to be changed in actual test!!
        # self.intrinsic_matrix = self.loadCameraCalibration()
        self.intrinsic_matrix = np.eye(3,dtype = float)
        # print self.intrinsic_matrix.shape
        self.co_eff_camera_2_world = np.eye(4, dtype = float)
        ######## hard code value
        # self.co_eff_camera_2_world[0][0] = 1.00390382e+03
        # self.co_eff_camera_2_world[0][1] = 6.81941154e+01
        # self.co_eff_camera_2_world[0][2] = -4.11826135e+01
        # self.co_eff_camera_2_world[0][3] = 9.41156288e+01

        # self.co_eff_camera_2_world[1][0] = -7.20439190e+01
        # self.co_eff_camera_2_world[1][1] = 1.01489943e+03
        # self.co_eff_camera_2_world[1][2] = -2.21034947e+01
        # self.co_eff_camera_2_world[1][3] = 1.44523136e+01

        # self.co_eff_camera_2_world[2][0] = 1.15061591e+01
        # self.co_eff_camera_2_world[2][1] = -3.91477233e+00
        # self.co_eff_camera_2_world[2][2] = -9.66161407e+02
        # self.co_eff_camera_2_world[2][3] = 9.08135219e+02

        # self.co_eff_camera_2_world[3][0] = 0.00000000e+00
        # self.co_eff_camera_2_world[3][1] = 0.00000000e+00
        # self.co_eff_camera_2_world[3][2] = 0.00000000e+00
        # self.co_eff_camera_2_world[3][3] = 1.00000000e+00
        # print self.co_eff_camera_2_world
        # print self.co_eff_camera_2_world.shape        
        self.kinectCalibrated = False
        ######### this line needs to be changed in actual test
        # self.kinectCalibrated = True
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((8,2),int)
        self.depth_click_points = np.zeros((8,2),int)

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])

        """ block info """
        self.block_contours = np.array([])

    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        if(self.kinectConnected):
            self.currentVideoFrame = freenect.sync_get_video()[0]
        else:
            self.loadVideoFrame()
        self.processVideoFrame()
        

    def processVideoFrame(self):
        cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,0,255),3)


    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            if(self.kinectCalibrated):
                self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
            else:
                self.currentDepthFrame = freenect.sync_get_depth()[0]
        else:
            self.loadDepthFrame()

    
    def loadVideoFrame(self):
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

    def convertFrame(self):
        """ Converts frame to format suitable for Qt  """
        try:
            img = QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for Qt  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """ 
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
            cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),3)

            img = QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2): # transform c1 to c2
        """
        Given 2 sets of corresponding coordinates, 
        find the affine matrix transform between them.

        TODO: Rewrite this function to take in an arbitrary number of coordinates and 
        find the transform without using cv2 functions
        """
        print('-----coord1----')
        print(coord1)
        pts1 = coord1[0:]

        print('----pts1----')
        print pts1

        pts1.astype(np.float32)
        # print('----pts1----')
        # print pts1
        pts2 = coord2[0:]
        pts2.astype(np.float32)   
        print('----pts2----')
        print pts2

        n_p = pts1.shape[0] # number of pts used in calculation
        nd = pts1.shape[1] # dimension
        # print nd*np
        # print np    


        # A = np.zeros(shape=(8,6))
        # B = np.zeros(shape=(8,1))
       
        # for i in xrange(0,7,2):
        #     A[i][0:2] = np.array([pts1[i/2][0], pts1[i/2][1]])
        #     A[i][2] = 1
        #     A[i+1][3:5] = np.array([pts1[i/2][0], pts1[i/2][1]])
        #     A[i+1][5] = 1
        #     #tmp2 = 
        #     B[i] = pts2[i/2][0]
        #     B[i+1] = pts2[i/2][1]

        ###############2019_02_07
        nAr = nd*n_p
        nAc = nd*(nd+1)
        A = np.zeros((nAr, nAc), dtype = float)
        nBr = nd*n_p
        B = np.zeros((nBr,1), dtype = float)   
        # print np.kron(np.eye(3), a)      
        for i in xrange(0,nd*n_p,nd):
            print np.kron(np.eye(nd), np.append(pts1[i/nd], 1 ) ) 
            A[i:i+nd][0:] = np.kron(np.eye(nd), np.append(pts1[i/nd], 1 )) 
            for j in xrange(0, nd, 1):
                B[i+j][0] = pts2[i/nd][j]
                
        print('----A-----')
        print A
        print('----B-----')
        print B        
        A_ = np.transpose(A)
        AAA = np.matmul(inv(np.matmul(A_,A)),A_)
        # temp = np.matmul(AAA,B).reshape(2,3)
        # self.depth2rgb_affine = temp.copy()
        # co_eff = np.append(temp.copy(),[[0,0,1]],axis=0)

        ###############2019_02_07
        temp = np.matmul(AAA,B).reshape(nd, nd+1)
        # print('---temp---')
        # print temp

        tmp_append = np.append([0.]*nd, 1)

        # print('-----tmp_append.size-----')
        # print tmp_append.size
        # print tmp_append

        co_eff = np.append(temp.copy(), tmp_append.reshape(1, nd+1), axis=0)
        print('---co-eff---')
        print co_eff

        return co_eff


    def registerDepthFrame(self, frame):
        # print self.depth2rgb_affine
        """
        TODO:
        Using an Affine transformation, transform the depth frame to match the RGB frame
        """
        
        # M = np.array([[0.9963,0.0865,0],
        #     [-0.0865,0.9963,0]])

        temp = np.zeros(frame.shape,dtype=int)
        # before = frame/10
        # cv2.imwrite('before.png',before.astype(np.uint8))
        rows,cols = temp.shape
        temp = cv2.warpAffine(frame, self.depth2rgb_affine, (cols,rows),1)
        '''
        for a in range(frame.shape[0]):
            for b in range(frame.shape[1]):
                cor = np.round(np.matmul(self.depth2rgb_affine,np.transpose(np.array([a,b,1]))))
                #print(self.depth2rgb_affine)
                # cor = np.round(np.matmul(M,np.transpose(np.array([a,b,1]))))
                cor[0]=int(cor[0])
                cor[1]=int(cor[1])
                # print cor
                if int(cor[0])< frame.shape[0] and int(cor[1])< frame.shape[1] and int(cor[0]) >= 0 and int(cor[1])>=0:
                    temp[int(cor[0])][int(cor[1])] = frame[a][b]
        '''
        return temp

    def loadCameraCalibration(self):
        """
        TODO:
        Load camera intrinsic matrix from file.
        """
        f = open('./util/calibration.cfg', 'r')
        str1 = ''.join(f.read().splitlines()[1:4])
        str2 = str1.replace("[", " ")
        str3 = str2.replace("]"," ")
        intrinsic_matrix = np.fromstring(str3,dtype=float, sep=' ')
        intrinsic_matrix = intrinsic_matrix.reshape([3,3])
        f.close() 
        return intrinsic_matrix       
        
    
    def blockDetector(self):
        """
        TODO:
        Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """
        #-------calll detectBlocksInDepthImage, return x,y, d, color, orientation
        keypoints, keypoints_depth, keypoints_color, keypoints_orientation = self.detectBlocksInDepthImage()
        # keypoints, keypoints_depth = self.detectBlocksInDepthImage()
        # print('------------DONE--------------')
        #-------convert (x,y,d) to world frame coordinate
        n_blob = len(keypoints_depth) # number of blobs
        #-------initialization
        keypoints_world = [[0 for x in range(3)] for y in range(n_blob)] 

        for i in xrange(0, n_blob, 1):
            # first convert d to Zc
            keypoints_depth[i] = np.clip(keypoints_depth[i],0,2**10 - 1)
            Zc = 0.1236 * math.tan(keypoints_depth[i]/2842.5+1.1863) 
            # convert image to camera
            # print('--------keypoint--------')
            # print keypoints[i].pt
            XYZ_camera = Zc*np.matmul(np.linalg.inv(self.intrinsic_matrix), keypoints[i]) 
            # convert camera to world
            XYZ_camera_world = np.matmul(self.co_eff_camera_2_world, np.append(XYZ_camera, 1))
            keypoints_world[i][0] = XYZ_camera_world[0]
            keypoints_world[i][1] = XYZ_camera_world[1]
            keypoints_world[i][2] = XYZ_camera_world[2]
        return keypoints_world, keypoints_color, keypoints_orientation


    def detectBlocksInDepthImage(self):
        # #---------- hard coded color HSV range-------
        # green_lower = np.array([30, 15, 100])
        # green_upper = np.array([100, 100, 200])
        # green = [green_lower, green_upper]

        # black_lower = np.array([100, 0, 40])
        # black_upper = np.array([180, 80, 80])
        # black = [black_lower, black_upper]

        # red_lower = np.array([150, 160, 100])
        # red_upper = np.array([200, 255, 150])
        # red = [red_lower, red_upper]

        # orange_lower = np.array([0, 150, 180])
        # orange_upper = np.array([20, 255, 230])
        # orange = [orange_lower, orange_upper]

        # yellow_lower = np.array([21, 200, 231])
        # yellow_upper = np.array([40, 255, 255])
        # yellow = [yellow_lower, yellow_upper]

        # blue_lower = np.array([100, 90, 120])
        # blue_upper = np.array([130, 180, 200])
        # blue = [blue_lower, blue_upper]

        # purple_lower = np.array([121, 30, 130])
        # purple_upper = np.array([186, 88, 180])
        # purple = [purple_lower, purple_upper]

        # pink_lower = np.array([150, 150, 180]) # how to differentiate pinnk and orange
        # pint_upper = np.array([200, 209, 203])
        # pink = [pink_lower, pint_upper]



        # color_all = [green, black, red, orange, yellow, blue, purple, pink]
        # color_strings = [ "green", "black", "red", "orange", "yellow", "blue", "purple", "pink" ]        
        """
        TODO:
        Implement a blob detector to find blocks
        in the depth image
        """
        rgb_frame = self.currentVideoFrame
        print('-----size of rgb_frame')
        print rgb_frame.shape
        hsv_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2HSV)
        self.captureDepthFrame()
        self.convertDepthFrame()
        depth_frame = self.DepthCM
        # depth_frame_crop = depth_frame[93:435, 135:475]
        params = cv2.SimpleBlobDetector_Params()
         
        # Change thresholds
        params.minThreshold = 10;
        # params.minThreshold = 50;
        params.maxThreshold = 300;
         
        # Filter by Area.
        params.filterByArea = True
        # params.minArea = 1000
        # params.maxArea = 40000
        params.minArea = 250
        params.maxArea = 1500
         
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1
        # params.maxCircularity = 0.85
         
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.01
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        detector = cv2.SimpleBlobDetector_create(params)
        # print('----detector good-----')
        keypoints = detector.detect(depth_frame) 
        # print('----keypoints good-----') 
        # cv2.imshow('depth_frame',depth_frame)
        # cv2.waitKey(0) 
        im_with_keypoints = cv2.drawKeypoints(depth_frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        imgplot = plt.imshow(im_with_keypoints)
        plt.show()
        # cv2.imshow("Keypoints", im_with_keypoints)
        # cv2.waitKey(0)      
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
        orange_h_l = np.array(0)
        orange_h_u = np.array(10)
        orange = [orange_h_l, orange_h_u]

        yellow_h_l = np.array(20)
        yellow_h_u = np.array(35)
        yellow = [yellow_h_l, yellow_h_u]

        green_h_l = np.array(50)
        green_h_u = np.array(80)
        green = [green_h_l, green_h_u]

        blue_h_l = np.array(110)
        blue_h_u = np.array(130)
        blue = [blue_h_l, blue_h_u]
        color_all_h = [orange, yellow, green, blue]
        color_strings_h = [ "orange", "yellow", "green", "blue"]

        pink_r_l = np.array(190)
        pink_r_u = np.array(255)
        pink = [pink_r_l, pink_r_u]

        # red_r_l = np.array(110)
        red_r_l = np.array(146)
        red_r_u = np.array(189)
        red = [red_r_l, red_r_u]

        # purple_r_l = np.array(80)
        purple_r_l = np.array(110)
        purple_r_u = np.array(145)
        # purple_r_u = np.array(109)
        purple = [purple_r_l, purple_r_u]

        black_r_l = np.array(0)
        black_r_u = np.array(109)
        # black_r_u = np.array(79)
        black = [black_r_l, black_r_u]
        color_all_r = [pink, red, purple, black]
        color_strings_r = [ "pink", "red", "purple", "black"]


        depth_frame_1layer = self.currentDepthFrame # 1 layer depth frame for depth value

        points = []
        keypoints_depth = []
        keypoints_color = []
        keypoints_orientation = []

        y_lower_bound = 90
        y_upper_bound = 470       
        x_lower_bound = 90
        x_upper_bound = 470
        for keyPoint in keypoints:
            # ----- find the rgb value
            print('------keyPoint.pt-----')
            print keyPoint.pt
            # print np.matmul(self.depth2rgb_affine, np.append([keyPoint.pt],[1]))
            if np.round(keyPoint.pt[0])>=x_lower_bound and np.round(keyPoint.pt[0])<=x_upper_bound and np.round(keyPoint.pt[1])>=y_lower_bound and np.round(keyPoint.pt[1])<=y_upper_bound \
            and depth_frame_1layer[int(np.round(keyPoint.pt[1]))][int(np.round(keyPoint.pt[0]))]<=720: #add another depth value info to make sure
                keyPoint_hsv = hsv_frame[int(np.round(keyPoint.pt[1]))][int(np.round(keyPoint.pt[0]))][0:] # x 
                keyPoint_rgb = rgb_frame[int(np.round(keyPoint.pt[1]))][int(np.round(keyPoint.pt[0]))][0:] # x 

                keyPoint_d = depth_frame_1layer[int(np.round(keyPoint.pt[1]))][int(np.round(keyPoint.pt[0]))]
                keypoints_depth.append(keyPoint_d)

                print('--keyPoint_LOCATION-----')
                print keyPoint.pt           
                print('--keyPoint_hsv-----')
                print keyPoint_hsv  
                print('--keyPoint_rgb-----')
                print keyPoint_rgb                 
                # ----- decide the color of each blob
                if keyPoint_hsv[0] <= 130:# can determine the color from h value
                    for ind_clr_h, colr_range_h in enumerate(color_all_h):
                        if (keyPoint_hsv[0] >= colr_range_h[0]) and (keyPoint_hsv[0] <= colr_range_h[1]):
                            print('-----color name------')
                            print color_strings_h[ind_clr_h]
                            #---------save the color name--------
                            keypoints_color.append(color_strings_h[ind_clr_h])                        
                            break

                else: # decide color from r value
                    for ind_clr_r, colr_range_r in enumerate(color_all_r):
                        if (keyPoint_rgb[0] >= colr_range_r[0]) and (keyPoint_rgb[0] <= colr_range_r[1]):
                            print('-----color name------')
                            print color_strings_r[ind_clr_r]
                            #---------save the color name--------
                            keypoints_color.append(color_strings_r[ind_clr_r])                          
                            break


                #------now figure out the orientation
                keyPoint_hsv_lower = np.zeros((3,), dtype=int)
                keyPoint_hsv_upper = np.zeros((3,), dtype=int)
                thereshold = 25
                for i in range(3): # Hue range is [0,179]
                    if keyPoint_hsv[i] <= thereshold:
                        keyPoint_hsv_lower[i] = 0
                    else:
                        keyPoint_hsv_lower[i] = keyPoint_hsv[i] - thereshold

                    # tmp = 255 - thereshold
                    if i == 0:
                        if keyPoint_hsv[i] >= 179 - thereshold:
                            keyPoint_hsv_upper[i] = 179
                        else:
                            keyPoint_hsv_upper[i] = keyPoint_hsv[i] + thereshold
                    else:
                        if keyPoint_hsv[i] >= 255 - thereshold:
                            keyPoint_hsv_upper[i] = 255
                        else:
                            keyPoint_hsv_upper[i] = keyPoint_hsv[i] + thereshold                    


                # keyPoint_hsv_lower = keyPoint_hsv - 20
                # print keyPoint_hsv_lower
                # print np.clip(keyPoint_hsv_lower, 0, 0)
                # keyPoint_hsv_upper = keyPoint_hsv + 20
                # print keyPoint_hsv_upper
                # print np.clip(keyPoint_hsv_upper, 0, 255)      
                # Threshold the HSV image to get only current colors
                # problem: backgraound is black, first crop image to only board
                hsv_frame_crop = hsv_frame[int(np.round(keyPoint.pt[1]))-25:int(np.round(keyPoint.pt[1]))+25, int(np.round(keyPoint.pt[0]))-25:int(np.round(keyPoint.pt[0]))+25]
                # imgplot = plt.imshow(rgb_frame_crop)
                # plt.show()
                # imgplot = plt.imshow(hsv_frame_crop)
                # plt.show()     

                mask = cv2.inRange(hsv_frame_crop, keyPoint_hsv_lower, keyPoint_hsv_upper)
                # imgplot = plt.imshow(mask)
                # plt.show()

                kernel = np.ones((4,4),np.uint8)
                closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                closingHSV = cv2.bitwise_and(hsv_frame_crop, hsv_frame_crop, mask = closing)

                # imgplot = plt.imshow(closing)
                # plt.show()     
                # print('----closing_hsv-----')
                # print closingHSV              
                # -----how to convert to grayscale

                closing_bgr = cv2.cvtColor(closingHSV, cv2.COLOR_HSV2BGR)
                # imgplot = plt.imshow(closing_bgr)
                # plt.show()              
                closing_gray = cv2.cvtColor(closing_bgr, cv2.COLOR_BGR2GRAY)

                # cv2.imwrite("/home/student/Desktop/Untitled/gray.png",closing_gray)
                # imgplot = plt.imshow(closing_gray)
                # plt.show()             
                ret,thresh = cv2.threshold(closing_gray,50,255,cv2.THRESH_BINARY)
                # print('----closing_bgr-----')
                # print closing_bgr      
                # print('----closing_gray-----')
                # print closing_gray.shape 
                # print('----thresh-----')
                # print thresh
                # plt.imshow(thresh, cmap="Greys")
                # plt.show()

                im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
                len_contours = len(contours)
                list_contours = np.zeros((len_contours,1), dtype = int)
                for i in range(len(contours)):   
                    list_contours[i] = len(contours[i])
                ind_maxlen = np.argmax(list_contours)
                rect = cv2.minAreaRect(contours[ind_maxlen])
                print('---orientation----')
                print rect[2]

                #-------save the orientation
                keypoints_orientation.append(rect[2])
                #     cnt = contours[i]
                #     print('--contours[i]----')
                #     print cnt
                #     rect = cv2.minAreaRect(cnt)
                #     print rect[2]

                # img, contours, hierarchy = cv2.findContours(opening,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                # for i in range(len(contours)):
                #     cnt = contours[i]
                #     rect = cv2.minAreaRect(cnt)
                #     # self.phi_img[self.centroids_cnt] = rect[2]
                #     print('---angle')
                #     print rect[2]
                points.append(np.array([keyPoint.pt[0], keyPoint.pt[1], 1]))

        return points, keypoints_depth, keypoints_color, keypoints_orientation
        # return keypoints, keypoints_depth
            # for ind_clr, colr_range in enumerate(color_all):
            #     # print('---ind_clr----')
            #     # print ind_clr
            #     # print('---colr_range----')
            #     # print colr_range
            #     if (keyPoint_hsv >= colr_range[0]).all() and (keyPoint_hsv <= colr_range[1]).all():
            #         print('-----color name------')
            #         print color_strings[ind_clr]
            #         break

            # print keyPoint.angle # y
            




        # params = cv2.SimpleBlobDetector_Params()
        # params.minThreshold = 704
        # params.maxThreshold = 706
        # detector = cv2.SimpleBlobDetector_create(params)
        # keypoints = detector.detect(depth_frame)
        # print keypoints

        #----block depthFrame: [704, 706] ------
        # blob_lower = 704
        # blob_upper = 706
        # nb = ((depth_frame>=blob_lower) & (depth_frame<=blob_upper)).sum()
        # loc = np.where(np.logical_and(depth_frame>=blob_lower, depth_frame<=blob_upper))
        # # # print loc[0] # x 
        # # # print loc[1] # y
        # # # print self.currentVideoFrame[loc[0]][0][0:]
        # for i in range(0, nb): # should group using rgb value
        #     print(loc[0][i], loc[1][i])
        #     # print  depth_frame[loc[0][i]][loc[1][i]]
        #     # print self.currentVideoFrame[loc[0][i]][loc[1][i]][0:]
        # # print depth_frame[loc[0][0]][loc[1][0]]
        # # print depth_frame[loc[0][5]][loc[1][5]]

        # np.clip(depth_frame[...,0],0,2**10 - 1,depth_frame[...,0])
        # depth_frame[...,0] >>= 2
        # depth_frame[...,0] = depth_frame[...,0].astype(np.uint8)
        # depth_frame[...,0] = np.round(depth_frame[...,0]/2047 *255)
        # print('----kinect good-----')

        