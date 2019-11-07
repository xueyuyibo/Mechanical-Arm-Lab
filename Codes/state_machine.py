import time
import numpy as np
import cv2
from copy import deepcopy
import freenect
from kinematics import *
import matplotlib.pyplot as plt
from trajectory_planner import TrajectoryPlanner
from copy import deepcopy
import math
from rexarm import *
import math
from random import randint
import config_tasks
"""
TODO: Add stat
es and state functions to this class
        to implement all of the required logic for the armlab
"""
class StateMachine():
    def __init__(self, rexarm, planner, kinect):
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.config = [[0]*self.rexarm.num_joints] #change when gripper is added
        # self.coeff_rgb_2_world = np.zeros((3,3),float)
        self.tp = TrajectoryPlanner(rexarm)
        self.world_mouse = [0,0,0]
        # self.offset = 85
        # self.remainder = 55
        self.offset = 110
        self.blob_height = 40
        self.remainder = 70
        self.config_event1 = []
        self.config_event3 = []
        self.config_event4 = []
        self.config_event5 = []

    def set_next_state(self, state):
        self.next_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        
        if(self.current_state == "execute"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "manual"):
                self.manual()
        
        if(self.current_state == "teach"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "manual"):
                self.manual()

        if(self.current_state == "repeat"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "manual"):
                self.manual()

        if(self.current_state == "manual"):
            if (self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()                
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "idle"):
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "rls"):
                self.rls()
            if(self.next_state == "teach"):
                self.teach()
            if(self.next_state == "repeat"):
                self.repeat()
            if(self.next_state == "execute"):
                self.execute()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
            if(self.next_state == "grab"):
                self.grab()
            if(self.next_state == "drop_off"):
                self.drop_off()
            if(self.next_state == "move"):
                self.move()    
            if(self.next_state == "event_1"):
                self.event_1()
            if(self.next_state == "event_2"):
                self.event_2()
            if(self.next_state == "event_3"):
                self.event_3()
            if(self.next_state == "event_4"):
                self.event_4()
            if(self.next_state == "event_5"):
                self.event_5()

        if(self.current_state == "estop"):
            self.next_state = "estop"
            self.estop()  

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):               
                self.idle()
               
        if(self.current_state == "grab"):
            if(self.next_state == "idle"):               
                self.idle()   

        if(self.current_state == "drop_off"):
            if(self.next_state == "idle"):               
                self.idle()  

        if(self.current_state == "move"):
            if(self.next_state == "idle"):               
                self.idle()   

        if(self.current_state == "event_1"):
            if(self.next_state == "idle"):               
                self.idle()   

        if(self.current_state == "event_2"):
            if(self.next_state == "idle"):               
                self.idle()   

        if(self.current_state == "event_3"):
            if(self.next_state == "idle"):               
                self.idle()  

        if(self.current_state == "event_4"):
            if(self.next_state == "idle"):               
                self.idle()   

        if(self.current_state == "event_5"):
            if(self.next_state == "idle"):               
                self.idle()                                     
    """Functions run for each state"""


    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"
        self.rexarm.send_commands()
        self.rexarm.get_feedback()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        self.rexarm.get_feedback()

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()

    def execute(self):
        self.status_message = "EXECUTE PLAN - Arm moves according to preset configuration"
        self.current_state = "execute"
        
        # config_space = [[ 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.5],
        #                 [ 1.0, 0.8, 1.0, 0.5, 1.0, -2.0, -1.0],
        #                 [-1.0,-0.8,-1.0,-0.5, -1.0, 3.0, 0.5],
        #                 [-1.0, 0.8, 1.0, 0.5, 1.0, 1.0, -1.0],
        #                 [1.0, -0.8,-1.0,-0.5, -1.0, -1.0, 0.5],
        #                 [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]]
        # config_space = [[ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0],
        #                 [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
        #                 [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0],
        #                 [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
        #                 [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0],
        #                 [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]]
        
        #IK test
        self.rexarm.pause(4)
        tmp = self.world_mouse
        # pose = [tmp[0],tmp[1],-20,0,0,0] #input desired position and orientation
        # pose = world2ik(tmp[0],tmp[1],-20,0,0,0)
        # print('pose = ',pose)
        oc = [-10,10,110]
        # config_space1 = IK(pose,oc)
        # config_space1 = np.append(config_space1,0)
        # print(config_space1)
        # #self.rexarm.set_positions(config_space1)
        # pose = [30,30,160,0,0,0]
        # pose1 = world2ik(150,100,-20,0,0,0)        
        # config_space1 = IK(pose1,oc)
        # config_space1 = np.append(config_space1,0)
        # config_space2 = IK(pose,oc)
        # config_space2 = np.append(config_space2,0)
        # config_space3 = deepcopy(config_space2)
        # config_space3[-1] = 60
        # pose = world2ik(0,0,0,0,0,0)
        # config_space4 = IK(pose,oc)
        # config_space4 = np.append(config_space4,np.deg2rad(80.0))
        # pose2 = world2ik (150,100,-40,0,0,0)
        # config_space5 = IK(pose2,oc)
        # config_space5 = np.append(config_space5,0)
        #print('config_space = ',config_space3)
        offset = 85
        remainder = 55
        x_factor = 15/150
        y_factor = 15/150
        pose = world2ik(200 *1.1,200 * 1.1,0-self.offset ,-np.pi/2, np.pi/2, -np.pi/2)
        # pose = world2ik(-150 - 20,100 + 10,40-self.offset,-np.pi/3,np.pi,0)
        config_space = IK(pose)
        cur_config = self.rexarm.get_positions();
        self.tp.execute_plan([cur_config,config_space])
        self.rexarm.pause(1)
        # pose = world2ik(-100 *1.1,100 * 1.1,0-self.offset ,-np.pi/2,np.pi/2,np.pi/2)
        # config_space = IK(pose,1)

        # cur_config = self.rexarm.get_positions();
        # self.tp.execute_plan([cur_config,config_space])
        # self.rexarm.pause(1)
        #print(self.rexarm.gripper_state)
        '''
        pose = world2ik(tmp[0],tmp[1],tmp[2]-offset+remainder,0,np.pi,-np.pi/2)
        config_space6 = IK(pose,oc)
        pose = world2ik(tmp[0],tmp[1],tmp[2]-offset,0,np.pi,-np.pi/2)
        config_space7 = IK(pose,oc)
        for i in [config_space6,config_space7]:#,config_space2,config_space4]:
            cur_config = self.rexarm.get_positions();
            self.tp.execute_plan([cur_config,i]);
            #self.rexarm.set_positions(i)
            self.rexarm.pause(2)
        self.rexarm.toggle_gripper()
        self.rexarm.pause(1)
        self.tp.execute_plan([self.rexarm.get_positions(),[0.0]*6])
        self.rexarm.pause(4)
        tmp = self.world_mouse
        pose = world2ik(tmp[0],tmp[1],tmp[2]-offset+remainder,0,-np.pi/2,-np.pi/2)
        config_space8 = IK(pose,oc)
        pose = world2ik(tmp[0],tmp[1],tmp[2]-offset,0,-np.pi/2,-np.pi/2)
        config_space9 = IK(pose,oc)
        for i in [config_space8,config_space9]:#,config_space2,config_space4]:
            cur_config = self.rexarm.get_positions();
            self.tp.execute_plan([cur_config,i]);
            #self.rexarm.set_positions(i)
            self.rexarm.pause(2)
        self.rexarm.toggle_gripper()
        cur_config = self.rexarm.get_positions();
        self.rexarm.pause(1)
        self.tp.execute_plan([cur_config,config_space8]);
        self.tp.execute_plan([self.rexarm.get_positions(),[0.0]*6])
        '''
        # self.rexarm.close_gripper()
        # self.rexarm.pause(1)
        # cur_config = self.rexarm.get_positions();
        # self.tp.execute_plan([cur_config,config_space4]);
        # #self.rexarm.set_positions(i)
        # self.rexarm.pause(2)
        # print(self.rexarm.get_positions())
        # self.rexarm.pause(2)
        # print(self.world_mouse)
        '''
        init = [0.0]*self.rexarm.num_joints;
        final = deepcopy(init) #[1.0]*self.rexarm.num_joints;
        final[1] = 2
        plan = [init,final]
        self.tp.execute_plan(plan)
        time.sleep(5)
        #self.rexarm.speeds = [1.0]*self.rexarm.num_joints
        init = deepcopy(final)
        final = [0.0]*self.rexarm.num_joints
        plan = [init,final]
        self.tp.execute_plan(plan)
        '''
        self.set_next_state("idle")
        
    def rls(self):
        self.status_message = "disable torque"
        self.rexarm.disable_torque()
        self.config = [[0]*self.rexarm.num_joints]
        self.set_next_state("idle")

    def teach(self):
        self.status_message = "TEACH - Please move the arm"
        #self.rexarm.disable_torque()
        #self.rexarm.pause(2)
        cur_config = self.rexarm.get_positions()
        #print(cur_config)
        self.config.append(deepcopy(cur_config))
        #print(self.config)
        #self.rexarm.enable_torque()
        config = np.genfromtxt("A.csv", delimiter=",")
        print(config)
        m,n = config.shape
        config_angles = []
        for i in range(m):
            row = [0.0,0.0,0.0,0.0,0.0,0.0]
            for j in range(n):
                row[j] = config[i][j]
            config_angles.append(row)
        config_angles.append(deepcopy(cur_config))
        np.savetxt("A.csv", config_angles, delimiter=",")

        print("configuration acquired\n")
        self.set_next_state("idle")

    def repeat(self):
        self.rexarm.enable_torque()
        self.status_message = "REPEAT - Arm moves according to what is taught"
        self.current_state = "repeat"
        self.rexarm.set_positions([0.0]*self.rexarm.num_joints)
        #print(self.config)
        # for j in self.config:
        #     self.rexarm.set_positions(j)
        #     #print(j)
        #     self.rexarm.pause(2)
        print(self.config)
        self.config_event5 = deepcopy(self.config) #change event when necessary
        print(self.config_event1)
        self.set_next_state("idle")

    def calibrate(self):
        self.current_state = "calibrate"
        self.set_next_state("idle")
        #self.tp.go(max_speed=2.0)
        # location_strings = ["lower left corner of board",
        #                     "upper left corner of board",
        #                     "upper right corner of board",
        #                     "lower right corner of board",
        #                     "center of 1 layer blob, right",
        #                     "center of 2 layer blob, right",
        #                     "center of 3 layer blob, right, upper board",
        #                     "center of 3 layer blob, right, lower board",
        #                     "center of 1 layer blob, left",
        #                     "center of 2 layer blob, left",
        #                     "center of 3 layer blob, left, upeer board",
        #                     "center of 3 layer blob, left, lower board"]
        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "center of 1 layer blob, right",
                            "center of 2 layer blob, right",
                            "center of 1 layer blob, left",
                            "center of 2 layer blob, left"]                            
                            # "center of shoulder motor",
        i = 0
        for j in range(len(location_strings)):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False        
        
        i = 0
        for j in range(len(location_strings)):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False
   
        print self.kinect.rgb_click_points.shape
        print self.kinect.depth_click_points.shape
        # print self.kinect.currentDepthFrame
        # print freenect.sync_get_depth()

        """TODO Perform camera calibration here"""
        # print ('----self.kinect.currentDepthFrame----')
        # print self.kinect.currentDepthFrame()
        # co_eff = self.kinect.getAffineTransform(self.kinect.rgb_click_points, self.kinect.depth_click_points)
        co_eff = self.kinect.getAffineTransform(self.kinect.depth_click_points, self.kinect.rgb_click_points)
        # print co_eff

        # ------rgb_2_world, need better measurement--------------------
        # hlf_len = 287.5
        # left_lower_world = np.array([-hlf_len, hlf_len]) 
        # left_upper_world = np.array([-hlf_len, -hlf_len])
        # right_upper_world = np.array([hlf_len, -hlf_len])
        # right_lower_world = np.array([hlf_len, hlf_len])
        # world_points = np.append([left_lower_world],[left_upper_world,right_upper_world, right_lower_world], axis=0)
        # print world_points
        # self.coeff_rgb_2_world = self.kinect.getAffineTransform(self.kinect.rgb_click_points, world_points)
        # print ('rgb_2_world',self.coeff_rgb_2_world)

        self.status_message = "Calibration - Completed Calibration"
        self.kinect.depth2rgb_affine = co_eff[0:2][0:]
        self.kinect.kinectCalibrated = True
        print('---print self.kinect.depth2rgb_affine-----')
        print self.kinect.depth2rgb_affine
        """TODO Perform camera calibration here"""
        self.kinect.intrinsic_matrix = self.kinect.loadCameraCalibration()
        # print('----intrinsic_matrix---')
        # print self.kinect.intrinsic_matrix
        # convert d to Zc, for d use only 10 bits
        d = np.zeros((len(location_strings),), dtype = int)
        
        time.sleep(2)
        self.kinect.currentDepthFrame = deepcopy(self.kinect.registerDepthFrame(freenect.sync_get_depth()[0]))
        # rows,cols = self.kinect.currentDepthFrame.shape
        # self.kinect.currentDepthFrame = cv2.warpAffine(self.kinect.currentDepthFrame, self.kinect.depth2rgb_affine, (cols,rows),1)
        # self.kinect.captureDepthFrame()
        for i in range(len(location_strings)):
            yi = self.kinect.rgb_click_points[i][1]
            xi = self.kinect.rgb_click_points[i][0]
            d[i] = self.kinect.currentDepthFrame[yi][xi] # the depth corresponding to points in rgb frame

        print('---before clip----')
        print d
        np.clip(d,0,2**10 - 1,d)
        print('---after clip')
        print d

        Zc = np.zeros((len(location_strings),), dtype = float)
        for i in range(len(location_strings)):
            Zc[i] = 0.1236 * math.tan(d[i]/2842.5+1.1863) 
        # print('---Zc-----')
        # print Zc
        # find the corresponding coordinate in camera frame
        camera_points = np.zeros((len(location_strings), 3))
        for i in range(len(location_strings)):
            XYZ = Zc[i]*np.matmul(np.linalg.inv(self.kinect.intrinsic_matrix), np.append(self.kinect.rgb_click_points[i],1)) 
            camera_points[i][0] = XYZ[0]
            camera_points[i][1] = XYZ[1]
            camera_points[i][2] = XYZ[2]
        # print('----camera_points----')
        # print camera_points
        # world frame coordinate
        hlf_len = 50
        blob_height = 38
        left_lower_world = np.array([-6*hlf_len, 6*hlf_len, 0]) 
        left_upper_world = np.array([-6*hlf_len, -6*hlf_len, 0])
        right_upper_world = np.array([6*hlf_len, -6*hlf_len, 0])
        right_lower_world = np.array([6*hlf_len, 6*hlf_len, 0])
        # center_shoulder_world = np.array([0, 0, 11*blob_height])
        one_layer_blob_world_right = np.array([3*hlf_len, -hlf_len , blob_height])
        two_layer_blob_world_right = np.array([3*hlf_len, 3*hlf_len, 2*blob_height])
        three_layer_blob_world_right_upper = np.array([4*hlf_len, -4*hlf_len , 3*blob_height])
        three_layer_blob_world_right_lower = np.array([4*hlf_len, 4*hlf_len , 3*blob_height])
        
        one_layer_blob_world_left = np.array([-3*hlf_len, -hlf_len, blob_height])
        two_layer_blob_world_left = np.array([-2*hlf_len, 3*hlf_len, 2*blob_height])
        three_layer_blob_world_left_upper = np.array([-4*hlf_len, -4*hlf_len, 3*blob_height])
        three_layer_blob_world_left_lower = np.array([-4*hlf_len, 4*hlf_len, 3*blob_height])
        # world_points = np.append([left_lower_world],[left_upper_world,right_upper_world, right_lower_world, \
        #     center_shoulder_world, one_layer_blob_world, two_layer_blob_world], axis=0)

        # world_points = np.append([left_lower_world],[left_upper_world,right_upper_world, right_lower_world, \
        #     one_layer_blob_world_right, two_layer_blob_world_right,\
        #     three_layer_blob_world_right_upper, three_layer_blob_world_right_lower,\
        #     one_layer_blob_world_left, two_layer_blob_world_left,\
        #     three_layer_blob_world_left_upper, three_layer_blob_world_left_lower], axis=0) 

        world_points = np.append([left_lower_world],[left_upper_world,right_upper_world, right_lower_world, \
            one_layer_blob_world_right, two_layer_blob_world_right,\
            one_layer_blob_world_left, two_layer_blob_world_left], axis=0)         
        print('----world_points----')
        print world_points      
        # get affine transformation between camera frame and world frame, inverse extrinsic matrix
        self.kinect.co_eff_camera_2_world = self.kinect.getAffineTransform(camera_points, world_points)
        print('---co_eff_camera_2_world---')
        print self.kinect.co_eff_camera_2_world

        # find transformation from rgb to world


        # find corresponding camera frame value
        # the corresponding world frame value
        # self.coeff_rgb_2_world = 


        # depth_frame = self.kinect.detectBlocksInDepthImage()
        # Setup SimpleBlobDetector parameters.
        # im_with_keypoints = self.kinect.detectBlocksInDepthImage()
        # cv2.imshow("Keypoints", im_with_keypoints)
        # cv2.waitKey(0) 


        # ######## error:  too many values to unpack
        # keypoints_world, keypoints_color, keypoints_orientation = self.kinect.blockDetector()
        # ############################

        # print('---keypoints_world----')
        # print keypoints_world 
        # print('---keypoints_color----')
        # print keypoints_color 
        # print('---keypoints_orientation----')
        # print keypoints_orientation 

        time.sleep(1)

    def event_1(self):
        self.rexarm.open_gripper()
        self.current_state = "event_1"
        self.status_message = "State: Event 1 - Pick n stack! "
        keypoints_world, keypoints_color, keypoints_orientation = self.kinect.blockDetector()
        # print('------keypoints_world-----')
        # print keypoints_world
        # print('-----keypoints_color----')
        # print keypoints_color
        # print('-----keypoints_orientation----')
        # print keypoints_orientation
        config_space_init_final = np.zeros((3,6))
        config_space = np.zeros((3,6))
        config_before = np.zeros((3,6))
        destination = np.zeros((3,6))
        # destination[0][0] = -100
        # destination[0][1] = -100
        # destination[1][0] = -100
        # destination[1][1] = -100      
        # destination[2][0] = -100
        # destination[2][1] = -100   
        print('-----destination-----')
        print destination   
        destination_before = np.zeros((3,6))
        destination_final = np.zeros((3,6))
        # ##---------------new----------------
        # config_obj_pre = np.zeros((3,6))
        # config_obj = np.zeros((3,6))

        factor = 1.08
        for i in range(3):
            #------------new-----------------
            # first move x, y position
            # pose_obj_pre = world2ik(keypoints_world[i][0], keypoints_world[i][1], keypoints_world[i][2] -self.offset -10 + 100, 0, np.pi, -np.pi-keypoints_orientation[i])
            # config_obj_pre[i] = IK(pose_obj_pre)
            # pose_obj = world2ik(keypoints_world[i][0], keypoints_world[i][1], keypoints_world[i][2] -self.offset -10, 0, np.pi, -np.pi-keypoints_orientation[i])
            # config_obj[i] = IK(pose_obj)            
            #-------------------------------------------
            #------convert angle
            # if keypoints_orientation[i]
            # if keypoints_world[i][0] > 0:
            #     keypoints_world[i][0] = keypoints_world[i][0] + 10
            # else:
            #     keypoints_world[i][0] = keypoints_world[i][0] - 10
            keypoints_world[i][0] = keypoints_world[i][0]*factor
            keypoints_world[i][1] = keypoints_world[i][1]*factor

            pose = world2ik(keypoints_world[i][0], keypoints_world[i][1], keypoints_world[i][2] - self.offset,0 , np.pi,keypoints_orientation[i]*3.141592/180.0 )
            config_space[i] = IK(pose,1)
            # pose = world2ik(keypoints_world[i][0],keypoints_world[i][1],keypoints_world[i][2]-self.offset + 40,0,np.pi,-np.pi-keypoints_orientation[i])
            # config_before[i] = IK(pose)
            pose = world2ik(keypoints_world[i][0],keypoints_world[i][1],keypoints_world[i][2] - self.offset + self.remainder, 0, np.pi, keypoints_orientation[i]*3.141592/180.0)
            config_space_init_final[i] = IK(pose,1)
            # pose = world2ik(-100,200,-60,0,np.pi,0)
            #if 1:
            pose = world2ik(-150,0, keypoints_world[i][2] - self.offset + self.blob_height*i, 0, np.pi, np.pi)
            destination[i] = IK(pose,1)
            pose = world2ik(-150,0, keypoints_world[i][2] - self.offset + self.blob_height*i + self.remainder, 0, np.pi, np.pi)
            destination_before[i] = IK(pose,1)
            pose = world2ik(-150,0, keypoints_world[i][2] - self.offset + self.blob_height*(i+1) + self.remainder,0,np.pi,np.pi)
            destination_final[i] = IK(pose,1)
            # else:
            #     pose = world2ik(-200,0, keypoints_world[i][2] - self.offset + self.blob_height*i, 0,np.pi/2,np.pi)
            #     destination[i] = IK(pose,1)
            #     pose = world2ik(-200,0, keypoints_world[i][2] - self.offset + self.blob_height*i + self.remainder, 0,np.pi/2,np.pi)
            #     destination_before[i] = IK(pose,1)
            #     # pose = world2ik(-100,100, keypoints_world[i][2] - self.offset + self.blob_height*(i+1) + self.remainder,0,np.pi/2,0)
            #     # destination_final[i] = IK(pose,1)
            #     destination_final[i] = deepcopy(destination[i])
            #     destination_final[i][-2] = 0
        for j in range(3):
            # self.tp.execute_plan([self.rexarm.get_positions(),config_obj_pre[j]])
            # self.rexarm.pause(3)
            # self.tp.execute_plan([self.rexarm.get_positions(),config_obj[j]])
            # self.rexarm.pause(3)
            # self.tp.execute_plan([self.rexarm.get_positions(),config_obj_pre[j]])
            ##################################################################
            self.tp.execute_plan([self.rexarm.get_positions(),config_space_init_final[j]]);
            #self.rexarm.set_positions(config_space_init_final[j])
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),config_space[j]]);
            #self.rexarm.set_positions(config_space[j])
            self.rexarm.pause(1)
            self.rexarm.close_gripper()
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),config_space_init_final[j]])
            #self.rexarm.set_positions(config_space_init_final[j])
            self.rexarm.pause(1)
            # self.tp.execute_plan([self.rexarm.get_positions(),[0.0]*6])
            # #self.rexarm.set_positions([0]*6)
            # self.rexarm.pause(1)
            '''
            self.tp.execute_plan([self.rexarm.get_positions(),destination_before[j]])
            #self.rexarm.set_positions(destination_before[j])
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),destination[j]])
            #self.rexarm.set_positions(destination[j])
            self.rexarm.pause(1)
            self.rexarm.open_gripper()
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),destination_final[j]])
            #self.rexarm.set_positions(destination_final[j])
            '''
            self.tp.execute_plan([self.rexarm.get_positions(),config_tasks.task1_position[2*j+1]])
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),config_tasks.task1_position[2*j]])
            self.rexarm.pause(1)
            self.rexarm.open_gripper()
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),config_tasks.task1_position[2*j+1]])
            self.rexarm.pause(1)
        self.tp.execute_plan([self.rexarm.get_positions(),[0]*6])
        self.set_next_state("idle")

    def event_2(self):
        self.current_state = "event_2"
        self.status_message = "State: Event 2 - Wacky Pick n place!"
        D2R = 3.141592/180.0
        keypoints_world, keypoints_color, keypoints_orientation = self.kinect.blockDetector()
        print('---keypoints_orientation------')
        print keypoints_orientation
        print('---keypoints_world------')
        print keypoints_world   
        for i in range(len(keypoints_color)):
            if keypoints_world[i][0] >= 0:
                pose1 = world2ik(keypoints_world[i][0]*1.35, keypoints_world[i][1]*1.35, keypoints_world[i][2] - 20 -self.offset+80 + 10,-keypoints_orientation[i]*D2R , 0,np.pi+np.pi/6)
                pose = world2ik(keypoints_world[i][0]-20*(keypoints_world[i][0]/abs(keypoints_world[i][0])), keypoints_world[i][1]-20*(keypoints_world[i][1]/abs(keypoints_world[i][1])), keypoints_world[i][2]-self.offset+80+60,-keypoints_orientation[i]*D2R , 0,np.pi+np.pi/6)
            else:
                pose1 = world2ik(keypoints_world[i][0]*1.35, keypoints_world[i][1]*1.35, keypoints_world[i][2] - 20 -self.offset+80 + 10,-keypoints_orientation[i]*D2R , 0,np.pi-np.pi/6)
                pose = world2ik(keypoints_world[i][0]-20*(keypoints_world[i][0]/abs(keypoints_world[i][0])), keypoints_world[i][1]-20*(keypoints_world[i][1]/abs(keypoints_world[i][1])), keypoints_world[i][2]-self.offset+80+60,-keypoints_orientation[i]*D2R , 0,np.pi-np.pi/6)
            config_heigh_space = IK(pose)     
            if keypoints_orientation[i] < -80:
                config_heigh_space[-1] = config_heigh_space[-1] + np.pi/2

            config_space = IK(pose1)
            config_space[-1] += np.pi/2
            config_heigh_space[-1] += np.pi/2

            self.tp.execute_plan([self.rexarm.get_positions(),config_heigh_space])
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),config_space])
            self.rexarm.pause(1)
            self.rexarm.toggle_gripper()
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),config_heigh_space])
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),[0]*6])
            self.rexarm.pause(1)

        self.set_next_state("idle")
    
    def event_3(self):
        self.current_state = "event_3"
        self.status_message = "State: Event 3 - Line them up! "
        self.rexarm.open_gripper()
        used_color_name = []
        block_interval = 50
        D2R = 3.141592/180.0

        # self.rexarm.pause(4)
        # tmp1 = self.world_mouse
        # self.rexarm.pause(4)
        # tmp2 = self.world_mouse
        # length = math.hypot(tmp1[0]-tmp2[0], tmp1[1]-tmp2[1])
        # temp_x = (tmp2[0]-tmp1[0])*block_interval/length
        # temp_y = (tmp2[0]-tmp1[0])*block_interval/length
        # location_list = []
        # for i in range(8):
        #     location_list.append([tmp1[0]+temp_x*i,tmp1[1]+temp_y*i])

        while True:
            keypoints_world, keypoints_color, keypoints_orientation = self.kinect.blockDetector()
            print('------keypoints_world-----')
            print keypoints_world
            print('-----keypoints_color----')
            print keypoints_color
            print('-----keypoints_orientation----')
            print keypoints_orientation
            print('-----number of blocks detected----')
            num = len(keypoints_color)
            print num

            flag2 = 0
            
            color_list = ['black', 'red', 'orange', 'yellow', 'green', 'blue', 'purple', 'pink']
            color_name_list = []
            for color in keypoints_color:
                for i in range(8):
                    if color == color_list[i]:
                        color_name_list.append(i)

            for i in range(num):
                flag1 = 0
                for used in used_color_name:
                    if color_name_list[i]==used:
                        flag1 = 1
                if flag1 == 0:
                    pose = world2ik(keypoints_world[i][0]*1.08,keypoints_world[i][1]*1.08,keypoints_world[i][2]-self.offset-15,0,0,keypoints_orientation[i]*D2R) # NEED TO EDIT: horizontally grab
                    initial_position = IK(pose,1)
                    pose = world2ik(keypoints_world[i][0]*1.08,keypoints_world[i][1]*1.08,keypoints_world[i][2]-self.offset-15+self.remainder,0,0,keypoints_orientation[i]*D2R) # NEED TO EDIT: horizontally grab
                    initial_heigh_position = IK(pose,1)
                    final_heigh_position = config_tasks.task3_config_destination[color_name_list[i]*2+1]
                    final_position = config_tasks.task3_config_destination[color_name_list[i]*2]
                    # pose = world2ik(location_list[color_name_list[i]][0],location_list[color_name_list[i]][1],0,0,0,0) # NEED TO EDIT: horizontally grab
                    # final_position = IK(pose,1)
                    # pose = world2ik(location_list[color_name_list[i]][0],location_list[color_name_list[i]][1],self.remainder,0,0,0) # NEED TO EDIT: horizontally grab
                    # final_heigh_position = IK(pose,1)
                    self.tp.execute_plan([self.rexarm.get_positions(),initial_heigh_position])
                    self.rexarm.pause(.5)
                    self.tp.execute_plan([self.rexarm.get_positions(),initial_position])
                    self.rexarm.pause(.5)
                    self.rexarm.close_gripper()
                    self.rexarm.pause(.5)
                    self.tp.execute_plan([self.rexarm.get_positions(),initial_heigh_position])
                    self.rexarm.pause(.5)
                    self.tp.execute_plan([self.rexarm.get_positions(),[0.0]*6])
                    self.rexarm.pause(.5)
                    self.tp.execute_plan([self.rexarm.get_positions(),final_heigh_position])
                    self.rexarm.pause(.5)
                    self.tp.execute_plan([self.rexarm.get_positions(),final_position])
                    self.rexarm.pause(.5)
                    self.rexarm.open_gripper()
                    self.rexarm.pause(1)
                    self.tp.execute_plan([self.rexarm.get_positions(),final_heigh_position])
                    self.rexarm.pause(1)
                    self.tp.execute_plan([self.rexarm.get_positions(),[0.0]*6])
                    self.rexarm.pause(1)
                else:
                    flag2 += 1

            if flag2 == len(color_name_list):
                break

            for color in color_name_list:
                flag = 0
                for used in used_color_name:
                    if used==color:
                        flag = 1
                if flag==0:
                    used_color_name.append(color)

        self.set_next_state("idle")

    def event_4(self):
        self.current_state = "event_4"
        self.status_message = "State: Event 4 - Stack them high!"
        self.rexarm.open_gripper()
        D2R = 3.141592/180.0
        used_color_name = []
        block_interval = 50

        # NEED TO ADD DEFINE OF IK &IK2

        # self.rexarm.pause(4)
        # tmp = self.world_mouse
        # stack_location = [tmp[0],tmp[1]]
        # stack_location_list = []
        # for i in range(8):
        #     stack_location_list.append([tmp[0],tmp[1],i*block_interval])
        
        for k in [3,2]:
            keypoints_world, keypoints_color, keypoints_orientation = self.kinect.blockDetector()
            print('------keypoints_world-----')
            print keypoints_world
            print('-----keypoints_color----')
            print keypoints_color
            print('-----keypoints_orientation----')
            print keypoints_orientation
            print('-----number of blocks detected----')
            num = len(keypoints_color)
            print num

            # used_location = []
            # used_location = [stack_location]
            # for i in range(num):
            #     used_location.append([keypoints_world[i][0],keypoints_world[i][1]])

            reput_block_num = 0
            xy_candidate = []
            location_candidate = []
            candidate_num = 0
            for j in range(len(config_tasks.task4_xy_waitlist)):
                flag = 0
                for i in range(num):
                    if math.hypot(keypoints_world[i][0]-config_tasks.task4_xy_waitlist[j][0],keypoints_world[i][1]-config_tasks.task4_xy_waitlist[j][1])<55:
                        flag = 1
                if flag==0:
                    xy_candidate.append(config_tasks.task4_xy_waitlist[j])
                    location_candidate.append(config_tasks.task4_position_waitlist[2*j])
                    location_candidate.append(config_tasks.task4_position_waitlist[2*j+1])
                    candidate_num += 1

            for i in range(num):
                if keypoints_world[i][2]>=k*30: # NEED TO EDIT: this 30 can be changed
                    # print('-----used_location-----'+str(used_location))
                    # num_tried = 0
                    # while True:
                    #     temp_location = [randint(-200,200),randint(-200,200)]
                    #     if temp_location[0]<70 and temp_location[0]>-70 and temp_location[1]>0:
                    #         num_tried += 1
                    #         print('-----'+str(num_tried)+'random points tried-----')
                    #         pass
                    #     else:
                    #         if math.hypot(temp_location[0],temp_location[1])<=math.hypot(120,150) and math.hypot(temp_location[0],temp_location[1])>100:
                    #             flag = 0
                    #             for j in range(len(used_location)):
                    #                 if math.hypot(used_location[j][0]-temp_location[0],used_location[j][1]-temp_location[1])<70:
                    #                     falg = 1
                    #             if flag==0:
                    #                 used_location.append(temp_location)
                    #                 print('-----temp_location-----'+str(temp_location))
                    #                 break
                    #             else:
                    #                 num_tried += 1
                    #                 print('-----'+str(num_tried)+'random points tried-----')
                    #         else:
                    #             num_tried += 1
                    #             print('-----'+str(num_tried)+'random points tried-----')

                    pose = world2ik(keypoints_world[i][0],keypoints_world[i][1],keypoints_world[i][2]-self.offset,0,0,keypoints_orientation[i]*D2R) # NEED TO EDIT: horizontally grab
                    initial_position = IK(pose,1)
                    pose = world2ik(keypoints_world[i][0],keypoints_world[i][1],keypoints_world[i][2]-self.offset+self.remainder,0,0,keypoints_orientation[i]*D2R) # NEED TO EDIT: horizontally grab
                    initial_heigh_position = IK(pose,1)
                    print('-----size & index-----')
                    print(candidate_num)
                    print(reput_block_num)
                    final_position = location_candidate[2*reput_block_num]
                    final_heigh_position = location_candidate[2*reput_block_num+1]
                    # pose = world2ik(temp_location[0],temp_location[1],38-self.offset,0,0,0) # NEED TO EDIT: horizontally grab
                    # final_position = IK(pose,1)
                    # pose = world2ik(temp_location[0],temp_location[1],38-self.offset+self.remainder,0,0,0) # NEED TO EDIT: horizontally grab
                    # final_heigh_position = IK(pose,1)
                    self.tp.execute_plan([self.rexarm.get_positions(),initial_heigh_position])
                    self.rexarm.pause(1)
                    self.tp.execute_plan([self.rexarm.get_positions(),initial_position])
                    self.rexarm.pause(1)
                    self.rexarm.toggle_gripper()
                    self.rexarm.pause(1)
                    self.tp.execute_plan([self.rexarm.get_positions(),initial_heigh_position])
                    self.rexarm.pause(1)
                    self.tp.execute_plan([self.rexarm.get_positions(),[0.0]*6])
                    self.rexarm.pause(1)
                    self.tp.execute_plan([self.rexarm.get_positions(),final_heigh_position])
                    self.rexarm.pause(1)
                    self.tp.execute_plan([self.rexarm.get_positions(),final_position])
                    self.rexarm.pause(1)
                    self.rexarm.toggle_gripper()
                    self.rexarm.pause(1)
                    self.tp.execute_plan([self.rexarm.get_positions(),final_heigh_position])
                    self.rexarm.pause(1)
                    self.tp.execute_plan([self.rexarm.get_positions(),[0.0]*6])
                    self.rexarm.pause(1)
                    reput_block_num += 1

        keypoints_world, keypoints_color, keypoints_orientation = self.kinect.blockDetector()
        print('------keypoints_world-----')
        print keypoints_world
        print('-----keypoints_color----')
        print keypoints_color
        print('-----keypoints_orientation----')
        print keypoints_orientation
        print('-----number of blocks detected----')
        num = len(keypoints_color)
        print num

        color_list = ['black', 'red', 'orange', 'yellow', 'green', 'blue', 'purple', 'pink']
        color_name_list = []
        for color in color_list:
            for i in range(num):
                if color == keypoints_color[i]:
                    color_name_list.append(i)

        level = 0
        for name in color_name_list:
            pose = world2ik(keypoints_world[name][0],keypoints_world[name][1],keypoints_world[name][2]-self.offset,0,0,keypoints_orientation[name]*D2R) # NEED TO EDIT: horizontally grab
            initial_position = IK(pose,1)
            pose = world2ik(keypoints_world[name][0],keypoints_world[name][1],keypoints_world[name][2]-self.offset+self.remainder,0,0,keypoints_orientation[name]*D2R) # NEED TO EDIT: horizontally grab
            initial_heigh_position = IK(pose,1)
            final_position = config_tasks.task4_config_destination[level]
            final_heigh_position = config_tasks.task4_config_destination[level+1]
            # pose = world2ik(stack_location_list[level][0],stack_location_list[level][1],stack_location_list[level][2],0,0,0) # NEED TO EDIT: horizontally grab
            # final_position = IK(pose,1)
            # pose = world2ik(stack_location_list[level][0],stack_location_list[level][1],stack_location_list[level][2]+self.remainder,0,0,0) # NEED TO EDIT: horizontally grab
            # final_heigh_position = IK(pose,1)
            self.tp.execute_plan([self.rexarm.get_positions(),initial_heigh_position])
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),initial_position])
            self.rexarm.pause(1)
            self.rexarm.toggle_gripper()
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),initial_heigh_position])
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),[0.0]*6])
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),final_heigh_position])
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),final_position])
            self.rexarm.pause(1)
            self.rexarm.toggle_gripper()
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),final_heigh_position])
            self.rexarm.pause(1)
            self.tp.execute_plan([self.rexarm.get_positions(),[0.0]*6])
            self.rexarm.pause(1)
            
            level += 1

        self.set_next_state("idle")
    
    def event_5(self):
        self.current_state = "event_5"
        self.status_message = "State: Event 5 - Block Mover!"
        for i in range(len(config_tasks.task5_position)):
        # for i in [len(config_tasks.task5_position)-1:0]:
            self.tp.execute_plan([self.rexarm.get_positions(),config_tasks.task5_position[i]])
            self.rexarm.pause(1)
            if i==2:
                self.rexarm.toggle_gripper()
                self.rexarm.pause(1)
            if i==len(config_tasks.task5_position)-2:
                self.rexarm.toggle_gripper()
                self.rexarm.pause(1)
        self.tp.execute_plan([self.rexarm.get_positions(),[0.0]*6])
        self.rexarm.pause(1)
        self.set_next_state("idle")
        pass

    def grab(self):
        self.current_state = "grab"
        self.status_message = "Grab block"
        #----grab
        self.rexarm.close_gripper()
        self.set_next_state("idle")
        pass

    def drop_off(self):
        self.current_state = "drop_off"
        self.status_message = "Drop off block"
        self.rexarm.open_gripper()
        # drop off
        self.set_next_state("idle")         
        # pass

    def move(self):
        self.current_state = "move"
        self.status_message = "Move from current location to destination"
        self.set_next_state("idle")    
        pass
