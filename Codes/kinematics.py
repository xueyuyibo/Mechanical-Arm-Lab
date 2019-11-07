import numpy as np
#expm is a matrix exponential function
from scipy.linalg import expm
import math

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""

def FK_dh(angles,link_num=6):
    l1 = 45.29
    l2 = 99.63
    l3 = 71.81
    l4 = 43.39
    l5 = 48.01
    l6 = 50.58
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm using DH convention

    return a transformation matrix representing the pose of the 
    desired link

    note: phi is the euler angle about the y-axis in the base frame

    """
    link_length = [l1,0,0,l3+l4,0,l5+l6] #measure the link length
    transform_matrix = np.identity(4)
    theta = [angles[0]+np.pi/2,angles[1]+np.pi/2,angles[2]-np.pi/2,angles[3],angles[4],angles[5]]
    offsets = np.zeros(6)
    offsets[1] = l2;
    #link_num = 6 #find the sequence of the link
    link_twist = [np.pi/2,0,-np.pi/2,np.pi/2,-np.pi/2,0]
    for i in range(link_num):
        transform_matrix = np.dot(transform_matrix,generate_transform_matrix(theta[i],link_length[i],offsets[i],link_twist[i]))
    return transform_matrix



def generate_transform_matrix(theta,d,a,alpha): #angle in randians
    z_rot = np.array([[np.cos(theta),-np.sin(theta),0,0],
                        [np.sin(theta),np.cos(theta),0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
    z_translation = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,d],
                        [0,0,0,1]])
    x_translation = np.array([[1,0,0,a],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
    x_rotation = np.array([[1,0,0,0],
                        [0,np.cos(alpha),-np.sin(alpha),0],
                        [0,np.sin(alpha),np.cos(alpha),0],
                        [0,0,0,1]])
    transform_matrix = np.dot(np.dot(np.dot(z_rot,z_translation),x_translation),x_rotation)
    return transform_matrix

def z_rotation(theta):
    z = np.array([[np.cos(theta),-np.sin(theta),0,0],
                    [np.sin(theta),np.cos(theta),0,0],
                    [0,0,1,0],
                    [0,0,0,1]])
    return z

def y_rotation(theta):
    y = np.array([[np.cos(theta),0,np.sin(theta),0],
                    [0,1,0,0],
                    [-np.sin(theta),0,np.cos(theta),0],
                    [0,0,0,1]])
    return y
def x_rotation(alpha):
    x =  np.array([[1,0,0,0],
                    [0,np.cos(alpha),-np.sin(alpha),0],
                    [0,np.sin(alpha),np.cos(alpha),0],
                    [0,0,0,1]])
    return x
def FK_pox(joint_angles):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm
    using product of exponential formulation

    return a 4-tuple (x, y, z, phi) representing the pose of the 
    desired link

    note: phi is the euler angle about y in the base frame

    """
    pass

def IK(pose,event = 0):
    #suppose pose is given with x,y,z,psi(x),theta(y),phi(z)
    """
    TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    """
    l1 = 45.29
    l2 = 99.63
    l3 = 71.81
    l4 = 43.39
    l3 = l3 + l4
    l5 = 48.01
    l6 = 91.51
    # l6 = 50.58
    x = pose[0]
    y = pose[1]
    z = pose[2]
    psi = pose[3]
    theta = pose[4]
    phi = pose[5]
    d6 = l5 + l6 #link length
    # if math.hypot(x,y) > 200:
    #     phi = -np.pi/2
    #     theta = np.pi/2
    if (event == 0):

        print('event = ',event)
        # l1 = 44.92
        # l2 = 80 + 20
        # l3 = 68.64
        # l4 = 45.30
        # l3 = l3 + l4
        # l5 = 51.37
        # l6 = 91.51
     #    R06 = [[ 7.75288575e-02, -7.57662631e-01, -6.48025165e-01],
     # [ 5.38446995e-01,  5.78856487e-01, -6.12372436e-01],
     # [ 8.39085281e-01, -3.01450668e-01,  4.52839249e-01]]
        # R06 = np.array([[np.cos(psi)*np.cos(theta)*np.cos(phi)-np.sin(phi)*np.sin(psi),-np.cos(phi)*np.cos(theta)*np.sin(psi)-np.sin(psi)*np.cos(phi),np.cos(phi)*np.sin(theta)],
        #     [np.sin(phi)*np.cos(theta)*np.cos(psi)+np.cos(phi)*np.sin(psi),-np.sin(phi)*np.cos(theta)*np.sin(psi)+np.cos(phi)*np.cos(psi),np.sin(phi)*np.sin(theta)],
        #     [-np.sin(theta)*np.cos(phi),np.sin(theta)*np.sin(phi),np.cos(theta)]]) #Rzyz
        R = np.dot(z_rotation(psi),np.dot(y_rotation(theta),x_rotation(phi)))
        R06 = R[:3,:3]
        #print('R06 = ',R06)
        # x = -122
        # y = -60
        # z = 291
        o = np.array([x,y,z])
        oc = np.subtract(o, d6*np.dot(R06,[0,0,1]))
        #print('oc=',oc)
        #print('oc=',oc)
        xc = oc[0]
        yc = oc[1]
        zc = oc[2]
        theta1 = np.arctan2(yc,xc) - np.pi/2
        if theta1 < - np.pi:
            theta1 = theta1 + 2*np.pi
        if theta1 > np.pi:
            theta1 = theta1 - 2*np.pi
        l_til = np.sqrt((zc-l1)**2+xc**2+yc**2)
        #print('l_til = ',l_til)
        #rio = (l_til**2-l2**2-l3**2)/(2*l2*l3)
        #print('rio = ',rio)
        tmp = (l_til**2-l2**2-l3**2)/(2*l2*l3)
        if tmp > 1:
            tmp = 1
        if tmp < -1:
            tmp = -1
        theta3 = -np.arccos(tmp)
        #print('theta3 = ',theta3)
        theta2 = np.arctan2((zc-l1),np.sqrt(xc**2+yc**2)) - np.arctan2(l3*np.sin(theta3),l2 + l3*np.cos(theta3))
        theta2 = theta2 - np.pi/2
        angles = [theta1,theta2,theta3,0,0,0]
        T03 = FK_dh(angles,3)
        #print('T03=',T03)
        # R03 = np.array([[np.cos(theta1)*np.cos(theta2+theta3),-np.cos(theta1)*np.cos(theta2+theta3),np.sin(theta1)],
        #     [np.sin(theta1)*np.cos(theta2+theta3),-np.sin(theta1)*np.sin(theta2+theta3),-np.cos(theta1)],
        #     [np.sin(theta2+theta3),np.cos(theta2+theta3),0]])
        R03 = T03[:3,:3]
        R36 = np.dot(np.transpose(R03),R06)
        r11 = R36[0][0]
        r12 = R36[0][1]
        r13 = R36[0][2]
        r21 = R36[1][0]
        r22 = R36[1][1]
        r23 = R36[1][2]
        r31 = R36[2][0]
        r32 = R36[2][1]
        r33 = R36[2][2]
        #case one: at least one of r13,r23 is nonzero
        if r13!=0 or r23!=0:
            theta5 = np.arctan2(-np.sqrt(1-r33**2),r33)
            theta4 = np.arctan2(r23,r13)
            theta6 = np.arctan2(r32,-r31)
        else:
            theta5 = 0
            theta4 = 0
            theta6 = arctan2(r21,r11)
        #print([theta2,theta3])
        #print('theta',[theta1,theta2,theta3,theta4,theta5,theta6])
        #print([theta1,theta2,theta3,theta4,theta5,theta6])
        
    #hard coded one:
    if event == 1:
        # l1 = 44.92
        # l2 = 80 + 20
        # l3 = 68.64
        # l4 = 45.30
        # l3 = l3 + l4
        # l5 = 51.37
        # l6 = 91.51
        oc = [x,y,z+l5+l6+10]
        xc = oc[0]
        yc = oc[1]
        zc = oc[2]
        theta1 = np.arctan2(yc,xc) - np.pi/2
        if theta1 < - np.pi:
            theta1 = theta1 + 2*np.pi
        if theta1 > np.pi:
            theta1 = theta1 - 2*np.pi
        l_til = np.sqrt((zc-l1)**2+xc**2+yc**2)
        #print('l_til = ',l_til)
        tmp = (l_til**2-l2**2-l3**2)/(2*l2*l3)
        if tmp > 1:
            tmp = 1
        if tmp < -1:
            tmp = -1
        theta3 = -np.arccos(tmp)
        #print('theta3 = ',theta3)
        
        theta2 = np.arctan2((zc-l1),np.sqrt(xc**2+yc**2)) - np.arctan2(l3*np.sin(theta3),l2 + l3*np.cos(theta3))
        theta2 = theta2 - np.pi/2
        theta5 = - theta2 - theta3 - np.pi 
        # if theta1 < -np.pi/4 and theta1 > -3*np.pi/4:
        #     theta6 = theta1 + phi + np.pi
        # elif theta1 < -3*np.pi/4:
        #     theta6 = theta1 + phi  + np.pi
        # elif theta1 < np.pi/4 and theta1 > -np.pi/4:
        #     theta6 = theta1 + phi 
        # elif theta1 < 3*np.pi/4 and theta1 > np.pi/4:
        #     theta6 = theta1 + phi  - np.pi/2
        # else:
        #     theta6 = theta1 + phi - np.pi
        theta6 = theta1 + phi
        if x < 0 and y < 0:
            theta6 = theta6 - np.pi/2
        # elif y < 0 and np.absolute(x) < 20:
        #     return IK([x,y,z,phi+np.pi/2,np.pi/2,-np.pi/2])
        theta4 = 0
    result = [theta1,theta2,theta3,theta4,theta5,theta6]
    angle_limits = np.array([
                [-180, 179.99],
                [-124, 124],
                [-115, 97],
                [-np.inf, np.inf],
                [-102, 102],
                [-np.inf, np.inf]], dtype=np.float)*3.141592/180.0
    for i in range(6):
        if result[i] > angle_limits[i][1]:
            result[i] = angle_limits[i][1]
        if result[i] < angle_limits[i][0]:
            result[i] = angle_limits[i][0]   
    return result

def world2ik(x,y,z,row,pitch,yaw):
    #psi(z),theta(y),phi(z)
    xi = y
    yi = x
    zi = z
    psi = row
    theta = pitch
    phi = yaw
    pose = [xi,yi,zi,psi,theta,phi]
    return pose


def get_euler_angles_from_T(T):
    """
    TODO: implement this function
    return the Euler angles from a T matrix
    
    """
    pass

def get_pose_from_T(T):
    """
    TODO: implement this function
    return the joint pose from a T matrix
    of the form (x,y,z,phi) where phi is rotation about base frame y-axis
    
    """
    pass




def to_s_matrix(w,v):
    """
    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)
    """
    pass


# if __name__ == '__main__':
# 	t = FK_dh([-np.pi/2,np.pi/6,-np.pi/7,np.pi/4,np.pi/3,np.pi/11],6)
# 	print(t)
# 	#print(FK_dh([np.pi/4,np.pi/4,np.pi/2,0,np.pi/3,0],4))
# 	pose = [1,1,1,1,1,1]
# 	oc = [1,1,1]
# 	T = IK(pose,oc)
# 	print(T)
# 	t_til = FK_dh([T[0],T[1],T[2],T[3],T[4],T[5]])
# 	print(t_til)

