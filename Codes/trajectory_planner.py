import numpy as np 
import time

"""
TODO: build a trajectory generator and waypoint planner 
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = self.rexarm.num_joints 
        self.initial_wp = [0.0]*self.num_joints #radians
        self.final_wp = [0.0]*self.num_joints 
        self.dt = 0.05 # command rate
        self.speeds = [0.0]*self.num_joints
        self.points = [0.0]*self.num_joints

    def set_initial_wp(self, waypoint=None):
        self.initial_wp = waypoint

    def set_final_wp(self, waypoint):
        self.final_wp = waypoint

        # num_joints X 1 vector

    def go(self, dt,max_speed = 2.5):
        for i in range(self.num_joints):
            if self.speeds[i] > max_speed:
                self.speeds[i] = max_speed
        self.rexarm.set_speeds(self.speeds)

        self.rexarm.set_positions(self.points)

        # self.rexarm.joints[-1].set_torque_limit(1.0)
        # self.rexarm.joints[-1].set_speed(0.8)
        time.sleep(dt)
        #self.rexarm.send_commands()
        #self.rexarm.get_feedback()

    def stop(self):
        #self.rexarm.set_speeds = [0.0]*self.num_joints
        pass
        
    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed=2.5): #time interval
        dis = np.amax(np.absolute((np.subtract(initial_wp,final_wp)))/max_speed)
        T = 2*dis
        # print('initial_wp'+str(initial_wp))
        # print('final_wp'+str(final_wp))
        # # print(T)
        return T

    def generate_cubic_spline(self, initial_wp, final_wp,T):
        M = np.array([[1,0,0,0],
                    [0,1,0,0],
                    [1,T,T**2,T**3],
                    [0,1,2*T,3*T**2]])
        v0 = [0.0]*self.num_joints
        vf = [0.0]*self.num_joints
        b = np.array([initial_wp,v0,final_wp,vf]) # 4 X num_jogints matrix
        #a = np.linalg.inv(M)*b # 4 X num_joints matrix
        a = np.linalg.solve(M,b)
        return a

    def execute_plan(self, plan, look_ahead=8): #plan: num_joints X 2 matrix
        self.speeds = [0.0] * self.rexarm.num_joints
        self.points = [0.0] * self.rexarm.num_joints
        '''
            self.set_initial_wp(plan[0])
            self.set_final_wp(plan[1])
            T = calc_time_from_waypoints(self.initial_wp,self.final_wp)
            a = self.generate_cubic_spline(self.initial_wp,self.final_wp,T)
            t_num = int(T/self.dt) + 1
            for i in range(t_num):
                t = i*self.dt
                if i+look_ahead <= t_num-1:
                    t_bar = t + look_ahead
                else:
                    t_bar = t
                self.speed = a[1,:] + 2*a[2,:]*t + 3*a[3,:]*t**2
                self.points = a[0,:] + a[1,:]*t_bar + a[2,:]*t_bar**2 + a[3,:]*t_bar**3
                self.go()
        '''
        self.set_initial_wp(plan[0])
        self.set_final_wp(plan[1])
        total_time = self.calc_time_from_waypoints(self.initial_wp,self.final_wp)
        coeff = self.generate_cubic_spline(self.initial_wp,self.final_wp,total_time)
        check_points_num = int(total_time/self.dt)
        dt = total_time/check_points_num
        for i in range(check_points_num):
            '''
            if (i + look_ahead <= check_points_num - 1):
                self.speeds = 
                self.points = 
                self.go()
            else:
                self.speeds = 
                self.points = 
                self.go()
            '''
            t_now = (i+1)*dt
            #T_now_point = [1,t_now,t_now**2,t_now**3]
            T_now_speed = [1,2*t_now,3*t_now**2]
            self.points = self.final_wp
            for j in range(self.num_joints):
                #self.points[j] = self.final_wp #np.dot(T_now_point,coeff[:,j])
                self.speeds[j] = np.dot(T_now_speed,coeff[1:,j])
                if self.speeds[j] < 0.1 and self.speeds[j] > 0:
                    self.speeds[j] = 0.1
                if self.speeds[j] > -0.1 and self.speeds[j] <= 0:
                    self.speeds[j] = -0.1                    
            self.go(dt)
        # self.rexarm.joints[-1].set_torque_limit(1.0)
        # self.rexarm.joints[-1].set_speed(0.8)
            