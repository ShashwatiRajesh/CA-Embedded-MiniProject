#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Imu
import numpy as np
import threading
import math
from math import cos, sin, tan, atan, atan2, pi

observation = np.array([0.0, 0.0, 0.0], float) #x, y, yaw angle

path = []
vel = 0

# Parameters
k = 0.1  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 1.83  # [m] wheel base of the vehicle

class State:

    def __init__(self):
        global observation, vel
        self.yaw = observation[2]
        self.v = 0.0
        self.rear_x = observation[0]
        self.rear_y = observation[1]

    def update(self, a):
        global observation
        self.yaw = observation[2]
        self.v += a * commander.time_interval
        self.rear_x = observation[0]
        self.rear_y = observation[1]

        return self.v

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            if(ind > len(self.cx) - 1):
                ind = len(self.cx) - 1
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break

                if (ind < len(self.cx) - 2):
                    ind = ind + 1 
                else:
                    ind

                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

class Commander(Node):

    def __init__(self):
        super().__init__('commander')

        self.knuckle_pos = np.array([0,0], float)  #left right
        self.wheel_vel= np.array([0,0], float)     #left right
        self.publisher_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.publisher_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        self.time_interval = 0.02
        self.T = 1.1  # track of the front and rear 
        self.L = 1.83 # wheel base
        self.Rw = 0.3 # Radius of the front and rear wheel
        self.t = 0

        self.target_creation = False

        self.timer = self.create_timer(self.time_interval, self.timer_callback)

        self.target_speed = 2.0 # (m/s)

        # initial state
        self.state = State()

    def pure_pursuit_steer_control(self, state, trajectory, pind):
        global observation, path
        ind, Lf = trajectory.search_target_index(state)

        if pind >= ind:
            ind = pind

        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1
            
        delta_x = state.rear_x - tx
        delta_y = state.rear_y - ty

        target_angle = -math.atan2(delta_y, delta_x)
        alpha = target_angle  - state.yaw
        delta = -alpha
        if(delta > 0):
            self.knuckle_pos[0] = delta
            self.knuckle_pos[1] = atan(self.L*tan(self.knuckle_pos[0])/(self.L + self.T*tan(self.knuckle_pos[0])))
        elif(delta < 0):
            self.knuckle_pos[0] = atan(self.L*tan(self.knuckle_pos[1])/(self.L - self.T*tan(self.knuckle_pos[1])))
            self.knuckle_pos[1] = delta
        else:
            self.knuckle_pos[0] = 0
            self.knuckle_pos[1] = 0            

        knuckle_pos_array1 = Float64MultiArray(data=self.knuckle_pos)    
        self.publisher_pos.publish(knuckle_pos_array1)  
            
        return alpha, ind, Lf

    def proportional_control(self, target, current):
        a = Kp * (target - current)
        return a

    def timer_callback(self):
        global observation, vel, path

        if(len(path) > 0):

            if(self.target_creation == False):
                cx_list = []
                cy_list = []
                for i in range(len(path)):
                    cx_list.append(path[i][0])
                cx = np.array(cx_list)
                for i in range(len(path)):
                    cy_list.append(path[i][1])
                cy = np.array(cy_list)
                self.target_course = TargetCourse(cx, cy)
                self.target_ind, _ = self.target_course.search_target_index(self.state)
                self.target_creation = True

            # Calc control input
            ai = self.proportional_control(self.target_speed, self.state.v)
            alpha, self.target_ind, Lf = self.pure_pursuit_steer_control(self.state, self.target_course, self.target_ind)
            state_vel = self.state.update(ai)  # Control vehicle
            if(self.target_ind < len(path) - 2):
                omega = 2.0 * state_vel * math.sin(alpha) / Lf
                self.wheel_vel[0] = (state_vel - omega*self.T/2)/self.Rw
                self.wheel_vel[1] = (state_vel + omega*self.T/2)/self.Rw  
            else: # goal
                omega = 0
                self.wheel_vel[0] = 0
                self.wheel_vel[1] = 0                 

            wheel_vel_array = Float64MultiArray(data=self.wheel_vel)    
            self.publisher_vel.publish(wheel_vel_array)  

class Get_modelstate(Node):

    def __init__(self):
        super().__init__('get_modelstate')
        self.subscription = self.create_subscription(
            TFMessage,
            '/model/fws_robot/pose',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global observation, vel

        for tfsf in data.transforms:
            if(tfsf.child_frame_id == 'fws_robot'):
                pose = tfsf.transform.translation
                orientation = tfsf.transform.rotation

                observation[0] = pose.x
                observation[1] = pose.y
                q0 = orientation.x
                q1 = orientation.y
                q2 = orientation.z
                q3 = orientation.w
                #vel = math.sqrt(data.twist[robot_1_id].linear.x**2 + data.twist[robot_1_id].linear.y**2)

                numerator = q0*q1 + q2*q3
                denominator = q0**2 - q1**2 - q2**2 + q3**2

                if(numerator > 0):
                    observation[2] = math.pi - math.atan2(2*numerator, denominator)
                elif(numerator <= 0):
                    observation[2] = -math.pi - math.atan2(2*numerator, denominator)

class Imu_subscriber(Node):
        
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.listener_callback,
            10)
        self.subscription
        self.prev_acc = 0
        self.time_interval = 0.01 #100Hz

    def listener_callback(self, msg):
        global vel

        acc = math.sqrt(msg.linear_acceleration.x**2 + msg.linear_acceleration.y**2)
        vel += (acc + self.prev_acc)*self.time_interval/2
        self.prev_acc = acc

class Get_path(Node):

    def __init__(self):
        super().__init__('get_path')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/path',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        global path

        path_num = np.array(msg.data)
        path = path_num.reshape([int(path_num.shape[0]/2), 2])

if __name__ == '__main__':
    rclpy.init(args=None)

    commander = Commander()
    get_modelstate = Get_modelstate()
    get_path = Get_path()
    imu_subscriber = Imu_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(get_modelstate)
    executor.add_node(get_path)
    executor.add_node(imu_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

