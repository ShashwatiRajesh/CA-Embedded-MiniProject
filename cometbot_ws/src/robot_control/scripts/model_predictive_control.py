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
import cvxpy

observation = np.array([0.0, 0.0, 0.0], float) #x, y, yaw angle

path = []
vel = 0

WB = 1.8  # [m] wheel base of the vehicle

# predictive control
NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 1  # horizon length # initially 5

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 8.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 5  # Search index number

DT = 0.05  # [s] time tick

MAX_STEER = np.deg2rad(60.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(60.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

class State:
    """
    vehicle state class
    """

    def __init__(self, x, y, yaw, v):
        global observation, vel
        self.x = x
        self.y = y
        self.yaw = yaw
        self.prev_yaw = 0.0
        self.v = v
        self.predelta = None

def angle_mod(x, zero_2_2pi=False, degree=False):
    """
    Angle modulo operation
    Default angle modulo range is [-pi, pi)

    Parameters
    ----------
    x : float or array_like
        A angle or an array of angles. This array is flattened for
        the calculation. When an angle is provided, a float angle is returned.
    zero_2_2pi : bool, optional
        Change angle modulo range to [0, 2pi)
        Default is False.
    degree : bool, optional
        If True, then the given angles are assumed to be in degrees.
        Default is False.

    Returns
    -------
    ret : float or ndarray
        an angle or an array of modulated angle.

    Examples
    --------
    >>> angle_mod(-4.0)
    2.28318531

    >>> angle_mod([-4.0])
    np.array(2.28318531)

    >>> angle_mod([-150.0, 190.0, 350], degree=True)
    array([-150., -170.,  -10.])

    >>> angle_mod(-60.0, zero_2_2pi=True, degree=True)
    array([300.])

    """
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle

def pi_2_pi(angle):
    return angle_mod(angle)

def get_linear_model_matrix(v, phi, delta):

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C

def update_state(state, a):
    global observation, vel

    state.x = observation[0]
    state.y = observation[1]
    state.yaw = observation[2]

    dyaw = state.yaw - state.prev_yaw
    while dyaw >= math.pi / 2.0:
        state.yaw -= math.pi * 2.0
        dyaw = state.yaw - state.prev_yaw
    while dyaw <= -math.pi / 2.0:
        state.yaw += math.pi * 2.0
        dyaw = state.yaw - state.prev_yaw

    state.v = state.v + a * DT

    if state.v > MAX_SPEED: 
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    return state

def get_nparray_from_matrix(x):
    return np.array(x).flatten()

def calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind

def predict_motion(x0, oa, od, xref):
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state = update_state(state, ai)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    MPC control with updating operational point iteratively
    """
    ox, oy, oyaw, ov = None, None, None, None

    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)
        poa, pod = oa[:], od[:]
        """
        xref: reference point
        xbar: operational point
        x0: initial state
        dref: reference steer angle
        """
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
        du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")

    return oa, od, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, dref):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(
            xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                            MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.CLARABEL, verbose=False) #ECOS

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def calc_ref_trajectory(state, cx, cy, cyaw, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref


def check_goal(state, goal, tind, nind):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = (d <= GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(state.v) <= STOP_SPEED)

    if isgoal and isstop:
        return True

    return False

def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)

    for i in range(len(cx) - 1):
        speed_profile[i] = - target_speed

    return speed_profile


def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw

def get_forward_course(dl, ax, ay):

    ryaw, rk = [], []
    for i in range(1, len(ax)):
        dx = ax[i] - ax[i-1]
        dy = ay[i] - ay[i-1]
        if(dy > 0):
            yaw = math.pi - math.atan2(dy, dx)
        else:
            yaw = -math.pi - math.atan2(dy, dx)
        if(i==1):
            ryaw.append(yaw)
            ryaw.append(yaw)
        else:
            ryaw.append(yaw)
      
    return ax, ay, ryaw

class Commander(Node):

    def __init__(self):
        super().__init__('commander')

        self.knuckle_pos = np.array([0,0], float) #left right
        self.wheel_vel= np.array([0,0], float)   #left right
        self.publisher_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.publisher_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        self.time_interval = DT
        self.Tread = 1.1  # track of the front and rear 
        self.L = WB       # wheel base
        self.Rw = 0.3     # Radius of the front and rear wheel
        self.time = 0.0

        self.timer = self.create_timer(self.time_interval, self.timer_callback)

        self.dl = 0.2
        self.target_creation = False

    def timer_callback(self):
        global path

        if(len(path) > 0):
            if(self.target_creation == False):
                """
                cx: course x position list
                cy: course y position list
                cyaw: course yaw position list
                sp: speed profile
                dl: course tick [m]
                """

                cx_list = []
                cy_list = []
                for i in range(len(path)):
                    cx_list.append(path[i][0])
                for i in range(len(path)):
                    cy_list.append(path[i][1])

                self.cx, self.cy, self.cyaw = get_forward_course(self.dl, cx_list, cy_list)
                self.sp = calc_speed_profile(self.cx, self.cy, self.cyaw, TARGET_SPEED)
                initial_state = State(x=self.cx[0], y=self.cy[0], yaw=self.cyaw[0], v=0.0)
                self.state = initial_state

                # initial yaw compensation
                if self.state.yaw - self.cyaw[0] >= math.pi:
                    self.state.yaw -= math.pi * 2.0
                elif self.state.yaw - self.cyaw[0] <= -math.pi:
                    self.state.yaw += math.pi * 2.0

                self.target_ind, _ = calc_nearest_index(self.state, self.cx, self.cy, self.cyaw, 0)

                self.odelta, self.oa = None, None

                self.cyaw = smooth_yaw(self.cyaw)

                self.target_creation = True

            xref, self.target_ind, dref = calc_ref_trajectory(
                self.state, self.cx, self.cy, self.cyaw, self.sp, self.dl, self.target_ind)

            x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]  # current state

            self.oa, self.odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
                xref, x0, dref, self.oa, self.odelta)

            di, ai = 0.0, 0.0
            if self.odelta is not None:
                di, ai = self.odelta[0], self.oa[0]
                self.state = update_state(self.state, ai)

            self.knuckle_pos[0] = atan(2*self.L*sin(di)/(2*self.L*cos(di)-self.Tread*sin(di)))
            self.knuckle_pos[1] = atan(2*self.L*sin(di)/(2*self.L*cos(di)+self.Tread*sin(di)))

            omega = -1*self.state.v * math.sin(di)
            self.wheel_vel[0] = (-1*self.state.v - omega*self.Tread/2)/self.Rw
            self.wheel_vel[1] = (-1*self.state.v + omega*self.Tread/2)/self.Rw  
 
            wheel_vel_array1 = Float64MultiArray(data=self.wheel_vel)    
            self.publisher_vel.publish(wheel_vel_array1)    

            knuckle_pos_array1 = Float64MultiArray(data=self.knuckle_pos)    
            self.publisher_pos.publish(knuckle_pos_array1) 

            self.time = self.time + DT


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
        global observation

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

        acc = msg.linear_acceleration.x
        vel += (acc + self.prev_acc)*self.time_interval
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

