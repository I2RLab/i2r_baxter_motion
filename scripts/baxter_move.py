#!/usr/bin/env python
__author__ = 'behzad'
import sys
from copy import copy
import argparse
import operator
import numpy as np
import csv
import baxter_pykdl
from math import atan, asin
from std_msgs.msg import (
    Float64,
    UInt8,
    Bool,
)
from geometry_msgs.msg import (
    Twist
)
import rospy
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION
import matplotlib.pyplot as plt
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from i2r_baxter_motion.msg import (
    TrustData,
    RobotData,
    Human
)
import baxter_dataflow
import os
current_dir = os.path.dirname(__file__)
parrent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
print (parrent_dir)

class PosPlayer(object):
    def __init__(self, filename, freq=100, emotion=False, use_direction = True):
        """ filename is the start position near human hand
        :param filename: start_pos
        :param freq:
        :param emotion:
        :param use_direction:
        """
        self._use_direction = use_direction
        self._filename = filename
        self._freq = freq
        self._rate = rospy.Rate(freq)
        self._rate_camera = rospy.Rate(freq*10)
        self._pub_path_is_started = rospy.Publisher('trust/path_is_started', Bool, queue_size=1)
        self._pub_path_is_started.publish(False)
        self._pub_path = rospy.Publisher('trust/robot_states', RobotData, queue_size=1)
        self._collision_detected = False
        self._inver_time = rospy.get_time() #last time human interveneted
        self._inverted = False #last time human interveneted
        self._path = RobotData()
        self._path.lr =1
        self._path.grad=1
        self._sub_path_vel_man = rospy.Subscriber('/trust/ur_manual', Float64, self.path_vel_man_callback)
        self._sub_trust = rospy.Subscriber('/trust/trust', TrustData, self.updateTrust)
        self._trust_mean = 0.5
        self._trust_count = 0
        self._trust_mean_last = 0.5
        self._head = baxter_interface.Head()
        self._head.set_pan(0.0)

        self._base_pos = []
        self._done = False
        self._limb = baxter_interface.Limb("left")
        self._joints = self._limb.joint_names()
        self._limb_kin = baxter_pykdl.baxter_kinematics('left')
        self._gripper = baxter_interface.Gripper("left", CHECK_VERSION)
        self._headers=[]
        self._positions=[]
        self._directions=[]
        self._dir = [] # curent direction
        self._dir_ds = 1 # gradient of curent direction wrt s
        self._vel = [] # current velocity vector
        self._path_vel = 0.75 # Needs to subscribes: path velocity
        self._limb.set_joint_position_speed(0.4)
        self._q_dot_max = np.array([[4, 4, 4, 2, 2, 2, 2]]).reshape(-1,)
        # for publishing
        # path.x ---> path_pos (s)
        # path.y ---> path_status (0,1,2)
        # baxter center pose
        config_file = parrent_dir + "/config.txt"
        reader = csv.reader(open(config_file, "rb"), delimiter=' ')
        reader.next()
        x = list(reader)
        config_values = np.array(x).astype('float')
        # self._center_pos = [-662, -3134, 1050]  # baxter center point in phasespace
        self._center_pos = config_values [2]
        self._center_deg = np.radians(180)  # baxter orientation with respect to phasespace
        self._rel_head_center = np.array([180,0,645]).astype('float')  # baxter head relative to baxter base
        self._head_center = self._center_pos + self._rel_head_center
        self._path_pos_0 = 0.0
        self._path_len = 0
        self._path_status = 1  # 1=forward, 2=pick, 3=backward
        self._K = .01 * np.identity(6)
        self._K[5,5] = .1
        self._path_offset = np.zeros((6, 1))
        self._way_point_coe = 0.5
        self._lambda = 0.1 #damping lambda
        # path waypoints minimum threshold
        self._eps = .05
        # Verify Grippers Have No Errors and are Calibrated
        if self._gripper.error():
            self._gripper.reset()
        if (not self._gripper.calibrated() and
                    self._gripper.type() != 'custom'):
            self._gripper.calibrate()
        self._gripper.calibrate()
        # Using the navigator for releasing during hand over
        self._nav_right = baxter_interface.Navigator('right')
        self._nav_right.button0_changed.connect(self._nav_b0_pressed)

        # Use left navigator for start and stop robot
        self._nav_left = baxter_interface.Navigator('left')
        self._nav_left.button0_changed.connect(self._nav_l0_pressed)
        self._started = False

        self._gripper_open = True
        self._is_hand_over = False
        # read the recorded file
        self._joint_first = []
        self._joint_last = []
        self._dir_test = []
        self.read_recorded()
        # for recording
        self._rec_positions = []
        self._rec_path_pos = 0
        self._rec_active = True
        # Blob Detection
        try:
            self._right_hand_camera = baxter_interface.CameraController('right_hand_camera')
            self._right_hand_camera.close()
        except AttributeError:
            pass
        try:
            self._left_hand_camera = baxter_interface.CameraController('left_hand_camera')
            self._left_hand_camera.close()
        except AttributeError:
            pass
        self._cvbridge = CvBridge()
        self._hand_image = None
        self.late_center = (570, 430)
        self.long_center = (620, 360)
        self._pos_eof = []
        self._pos_joint = []
        self._top_pos_eof = []
        self._top_pos_joint = []
        self._obj_pos_eof = []
        self._obj_pos_joint = []
        # pick up positions
        self._pick_positions = []
        self._pick_idx = 0
        # place positions
        self._place_positions =[]
        self._place_idx = 0
        self.read_place_pos(filename)
        # Display Image
        self._display_image = np.zeros((1024, 600, 3), np.uint8)
        self._pub_img = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)

        # collision detection.
        self.camera_detect = False  # Whether to use visual detection for safety
        self._camera_area_threshold = (400, 800)  # Theshold number of pixels to indicate stop signal
        self._camera_area_rectangle = (660, 800, 372, 660)  # Detection rectangle for stop signal (y1, y2, x1, x2)
        self._debug_collision = False
        self._debug = False
        self._pub_debug = rospy.Publisher('trust/debug_player', Twist, queue_size=1)
        self._bgsubtractor = cv2.BackgroundSubtractorMOG()
        self._camera_workspace_begin = 402 #image
        self._camera_workspace_end = 600
        self._camera_workspace_offset = 150
        self._limb_y_workspace_begin = 0.4810551107413326 #workspace
        self._limb_y_workspace_end = 0.1497036417856029
        self._limb_y_workspace_start = 0.58950401153522277
        # self._collision_detected = False
        self._collision_path_vel = self._path_vel
        self._delay_counter = 0;
        img_file = parrent_dir + '/img/idle.png'
        self.set_image(img_file)
        if self.camera_detect:
            self._head_camera = baxter_interface.CameraController('head_camera')
            self._head_camera.resolution = (1280, 800)
            self._head_camera.exposure = 20
            self._head_camera.open()
            self._sub_head_img = rospy.Subscriber('/cameras/head_camera/image', Image,
                                                  self.detection_callback)  # Subscribe to head camera topic
        # PhaseSpace Collision Detection
        self._tracking_detect = emotion  # Whether to use phasespace detection for safety
        self._xh = self._center_pos[0]  #human x in ref frame (phasespace)
        self._xr = self._center_pos[1]  #human y in ref frame (phasespace)
        self._xz = self._center_pos[2]  #human z in ref frame (phasespace)
        self._d_safe = pow(200, 2)   #change the first argument of pow(distance <---, 2)
        if self._tracking_detect:
            self._sub_human = rospy.Subscriber('/trust/human_states', Human, self.updateHuman)
            self._pub_emotion = rospy.Publisher('trust/emotion', UInt8, queue_size=1)
            # self._emotion_state = 0
            # 0: Happy
            # 1: Worried (Collision)
            # 2: Late
            # 3: Ahead
            self.__img_path = [parrent_dir + '/img/happy.png',
                               parrent_dir + '/img/worried.png',
                               parrent_dir + '/img/bored.png',]
            # self.set_image()
        self._sub_path_vel = rospy.Subscriber('/trust/ur', Float64, self.path_vel_callback)
        #TASE
        self._start_point = self._joint_first

    def joint_current(self):
        joints_left = self._limb.joint_names()
        # forward kinematics
        limb_pos = self._limb_kin.forward_position_kinematics()
        angles_left = [self._limb.joint_angle(j) for j in joints_left]
        cur_pos =  [0.0]+list(limb_pos)+[0.0]+angles_left+[self._gripper.position()]
        return np.array(cur_pos).astype('float')


    def show_speed(self):
        img = self._image.copy()
        cv2.putText(img, 'Speed: {0:.2f}'.format(self._path_vel), (530, 380), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 4,
                    v2.CV_AA)
        self.send_cv_image(img)

    def force_velocity(self, vel):  # use for debug only: this method sets the robot velocity and overrides other programs commands
        # vel should be between 0 and 1
        self._path_vel = vel
        self._limb.set_joint_position_speed(vel)

    def send_cv_image(self, img):
        msg = self._cvbridge.cv2_to_imgmsg(img, "bgr8")
        self._pub_img.publish(msg)

    def set_image(self, img_path = parrent_dir + '/img/happy.png', left_eye_position = None, right_eye_position = None):
        self._image = cv2.imread(img_path)
        if np.any(left_eye_position):
            cv2.circle(self._image,left_eye_position, 36, (39, 78, 102), -1)
            cv2.circle(self._image,right_eye_position, 36, (39, 78, 102), -1)
        self.send_cv_image(self._image)
        self._rate.sleep()

    def detection_callback(self, imagedata):
        head_image = self._cvbridge.imgmsg_to_cv2(imagedata, 'bgr8')
        if self._debug:
            cv2.imshow('head camera', cv2.resize(head_image, (600, 400)))

        # Detect for (left) limb collision using camera
        mask = self._bgsubtractor.apply(head_image)

        if self.camera_detect:
            limb_y = self._limb.endpoint_pose()['position'].y
            if limb_y < self._limb_y_workspace_start:
                rect = (
                    self._camera_area_rectangle[0],
                    self._camera_area_rectangle[1],
                    int(self._camera_workspace_begin + (limb_y - self._limb_y_workspace_begin) * (
                        self._camera_workspace_end - self._camera_workspace_begin) / (self._limb_y_workspace_end - self._limb_y_workspace_begin)),
                    min(int(self._camera_workspace_offset + self._camera_workspace_begin + (limb_y - self._limb_y_workspace_begin) * (
                        self._camera_workspace_end - self._camera_workspace_begin) / (self._limb_y_workspace_end - self._limb_y_workspace_begin)),
                        self._camera_area_rectangle[3])
                )
                submask = mask[rect[0]:rect[1], rect[2]:rect[3]]
                area = cv2.countNonZero(submask)
                camera_stop = area > self._camera_area_threshold[0] and area < self._camera_area_threshold[1]

                if self._debug_collision:
                    print 'Area:', area  # debug
                    cv2.rectangle(mask, (rect[2], rect[0]), (rect[3], rect[1]), (255, 255, 255), 1)  # debug
            else:
                camera_stop = False
        else:
            camera_stop = False

        if self._debug_collision:
            cv2.imshow('debug', cv2.resize(mask, (600, 400)))  # debug

        cv2.waitKey(10)

        if camera_stop:
            if not self._collision_detected:
                if self._path_vel > 0:
                    self._collision_path_vel = self._path_vel
                    self._path_vel = 0.0
                    self._delay_counter = 0
                print ('Baxter stops to prevent a possible collision')
                self.set_image(parrent_dir + '/img/worried.png')
        else:
            # if self._collision_detected:
            if self._delay_counter > 3:
                self._collision_detected = False
                print ('Baxter moves again')
                self.set_image()
                self._path_vel = self._collision_path_vel
                print(self._path_vel)
            else: # No Collision
                self._delay_counter += 1
                print ('Delay counter', self._delay_counter)

        self._collision_detected = camera_stop

    def reset_trust(self):
        self._trust_count = 1

    def updateTrust(self, data):
        self._trust_mean = (self._trust_mean_last * self._trust_count + data.trust) / (self._trust_count+1)
        self._trust_mean_last = self._trust_mean
        self._trust_count += 1

    def updateHuman(self, data):
        self._xh = data.x0
        self._yh = data.y0
        self._zh = data.z0
        self.get_end_point()
        pos_human = self.get_tracking_point([data.x0, data.y0, data.z0])

        if self._debug:
            debug_state = Twist()
            debug_state.linear.x = pos_human[0]
            debug_state.linear.y = pos_human[1]
            debug_state.linear.z = pos_human[2]
            base_pos = self._limb_kin.forward_position_kinematics()
            debug_state.angular.x = base_pos[0]*1000
            debug_state.angular.y = base_pos[1]*1000
            debug_state.angular.z = base_pos[2]*1000
            self._pub_debug.publish(debug_state)

        #  sight range of the robot, width along the Y axis and height along the Z axis,
        #  unit: mm, coordinates with respect to head
        sight_range = np.array([1000, 1000]).astype('float')

        #  eye range of motion in head screen, width along the x axis and height along the y axis
        #  unit: pixels
        eye_range = np.array([50, 50]).astype('float')

        #  sight_range at X=5000 mm
        depth_ratio = min(pos_human[0] / 5000, 1)

        eye2sight = eye_range / sight_range * (1 - depth_ratio)
        eye_s_l_center = np.array([417, 277]).astype('int')
        eye_s_r_center = np.array([606, 277]).astype('int')
        eye_l_pos = np.array([0, 20, 0]).astype('float')
        eye_r_pos = np.array([0, -20, 0]).astype('float')

        hand_l_pos = (pos_human - eye_l_pos)
        hand_r_pos = (pos_human - eye_r_pos)
        dl = np.array([hand_l_pos[1] * eye2sight[0], -hand_l_pos[2] * eye2sight[1]]).astype('int')
        len_dl = np.linalg.norm(dl)
        len_eye = np.linalg.norm(eye_range) / 2
        if len_dl > len_eye:
            dl = (dl / len_dl * len_eye).astype('int')
        dr = np.array([hand_r_pos[1] * eye2sight[0], -hand_r_pos[2] * eye2sight[1]]).astype('int')
        len_dr = np.linalg.norm(dr)
        if len_dr > len_eye:
            dr = (dr / len_dr * len_eye).astype('int')
        eye_s_l = eye_s_l_center + dl
        eye_s_r = eye_s_r_center + dr

        # Collision Checking using PhaseSpace tracking system
        l = pow((self._xh-self._path.xr), 2) + pow((self._yh-self._path.yr), 2) + pow((self._zh-self._path.zr), 2)
        tracking_stop = l < self._d_safe
        # print('l=', l)
        # print(self._xh, self._path.xr, self._yh, self._path.yr, self._zh, self._path.zr)
        if tracking_stop:
            if not self._collision_detected:
                # self._emotion_state = 1
                # printr('l=', l)
                # print(self._xh, self._path.xr, self._yh, self._path.yr, self._zh, self._path.zr)
                if self._path_vel > 0:
                    self._collision_path_vel = self._path_vel
                    self._path_vel = 0.0
        else:
            if self._collision_detected:
                self._collision_detected = False
                # print ('Baxter moves again')
                # self._emotion_state = 1
                # self.set_image()
                self._path_vel = self._collision_path_vel
                # print(self._path_vel)
        self._collision_detected = tracking_stop
        sh = data.sh
        sr = self._path.sr
        delta = sr - sh
        emotion = 0
        if delta > 0.5:  # robot is too far ahead
            emotion = 2
        elif delta < -0.5: # robot is late
            # emotion = 2
            emotion = 0
        if tracking_stop:
            emotion = 1
        self.set_image(self.__img_path[emotion], (eye_s_l[0],eye_s_l[1]), tuple(eye_s_r))
        self._pub_emotion.publish(emotion)

    def reset_positions(self, positions):
        self._done = False
        self._dir = [] # curent direction
        self._pos = [] # current position
        self._vel = [] # current velocity vector
        self._dir_test = []
        self.make_pos(positions)

    def hand_callback(self, data):
        self._hand_image = self._cvbridge.imgmsg_to_cv2(data, 'bgr8')
        # if debug:
        # cv2.imshow('hand camera', cv2.resize(self._hand_image, (600, 400)))
        # cv2.waitKey()
        # print ("I've got the hand imgae")

    def path_vel_callback(self, data):
        vel = 0.35
        if not self._collision_detected:
            if self._inverted:
                dt = rospy.get_time()-self._inver_time
                # print(dt)
                if dt > 10:
                    vel = data.data
            elif data.data >= 0.3:
                vel = data.data
                # print('change the velocity to ', data.data)
            else:
                # print('the velocity should be greater than 0.3: ', data.data)
                vel = 0.3
                # None	
            vel = max(0.3, vel)
            self._path_vel = vel
            self._path.vel = vel
            self._pub_path.publish(self._path)

    def path_vel_man_callback(self, data):
        if not self._collision_detected:
            if data.data >= 0.3:
                self._path_vel = data.data
                self._path.vel = data.data
                self._pub_path.publish(self._path)
                self._inver_time = rospy.get_time()
                self._inverted = True
            else:
                self._path_vel = 0.3
                self._path.vel = 0.3
                self._pub_path.publish(self._path)

    def stop(self):
        """
        Stop moving.
        """
        self._done = True
        self._pub_path_is_started.publish(False)
        self._limb.set_joint_position_speed(0.3)
        self._head.set_pan(0.0)
        if self.camera_detect:
            self._head_camera.close()
        self.set_image(parrent_dir + '/img/end.png')
        print('Baxter Rocks!')
        rospy.loginfo("Robot Velocity is now %s", self._path_vel)

    def done(self):
        """
        Return whether or not moving is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def refine_pos(self, posvalues):
        """
        Refine the position values based on the epsilon and calculates the correspondent directions.
        """
        self._positions = []
        self._directions = []
        line_cur=posvalues[0]
        # print(line_cur)
        idx=0
        # print 'refine_pos'
        for values in posvalues[0:]:
            line_next = values
            unit_dir, grads=calc_dir(line_cur,line_next, self._eps/3)
            if unit_dir!=None or not self._use_direction:
                self._positions.append(line_cur)
                if unit_dir!=None:
                    vec = unit_dir.tolist()
                    self._directions.append(vec)
                line_cur=line_next
                idx +=1
        # add the last point and zero direction at the end
        zero_dir = np.zeros(7)
        self._positions.append(line_cur)
        vec=zero_dir.tolist()
        vec[0]=line_cur[0]
        self._directions.append(vec)
        return idx>1

    def make_pos(self, posvalues):
        """
        Refine the position values based on the epsilon and calculates the correspondent directions.
        """
        self._positions = []
        self._directions = []
        pos_cur = posvalues[0]
        line_cur = np.insert(pos_cur, 0, 0)
        self._joint_first = line_cur

        current_length = 0
        idx=0

        for values in posvalues[0:]:
            # print 'values=', values
            pos_next = values
            current_length += arc_length(pos_cur, pos_next)
            line_next = np.insert(pos_next, 0, current_length)
            unit_dir, grads=calc_dir(line_cur,line_next, self._eps/3)
            if unit_dir!=None:
                self._positions.append(line_cur)
                vec=unit_dir.tolist()
                self._directions.append(vec)
                line_cur=line_next
                pos_cur=pos_next
            idx +=1
        # add the last point and zero direction at the end
        zero_dir = np.zeros(7)
        self._positions.append(line_cur)
        self._joint_last = line_cur

        vec=zero_dir.tolist()
        vec[0]=line_cur[0]
        self._directions.append(vec)

    def read_recorded(self):
        """
        Read the recorded file for positions and directions
        """
        reader=csv.reader(open(self._filename,"rb"),delimiter=',')
        self._headers = reader.next()
        x=list(reader)
        recorded_positions=np.array(x).astype('float')
        self.refine_pos(recorded_positions)
        self._joint_first = recorded_positions[0]
        self._joint_last = recorded_positions[-1]

    def open_file(self, filename):
        self._filename = filename
        self._done = False
        self._path_pos = 0  # Distance robot moved along the path
        self.read_recorded()

    def read_pick_up_pos(self, filename):
        reader = csv.reader(open(filename,"rb"),delimiter=',')
        self._headers = reader.next()
        x = list(reader)
        posvalues = np.array(x).astype('float')
        for values in posvalues[0:]:
            self._pick_positions.append(values)

    def read_place_pos(self, filename):
        reader = csv.reader(open(filename,"rb"),delimiter=',')
        self._headers = reader.next()
        x = list(reader)
        posvalues = np.array(x).astype('float')
        for values in posvalues[0:]:
            self._place_positions.append(values)
        # print(self._place_positions)

    def move_to_pos(self, position, timeout = 3, threshold = 0.008726646):
        # To Do : use header to find joint_names
        pos = position[9:17]
        combined = zip(self._joints, pos)
        # take out any tuples that have a none value
        cleaned = [x for x in combined if x[1] is not None]
        # convert it to a dictionary with only valid commands
        command = dict(cleaned)
        self._limb.move_to_joint_positions(command, timeout=timeout, threshold=threshold)
        return command

    def move_to_pos_raw(self, position, timeout = 3, threshold = 0.008726646, joints = False, c1 = 0.9751):
        # To Do : use header to find joint_names
        if joints:
            pos = position
        else:
            pos = position[9:17]
        combined = zip(self._joints, pos)
        # take out any tuples that have a none value
        cleaned = [x for x in combined if x[1] is not None]
        # convert it to a dictionary with only valid commands
        positions = dict(cleaned)

        cmd = self._limb.joint_angles()
        def filtered_cmd():
            # First Order Filter - 0.2 Hz Cutoff
            for joint in positions.keys():
                cmd[joint] = (1-c1) * positions[joint] + c1 * cmd[joint]
            return cmd


        def genf(joint, angle):
            def joint_diff():
                return abs(angle - self._limb._joint_angle[joint])
            return joint_diff

        diffs = [genf(j, a) for j, a in positions.items() if
                 j in self._limb._joint_angle]

        self._limb.set_joint_positions(filtered_cmd())
        baxter_dataflow.wait_for(
            test=lambda: (all(diff() < threshold for diff in diffs)),
            timeout=-timeout,
            timeout_msg=("Left limb failed to reach commanded joint positions"),
            rate=100,
            raise_on_error=True,
            body=lambda: self._limb.set_joint_positions(filtered_cmd())
            )
        # self._limb.set_joint_positions(filtered_cmd())
        # return positions

    def set_task_velocity(self, next_path_pos = None, direction = None, gripper_rotation = True, velocity_ratio = 0.75):
        if direction:
            # direction towards the next point based on the path position
            self._dir = direction
        if next_path_pos:
            # direction towards the next point using actual position
            actual_point = self._limb_kin.forward_position_kinematics()
            self.get_end_point(actual_point)

            act_point = np.zeros(8)
            act_point[0]=0.00001
            act_point[1:]=actual_point
            next_point = self.path_point(next_path_pos)
            dir_act, grads = calc_dir(act_point, next_point, 0.0)
            self._dir_ds = grads
            self._dir_test.append(dir_act)
            if np.any(dir_act):
                dir_ef=self._way_point_coe*np.array(self._dir)+(1-self._way_point_coe)*np.array(dir_act[1:])
                dir_eff = calc_unit_vector(dir_ef, self._eps)
                if np.any(dir_eff):
                    tmp = [i * self._path_vel for i in dir_eff]
                else:
                    tmp = np.zeros(6)
            else:
                tmp = np.zeros(6)
        else:
            tmp = [i * self._path_vel for i in self._dir]
        self._vel = np.matrix(tmp).transpose()
        # adding closed-loop kinematics
        t_dot = self._vel + self._path_offset
        J = self._limb_kin.jacobian()
        Js = pinv_damped(J, self._lambda)
        q_dot = Js * t_dot
        # scaling joint velocities wrt to the joint with the maximal velocity
        sc_factor = np.amin(np.absolute(self._q_dot_max/q_dot))
        if sc_factor < 1 :
            q_dot *= sc_factor
        q_dot *= velocity_ratio

        if gripper_rotation:
            s_end = self._positions[-1][0]
            ds = s_end
            dw = self._joint_last[15] - self._joint_first[15]
            if ds > 0:
                w_dot_mean = dw/ds*self._path_vel*.5
            else:
                w_dot_mean = 0
            q_dot[6]=[w_dot_mean]

        cmd_values = np.array(q_dot).reshape(-1,).tolist()
        cmd = dict(zip(self._joints, cmd_values))
        #find sigular values of Jacobian
        singles = np.linalg.svd(J, compute_uv=False)
        # record singular jacobian values
        # singular_values=[]
        # singular_values.append(singles)

        self._limb.set_joint_velocities(cmd)
        self._rate.sleep()
        return singles

    def set_status(self, status):
        self._path.status = status
        self._limb.set_joint_position_speed(0.3+self._path_vel*0.2)
        self._pub_path.publish(self._path)
        self._rate.sleep()

    def step_passed_length(self):
        """
        :type pre_pos: forward kinematics position
        :type _base_pos: forward kinematics position
        """
        cur_pos = self._limb_kin.forward_position_kinematics()
        diffs = map(operator.sub, self._base_pos[0:2], cur_pos[0:2])
        return np.linalg.norm(diffs)

    def get_end_point(self, base_pos=None):
        """
        :param base_pos: the position of the end-effector in robot coordinate system
        :return: returns the position of the end-effector in phasespace tracking system coordinate system
        """
        if not np.any(base_pos):
            base_pos = self._limb_kin.forward_position_kinematics()
        pos_left = np.array(base_pos[0:3])*1000
        self._path.xr = +pos_left[0] * np.cos(self._center_deg) - pos_left[1] * np.sin(self._center_deg) + self._center_pos[0]
        self._path.yr = +pos_left[0] * np.sin(self._center_deg) + pos_left[1] * np.cos(self._center_deg) + self._center_pos[1]
        self._path.zr = pos_left[2] + self._center_pos[2]

    def get_tracking_point(self, pos):
        """
        :param pos[3]: position of the human hand in phasespace tracking system coordiante system
        :return: pos_in_head[3] returns position of the human hand in robot's head coordinate system
        :return: pos_in_robot[3] returns position of the human hand in robot coordinate system
        """
        # pos_left = np.array(base_pos[0:3])*1000
        pos_in_robot = np.zeros(3)
        dx = pos[0]-self._center_pos[0]
        dy = pos[1]-self._center_pos[1]
        pos_in_robot[0] = +dx * np.cos(self._center_deg) + dy * np.sin(self._center_deg)
        pos_in_robot[1] = -dx * np.sin(self._center_deg) + dy * np.cos(self._center_deg)
        pos_in_robot[2] = pos[2] - self._center_pos[2]
        pos_in_head = pos_in_robot - self._rel_head_center
        # return pos_in_head, pos_in_robot,
        return pos_in_head
        # return pos_in_robot

    def reached_task_goal(self, goal, threshold = 0.0009,):
        cmd = self._limb_kin.forward_position_kinematics()[0:3].tolist()
        diff=map(lambda x,y:x-y, goal, cmd)
        err = (all(x*x < threshold for x in diff))
        return err

    def move_tase(self,  gripper_rotation = False, first_timeout=10, timeout=5, velocity_ratio = 0.75):
        if self._rec_active:
            point = self._limb_kin.forward_position_kinematics()
            rec_point = np.insert(point, 0, self._rec_path_pos)
            self._rec_positions.append(rec_point)
        s_passed = 0
        s_end = self._positions[-1][0]
        self._path_len = s_end*4
        ds_next = self._positions[1][0]
        pos_idx = 0
        self._dir = self._directions[0][1:]
        tmp = [i * self._path_vel for i in self._dir]
        self._vel = np.matrix(tmp).transpose()
        singular_values = []
        # singular_values.append(singles)
        last_time = rospy.get_time()
        ds = 0
        self._base_pos = self._limb_kin.forward_position_kinematics()
        # publish info
        self._pub_path_is_started.publish(True)
        alpha=0.60 #path completed for maximum speed
        beta = alpha+(1-alpha)*self._path_vel
        self._path.lr = s_end
        #beta is one at vel=0
        #beta is alpha at vel =1
        goal = self._positions[-1][1:4]
        goal = copy(goal)
        while s_passed<s_end*beta and not rospy.is_shutdown() and not self._done:
            if self._path_vel > 0.01:
                singles = self.set_task_velocity(s_passed+ds_next, gripper_rotation=gripper_rotation, velocity_ratio = velocity_ratio)
                self._rate.sleep()
                # singular_values.append(singles)
                ds = self.step_passed_length()
                self._path.grad = self._dir_ds
                self._path.sr = self._path_pos_0 + (s_passed+ds)/self._path_len
                self._path.vel = self._path_vel
                self._pub_path.publish(self._path)
                if (ds>ds_next):
                    s_passed += ds
                    self._base_pos = self._limb_kin.forward_position_kinematics()
                    self.get_end_point()
                    self.calc_error(s_passed)
                    singles = self.set_task_velocity(s_passed+ds, gripper_rotation=gripper_rotation, velocity_ratio=velocity_ratio)
                    self._rate.sleep()
                    # singular_values.append(singles)
                    ds = self.step_passed_length()
                    pos_idx += 1
                    try:
                        self._dir = self._directions[pos_idx][1:]
                        ds_next = self._directions[pos_idx][0] - self._directions[pos_idx-1][0]
                    except IndexError:
                        self._dir=[0, 0, 0, 0, 0, 0]
                        self._done = True
                        break
                else:
                    self.calc_error(s_passed)
            else:
                self._path.sr = self._path_pos_0 + (s_passed+ds)/self._path_len
                self._path.vel = self._path_vel
                self._pub_path.publish(self._path)
                self._rate.sleep()

            # self._rate.sleep()
        self._pub_path_is_started.publish(False)
        self._path.sr = self._path_pos_0 + 0.25
        self._pub_path.publish(self._path)

        cmd_values=[0, 0, 0, 0, 0, 0, 0]
        cmd = dict(zip(self._joints, cmd_values))
        self._limb.set_joint_velocities(cmd)
        self._rate.sleep()
        # record point
        point = self._limb_kin.forward_position_kinematics()
        self.rec_point(point)

        with open('rec_path_quad--.csv', 'wb') as csvfile:
            writer = csv.writer(csvfile, delimiter=',',
                                    quotechar='|', quoting=csv.QUOTE_MINIMAL)
            [writer.writerow(r) for r in self._rec_positions]
        with open('singular_values.csv', 'wb') as csvfile:
            writer = csv.writer(csvfile, delimiter=',',
                                    quotechar='|', quoting=csv.QUOTE_MINIMAL)
            [writer.writerow(r) for r in singular_values]

    def move_waypoint(self, point_number, point_timeout=10):
        # starts from 1 (first point = 1)
        try:
            values = self._positions[point_number-1]
        except IndexError:
            values = self._positions[-1]
        self.move_to_pos(values, timeout=point_timeout)

    def move_debug(self, point_timeout=10):
        for i, values in enumerate(self._positions):
            self.move_to_pos(values, timeout=point_timeout)
            rospy.loginfo("Moved to Point #"+str(i))
            c = baxter_external_devices.getch()
            if c:
                del c
                sys.stdout.flush()
                sys.stdin.flush()
                self._rate.sleep()

    def create_line(self, dir_ref=[0, -1, 0, 0, 0, 0, 0]):
        # To Do : use header to find joint_names
        s_passed = 0
        # s_end = 1
        ds = self._eps
        point_first = self._joint_first
        point_last = self._joint_last
        s_end = self._joint_last[0]
        dir_ref, grads = calc_dir(point_first, point_last, ds)
        # print point_first, point_last
        # print dir_ref, s_end
        point_cur = point_first[0:8]
        # print ('first=', point_first)
        points=[]
        for i in range(0, int(s_end/ds)):
            points.append(point_cur)
            s = (i+1) * ds
            point_increment =  [ds * u_j for u_j in dir_ref]
            point_increment[0]=ds
            point_increment.append(0.0)
            point_next= point_cur + np.array(point_increment)
            point_cur=point_next
        positions=np.array(points).astype('float')
        self.refine_pos(positions)
        with open('line_trajectory--.csv', 'wb') as csvfile:
            writer = csv.writer(csvfile, delimiter=',',
                                    quotechar='|', quoting=csv.QUOTE_MINIMAL)
            [writer.writerow(r) for r in positions]

    def create_quad_z(self):
        # To Do : use header to find joint_names
        s_passed = 0
        ds = self._eps
        point_first = self._joint_first
        point_last = self._joint_last
        point_mid = point_next=map(lambda a,b:(a+b)/2, point_first, point_last)
        point_mid[2] += .05
        yy = [point_first[2], point_mid[2], point_last[2]]
        zz = [point_first[3], point_mid[3], point_last[3]]
        a, b, c = quad_coefficient(np.array(yy).astype('float'), np.array(zz).astype('float'))
        # s_end = self._joint_last[0]
        y_end = point_last[2] - point_first[2]
        x_end = point_last[1] - point_first[1]
        point_cur = point_first[0:8]
        points = []
        for i in range(0, int(y_end/ds)+1):
            points.append(point_cur)
            y = ds + point_cur[2]
            x = ds * x_end/y_end + point_cur[1]
            z = a*y*y + b*y + c
            diffs = map(operator.sub, point_cur[1:4], [x, y, z])
            dist = np.linalg.norm(diffs)
            s_passed += dist
            point_next = [s_passed, x, y, z, point_cur[4], point_cur[5], point_cur[6], point_cur[7]]
            point_cur=point_next
        positions=np.array(points).astype('float')

        self.refine_pos(positions)
        with open('quad_trajectory--.csv', 'wb') as csvfile:
            writer = csv.writer(csvfile, delimiter=',',
                                    quotechar='|', quoting=csv.QUOTE_MINIMAL)
            [writer.writerow(r) for r in positions]

    def create_path(self, axis = 3, mid_offset=.01, mid_offset_vector = None, mid_offset_knot = 0.5, firstpoint = None, lastpoint = None):
        """
        :param axis: 1=x, 2=y, 3=z
        :param mid_offset: midpoint offset
        :param mid_offset_vector:
        :param mid_offset_knot: mid_point knot position: 0.1: begin, 0.5: middle, .9: end
        :return either use "axis and mid_ffset" or "mid_offset_vector"
        """
        s_passed = 0
        ds = self._eps
        if np.any(firstpoint):
            point_first = firstpoint
        else:
            # point_first = self._joint_first
            point_first = self.joint_current()
        if np.any(lastpoint):
            point_last = lastpoint
        else:
            point_last = self._joint_last
        self._joint_last = point_last

        point_mid = map(lambda a,b:(a+b)/2, point_first, point_last)
        if mid_offset_vector:
            point_mid[1] += mid_offset_vector[0]
            point_mid[2] += mid_offset_vector[1]
            point_mid[3] += mid_offset_vector[2]
        else:
            point_mid[axis] += mid_offset
        l = arc_length(point_first, point_mid)+arc_length(point_mid, point_last)
        sp = [0, mid_offset_knot*l, l]
        xp = [point_first[1], point_mid[1], point_last[1]]
        yp = [point_first[2], point_mid[2], point_last[2]]
        zp = [point_first[3], point_mid[3], point_last[3]]
        coeff_x = np.polyfit(sp, xp, 2)
        coeff_y = np.polyfit(sp, yp, 2)
        coeff_z = np.polyfit(sp, zp, 2)
        poly_x = np.poly1d(coeff_x)
        poly_y = np.poly1d(coeff_y)
        poly_z = np.poly1d(coeff_z)
        sr = np.linspace(0, l, l/ds)
        xr = poly_x(sr)
        yr = poly_y(sr)
        zr = poly_z(sr)
        point_cur = point_first[0:8]
        points = []
        for i in range(1, sr.size):
            points.append(point_cur)
            x = xr[i]
            y = yr[i]
            z = zr[i]
            diffs = map(operator.sub, point_cur[1:4], [x, y, z])
            dist = np.linalg.norm(diffs)
            s_passed += dist

            point_next = [s_passed, x, y, z, point_cur[4], point_cur[5], point_cur[6], point_cur[7]]
            point_cur=point_next
        points.append(point_cur)
        positions=np.array(points).astype('float')

        find_path = self.refine_pos(positions)
        return find_path
        # with open('quad_xy_trajectory.csv', 'wb') as csvfile:
        # with open('quad_xy_trajectory--.csv', 'wb') as csvfile:
        #     writer = csv.writer(csvfile, delimiter=',',
        #                             quotechar='|', quoting=csv.QUOTE_MINIMAL)
        #     # [writer.writerow(r) for r in positions]

    def create_path_dir(self, axis = 3, mid_offset=.05, mid_offset_vector = None, mid_offset_knot = 0.5, direction=1):
        # direction: 1=fwd  or True
        # direction: 0:bwd  or False
        firstpoint = self.joint_current()
        if direction: #forward toward the pick position
            # firstpoint = self._start_point
            lastpoint = self._pick_positions[self._pick_idx]
            lastpoint[3]+=.15
            self.create_path(axis, mid_offset, mid_offset_vector, mid_offset_knot, firstpoint, lastpoint)
            lastpoint[3] -= .15
        else:
            lastpoint = self._place_positions[self._place_idx]
            lastpoint[3]+=.15
            self.create_path(axis, mid_offset, mid_offset_vector, mid_offset_knot, firstpoint, lastpoint)
            lastpoint[3] -= .15

    def plot_result(self):
        print self._positions[:][0]
        print self._positions[:][1]
        print self._rec_positions[:][0]
        print self._rec_positions[:][1]
        plt.plot(self._positions[:][1], self._positions[:][2], 'ro')
        plt.legend(['ref','robot'])
        plt.title('x-y trajectory')
        plt.show()

    def rec_point(self, point):
        if self._rec_active:
            self._rec_path_pos += arc_length(self._rec_positions[-1][1:], point)
            rec_point  = np.insert(point, 0, self._rec_path_pos)
            self._rec_positions.append(rec_point)

    def calc_error(self, path_pos):
        point_ref = self.path_point(path_pos)
        task_ref = task_point(point_ref)
        point_cur = self._limb_kin.forward_position_kinematics()
        self.rec_point(point_cur)
        rot = euler2(point_cur[3:7])
        task_cur = point_cur[0:3].tolist() + rot
        e=map(lambda x,y:x-y, task_ref, task_cur)
        err = np.dot(self._K, np.matrix(e).transpose())
        self._path_offset = err

    def path_point(self, value, idx = 0):
        new = self._positions[-1]
        for index, entry in enumerate(self._positions):
            if value < entry[idx]:
                pr = self._positions[index-1]
                for j in range(len(pr)):
                    if j != idx:
                        new[j] = (pr[j] - entry[j])/(pr[idx] - entry[idx])*(value - pr[idx]) + pr[j]
                    else:
                        new[j] = value
                break
        else:
            index += 1
        return new

    def hand_over(self):
        self._is_hand_over = True
        warn = False
        while not self._gripper_open and not rospy.is_shutdown():
            if not warn:
                rospy.loginfo("Waiting for hand-over, Press <Enter> for handover")
                warn = True
            c = baxter_external_devices.getch()
            if c:
                self._gripper_open = True
                del c
                sys.stdout.flush()
                sys.stdin.flush()
                break
            self._rate.sleep()
        self.open_gripper()
        self._is_hand_over = False
        rospy.loginfo("~hand-over~")

    def wait(self, msg=None):
        warn = False
        while not rospy.is_shutdown():
            if not warn:
                if msg:
                    rospy.loginfo(msg+" press enter to continue")
                else:
                    rospy.loginfo("Waiting, press enter to continue")
                warn = True
            c = baxter_external_devices.getch()
            if c:
                del c
                sys.stdout.flush()
                sys.stdin.flush()
                break
            self._rate.sleep()
        rospy.loginfo("~wait is over~")

    def sleep(self, duration=60):
        for i in range(duration):
            self._rate.sleep()

    def _nav_b0_pressed(self, v):
        if v and self._is_hand_over:
            self._gripper_open = True

    def _nav_l0_pressed(self, v):
        self._started = not self._started


    def change_img_to_limb(self, x, y, direction, limb_pos):
        if direction == LATERAL:
            return (x - self.late_center[0])*late_ratio[0] + limb_pos[0], \
                   (self.late_center[1] - y)*late_ratio[1] + limb_pos[1]
        elif direction == LONGITUDINAL:
            return (y - self.long_center[1])*long_ratio[0] + limb_pos[0], \
                   (x - self.long_center[0])*long_ratio[1] + limb_pos[1]

    def open_gripper(self):
        self._gripper.open()
        self._rate.sleep()
        self._gripper_open = True

    def close_gripper(self):
        self._gripper.close()
        self._rate.sleep()
        self._gripper_open = False
        rospy.sleep(1)

    def pick_blob_pos(self):
        pos = self._pick_positions[self._pick_idx][:]
        pos[3] += .15
        joint_pos=self._limb_kin.inverse_kinematics(list(pos[1:4]), list(pos[4:8]))
        self.move_to_pos_raw(joint_pos, timeout=0.5, threshold = 0.008726646 * 3, joints=True, c1 = 0.95)
        for i in range(90):
            self._rate.sleep()
        self.move_to_pos_raw(self._pick_positions[self._pick_idx], timeout=.5, threshold = 0.008726646 * 2, c1 = 0.988)
        self._rate.sleep()
    	# added for video
    	pos[3] -= .17
        joint_pos=self._limb_kin.inverse_kinematics(list(pos[1:4]), list(pos[4:8]))
        self.move_to_pos_raw(joint_pos, timeout=0.5, threshold = 0.008726646 * 2, joints=True, c1 = 0.95)

        self._gripper.close()
        for i in range(90):
            self._rate.sleep()
        self._pick_idx += 1
        self._pick_idx %= len(self._pick_positions)

    def place_blob_pos(self):
        last_pos = self.joint_current()
        while self._collision_detected:
            self._rate.sleep()
        # print 'wait-!'
        for i in range(10):
            self._rate.sleep()
        pos = self._place_positions[self._place_idx]
        while self._collision_detected:
            self._rate.sleep()
        self.move_to_pos_raw(pos, timeout=.5, threshold = 0.008726646 * 3, c1 = 0.95)
        for i in range(10):
            self._rate.sleep()
        # print 'place-!'
        while self._collision_detected:
            self._rate.sleep()
        self.open_gripper()
        while self._collision_detected:
            self._rate.sleep()
        for i in range(50):
            self._rate.sleep()

        while self._collision_detected:
            self._rate.sleep()
        self.move_to_pos_raw(last_pos, timeout=.5, threshold = 0.008726646 * 3, c1 = 0.95)

        # to avoid the car console center
        while self._collision_detected:
            self._rate.sleep()
        self._rate.sleep()
        self._place_idx += 1
        self._place_idx %= len(self._place_positions)


def task_point(point):
    rot = euler2(point[4:9])
    tmp = point[1:4].tolist()+rot
    return tmp


def euler2(q):
    phi = atan(2*(q[0]*q[1]+q[2]*q[3])/(1-2*(q[1]*q[1]+q[2]*q[2])))
    theta = asin(2*(q[0]*q[2]-q[3]*q[1]))
    psi = atan(2*(q[0]*q[3]+q[1]*q[2])/(1-2*(q[2]*q[2]+q[3]*q[3])))
    eulers=[phi, theta, psi]
    return eulers


def arc_length(pre_pos, cur_pos):
    """
    :type pre_pos: forward kinematics position
    :type cur_pos: forward kinematics position
    """
    diffs = map(operator.sub, pre_pos[0:3], cur_pos[0:3])
    dist = np.linalg.norm(diffs)
    return dist


def quad_coefficient(x,y):
    x_1 = x[0]
    x_2 = x[1]
    x_3 = x[2]
    y_1 = y[0]
    y_2 = y[1]
    y_3 = y[2]
    a = y_1/((x_1-x_2)*(x_1-x_3)) + y_2/((x_2-x_1)*(x_2-x_3)) + y_3/((x_3-x_1)*(x_3-x_2))
    b = (-y_1*(x_2+x_3)/((x_1-x_2)*(x_1-x_3))
         -y_2*(x_1+x_3)/((x_2-x_1)*(x_2-x_3))
         -y_3*(x_1+x_2)/((x_3-x_1)*(x_3-x_2)))
    c = (y_1*x_2*x_3/((x_1-x_2)*(x_1-x_3))
        +y_2*x_1*x_3/((x_2-x_1)*(x_2-x_3))
        +y_3*x_1*x_2/((x_3-x_1)*(x_3-x_2)))
    return a,b,c


def pinv_damped(a, l, rcond=1e-15 ):
    """
    Compute the (Moore-Penrose) pseudo-inverse of a matrix.

    Calculate the generalized inverse of a matrix using its
    singular-value decomposition (SVD) and including all
    *large* singular values.

    Parameters
    ----------
    a : (M, N) array_like
      Matrix to be pseudo-inverted.
    l : float
      lamped lambda
    rcond : float
      Cutoff for small singular values.
      Singular values smaller (in modulus) than
      `rcond` * largest_singular_value (again, in modulus)
      are set to zero.

    Returns
    -------
    B : (N, M) ndarray
      The pseudo-inverse of `a`. If `a` is a `matrix` instance, then so
      is `B`.

    """
    a = a.conjugate()
    u, s, vt = np.linalg.svd(a, 0)
    m = u.shape[0]
    n = vt.shape[1]
    cutoff = rcond*np.maximum.reduce(s)
    # estimate of smallest singular value
    # s_m = 0.00065
    s_m = np.amin(s)
    # size of the singular region
    eps = .10
    if s_m > eps:
        lambda_2 =0
    else:
        lambda_2 = (1-(s_m/eps)**2)*l*l
    for i in range(min(n, m)):
        if s[i] > cutoff:
            s[i] = s[i]/(s[i]*s[i]+lambda_2)
        else:
            s[i] = 0.;
            print('singularity: ', s)
    res = np.dot(np.transpose(vt), np.multiply(s[:, np.newaxis], np.transpose(u)))
    return res


def calc_unit_vector(vec, eps):
    # l_x,l_y,l_z,l_rot_i, l_rot_j, l_rot_k, l_rot_w,
    result=np.zeros(6)
    vec_norm=np.linalg.norm(vec[0:3])
    if vec_norm>=eps:
        vec_x=vec[0:3]/vec_norm
    else:
        vec_x=None
    vec_e=[0, 0, 0]

    if vec_x==None:
        return None
    else:
        result[0:3]=vec_x[0:]
        result[3:]=vec_e[0:]
        return result


def calc_dir(line1, line2, eps):
    # l_arclength, l_x,l_y,l_z,l_rot_i, l_rot_j, l_rot_k, l_rot_w,
    # time,left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2,left_gripper,

    # position
    result = np.zeros(7)
    result[0] = line1[0]
    p1 = line1[1:4]
    p2 = line2[1:4]
    dp = p2-p1
    p_norm = np.linalg.norm(dp)

    # quaternion
    q1 = line1[4:]
    q2 = line2[4:]
    # print(q1)
    # print('q1=', q1)

    o1 = tf.transformations.euler_from_quaternion(q1)
    o2 = tf.transformations.euler_from_quaternion(q2)
    # print(o1)

    # orientation
    # o1 = euler(q1[3], q1[0], q1[1], q1[2])
    # o2 = euler(q2[3], q2[0], q2[1], q2[2])
    do = map(lambda x,y:x-y, o2, o1)
    o_norm = np.linalg.norm(do)
    # print(do)
    if p_norm >= eps:
        p_hat = dp/p_norm
        o_hat = do/p_norm
        result[1:4] = p_hat[0:]
        result[4:] = o_hat[0:]
        # result[4:] = [0, 0, 0]
        # print(result)
        ds = line2[0] - line1[0]
        grads = p_norm/ds
        # print grads
        return result, grads
    else:
        return None, None


def main():
    """Baxter motion in cartesian with desired path velocity

    """
    epilog = """
    Related examples:
    exp.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--filename', metavar='PATH',
        default=parrent_dir + '/scripts/configs/place_pos.csv',
        help='path to input file'
    )
    parser.add_argument(
        '-r', '--command_rate', type=int, default=100, metavar='COMRATE',
        help='rate at which to send commands (default: 100)'
    )
    parser.add_argument(
        '-e', '--emotion', type=bool, default=False, metavar='EMOTION',
        help='whether or not the robot displays emotion (default: False)'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Please Enter to begin")
    # raw_input()
    print("Initializing node... ")
    rospy.init_node("path_player")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    try:
        emotion = rospy.get_param('~emotion1')
    except KeyError:
        emotion = args.emotion
    emotion = True
    mover = PosPlayer(args.filename, args.command_rate, emotion)
    rospy.on_shutdown(mover.stop)
    mover.read_pick_up_pos(parrent_dir + '/scripts/configs/pick_pos.csv')

    print("Moving. Press Ctrl-C to stop.")
    mover.set_image(parrent_dir + '/img/idle.png')

    mover.move_to_pos(mover._joint_first, timeout=15)

    j = 0
    while not rospy.is_shutdown() and j < 6:
        j+=1
        mover.create_path_dir(axis=3, mid_offset=.05,direction=1)
        mover.move_tase(first_timeout=1)
        mover.pick_blob_pos()
        mover.create_path_dir(axis=3, mid_offset=.05, direction=0)
        mover.move_tase(first_timeout=1)
        mover.place_blob_pos()
    mover.create_path_dir(3, mid_offset=.10,direction=1)
    mover.set_status(1)
    mover.move_tase(first_timeout=1)

    print("\nDone.")


if __name__ == '__main__':
    main()