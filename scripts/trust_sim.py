#!/usr/bin/env python
__author__ = 'behzad'
import rospy
from geometry_msgs.msg import (
    Vector3
)
import math
from i2r_baxter_motion.msg import (
    Human,
    TrustData,
    RobotData
)
import numpy as np
from scipy import interpolate
import csv
current_dir = os.path.dirname(__file__)
parrent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
print (parrent_dir)


# please sync with model.cpp
# define a_t 0.8
# define b_t 0.1
# define c_t 0.1
a_t = 0.9
b_t = 0.05
c_t = 0.05
pr = .5
ph = .5
T = .5
S_R = 0.0
S_H = 0.0
pos_i = Vector3()
pos_f = Vector3()

reader = csv.reader(open(parrent_dir + "/config.txt", "rb"), delimiter=' ')
reader.next()
x = list(reader)
config_values = np.array(x).astype('float')
pos_i.x = config_values[0][0]
pos_i.y = config_values[0][1]
pos_i.z = config_values[0][2]
pos_f.x = config_values[1][0]
pos_f.y = config_values[1][1]
pos_f.z = config_values[1][2]
# self._center_pos = [-662, -3134, 1050]  # baxter center point in phasespace
xr = config_values[2][0]
yr = config_values[2][1]
zr = config_values[2][2]

v_h_max = 1500
_xm = (pos_i.x+pos_f.x)/2
_ym = (pos_i.y+pos_f.y)/2
_range = (math.pow(pos_i.x-pos_f.x,2) + math.pow(pos_i.y-pos_f.y,2))*2
dir_h=[pos_f.x-pos_i.x, pos_f.y-pos_i.y]
len_h=math.sqrt(dir_h[0]*dir_h[0] + dir_h[1]*dir_h[1])
dir_h[0]=dir_h[0]/len_h
dir_h[1]=dir_h[1]/len_h
len_h=len_h*2
w1 = 0.5
w2 = 0.5

x_ref = [0.0, 0.16*.5, 0.37*.5, 0.5*.5,  0.66*.5, 0.87*.5, 1.0*.5, 0.5 + 0.16*.5, 0.5+0.37*.5, 0.5+0.5*.5, 0.5+0.66*.5, 0.5+0.87*.5, 1.0]
y_ref = [0.0, 0.8,    0.60,     0.0,     0.8,      0.6,    0.0,    0.8,           0.6,         0.0,        0.8,         0.6,         0.0]

tck = interpolate.splrep(x_ref, y_ref, s=0)
# global pub_trust
# trust_states = Vector3()
trust_states = TrustData()
T_last = T
xh = pos_i.x
yh = pos_i.y
zh = pos_i.z
xdot = 0
ydot = 0
vh = 0
vh_ref = 0
vr_ref = 0
u_r = 0
N_p = 30
pr_data = np.ones(N_p)*pr
ph_data = np.ones(N_p)*ph
# pr_data = range(0,10,1)


def trust_model(T_pre):
    global T
    global pr_data, pr_data_N, pr
    global ph_data, ph_data_N, ph
    T = a_t*T_pre+b_t*pr+c_t*ph
    trust_data = TrustData()
    trust_data.trust = T
    trust_data.pr_data = pr_data
    trust_data.pr = pr
    trust_data.ph_data = ph_data
    trust_data.ph = ph
    return trust_data
    # pub_trust.publish(trust_states)


def robotCallback(data):
    global P_R, S_R
    global u_r, vr_ref
    global pr_data, pr
    global xr, yr, zr
    S_R = data.sr
    status_r = data.status
    u_r = data.vel
    xr = data.xr
    yr = data.yr
    zr = data.zr
    # if status_r-2<0.01:         #pickup
    if abs(status_r-2)<0.01 or abs(status_r-3)<0.01:         #pickup
        r_status_coef=[0.0, 0.5]
    else:                       #motion
        r_status_coef=[1.0, 0.0]

    s_diff = abs(S_R - S_H)
    if s_diff > 1:
        s_diff = 1.0
    vr_ref = v_r_ref(S_R)
    P_R = 1-(1-w2)*r_status_coef[0]*abs(u_r-vr_ref)-w2*s_diff
    pr_data = np.roll(pr_data,1)
    pr_data[0] = P_R
    pr = np.sum(pr_data)/N_p


def humanCallback(data):
    global S_H, P_H
    global xh, yh, xdot, ydot, vh, vh_ref, zh
    global ph_data, ph_data_N, ph
    S_H = data.sh
    dir_sgn  = data.dir_sgn
    xdot = data.xd0
    ydot = data.yd0
    # s_diff = max([S_R - S_H, 1.0])
    s_diff = S_R - S_H
    if s_diff < 0.0:
        s_diff = 0
    if s_diff > 1:
        s_diff = 1
    # P_H = 1-(1-w1)*abs(1-dir_sgn*(dir_h[0]*xdot+dir_h[1]*ydot)/v_h_max)-w1*s_diff*u_s_diff
    vh = abs(dir_h[0]*xdot+dir_h[1]*ydot)/v_h_max
    vh_ref = v_h_ref(S_H)
    # P_H = 1-(1-w1)*abs(vh-vh_ref)-w1*s_diff
    P_H = 1-(1-w1)*abs(vh-vh_ref)-w1*s_diff
    xh = data.x0
    yh = data.y0
    zh = data.z0
    ph_data = np.roll(ph_data,1)
    ph_data[0] = P_H
    ph = np.sum(ph_data)/N_p
    # ph = P_H


def v_r_ref(s_r):
    return interpolate.splev(s_r % 1, tck, der=0)


def v_h_ref(s_h):
    return interpolate.splev(s_h % 1, tck, der=0)


def record_data(filename):
    global pr, ph, T
    global S_H, P_H
    global xh, yh, xdot, ydot, vh, vh_ref, zh
    global xr, yr, zr
    global P_R, S_R
    global u_r, vr_ref
    with open(filename, 'a') as f:
        f.write("%f," % (time_stamp(),))
        f.write(str(xh) + ',' + str(xdot) + ',' + str(yh) + ',' + str(ydot) + ',' + str(zh) + ',' +
                str(xr) + ',' + str(yr) + ',' + str(zr) + ',' +
                str(S_H) + ',' + str(S_R) + ',' + str(ph) + ',' + str(pr) + ',' +
                str(T) + ',' +
                str(u_r)+',' + str(vr_ref) + ',' + str(vh) + ',' + str(vh_ref) + '\n')


def time_stamp():
    global start_time
    return rospy.get_time() - start_time

def trustupdate():
    global pub_trust, T_last
    rospy.init_node('trust_sim', anonymous=True)
    pub_trust = rospy.Publisher('/trust/trust', TrustData, queue_size=1)
    # rospy.Subscriber('/trust/trust_states', Vector3, trustCallback, queue_size=1)
    rospy.Subscriber('/trust/robot_states', RobotData, robotCallback, queue_size=1)
    rospy.Subscriber('/trust/human_states', Human, humanCallback,  queue_size=1)
    r = rospy.Rate(10)
    try:
        filename = parrent_dir + '/data/' +rospy.get_param('~log_file')
    except KeyError:
        filename = parrent_dir + '/data/Subject_XX_Type_X.csv'

    with open(filename, 'w') as f:
        f.write('time_stamp, xh, xh_dot, yh, yh_dot, zh, xr, yr, zr, sh, sr, ph, pr, trust, u, ')
        f.write('vr_ref, vh, vh_ref\n')
    global start_time
    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        t_states = trust_model(T_last)
        pub_trust.publish(t_states)
        record_data(filename)
        # print t_states
        T_last = t_states.trust
        r.sleep()
    # rospy.spin()


if __name__ == '__main__':
    trustupdate()