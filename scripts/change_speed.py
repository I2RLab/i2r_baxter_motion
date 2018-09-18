#!/usr/bin/env python
__author__ = 'behzad'

# Modifed code of Rethink Robotics Recorder Exmaple

import argparse
import rospy
import baxter_interface
from baxter_interface import (
    DigitalIO,
    Gripper,
    Navigator,
    CHECK_VERSION,
)
import operator
import numpy
from std_msgs.msg import (
    Float64,
)
from i2r_baxter_motion.msg import RobotData
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def arc_length(pre_pos, cur_pos):
    """
    :type pre_pos: forward kinematics position
    :type cur_pos: forward kinematics position
    """
    diffs = map(operator.sub, pre_pos[0:2], cur_pos[0:2])
    dist = numpy.linalg.norm(diffs)
    return dist
current_dir = os.path.dirname(__file__)
parrent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
print (parrent_dir)


class SpeedChanger(object):
    def __init__(self):
        self._done = False
        self._ur = 0.75
        self._sub_ur = rospy.Subscriber('/trust/robot_states', RobotData, self._get_robot)
        self._pub_ur = rospy.Publisher('trust/ur_manual', Float64, queue_size=1)

        # Use navigator for start and stop of recording
        self._nav_right = baxter_interface.Navigator('right')
        self._nav_right.button0_changed.connect(self._nav_b0_pressed)

        self._nav_right.wheel_changed.connect(self._nav_wheel_changed)

        self._cvbridge = CvBridge()
        self._image = numpy.zeros((1024, 600, 3), numpy.uint8)

        self._pub_img = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)

    def _get_robot(self, data):
        self._ur = data.vel

    def _nav_wheel_changed(self, v):
        if v==1:
            self._ur -=.1
        elif v==-1:
            self._ur +=.1
        if self._ur>1:
            self._ur = 1
        if self._ur<0.3:
            self._ur = 0.3

        self._pub_ur.publish(self._ur)
        print ("Speed is : %s" % (self._ur,))

    def _nav_b0_pressed(self, v):
        pass


    def show_speed(self):
        img = self._image.copy()
        cv2.putText(img, 'Speed: {0:.2f}'.format(self._ur), (530, 380), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 4, cv2.CV_AA)
        self.send_cv_image(img)

    def send_cv_image(self,img):
        msg = self._cvbridge.cv2_to_imgmsg(img, "bgr8")
        self._pub_img.publish(msg)

    def set_image(self):
        self._image = cv2.imread(parrent_dir + '/img/idle.png')
        self.send_cv_image(self._image)
        rospy.sleep(1)



    def stop(self):
        """
        Stop recording.
        """
        # self.set_image()
        self._done = True

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done


def main():

    print("Initializing node... ")
    rospy.init_node("speed_changer")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    changer = SpeedChanger()
    rospy.on_shutdown(changer.stop)
    # changer.set_image()
    print("Change the speed of left arm motion using the wheel on the right arm.")


    rospy.spin()
    print("\nDone.")


if __name__ == '__main__':
    main()
