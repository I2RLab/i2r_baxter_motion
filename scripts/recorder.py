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
import baxter_pykdl
import operator
import numpy


def arc_length(pre_pos, cur_pos):
    """
    :type pre_pos: forward kinematics position
    :type cur_pos: forward kinematics position
    """
    diffs = map(operator.sub, pre_pos[0:2], cur_pos[0:2])
    dist = numpy.linalg.norm(diffs)
    return dist


class JointRecorder2(object):
    def __init__(self, filename, rate):
        """
        Records joint data to a file at a specified rate.
        """
        self._filename = filename
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False

        self._limb_left = baxter_interface.Limb("left")
        self._limb_right = baxter_interface.Limb("right")
        # forward kinematics
        self._limb_kin_left = baxter_pykdl.baxter_kinematics('left')
        self._limb_kin_right = baxter_pykdl.baxter_kinematics('right')
        self._arc_length_left = 0.0;
        self._arc_length_right = 0.0;

        self._gripper_left = baxter_interface.Gripper("left", CHECK_VERSION)
        self._gripper_right = baxter_interface.Gripper("right", CHECK_VERSION)
        self._io_left_lower = baxter_interface.DigitalIO('left_lower_button')
        self._io_left_upper = baxter_interface.DigitalIO('left_upper_button')
        self._io_right_lower = baxter_interface.DigitalIO('right_lower_button')
        self._io_right_upper = baxter_interface.DigitalIO('right_upper_button')

        self._io_left_lower.state_changed.connect(self._left_open_action)
        self._io_left_upper.state_changed.connect(self._left_close_action)

        # Use navigator for start and stop of recording
        self._nav_left = baxter_interface.Navigator('left')
        self._nav_left.button0_changed.connect(self._nav_b0_pressed)
        self._is_started = False
        self._pre_pos_left = None

        # Verify Grippers Have No Errors and are Calibrated
        if self._gripper_left.error():
            self._gripper_left.reset()
        if self._gripper_right.error():
            self._gripper_right.reset()
        if (not self._gripper_left.calibrated() and
                    self._gripper_left.type() != 'custom'):
            self._gripper_left.calibrate()
        if (not self._gripper_right.calibrated() and
                    self._gripper_right.type() != 'custom'):
            self._gripper_right.calibrate()

        # self._gripper.on_type_changed.connect(self._check_calibration)
        # self._open_io.state_changed.connect(self._open_action)
        # self._close_io.state_changed.connect(self._close_action)

    def _left_open_action(self, value):
        # if value and self._is_grippable():
        if value:
            rospy.logdebug("gripper open triggered")
            self._gripper_left.open()

    def _left_close_action(self, value):
        # if value and self._is_grippable():
        if value:
            rospy.logdebug("gripper close triggered")
            self._gripper_left.close()


    def _nav_b0_pressed(self, v):
        self._is_record = v
        # print ("Button 0: %s" % (v,))
        if v and self._is_started:
            self.record_data()

    def record_data(self):
        joints_left = self._limb_left.joint_names()
        # forward kinematics
        limb_pos_left = self._limb_kin_left.forward_position_kinematics()
        angles_left = [self._limb_left.joint_angle(j)
                       for j in joints_left]
        if numpy.any(self._pre_pos_left):
            self._arc_length_left += arc_length(self._pre_pos_left, limb_pos_left)
            print ("Another point is recorded.")
        else:
            print ("First point is recorded.")

        with open(self._filename, 'a') as f:
            f.write(str(self._arc_length_left) + ',')
            print(str(self._arc_length_left) + ',')
            f.write(','.join([str(x) for x in limb_pos_left]) + ',')

            f.write("%f," % (self._time_stamp(),))

            f.write(','.join([str(x) for x in angles_left]) + ',')
            f.write(str(self._gripper_left.position()) + '\n')

        self._pre_pos_left = limb_pos_left


    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def stop(self):
        """
        Stop recording.
        """
        self._done = True

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def record(self):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        This function does not test to see if a file exists and will overwrite
        existing files.
        """
        if self._filename:
            joints_left = self._limb_left.joint_names()
            joints_right = self._limb_right.joint_names()
            with open(self._filename, 'w') as f:
                # f.write('time,')
                # f.write(','.join([j for j in joints_left]) + ',')
                # f.write('left_gripper,')
                # f.write(','.join([j for j in joints_right]) + ',')
                # f.write('right_gripper\n')
                # f.write('right_gripper,')
                f.write('l_arclength, l_x,l_y,l_z,l_rot_i, l_rot_j, l_rot_k, l_rot_w,')
                f.write('time,')
                f.write(','.join([j for j in joints_left]) + ',')
                f.write('left_gripper\n')

                # start recording
                self._is_started = True

                # while not self.done():
                #     # Look for gripper button presses
                #     if self._io_left_lower.state:
                #         self._gripper_left.open()
                #     elif self._io_left_upper.state:
                #         self._gripper_left.close()
                #     if self._io_right_lower.state:
                #         self._gripper_right.open()
                #     elif self._io_right_upper.state:
                #         self._gripper_right.close()
                #     self._rate.sleep()

def main():
    """Modified from RSDK Joint Recorder Example

    Record timestamped joint and gripper positions to a file for
    later play back.

    Run this example while moving the robot's arms and grippers
    to record a time series of joint and gripper positions to a
    new csv file with the provided *filename*. This example can
    be run in parallel with any other example or standalone
    (moving the arms in zero-g mode while pressing the cuff
    buttons to open/close grippers).

    You can later play the movements back using one of the
    *_file_playback examples.
    """
    epilog = """
Related examples:
  joint_position_file_playback.py; joint_trajectory_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        # '-f', '--file', dest='filename', required=True,
        '-f', '--file', dest='filename', default='/home/baxter/ros_ws/src/i2r_motion_baxter/scripts/configs/??', required=False,  # part 1, type 1
        help='the file name to record to'
    )
    parser.add_argument(
        '-r', '--record-rate', type=int, default=100, metavar='RECORDRATE',
        help='rate at which to record (default: 100)'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("path_recorder")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    # arm = 'left'
    # grip_ctrls = GripperConnect(arm, False)

    # print("Press cuff buttons to control grippers. Spinning...")
    # rospy.spin()
    # print("Gripper Button Control Finished.")

    recorder = JointRecorder2(args.filename, args.record_rate)
    rospy.on_shutdown(recorder.stop)

    print("Recording. Press Ctrl-C to stop.")
    recorder.record()

    rospy.spin()
    print("\nDone.")


if __name__ == '__main__':
    main()
