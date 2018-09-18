#!/usr/bin/env python
__author__ = 'behzad'
import argparse
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import baxter_move
import os
current_dir = os.path.dirname(__file__)
parrent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
print (parrent_dir)

def main():
    """baxter_move: usage example

    Reads a file for the position of the workspace (place position).
    Uses the player_tase to picks a part from the bin of parts
    and places it the shared workspace.

    Run recorder_1.py first to create a recording file for
    parts in the bin and a file for the position of the workspace.

    For Human-Robot Interaction run a launch file instead of this.
    """
    epilog = """
    Recorder:
    recorder_1.py;
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--filename', metavar='PATH',
        default = parrent_dir + '/scripts/configs/place_pos.csv',
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
    parser.add_argument(
        '-t', '--trust_path', type=bool, default=False, metavar='TRUST_PATH',
        help='whether or not the robot selects the path based on trust (default: False)'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("path_player")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    try:
        emotion = rospy.get_param('~emotion')
    except KeyError:
        emotion = args.emotion
    # emotion = True
    try:
        trust_path = rospy.get_param('~trust_path')
    except KeyError:
        trust_path = args.trust_path
    # trust_path = True
    mover = baxter_move.PosPlayer(args.filename, args.command_rate, emotion)
    mover.set_image(parrent_dir + '/img/start.png')
    rospy.on_shutdown(mover.stop)
    print("\nMoving. Press Ctrl-C to stop.")

    mover.read_pick_up_pos(parrent_dir + '/scripts/configs/pick_pos.csv')
    mover.move_to_pos(mover._joint_first, timeout=15)
    # mover._path_pos_0 = 0.0
    if trust_path or emotion:
        trust_threshold = 0.75
    else:
        trust_threshold = 0.0
    if not emotion:
        mover.set_image(parrent_dir + '/img/idle.png')
    j=0
    while not rospy.is_shutdown() and j < 6:
        j+=1
        if trust_path and mover._trust_mean<trust_threshold:
            mover.create_path_dir(mid_offset_vector = [-.05, 0, 0.05], mid_offset_knot=0.65, direction=1)
        else:
            mover.create_path_dir(axis=3, mid_offset=.05,direction=1)
        mover.reset_trust()
        mover.set_status(1)
        mover.move_tase(first_timeout=1)
        mover._path_pos_0 += 0.25
        mover._path.sr = mover._path_pos_0
        mover.set_status(2)
        mover.pick_blob_pos()
        if trust_path and  mover._trust_mean<trust_threshold:
            mover.create_path_dir(mid_offset_vector = [-.05, 0, 0.05], mid_offset_knot=0.35, direction=0)
        else:
            mover.create_path_dir(axis=3, mid_offset=.05, direction=0)
        mover.reset_trust()
        mover.set_status(1)
        mover.move_tase(first_timeout=1)
        mover.set_status(2)
        mover.place_blob_pos()
        mover._path_pos_0 += 0.25
        mover._path.sr = mover._path_pos_0
    mover.create_path_dir(3, mid_offset=.10,direction=1)
    mover.set_status(1)
    mover.move_tase(first_timeout=1)
    mover._path_pos_0 += 0.5

    print("\nDone.")


if __name__ == '__main__':
    main()