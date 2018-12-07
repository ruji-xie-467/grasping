#!/usr/bin/env python
import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

def calibrate_gripper(left):
    print('Calibrating...')
    left.calibrate()
    rospy.sleep(2.0)    
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    left = baxter_interface.Gripper('left', CHECK_VERSION)


def close_gripper(left):
    print('Closing...')
    rospy.sleep(1)
    left.close()


def open_gripper(left):
    print('Openning...')
    rospy.sleep(1)
    left.open()


# set_holding_force() method is used to set the force of 
# the grippers when the fingers come in contact with an object between them.

def offset_holding(gripper):
    if gripper.type() != 'electric':
        capability_warning(gripper, 'set_holding_force')
        return
    current = gripper.parameters()['holding_force']
    print("The previous force: ")
    print(current)
    offset = raw_input('Input offset:\n')
    gripper.set_holding_force(current + float(offset))
    after_set = gripper.parameters()['holding_force']
    print("Current force: ")
    print(after_set)



# set_dead_band() method is used to set the dead zone of the grippers. 
# Precisely, it refers to the minimum distance between the gripper fingers when they are in closed state.

def offset_dead_band(gripper):
    if gripper.type() != 'electric':
        capability_warning(gripper, 'set_dead_band')
        return
    current = gripper.parameters()['dead_zone']
    print("The previous min-distance: ")
    print(current)
    offset = raw_input('Input offset:\n')
    gripper.set_dead_band(current + float(offset))
    after_set = gripper.parameters()['dead_zone']
    print("Current min-distance: ")
    print(after_set)


if __name__ == "__main__":
    rospy.init_node('gripper_calibration', anonymous=False)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    left = baxter_interface.Gripper('left', CHECK_VERSION)
    while True:
        print("c: calibrate")
        print("a: open")
        print("b: close")
        print("d: dead_bind")
        print("f: set_force")
        print("e: exit")
        key = raw_input("Choose one:\n")

        if key == 'c':
            calibrate_gripper(left)
        elif key == 'a':
            open_gripper(left)
        elif key == 'b':
            close_gripper(left)
        elif key == 'd':
            offset_dead_band(left)
        elif key == 'f':
            offset_holding(left)
        elif key == 'e':
            break
        else:
            pass





