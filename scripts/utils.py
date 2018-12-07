import rospy


def count_down_clock(time):
    for i in range(time):
        print(time-i-1)
        rospy.sleep(1)