#!/usr/bin/env python3

import rospy
from franka_msgs.msg import ErrorRecoveryActionGoal

if __name__ == '__main__':
    rospy.init_node('franka_automatic_error_recovery')
    pub = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=1)
    msg = ErrorRecoveryActionGoal()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.loginfo('Trying recovery')
        r.sleep()

