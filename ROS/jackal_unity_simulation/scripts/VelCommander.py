#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

def VelCommander():
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.init_node('VelCommander', anonymous=True)
    rate = rospy.Rate(1)
    linear = Vector3()
    linear.x = 0.5
    linear.y = 0.0
    linear.z = 0.0
    angular = Vector3()
    angular.x = 0.0
    angular.y = 0.0
    angular.z = 0.5
    command = Twist()
    command.linear = linear
    command.angular = angular
    log_str = "New command is being sent"
    while not rospy.is_shutdown():
        rospy.loginfo(log_str)
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        VelCommander()
    except rospy.ROSInterruptException:
        pass
