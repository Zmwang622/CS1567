#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def send_command():
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.init_node('cmdExampleNode', anonymous=True)
    while pub.get_num_connections() == 0:
        pass
    command = Twist()
    command.linear.x = 0.5
    command.angular.z = 0.0
    rospy.loginfo(command)
    pub.publish(command)
    rospy.sleep(2)
    command.linear.x = 0.0
    pub.publish(command)
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        send_command()
    except rospy.ROSInterruptException:
        pass
