#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist


pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
currentCommand = Twist()
currentCommand.linear.x = 0.0
currentCommand.angular.z = 0.0
targetCommand = Twist()
targetCommand.linear.x = 0.0
targetCommand.angular.z = 0.0

def updateCommand(data):
    global targetCommand
    targetCommand = data

def cleanUp():
    global currentCommand
    currentCommand.linear.x = 0.0
    currentCommand.angular.z = 0.0
    pub.publish(currentCommand)
    rospy.sleep(1)

def almostEqual(x,y, epsilon=1*10**(-8)):
   return abs(x-y) <= epsilon

def velSmoother():
    global pub, targetCommand, currentCommand
    rospy.init_node("velocitySmoother", anonymous=True)
    rospy.Subscriber("kobuki_command", Twist, updateCommand)
    rospy.on_shutdown(cleanUp)

    while pub.get_num_connections() == 0:
        pass

    while not rospy.is_shutdown():
        curr_lin_x = currentCommand.linear.x
        tar_lin_x = targetCommand.linear.x
        if tar_lin_x == -2:
            currentCommand.linear.x = 0
            rospy.signal_shutdown("Emergency stop!!!")
        elif almostEqual(curr_lin_x, tar_lin_x) == False:
            if curr_lin_x < tar_lin_x:
               currentCommand.linear.x += 0.06
            elif curr_lin_x > tar_lin_x:
               currentCommand.linear.x -= 0.06
        currentCommand.angular.z = targetCommand.angular.z
        pub.publish(currentCommand)
        rospy.sleep(0.1)
if __name__ == '__main__':
    velSmoother()

