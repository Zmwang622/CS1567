#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

pub = rospy.Publisher('/mobile_base/commands/velocity',Twist,queue_size = 10)

command = Twist()
command.linear.x = 0
command.angular.z = 0

target = Twist()
target.linear.x = 0
target.angular.z = 0

revved = False

def updateCommand(data):
    global revved, target, pub, command
    ascii_code = data.data
    if ascii_code == 119:
        if command.linear.x < 0:
            command.linear.x = 0
        temp  = command.linear.x + 0.05
        target.linear.x = min(0.8,temp)
    elif ascii_code == 115:    
        if command.linear.x > 0:
            command.linear.x = 0
        temp = command.linear.x - 0.05
        target.linear.x = max(-0.8, temp)
    elif ascii_code == 97:
        """
        if command.angular.z < 0:
            command.angular.z = 0
        target.angular.z = command.angular.z + 0.2
        target.angular.z = min(target.angular.z, 0.8)
        """
        if target.angular.z < 0:
            target.angular.z = 0
        else:
            target.angular.z = 0.8
    elif ascii_code == 100:
        """
        if command.angular.z > 0:
            command.angular.z = 0
        target.angular.z = command.angular.z - 0.2
        target.angular.z = max(target.angular.z, -0.8) 
        """
        if target.angular.z > 0:
            target.angular.z = 0
        else:
            target.angular.z = -0.8
    elif ascii_code == 120:
        if not revved:
            target.linear.x = 0.1
            pub.publish(command)
            revved = True
        if revved:
            target.linear.x = 0
            command.linear.x = 0
            target.angular.z = 0
            command.angular.z = 0
            pub.publish(command)
    
def cleanUp():
    global command, pub
    command.linear.x = 0
    command.angular.z =0
    pub.publish(command)
    rospy.sleep(1)

def almostEqual(x,y,epsilon = 1*10**(-8)):
    return abs(x-y) <= epsilon

def rover():
    global command,target,pub,revved
    rospy.init_node("smooth_rover", anonymous = True)
    rospy.Subscriber("ascii_node", Int32, updateCommand)
    rospy.on_shutdown(cleanUp)
    rospy.Rate(10)    

    while pub.get_num_connections()==0:
        pass

    while not rospy.is_shutdown():
        if revved:

            # Check if speed or angle equals its target  
            speedEqual = almostEqual(target.linear.x, command.linear.x)
            angleEqual = almostEqual(target.angular.z, command.angular.z)

            # Slowly increase the speed
            if speedEqual == False and target.linear.x > command.linear.x:
                command.linear.x += 0.05
                pub.publish(command)
                rospy.sleep(0.1)
            elif speedEqual == False and target.linear.x < command.linear.x:
                command.linear.x -= 0.05
                pub.publish(command)
                rospy.sleep(0.1)
               
            # Slowly increase the angle
            if angleEqual == False and target.angular.z < command.angular.z:
                command.angular.z = target.angular.z
                pub.publish(command)
                rospy.sleep(0.5)
                # Reset to 0
                #command.angular.z = 0
                # target.angudlar.z = 0
                angleEqual = True 

            elif angleEqual == False and  target.angular.z > command.angular.z:
                command.angular.z = target.angular.z
                pub.publish(command)
                rospy.sleep(0.5)
                # Reset to 0
                #command.angular.z = 0
                #target.angular.z= 0
        pub.publish(command)
        rospy.sleep(0.1)
       

if __name__ == "__main__":
    rover()
