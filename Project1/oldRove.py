#!/usr/bin/env python

import rospy
import math
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

pub = rospy.Publisher('/mobile_base/commands/velocity',Twist,queue_size = 10)

# Store the current command
command = Twist()
command.linear.x = 0
command.angular.z = 0

# Store the target commnand
target = Twist()
target.linear.x = 0
target.angular.z = 0

# Keep track of whether a command was received
received = False

def updateCommand(data):
    global target, command, pub, received
    received = True
    ascii_input = data.data
    if ascii_input == 119:
        temp_lin = command.linear.x + 0.05
        target.linear.x = min(0.8, temp_lin)
    elif ascii_input == 115:
        temp_lin = command.linear.x - 0.05
        target.linear.x = max(-0.8, temp_lin)
    elif ascii_input == 97:
        target.angular.z += 0.8
        target.angular.z = min(0.8, target.angular.z)
    elif ascii_input == 100:
        target.angular.z -= 0.8
        target.angular.z = max(-0.8, target.angular.z)

def bumperCallback(data):
    global command, pub
    if data.state == 1:
        command.linear.x = 0
        command.angular.z = 0
        pub.publish(command)
        

def cleanUp():
    global command, pub
    command.linear.x = 0
    command.angular.z = 0
    pub.publish(command)
    rospy.sleep(1)

def almostEqual(x, y, epsilon=1*10**(-8)):
    return abs(x-y) <= epsilon    

def rover():
    global command, target, pub, received
    rospy.init_node("newRover", anonymous = True)
    rospy.Subscriber("ascii_node", Int32, updateCommand)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumperCallback)
    rospy.on_shutdown(cleanUp)

    # Ensure a connection was established
    while pub.get_num_connections() == 0:
        pass

    while not rospy.is_shutdown():
        # If a key was pressed
        if received == True:
            # Control forward and backwards movement
            while almostEqual(command.linear.x, target.linear.x) == False:
                # Slowly move forward
                if target.linear.x > command.linear.x:
                    print 'w'
                    command.linear.x += 0.05
                    pub.publish(command)
                # Slowly move backward
                elif target.linear.x < command.linear.x:
                    print 's'
                    command.linear.x -= 0.05
                    pub.publish(command)
            # Control rotation 
            while almostEqual(command.angular.z, target.angular.z) == False:
                if target.angular.z > command.angular.z:
                    print 'a'
                    command.angular.z += 0.8
                    pub.publish(command)
                elif target.angular.z < command.angular.z:
                    print 'd'
                    command.angular.z -= 0.8
                    pub.publish(command)
            pub.publish(command)
        else:
            target.linear.x = 0
            if command.linear.x > 0:
                # Decelerate forward motion
                while command.linear.x != 0 and  command.linear.x > 0:
                    command.linear.x -= 0.01
                    rospy.sleep(0.005)
                    pub.publish(command)
            else:
                # Decelerate backwards motion
                while command.linear.x != 0 and command.linear.x < 0:
                    command.linear.x += 0.01
                    rospy.sleep(0.005)
                    pub.publish(command)

            # Reset the speed to 0
            command.linear.x = 0
            command.angular.z = 0
            target.angular.z = 0
            pub.publish(command)

        # Reset received parameter
        received = False
        rospy.sleep(0.1)

if __name__ == "__main__":
    rover()
