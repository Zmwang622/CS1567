#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

pub = rospy.Publisher("kobuki_command", Twist, queue_size=10)
command = Twist()
controls = Joy()

def joystickCallback(data):
    global pub, command, controls
    controls = data

def cleanUp():
    global pub, command
    command.linear.x = 0.0
    command.angular.z = 0.0
    pub.publish(command)
    rospy.sleep(1)

def remoteController():
    global command, controls
    rospy.init_node("remoteControl", anonymous=True)
    rospy.Subscriber("joy", Joy, joystickCallback)
    rospy.on_shutdown(cleanUp)
    
    # Initialize the butons and axes array
    controls.buttons = [0.0] * 11
    controls.axes = [0.0] * 8

    while pub.get_num_connections() == 0:       
       pass

    while not rospy.is_shutdown():
       # Set RT value
       command.linear.x = min(1 - controls.axes[5], 0.8)
       if controls.buttons[0] == 1:
          command.linear.x = command.linear.x*-1

       # Set rotation value
       command.angular.z = controls.axes[0]

       # Stop if A is clicked
       if controls.axes[2] == -1:
          command.linear.x = 0
          command.angular.z = 0

       # Quit if B is clicked
       if controls.buttons[1] == 1:
         command.linear.x = -2
         pub.publish(command)
#         rospy.signal_shutdown("Emergency Stop!!!")

       pub.publish(command)
    
    # rospy.spin()

if __name__ == '__main__':
    remoteController()
