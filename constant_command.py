#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led
from kobuki_msgs.msg import ButtonEvent
from kobuki_msgs.msg import BumperEvent

# Setup publishers
ledPub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)
veloPub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
queue_size=10)

# Initialize global variables
led = Led()
target = Twist()
target.linear.x = 0
target.angular.z = 0
isPaused = False
temp = Twist()
temp.linear.x = 0

def cleanUp():
    global temp
    veloPub.publish(temp)
    led.value = 0
    ledPub.publish(led)
    rospy.sleep(1)

def updateCommand(data):
   pass

def stopOperation():
    global isPaused
    isPaused = True

def resumeOperation():
    global isPaused
    isPaused = False

def moveCallback(data):
    global target
    target = data

def bumperCallback(data):
   # all bumpers
   if data.state != 0 :
      led.value = 3
      ledPub.publish(led)
      stopOperation()
def buttonCallback(data):
   if data.button == 0 and data.state == 1 and led.value == 1:
      led.value = 3
      ledPub.publish(led)
      stopOperation()
   elif data.button == 0 and data.state== 1 and  led.value == 3:
      led.value= 1
      ledPub.publish(led)
      resumeOperation()
def constantCommand():
    rospy.init_node('constant_command',anonymous = True)
    rospy.Subscriber("kobuki_command", Twist, updateCommand)
    rate = rospy.Rate(10) 
    rospy.Subscriber('/mobile_base/events/button', ButtonEvent, buttonCallback)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumperCallback)
    rospy.Subscriber('move', Twist, moveCallback)
    rospy.on_shutdown(cleanUp)
    while ledPub.get_num_connections() == 0:
        pass

    # Sets the light to Green
    led.value = 1
    ledPub.publish(led)

    while not rospy.is_shutdown():    
        if isPaused is False:
            print(target)
            veloPub.publish(target)
        else:
            # print("Hello")
            veloPub.publish(temp)
   
    stopOperation()
    led.value = 0
    ledPub.publish(led)

if __name__ == '__main__':
    try:
        constantCommand()
    except rospy.ROSInterruptException:
        pass
