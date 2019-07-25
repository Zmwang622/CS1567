#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)

turner = Twist()
turner.linear.x = 0 
turner.angular.z = 0 
lastTime = 0 # time t
lastError = 0 # error at time t
changeError = 0 # derivative
totalError = 0 # integral
p = 0.009  # proportion constant
i = 0 # integral constant
d = 0 # derivative constant

moveStatus = False

def calculateTrap(base,height1,height2):
    return (((height1+height2)/2) * base)

def blobCallback(data):
    global lastTime,changeError, totalError, lastError, moveStatus
    moveStatus = True
    xPos = data.data
    error = 320 - xPos
    newTime = rospy.Time.now().to_sec()
    changeError = (lastError - error)/(lastTime-newTime) # Derivative of e(t)
    totalError += calculateTrap(changeError,error,lastError) # this is integral(e(t))
    lastTime = newTime
    lastError = error # this is e(t)

def stop():
    global pub,turner
    turner.angular.z = 0
    turner.linear.x = 0
    pub.publish(turner)

def cleanUp():
    stop()
    rospy.sleep(1)

def pidControl():
    global p, i, d, lastError, changeError, totalError
    print (p * lastError + i * totalError + d * changeError)
    return (p * lastError + i * totalError + d * changeError)

def follow():
    global pub, turner, lastTime, moveStatus
    rospy.init_node('follower', anonymous = True)
    rospy.Subscriber('blob_move',Int32,blobCallback)
    rospy.on_shutdown(cleanUp)
    
    while pub.get_num_connections() == 0:
        pass

    while not rospy.is_shutdown():
    #angular.z =  p*e(t) + i*integral(e(t), 0, t) + d*(change in e/change in t) 
        if (rospy.Time.now().to_sec() - lastTime) >= 1:
            moveStatus = False    
        if moveStatus:
            turner.linear.x = 0.4
            turner.angular.z = pidControl()
            pub.publish(turner)
        else:
            stop()
if __name__ == "__main__":
    follow()

