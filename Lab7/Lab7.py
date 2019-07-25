#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Image
from struct import *
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
dist = 0

moveStatus = False

depthData = Image();
isDepthReady = False;

def depthCallback(data):
    global depthData, isDepthReady
    depthData = data
    isDepthReady = True

def blobCallback(data):
    global lastTime,changeError, totalError, lastError, moveStatus, veloError, dist
    moveStatus = True
    xPos = data.data
    error = 320 - xPos
    newTime = rospy.Time.now().to_sec()
    changeError = (lastError - error)/(lastTime-newTime) # Derivative of e(t)
    lastTime = newTime
    lastError = error # this is e(t)
    veloError = dist - 1 

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

def veloPid(): 
    global p, i, d, veloError, dist 
    print (0.8 * veloError)
    return (0.55 * veloError)

def main():
    global depthData, isDepthReady, pub, turner, lastTime, moveStatus, dist
    rospy.init_node('depth_example', anonymous=True)
    rospy.Subscriber('blob_move',Int32,blobCallback)
    rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)
    rospy.on_shutdown(cleanUp)

    while not isDepthReady:
        pass

    while not rospy.is_shutdown():
        step = depthData.step
        midX = 320
        midY = 240
        offset = (240 * step) + (320 * 4)
        (dist,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
        if (rospy.Time.now().to_sec() - lastTime) >= 1: 
            moveStatus = False
        print "Distance: %f" % dist

        if moveStatus:
            print "velo PID: ", veloPid()
            turner.linear.x = veloPid() 
            turner.angular.z = pidControl()
            pub.publish(turner)
        else:
            stop()

if __name__ == '__main__':
    main()
