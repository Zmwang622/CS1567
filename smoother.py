#!/usr/bin/env python
 
import rospy
import math
import time
from std_msgs.msg import Float32

def almostEqual(x,y, epsilon=1*10**(-8)):
   return abs(x-y) <= epsilon

def callback(data):
   global target
   target = round(data.data, 2)

def smoother():
   global target
   global curr
   target = 0
   curr = 0
   rospy.init_node("smoother", anonymous=True)
   rospy.Subscriber("command", Float32, callback)
   while not rospy.is_shutdown():
      if almostEqual(curr, target) == False:
         if curr < target:
            curr += 0.01
         elif curr > target:
            curr -= 0.01
      time.sleep(0.1)

if __name__ == '__main__':
   smoother()
