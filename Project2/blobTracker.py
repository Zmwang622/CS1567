#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
turner = Twist()
turner.linear.x = 0
turner.angular.z = 0
lastTime = 0
moveStatus = 0

def turnerCallback(data):
    global moveStatus, lastTime
    xPos = data.data
    lastTime = rospy.Time.now().to_sec()
    if xPos >= 310 and xPos <= 330:
        moveStatus = 0
    elif xPos >=310:
        moveStatus = 1
    else:
        moveStatus = -1

def cleanUp():
    global turner, pub
    turner.angular.z = 0
    pub.publish(turner)
    rospy.sleep(1)

def react():
    global pub, turner,moveStatus, lastTime
    rospy.init_node('turner',anonymous=True)
    rospy.Subscriber('blob_move',Int32, turnerCallback)
    rospy.on_shutdown(cleanUp)
    lastTime = rospy.Time.now().to_sec()

    while pub.get_num_connections() == 0:
        pass

    while not rospy.is_shutdown():
        if (rospy.Time.now().to_sec() - lastTime) >= 1:
            moveStatus = 0
        if moveStatus == 0:
            turner.angular.z = 0
            pub.publish(turner)
        elif moveStatus < 0:
            turner.angular.z = 0.65
            pub.publish(turner)
        else:
            turner.angular.z = -0.65
            pub.publish(turner)


if __name__ == "__main__":
    react()



