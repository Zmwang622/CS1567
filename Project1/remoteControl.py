#!/usr/bin/python

import sys, tty, termios
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

pub = rospy.Publisher('ascii_node', Int32, queue_size=10)

def remoteCtrl():
    rospy.init_node('newRemoteCtrl', anonymous = True)
    
    while pub.get_num_connections()==0:
        pass

    print 'Press q to terminate the program'
    ch = ' '
    while not ch == 'q':
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print ch
        pub.publish(ord(ch))
        
if __name__ == '__main__':
    try:
        remoteCtrl()
    except rospy.ROSInterruptionException:
        pass
