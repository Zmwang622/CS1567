#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from kobuki_msgs.msg import BumperEvent
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
roboCommand = Twist()
roboCommand.linear.x = 0
roboCommand.linear.y = 0

xPos = 0
yPos = 0
degree = 0

bumperStop = False

isPaused = False

def odomCallback(data):
    # Declare global variables
    global xPos, yPos, degree

    # Convert quaternion to degree
    q = [data.pose.pose.orientation.x,
         data.pose.pose.orientation.y,
         data.pose.pose.orientation.z,
         data.pose.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)

    # roll, pitch, and yaw are in radian
    degree = yaw * 180 / math.pi
    xPos = data.pose.pose.position.x
    yPos = data.pose.pose.position.y
    msg = "(%.6f,%.6f) at %.6f degree." % (xPos, yPos, degree) 
    #rospy.loginfo(msg)

def resetOdom():
    resetPub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    resetPub.publish(Empty())
    rospy.sleep(0.1)

def bumperCallback(data):
    global isPaused 
    isPaused = True  
    str = ""
    if data.bumper == 0:
        str = "Left bumper is "
    elif data.bumper == 1:
        str = "Central bumper is "
    else:
        str = "Right bumper is "
    if data.state == 0:
        str = str + "released."
    else: 
        str =str + "pressed."
    rospy.loginfo(str)

def batchDriving():
    global roboCommand, pub

    rospy.init_node('batchDriving', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumperCallback)
    rospy.Rate(1000)
    # Ensure a connection was established
    while pub.get_num_connections() == 0:
        pass

    # Reset odometry
    resetOdom()
    
    # Get user commands and insert them into a list
    print "Format of a command: (direction, speed, x, y, r)"
    print "Available directions: (m= move, r = rotate, c = circle)"
    commands = []
    command = ""
    while not command == "q":
        command = raw_input("Please enter your command: \n")
        commands.append(command)

    # Remove 'q' from the commands list 
    commands = commands[:-1]

    # Split the user command into parts
    for cmd in commands:
        resetOdom()
        cmd = cmd.split()
        direction = cmd[0]
        speed = float(cmd[1])
        x = float(cmd[2])
        r = float(cmd[3])

        if direction == "m":
            move(speed, x)
        elif direction == "c":
            circle(speed, r)
        elif direction == "r":
            rotate(r)
        elif direction == "h":
            circle(speed,r,1)
        else:
            print "invalid command"
        rospy.sleep(0.5) 
        
def move(speed, x):
    global roboCommand, pub, isPaused

    # Determine if speed is negative (going backwards)
    negStatus = -1 if speed < 0 else 1
    x *= negStatus
    
    # Set initial speed
    roboCommand.linear.x = 0
    pub.publish(roboCommand)

    # Accelerate in 3 parts
    step = x / 3

    print "Initial Position: ", xPos

    # First acceleration
    acceleration = ( (speed * speed) / (2 * step) ) / 10
    if negStatus > 0:    
        while xPos < step and isPaused == False:
            roboCommand.linear.x += acceleration
            roboCommand.linear.x = min(roboCommand.linear.x, speed)
            #print "linear.x: ", roboCommand.linear.x 
            pub.publish(roboCommand)
            rospy.sleep(0.1)
   
        print "Second Position ", xPos

        # Second acceleration
        while xPos < (step * 2) and isPaused == False:
            #print "linear.x: ", roboCommand.linear.x 
            pub.publish(roboCommand)
            rospy.sleep(0.1)
        
        print "Third Position: " , xPos

        # Deacceleration
        while xPos <= (step * 3) and isPaused == False:
            print "linear.x:  ", roboCommand.linear.x
            if roboCommand.linear.x == 0:
                break
            roboCommand.linear.x -= acceleration
            roboCommand.linear.x = max(0,roboCommand.linear.x)
            pub.publish(roboCommand)
            rospy.sleep(0.1)
    
    else: 
        # First acceleration
        while xPos > step and isPaused == False:
            roboCommand.linear.x += acceleration
            roboCommand.linear.x = max(roboCommand.linear.x,speed)
            #print "linear.x: ", roboCommand.linear.x
            pub.publish(roboCommand)
            rospy.sleep(0.1)
        
        print "Second Position ", xPos
        
        # Second Acceleration
        while xPos >  (step * 2) and isPaused == False:
            print "linear.x: ", roboCommand.linear.x
            pub.publish(roboCommand)
            rospy.sleep(0.1)
        
        print "Third Position", xPos
        
        # Third Acceleration
        while xPos > (step * 3) and isPaused == False:
            #print "linear.x: ", roboCommand.linear.x
            if roboCommand.linear.x == 0:
                break
            roboCommand.linear.x -= acceleration
            roboCommand.linear.x = min(0, roboCommand.linear.x)
            pub.publish(roboCommand)
            rospy.sleep(0.1)

        print "Fourth Position: ", xPos
        
    # Stop the robot
    roboCommand.linear.x = 0
    pub.publish(roboCommand)

def circle(speed, radius, half = 0):
    global pub, degree, roboCommand
    rTime = ( 2 * math.pi * radius) / speed
    roboCommand.linear.x = speed
    roboCommand.angular.z = (2 * math.pi) / rTime
    if half == 1:
        rTime /= 2
    print "rTime", rTime
    t0 = rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec() - t0) < (rTime * 1.25) and isPaused == False:
        if degree < 0:
            degree = 180 + (180 + degree)
        pub.publish(roboCommand)
    print "degree ", degree
    roboCommand.linear.x = 0
    roboCommand.angular.z = 0
    pub.publish(roboCommand)


def almostEqual(x,y, epsilon = 1*10**(-1)):
    return abs(x-y) <= epsilon

def rotate(r):
    # Declare global variables
    global pub, degree, roboCommand
    roboCommand.linear.x = 0
    # Move at a constant speed
    roboCommand.angular.z = 0.75
    pub.publish(roboCommand)
    rospy.sleep(0.1)
    print degree
    while isPaused == False and almostEqual(degree, r, epsilon = 1) == False:
         # print "degree: ", degree, "r: ", r
         if degree < 0:
             degree = 180 + (180 + degree)
         pub.publish(roboCommand)
    roboCommand.angular.z = 0
    pub.publish(roboCommand)
    print degree
if __name__ == '__main__':
   batchDriving()
