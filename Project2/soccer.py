#!/usr/bin/env python

import rospy
import math
from math import radians, degrees
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from cmvision.msg import Blobs, Blob
from geometry_msgs.msg import Twist

from std_msgs.msg import Int32, Empty

# Setup kobuki robot publisher
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
turner = Twist()
turner.linear.x = 0
turner.angular.z = 0

test = 0

# Initialize variables
ballFirst = False
goalFirst = False
alpha = 0
alpha_count = 0
beta = 0
beta_count = 0
alpha_prime = 0
beta_prime = 0
degree = 0
findObj = 0
p = 0
X = 0
R = 0

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
    # Reset odometry by publishing an empty message
    resetPub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    resetPub.publish(Empty())
    rospy.sleep(0.5)

def turnerCallback(data):
    global alpha, alpha_prime, beta,beta_prime, alpha_count, beta_count, ballFirst, goalFirst, test

    # Retrieve x position of blobs
    pinkX = data.blobs[0].x
    redX = data.blobs[1].x

    # Goal is in the middle of the screen
    if pinkX >= 300 and pinkX <= 340:
        #if ballFirst == False:
            #goalFirst = True
        if test == 1:
            alpha_count+=1
            alpha += degree
        if test == 2:
            alpha_prime += degree
            alpha_count+=1
        

    # Ball is in the middle of the screen
    if redX >= 300 and redX <= 340:
        #if goalFirst == False:
         #   ballFirst = True
        if test == 1:
            beta += degree
            beta_count += 1
        if test == 2:
            beta_prime += degree
            beta_count += 1

def almostEqual(x,y, epsilon = 1*10**(-1)):
    return abs(x-y) <= epsilon

def rotate(angle):
    # Reset odometry
    resetOdom()

    # Handle negative angles (i.e. rotations)
    negStatus = -1 if angle < 0 else 1

    # Declare global variables
    global pub, degree, turner
    turner.linear.x = 0

    # Move at a constant speed
    turner.angular.z = 0.5 * negStatus
    pub.publish(turner)
    rospy.sleep(0.1)

    # print "1) degree: ", degree
    while almostEqual(degree, angle, epsilon = 1) == False:
         #print "degree: ", degree, "r: ", angle
         if degree < 0:
             degree = 180 + (180 + degree)
         pub.publish(turner)

    turner.angular.z = 0
    pub.publish(turner)
    # print "2) degree: ", degree

def move(dist, speed = False):
    resetOdom()

    # Set initial speed
    turner.linear.x = 0.8 if speed==True else 0.25
    turner.angular.z = 0

    # Move the robot at a specific distance 'dist'
    while xPos < dist:
        pub.publish(turner)
        rospy.sleep(0.1)
    
    # Reset speed to 0
    turner.linear.x = 0
    pub.publish(turner)
    rospy.sleep(0.1)

def calculateRight():
    global alpha, beta, alpha_prime, beta_prime, X, R, p, ballFirst, goalFirst
    a = 180 - alpha_prime
    g = 180 - (alpha + a)
    print "Alpha: ", alpha
    print "Beta: ", beta
    print "Alpha Prime: ", alpha_prime
    print "Beta Prime: ", beta_prime
    Q = math.sin(radians(alpha)) / math.sin(radians(g))
    b = 180 - beta_prime
    d = 180 - (beta + b) 
    K = math.sin(radians(b)) / math.sin(radians(d))
    N = math.sin(radians(a)) / math.sin(radians(g))
    h = alpha - beta
    O = math.sqrt( (N*N) + (K*K) - ((2 * (N*K)) * math.cos(radians(h))))
    GM = degrees( math.asin( (math.sin(radians(h)) * K) / O ) )
    m = 0
    if (GM > g):
        m = GM - g
    else:
        m = g - GM
    p = 180 - (m + alpha_prime) 
    if p < 0:
        p = -p
    y = 180 - (beta_prime + p)
    if y < 0:
        y = -y
    F = (math.sin(radians(beta)) * K) / math.sin(radians(b))
    X = (math.sin(radians(y)) * F) / math.sin(radians(p))
    #X = (math.sin(m) * Q) / math.sin(p)
    print "Alpha: ", alpha
    print "Beta: ", beta
    print "Alpha Prime: ", alpha_prime
    print "Beta Prime: ", beta_prime
    print "X: ", X
    print "y: ", y
    print "Q: ", Q
    print "M: ", m
    print "G: ", g
    print "GM: ", GM
    print "O: ", O
    R = (math.sin(radians(alpha_prime)) * X) / math.sin(radians(y))

def calculateLeft():
    global alpha, beta, alpha_prime, beta_prime, X, p, R
    alpha_prime = abs(180 - alpha_prime)
    beta_prime = abs( 180 - beta_prime)
    print "Alpha: ", alpha
    print "Beta: ", beta
    print "Alpha Prime: ", alpha_prime
    print "Beta Prime: ", beta_prime
    a = 180 - beta
    b = 180 - alpha
    c = 180 - alpha_prime
    d = 180 - beta_prime
    e = 180 - (beta_prime + a)
    g = 180 - (alpha_prime + b)
    f = 180 - (g+b)
    h = 180-f
    i = 180 - (e+f)
    J = math.sin(radians(a))/math.sin(radians(e))
    K = math.sin(radians(alpha_prime))/ math.sin(radians(g))
    L = (J*math.sin(radians(i))) / math.sin(radians(f))
    n = b - a
    O = (K * math.sin(radians(n))) / math.sin(radians(f))
    Q = math.sqrt( (O*O) + (L*L) - (( 2 * O * L) * math.cos(radians(h))))
    m = degrees(math.asin( (math.sin(h) * L) / Q))
    p = 180 - (m + c)
    s = 180 - (p + d)
    R = (J * math.sin(radians(d))) / math.sin(radians(p))
    X = (R * math.sin(radians(s))) / math.sin(radians(d))
    
    if p < 0:
        p = -p

    print "X: ", X
    print "R: ", R
    print "p: ", p
def cleanUp():
    global turner, pub
    turner.angular.z = 0
    pub.publish(turner)
    rospy.sleep(1)

def soccer():
    global test, pub, turner, alpha, beta, alpha_prime, beta_prime, X, R, p, alpha_count, beta_count
    rospy.init_node('turner',anonymous=True)
    rospy.Subscriber('blob_merger',Blobs, turnerCallback)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rospy.on_shutdown(cleanUp)
    lastTime = rospy.Time.now().to_sec()

    # Ensure a connection was established
    while pub.get_num_connections() == 0:
        pass

    # Reset odometry
    resetOdom()
    
    # Test 1: Find alpha and beta
    rotate(-90)
    test += 1
    rospy.sleep(0.1)
    rotate(180)
    
    goalFirst = True if alpha < beta else False
    if goalFirst == False:
        rotate(180)
    resetOdom()
    #if goalFirst == True:
        #rotate(180)
     
    # Test 2: Move 1 meter to get a default distance
    move(1)
    resetOdom()
    test += 1 

    # Calculate averages for alpha and beta
    alpha = alpha / alpha_count
    beta = beta / beta_count
    print "VALUE OF ALPHA: ", alpha
    print "ALPHA_COUNT ", alpha_count
    alpha_count = 0
    beta_count = 0
    # Test 3: Find alpha and beta prime
    if goalFirst == True:
        resetOdom()
        rotate(-180)
    else:
        rotate(180)
    test += 2

    # Calculate averages for alpha_prime and beta_prime
    alpha_prime = alpha_prime / alpha_count
    beta_prime = beta_prime / beta_count
    
    if alpha_prime == beta_prime:
        alpha_prime+=1

    # Find distance to move and rotate in order the ball into the goal
    if goalFirst == True:
        calculateLeft()
    else:
        calculateRight()

    if p < 0:
        p = -p

    resetOdom()

    if goalFirst == True:
        if X > 0:
            rotate(180)
        if R < 0:
            R *= -1

    if X > 0:
        rotate(180)
        p = 180 - p
    elif X < 0:
        X *= -1
    print "X: ", X
    print "p: ", p
    print "R: ", R
    move(X)
    rotate(p)
    rospy.sleep(0.1)
    move(R, True)
    while not rospy.is_shutdown():
        pass

if __name__ == "__main__":
    soccer()



