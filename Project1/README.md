# Sennott Square Rover

Imagine that you are responsible for writing a programs to control a rover in a distance (e.g., Mars
rovers). However, you are so far away to physically see the rover. The only eye you have is a single
camera mounting on top of your rover.

So, for this project, your rover is your Kobuki robot. The camera will be your smart phone unless
we can get a video conference program to work on an old Ubuntu. The goal are as follows:

1. Add the ability to control the robot in real time using another laptop viasshas a joystick
2. Send a batch of command to the robot (viassh) and let it executes
3. Add the ability to stop if a bumper is pressed

I will grade this project by letting your team control the rover to complete a series of tasks. Again,
your team will not be able to physically see the robot or the environment around it. The only
information you have is your phone’s camera. So, make sure your team can perform the following:

- Video conferencing between two mobile phones
- You must be able tosshto your give laptop

# Part I: Real-Time Driving (50 Points)

Recall that for this project, there are two laptops. Your laptop and the robot’s laptop. The robot’s
laptop will be connected to the robot. You must use your laptop to remotely control the robot via
ssh.

For this part, it will be a real-time controlling using your laptop as a joystick. Pick four keys to
be used to tell the robot to move forward, move backward, turn left, and turn right. Because we
want to be able to just press a key and make the robot movewithoutpressing the ”Enter” key,
we need to use thegetch()style. Consider the following Python program:

```
#!/usr/bin/python
```
```
import sys, tty, termios
```
```
print ’Press q to terminate the program’
ch = ’ ’
while not ch == ’q’:
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setraw(sys.stdin.fileno())
ch = sys.stdin.read(1)
termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
sys.stdout.write(ch)
#print ord(ch)
```
The above program will not wait for you to press the ”Enter” key. Once you press a key, it will
print the character associated with the key immediately. With some minor modification to the


above code you should be able to make your laptop behaves like a joystick.

Note that, if you want to use the arrow keys to drive your robot, you need to understand how
arrow keys are handled. When an arrow key is pressed, it is the same as a sequence of three keys
is pressed. Recall that each character has its associated ASCII value. You can convert a character
to ASCII value in Python using the functionord(). If you comment the second to last line of
the above code and uncomment the last line, when you press an up arrow once, you will see the
following on the console screen:

```
27
91
65
```
It tells you that three keys have been pressed, key 27 (ESC), key 91, and key 65. For each arrow
key, the first two are always 27 followed by 91. The last number indicates which arrow key has
been pressed where 65 is the up arrow key, 66 is the down arrow key, 67 is the right arrow key, and
68 is the left arrow key. Note that for the above code, it also supports press and hold. Try it.

Note that your robot must be able to move and turn at various speed. It must be able to move
forward and backward and turn at the same time as well. Again, the goal is for you to use your
computer as a remove joystick viasshin real time. You can use as many keys as you wish.

# Part II: Batch Driving (50 points)

For this part, your robot must be able to receive a sequence of commands without executing them
yet. Once all commands are received, your robot must follow all those command. Note that for
this part, we are going to focus onprecision. Therefore, using the odometry of the robot is a
must. At least, your robot must be able to perform the following:

- Move forward or backward with a given maximum speed (linear.xbetween -0.8 to 0.8) for
    a specific distance in meters (floating-point).
- Stationary turn left or right with a given maximum speed (angular.zbetween -1.0 to 1.0)
    for a specific angle in degrees (floating-point).
- Move forward in a circle (arch) with a given maximum speed (linear.xbetween -0.8 to 0.8)
    with a specific diameter in meters (floating-point).

You can design the interface whichever way you want as long as you can send a sequence of
commands to your robot and then tells it to execute those command.The number of commands
in the sequence should be one or more.

Since this will be a batch of commands, we need to put some safety into the robot.If your rover
bumps into something while it is moving, your robot must stop immediately, cancel
the current command (or series of command), and it must notifies the controller which
side of the rover (left, front, or right) bump to something. It is up to you whether you
want to use the constant command node or implement constantly publishing into your own node.

## Moving Behaviors

As you may notice, if you tell robot to move forward at speed 0.7, it will try to move at that speed
right away. This generally causes wheels to slip and it can even pop a wheelie. As a result, instead


of going straight, it may move to the left/right out of your desire destination. Similar situation also
happens when it tries to stop. When you tell the robot to stop, it will try to stop immediately. As a
result, it may move beyond your desire destination or facing a little bit off to the left/right instead
of straight. This type of problem also occurs when you turn. Note that when robot’s wheels slip,
the reading from odometry may be off by a lot.

To solve this problem, instead of moving at the desire speed right away and stop when it reaches
the destination, your robot should slowly accelerate to the desire speed. When it is getting close
to the destination, it should slows down and stop. The graph below show the speed of robot when
we tell the robot to move forward at the maximum speed of 0.75 for 1.5 meters.

```
0.
```
```
1.
```
```
0.5 1.0 1.5 Distance
```
```
Speed
```
Note that the robot should do the same when it tries to turn for a specific degrees.

This is almost the same asvelocitySmoother. However, for this project, it has to be controlled
directly from your control node since it needs to know where it is and when to stop. To get a very
precise start/stop, you may need to increase the publishing rate of yourconstantcommandnode.
For example, if you set the publishing rate to 10 Hz. It may take at most 100 ms for it to publish
the current command to the robot. Increase it to 100 Hz should do it.

## Research/Test

To move the robot in a circle with a specific radius, there must be a relation betweenlinear.x
andangular.z. Try to find out whether by researching or testing about the relation. You will be
asked to move the robot in a circle with a specificlinear.xvalue and a specific radius. You must
be able to figure it out what would be the value ofangular.z.

# Due Date and Submission

Zip all necessary files into one single file and submit it onto the CourseWeb before the due date
stated on the CourseWeb. You must demonstrate your project to Dr. Tan on the due date.


