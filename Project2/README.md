# Project 2: Line Soccer

For this project, just imagine that you robot is a soccer player. First it has to be able to run
around the field and then kick the ball in to the goal.

# 1 Line Follower (40 Points)

For this project, you robot must be able to use its camera to move itself along a predefine line.
Note that you may have to tilt the camera downward for your robot to see the line right in front
of it. The line will consist of the following:

- straight sections
- various left and right curve
- various left and right angle turn (maximum turn will be limited at 90 degree)

The line will be a green tape which can be easily distinguish from the dark color carpet. It will be
place in the robotic lab for you to test your robot. Here are requirements for this part:

- Your robot must be able to follow the line from one end of a line to the other end.
- The forward speed can be a constant speed or variable speed but preferable not slower than
    0.2 forlinear.x.
- Your robot must show a smooth move from one end to the other. No shaking or sudden stops.

# 2 Soccer (60 Points)

For the second task, your robot will automatically kick the ball into the goal by itself without
human interaction. The ball will be a red ball and the goal will be a bright color inside another
bright color. I will set this up in the lab for you to test your robot. Your robot must be able to
find the ball and the goal by it self. It may need to scan 90 degree to the left and 90 degree to the
right. For this part, there are requirements:

- Yout robot can make as many stationary turns as you wish but only atthree locations
- Your robot can only make three forward/backward move.

To achieve this part, your robot must be able to make precise moves (e.g., exact amount of cen-
timeters or degrees).

CS 1567 – Programming and System Design on a Mobile Robot Platform Page 1


