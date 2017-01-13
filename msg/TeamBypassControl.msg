# Message that informs that the control should be bypassed for a robot.
#

std_msgs/Header header

# True if the control should not compute the pwm. One for each robot.
bool[3] bypass

# PWM signal to be used instead. One for each robot.
RobotPWM[3] robot_pwm