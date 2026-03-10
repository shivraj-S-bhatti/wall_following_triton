#!/usr/bin/env python3
"""Minimal keyboard teleop for Triton using the Project 2 frame."""

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


HELP = """
Triton teleop keys
------------------
w/s : forward/back along +Y/-Y
a/d : rotate left/right
j/l : strafe left/right along -X/+X
space or x : stop
q : quit
"""


def main():
    rospy.init_node("triton_key_teleop", anonymous=False)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    linear_speed = float(rospy.get_param("~linear_speed", 0.25))
    strafe_speed = float(rospy.get_param("~strafe_speed", 0.20))
    angular_speed = float(rospy.get_param("~angular_speed", 0.9))

    bindings = {
        "w": (0.0, linear_speed, 0.0),
        "s": (0.0, -linear_speed, 0.0),
        "a": (0.0, 0.0, angular_speed),
        "d": (0.0, 0.0, -angular_speed),
        "j": (-strafe_speed, 0.0, 0.0),
        "l": (strafe_speed, 0.0, 0.0),
        " ": (0.0, 0.0, 0.0),
        "x": (0.0, 0.0, 0.0),
    }

    print(HELP)
    old_attrs = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        rate = rospy.Rate(20)
        idle_sent = False
        while not rospy.is_shutdown():
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                if not idle_sent:
                    pub.publish(Twist())
                    idle_sent = True
                rate.sleep()
                continue

            key = sys.stdin.read(1)
            if key == "q":
                break
            if key not in bindings:
                continue

            linear_x, linear_y, angular_z = bindings[key]
            cmd = Twist()
            cmd.linear.x = linear_x
            cmd.linear.y = linear_y
            cmd.angular.z = angular_z
            pub.publish(cmd)
            idle_sent = False
            rate.sleep()
    finally:
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)


if __name__ == "__main__":
    main()
