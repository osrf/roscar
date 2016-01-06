#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Twist

class RosCar:

    def __init__(self):
        rospy.init_node('roscar')
        # Control modes
        self.modes = ['teleop', 'follow']
        self.mode_idx = 0
        # Overall config
        self.max_abs_linvel = 1.5
        self.max_abs_angvel = 0.5
        # Teleop config
        self.x_axis = 1
        self.y_axis = 0
        self.last_teleop_time = None
        self.last_teleop_cmd = None
        self.teleop_timeout = rospy.Duration.from_sec(5.0)
        # Follower config
        self.angle_min = -math.pi/4.0
        self.angle_max = math.pi/4.0
        self.goal_x = 0.6
        self.x_scale = 2.0
        self.y_scale = 7.0
        # Subs and pubs
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_cb, queue_size=1)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_cb, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.05), self.timer_cb)

    def timer_cb(self, data):
        if self.modes[self.mode_idx] != 'teleop':
            return
        if self.last_teleop_cmd is not None and self.last_teleop_time is not None:
            if (rospy.Time.now() - self.last_teleop_time) <= self.teleop_timeout:
                self.cmd_vel_pub.publish(self.last_teleop_cmd)

    def joy_cb(self, msg):
        if msg.buttons[5] == 1:
            idx = (self.mode_idx + 1) % len(self.modes)
            print('Switching from %s to %s'%(self.modes[self.mode_idx], self.modes[idx]))
            self.mode_idx = idx

        if self.modes[self.mode_idx] != 'teleop':
            return

        vx = msg.axes[self.x_axis] * self.max_abs_linvel
        vyaw = msg.axes[self.y_axis] * self.max_abs_angvel
        self.publish_cmd(vx, vyaw)

    def scan_cb(self, msg):
        if self.modes[self.mode_idx] != 'follow':
            return
        self.scan = msg
        i = 0
        x = 0
        y = 0
        n = 0
        while i < len(msg.ranges):
            angle = msg.angle_min + i*msg.angle_increment
            if angle >= self.angle_min and angle <= self.angle_max:
                px = msg.ranges[i] * math.cos(angle)
                py = msg.ranges[i] * math.sin(angle)
                x += px
                y += py
                n += 1
            i += 1
        x /= n
        y /= n
        vx = (x - self.goal_x) * self.x_scale
        vyaw = y * self.y_scale
        self.publish_cmd(vx, vyaw)

    def publish_cmd(self, vx, vyaw):
        cmd = Twist()
        cmd.linear.x = min(vx, max(vx, -self.max_abs_linvel))
        cmd.angular.z = min(vyaw, max(vyaw, -self.max_abs_angvel))
        self.cmd_vel_pub.publish(cmd)
        if self.modes[self.mode_idx] == 'teleop':
            self.last_teleop_cmd = cmd
            self.last_teleop_time = rospy.Time.now()

    def go(self):
        rospy.spin()

if __name__ == '__main__':
    rc = RosCar()
    rc.go()
