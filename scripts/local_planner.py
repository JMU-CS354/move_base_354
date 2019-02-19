#!/usr/bin/env python
"""Local path planner using pure-pursuit.  

Subscribed Topics:
   path - (nav_msgs/Path)
     The path to follow.

   odom - (nav_msgs/Odometry)
      Odom messages are used to estimate the robot's current speed, which
      is needed to determine steering angles for pure pursuit.

   scan - (sensor_msgs/LaserScan)
      Laser scans are used to avoid colliding with obstacles that may be
      in the way of the path.

Published Topics:
    /cmd_vel_mux/input/navi - (geometry_msgs/Twist)
        Output velocity

    /target_marker - (visualization_msgs/Marker)
        Rviz marker illustrating current steering target.

Parameters:
    ~target_speed - (float, default: .4)

    ~goal_threshold - (float, default: .1)
       Planner will stop moving when this close to the end of the
       path.

    ~blocked_distance - (float, default: .5)
       Planner will stop at this distance from detected obstacles in
       the path.

    ~wheel_base - (float, default: .3)
       Distance between robot wheels.

Author: Nathan Sprague
Version: 2/19/19

"""
import threading
import numpy as np
import rospy
import tf2_ros

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

import tf2_geometry_msgs

import util
import pure_pursuit
from pure_pursuit import pure_pursuit_control


def make_sphere_marker(x, y, z, header,
                       ident=0, color=(1.,0.,0.), scale=.1):
    """ Create a sphere marker message at the indicated position. """
    marker = Marker()
    marker.header = header
    marker.ns = 'spheres'
    marker.id = ident
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.scale.x = marker.scale.y = marker.scale.z = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(0) # forever
    return marker

class LocalPlanner(object):
    """ 
    Handle path following.
    """

    def __init__(self):
        """ Set up the node, publishers and subscribers. """
        rospy.init_node('local_planner')

        rospy.Subscriber('path', Path, self.path_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.Subscriber('scan', LaserScan, self.scan_callback)

        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist,
                                       queue_size=10)
        self.target_marker_pub = rospy.Publisher('/target_marker',
                                                 Marker, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        self.target_speed = rospy.get_param('~target_speed', .4)
        self.goal_threshold = rospy.get_param('~goal_threshold', .1)
        self.blocked_distance = rospy.get_param('~blocked_distance', .5)
        self.wheel_base = rospy.get_param('~wheel_base', .3)
        self.path_lock = threading.Lock()
        self.path = None
        self.odom = None
        self.blocked = False

    def odom_callback(self, odom_msg):
        """ Save the odom message (used to monitor robot speed.)  """
        self.odom = odom_msg

    def scan_callback(self, scan_msg):
        """ Examine the latest scan to see if the path is blocked. """
        start = int(len(scan_msg.ranges) * .3)
        end = int(len(scan_msg.ranges) * .6)
        ranges = np.array(scan_msg.ranges[start:end])
        ranges = ranges[np.logical_not(np.isnan(ranges))]
        if len(ranges) > 0:
            self.blocked = np.min(ranges) < self.blocked_distance
        else:
            self.blocked = True

    def get_pursuit_state(self):
        """Get the pure pursuit state variable from the pure pursuit library.

        """
        cur_pose = self.get_current_pose()
        state = pure_pursuit.State(cur_pose.pose.position.x,
                                   cur_pose.pose.position.y,
                                   util.yaw_from_pose(cur_pose.pose),
                                   self.odom.twist.twist.linear.x)
        return state

    def path_callback(self, path_msg):
        """  Initialize pure pursuit when a new path is received. """
        self.path_lock.acquire()
        rospy.loginfo("Path received")
        if len(path_msg.poses) == 0:
            rospy.loginfo("Empty path. Ignoring.")
            self.path = None
        else:
            self.path = path_msg
            self.path_xs = [pose.pose.position.x for pose in path_msg.poses]
            self.path_ys = [pose.pose.position.y for pose in path_msg.poses]
            state = self.get_pursuit_state()
            self.path_index = pure_pursuit.calc_target_index(state,
                                                             self.path_xs,
                                                             self.path_ys)
            marker = make_sphere_marker(self.path_xs[self.path_index],
                                        self.path_ys[self.path_index],
                                        0.0, path_msg.header)
            self.target_marker_pub.publish(marker)
        self.path_lock.release()

    def get_current_pose(self):
        """Helper method to get the robot's current pose."""
        pose_base = tf2_geometry_msgs.PoseStamped()
        pose_base.header.frame_id = 'base_link'
        pose_base.header.stamp = rospy.get_rostime()
        pose_base.pose.orientation.w = 1.0
        return util.pose_transform(self.tf_buffer, pose_base, 'map')

    def run(self):
        """ Main loop.  Pure pursuit on the most recently received path. """

        rospy.loginfo("Local planner waiting for odometry")
        while self.odom is None and not rospy.is_shutdown():
            rospy.sleep(.1)
        rospy.loginfo("odometry received")

        rate = rospy.Rate(20)

        # This is the main loop.
        while not rospy.is_shutdown():
            self.path_lock.acquire()
            twist = Twist()

            if self.path is None:  # Bail if there is no path
                self.vel_pub.publish(twist)
                self.path_lock.release()
                rate.sleep()
                continue

            state = self.get_pursuit_state()
            ai = pure_pursuit.PIDControl(self.target_speed, state.v)
            di, self.path_index = pure_pursuit_control(state,
                                                       self.path_xs,
                                                       self.path_ys,
                                                       self.path_index)
            behind = False
            target = util.pose_transform(self.tf_buffer,
                                         self.path.poses[self.path_index],
                                         'base_link')

            if target.pose.position.x < 0 : # behind the robot!
                behind = True

            goal = self.path.poses[-1]
            dist = np.sqrt((state.x - goal.pose.position.x)**2 +
                           (state.y - goal.pose.position.y)**2)

            if dist < self.goal_threshold:
                twist.linear.x = 0
                # Rotate to the angle of the final pose...
                goal_in_base = util.pose_transform(self.tf_buffer,
                                                   goal, 'base_link')
                angle_error = util.yaw_from_pose(goal_in_base.pose)
                twist.angular.z = np.clip(angle_error * 3, -1, 1)
                
            elif behind or self.blocked:
                twist.linear.x = 0
                twist.angular.z = np.sign(di) * .5
                
            else:
                twist.linear.x = ai
                wheel_base = self.wheel_base
                omega = di * state.v / wheel_base
                twist.angular.z = omega # very approximate

            marker = make_sphere_marker(self.path_xs[self.path_index],
                                        self.path_ys[self.path_index],
                                        0.0, self.path.header)
            self.target_marker_pub.publish(marker)
            self.vel_pub.publish(twist)
            self.path_lock.release()
            rate.sleep()


if __name__ == "__main__":
    approach = LocalPlanner()
    approach.run()
