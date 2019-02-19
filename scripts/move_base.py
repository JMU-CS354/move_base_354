#!/usr/bin/env python
"""Simple action server interface to the 354 pure-python navigation
library.

Action API
    Action Subscribed Topics

    move_base/goal (move_base_msgs/MoveBaseActionGoal)
        A goal for move_base to pursue in the world.

    move_base/cancel (actionlib_msgs/GoalID)
        A request to cancel a specific goal.

    Action Published Topics

    move_base/feedback (move_base_msgs/MoveBaseActionFeedback)
        Feedback contains the current position of the base in the world.

     move_base/status (actionlib_msgs/GoalStatusArray)
        Provides status information on the goals that are sent to the
        move_base action.

    move_base/result (move_base_msgs/MoveBaseActionResult)
        Result is empty for the move_base action.

Subscribed Topics:
   move_base_simple/goal - (geometry_msgs/PoseStamped)
     Provides a non-action-client based mechanism for sending
     navigation goals.  This can be used to send goals through the "2d
     Nav Goal" button in Rviz.  It doesn't allow for monitoring the
     progress of navigation.


Parameters:
    ~xy_goal_tolerance - (float, default .15)
       How close we need to get to the goal before declaring success.

    ~stuck_delay - (float, default 10.0)
       How long we wait to declare failure if no progress is being made.


(Some documentation borrowed from http://wiki.ros.org/move_base
Creative Commons Attribution 3.0)

"""

from collections import deque
import rospy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback
from geometry_msgs.msg import PoseStamped
import tf2_ros
import actionlib

import util

class MoveBase(object):
    """
    Class handling navigation logic for a simple-action-server-based
    navigation system.
    """

    def __init__(self):
        """
        Set up the node and read the goal parameters.
        """

        rospy.init_node('move_base')

        self.xy_goal_tolerance = rospy.get_param('~xy_goal_tolerance', .15)
        self.stuck_delay = rospy.get_param('~stuck_delay', 10.0)
        self.stuck_threshold = .2
        self.action_s = actionlib.SimpleActionServer('move_base',
                                                     MoveBaseAction,
                                                     execute_cb=self.execute_cb,
                                                     auto_start=False)

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(1.0) # accumulate some transforms

        rospy.Subscriber('move_base_simple/goal', PoseStamped,
                         self.goal_callback)



        self.path_pub = rospy.Publisher('/path', Path, latch=True,
                                        queue_size=10)


        rospy.loginfo("Waiting for create_global_plan service...")
        rospy.wait_for_service('create_global_plan')
        rospy.loginfo("OK")

        self.action_s.start()


        
        rospy.spin()

    def _check_stuck(self, poses, pose):
        """Return true if we haven't moved for a while.

        """
        max_delay = rospy.Duration.from_sec(self.stuck_delay)
        poses.append(pose)
        if (poses[-1].header.stamp - poses[0].header.stamp) > max_delay:
            dist = util.distance(poses[-1], poses[0])
            if dist < self.stuck_threshold:
                return True
            poses.popleft()
        return False

    def execute_cb(self, goal):
        """This is the callback that will occur when a new goal arrives
        through the simple action client. goal will have a
        .target_pose field

        """

        # Store the goal pose in the global frame so we can
        # monitor progress.
        goal_pose_world = util.pose_transform(self.tf_buffer,
                                              goal.target_pose, 'map')

        path = self.request_plan(goal.target_pose)

        if path is None:
            msg = "Global planner couldn't find plan. Giving up."
            self.action_s.set_aborted(text=msg)
            return

        # After this happens the local planner should be working.
        self.path_pub.publish(path.plan)

        at_goal = False
        stuck = False

        rate = rospy.Rate(10)
        poses = deque() # store recent poses to see if we are stuck.

        while not at_goal and not stuck and not rospy.is_shutdown():

            if self.action_s.is_preempt_requested():
                rospy.loginfo('Navigation Preempted')
                self.path_pub.publish(Path())
                self.action_s.set_preempted()
                break

            robot_pose = self.get_current_pose()

            distance = util.distance(robot_pose, goal_pose_world)
            if distance < self.xy_goal_tolerance:
                at_goal = True
                rospy.loginfo("Goal reached.")
                self.action_s.set_succeeded()

            elif self._check_stuck(poses, robot_pose):
                stuck = True
                msg = "Not making progress. Giving up."
                self.path_pub.publish(Path())
                self.action_s.set_aborted(text=msg)

            else:
                feedback = MoveBaseFeedback()
                feedback.base_position = robot_pose
                self.action_s.publish_feedback(feedback)

            rate.sleep()


    def goal_callback(self, goal_pose):
        """ This callback handles non-action-client goal requests."""
        path = self.request_plan(goal_pose)
        if path is None:
            rospy.loginfo("Global planner couldn't find plan.")
        else:
            self.path_pub.publish(path.plan)


    def get_current_pose(self):
        """ Return the current robot pose in the world frame. """
        pose_base = util.create_pose_stamped(0, 0, frame_id='base_link')
        return util.pose_transform(self.tf_buffer, pose_base, 'map')

    def request_plan(self, goal_pose):
        """
        Create a service proxy and use it to request a plan from the
        global planner.  Returns the plan, or None if no plan
        is available.
        """
        rospy.wait_for_service('create_global_plan')
        try:
            get_plan = rospy.ServiceProxy('create_global_plan',
                                          GetPlan)
        
            robot_pose = self.get_current_pose()

            rospy.loginfo("Requesting plan...")
            resp = get_plan(robot_pose, goal_pose, self.xy_goal_tolerance)
            rospy.loginfo("Plan received.")
            if len(resp.plan.poses) == 0:
                resp = None
        except rospy.ServiceException:
            resp = None
        return resp

if __name__ == "__main__":
    MoveBase()
