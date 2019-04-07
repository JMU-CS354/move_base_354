"""Testing code for global path planners.

Author: Nathan Sprague
Version: 3/14/2019

"""
import copy
import skimage.draw
import rospy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path, OccupancyGrid
import tf2_ros

import util
import map_utils



class PlanTester(object):
    """
    Testing class.
    """

    def __init__(self):
        """
        Set up subscribers.
        """

        rospy.init_node('plan_tester')

        rospy.Subscriber("costmap",
                         OccupancyGrid, self.costmap_callback)
        self.path_pub = rospy.Publisher('/path', Path, latch=True,
                                        queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        self.costmap = None
        rospy.loginfo("Waiting for costmap...")
        while self.costmap is None and not rospy.is_shutdown():
            rospy.sleep(.1)
        rospy.loginfo("Costmap received.")

    def costmap_callback(self, costmap):
        """
        Get the costmap and covert it to a Map object.
        """
        self.costmap = map_utils.Map(costmap)

    def plan_cost(self, path_msg, start, goal):
        """Return the total cost of the provided plan. The cost is the total
        Euclidean distance between all waypoints, plus the cost of
        every cell in the costmap that is traversed.

        """
        poses = [start] + path_msg.poses + [goal]

        prev_pose = poses[0]
        total_distance = 0
        cell_cost = 0

        for cur_pose in poses[1::]:
            total_distance += util.distance(prev_pose, cur_pose)
            row1, col1 = self.costmap.cell_index(prev_pose.pose.position.x,
                                                 prev_pose.pose.position.y)
            row2, col2 = self.costmap.cell_index(cur_pose.pose.position.x,
                                                 cur_pose.pose.position.y)
            rr, cc = skimage.draw.line(row1, col1, row2, col2)
            for row, col in zip(rr[1::], cc[1::]):
                cur_cost = self.costmap.grid[row, col]
                if cur_cost == -1 or cur_cost == 100:
                    cur_cost = float('inf')
                cell_cost += cur_cost
            prev_pose = cur_pose
        return total_distance + cell_cost

    def test_case(self, start_pose, end_pose, cost4, cost8):
        """ Run one planning test and print the results. """

        print("")
        msg = ("Testing ({}, {}) -> ({}. {})\n\t4-connected optimal: " +
               "{}\n\t8-connected optimal: {}")
        print(msg.format(start_pose.pose.position.x,
                         start_pose.pose.position.y,
                         end_pose.pose.position.x,
                         end_pose.pose.position.y, cost4, cost8))
        path_msg = self.request_plan(start_pose, end_pose)
        if path_msg is None:
            print("Planner service failed to return a plan.")
        elif len(path_msg.plan.poses) == 0:
            if cost8 == float('inf'):
                print("Correct: empty plan for unreachable goal.")
            else:
                print("Error: empty plan.")
        else:
            self.path_pub.publish(path_msg.plan)
            cost = self.plan_cost(path_msg.plan, start_pose, end_pose)
            if util.distance(path_msg.plan.poses[0], start_pose) > .01:
                print("Path doesn't start at the start location!")
                print path_msg.plan.poses[0], start_pose
            if util.distance(path_msg.plan.poses[-1], end_pose) > .1:
                print("Path doesn't end at the goal location!")

            print("Path cost: {:.2f}".format(cost))

    def request_plan(self, start_pose, end_pose):
        """ Request a plan from the global planner service. """
        rospy.wait_for_service('create_global_plan')
        try:
            get_plan = rospy.ServiceProxy('create_global_plan',
                                          GetPlan)
            resp = get_plan(start_pose, end_pose, .1)
        except rospy.ServiceException:
            resp = None
        return resp

def main():
    pt = PlanTester()
    print("STARTING TESTS...")
    pt.test_case(util.create_pose_stamped(-1, -1),
                 util.create_pose_stamped(0, -1), 21.0, 21.0)
    
    pt.test_case(util.create_pose_stamped(1, 1),
                 util.create_pose_stamped(2, 2), 57.0, 37.41)

    pt.test_case(util.create_pose_stamped(-2, -2),
                 util.create_pose_stamped(2, 2), 168.0, 107.27)

    pt.test_case(util.create_pose_stamped(1.96, -3.6),
                 util.create_pose_stamped(.93, .9), 487.05, 406.66)

    pt.test_case(util.create_pose_stamped(0, 0),
                 util.create_pose_stamped(20, 20), float('inf'), float('inf'))    
    pt.test_case(util.create_pose_stamped(20, 20),
                 util.create_pose_stamped(0, 0), float('inf'), float('inf'))

if __name__ == "__main__":
    main()
