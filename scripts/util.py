""" Utility methods for simplifying some common tf-related tasks.

Author: Nathan Sprague
Version: 2/19/2019
"""

import numpy as np
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import rospy
import tf.transformations

def create_pose_stamped(x, y, yaw=0, frame_id='map',
                        stamp=None):
    """ Create a PoseStamped object from a 2d location """

    pose = PoseStamped()
    if stamp is None:
        pose.header.stamp = rospy.Time.now()
    else:
        pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    return pose

def pose_to_tf2(pose):
    """Convert a geometry_msgs/PoseStamped object to a
    tf2_geometry_msgs/PoseStamped object.

    """
    new_pose = tf2_geometry_msgs.PoseStamped()
    new_pose.header.stamp = pose.header.stamp
    new_pose.header.frame_id = pose.header.frame_id
    new_pose.pose.orientation = pose.pose.orientation
    new_pose.pose.position = pose.pose.position
    return new_pose

def pose_to_tf1(pose):
    """Convert a tf2_geometry_msgs/PoseStamped object to a
    geometry_msgs/PoseStamped object.

    """
    new_pose = PoseStamped()
    new_pose.header.stamp = pose.header.stamp
    new_pose.header.frame_id = pose.header.frame_id
    new_pose.pose.position = pose.pose.position
    new_pose.pose.orientation = pose.pose.orientation
    return new_pose

def pose_transform(tf_buffer, pose, target_frame):
    """Use a properly initilized tf2 Buffer to transform the provided pose
    into the target coordinate frame.

    Args:
        tf_buffer -  a properly initilized tf2 Buffer
        pose -  geometry_msgs/PoseStamped object
        target_frame - string representing the target frame

    Returns: a geometry_msgs/Pose object or None if something went wrong

    """
    pose2 = pose_to_tf2(pose)
    try:
        pose2 = tf_buffer.transform(pose2, target_frame,
                                    rospy.Duration(.1))
        # convert to standard PoseStamped
        pose_new = pose_to_tf1(pose2)
        return pose_new

    except Exception as e:
        print(type(e))
        print(e)
        return None


def yaw_from_pose(pose):
    """ Extract the yaw (orientation) from a pose message. """
    quat = np.array([pose.orientation.x, pose.orientation.y,
                     pose.orientation.z, pose.orientation.w])
    euler = tf.transformations.euler_from_quaternion(quat)
    return euler[2]

def distance(pose1, pose2):
    """calculate the distance between two poses in the same coordinate
    frame, under the assumption that they have the same time stap.

    """

    assert pose1.header.frame_id == pose2.header.frame_id
    return np.sqrt((pose1.pose.position.x - pose2.pose.position.x) **2 +
                   (pose1.pose.position.y - pose2.pose.position.y) **2)

def distance_tf(tf_buffer, pose1, pose2):
    """Use tf to calculate the distance between any two PoseStamped
    objects.

    """
    #convert to the same coordinate frame...
    pose1 = pose_transform(tf_buffer, pose1, pose2.header.frame_id)
    return distance(pose1, pose2)
