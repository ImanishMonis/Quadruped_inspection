#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

joint_pub = None


def trajectory_callback(msg):

    rospy.loginfo(
        "Received trajectory with %d points",
        len(msg.points)
    )
    
    previous_time = rospy.Duration(0)
    for point in msg.points:

        joint_msg = JointState()

        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = msg.joint_names
        joint_msg.position = point.positions

        joint_pub.publish(joint_msg)

        delay = point.time_from_start - previous_time
        if delay.to_sec() > 0:
            rospy.sleep(delay)
        
        previous_time = point.time_from_start


def main():

    global joint_pub

    rospy.init_node("trajectory_bridge")

    joint_pub = rospy.Publisher(
        "/joint_command",
        JointState,
        queue_size=10
    )

    rospy.Subscriber(
        "/joint_trajectory",
        JointTrajectory,
        trajectory_callback
    )

    rospy.loginfo("Trajectory bridge ready.")

    rospy.spin()


if __name__ == "__main__":
    main()