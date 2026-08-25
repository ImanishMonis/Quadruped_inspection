#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

def main():
    rospy.init_node("joint_command_test")

    pub = rospy.Publisher("/joint_command",JointState,queue_size=10)
    rate = rospy.Rate(20)
    direction = 1 
    counter = 0 
    rospy.loginfo("Joint command test started.")

    while not rospy.is_shutdown():
        joint_positions = [
            0.5 * direction,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]

        msg = create_joint_state(joint_positions)

        pub.publish(msg)

        counter += 1 
        if counter >= 40:
            direction *= -1
            counter = 0  
        rate.sleep()



def create_joint_state(joint_positions):
    """
    Creates a JointState message from a list of joint positions.
    """

    msg = JointState()

    msg.header.stamp = rospy.Time.now()

    msg.name = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "gripper_left_joint",
        "gripper_right_joint"
    ]

    msg.position = joint_positions

    return msg

if __name__ == "__main__":
    main()