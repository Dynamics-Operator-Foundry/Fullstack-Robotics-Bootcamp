#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelConfiguration
import math
import time

def main():
    rospy.init_node('set_joint_angles_gazebo')

    rospy.wait_for_service('/gazebo/set_model_configuration')
    set_config = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

    model_name = "fredo1"  # must match your model name in Gazebo
    urdf_param_name = "robot_description"
    joint_names = ["joint_1", "joint_2", "joint_3"]  # match your URDF

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        
        print(1)
        t = time.time()

        joint_positions = [
            math.sin(t),
            math.cos(t),
            math.sin(t) * math.cos(t)
        ]
        print(2)
        try:
            set_config(model_name, urdf_param_name, joint_names, joint_positions)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to set joint configuration: %s", e)
        print(3)
        rate.sleep()
        print(4)

if __name__ == '__main__':
    main()
