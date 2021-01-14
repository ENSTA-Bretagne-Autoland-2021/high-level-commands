#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Vector3
import numpy as np


force = Vector3()


def callback(data):
    
    global force
    force = data







if __name__ == '__main__':
    try:
        rospy.Subscriber("/vector", PoseStamped, callback)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('vel_node', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        cmd_vel = Twist()
        dt = 0.1
        t = 0.
        while not rospy.is_shutdown():

            
            rospy.loginfo(force)
            cmd_vel.linear.x = force.x*dt
            cmd_vel.linear.y = force.y*dt
            cmd_vel.linear.z = force.z*dt
            pub.publish(cmd_vel)
            
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
