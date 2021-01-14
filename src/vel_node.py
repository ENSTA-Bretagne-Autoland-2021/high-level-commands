#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np



drone_position = PoseStamped()
topic = 'target_marker'



def callback(data):
    
    global drone_position 
    drone_position = data


def compute_force(wpt, pose):
    force = 5*np.array([[wpt[0] - pose.pose.position.x, wpt[1] - pose.pose.position.y, wpt[2] - pose.pose.position.z]]).T
    
    return force.flatten()




if __name__ == '__main__':
    try:
        rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, callback)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        marker_pub = rospy.Publisher(topic, Marker)
        rospy.init_node('vel_node', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        cmd_vel = Twist()
        dt = 0.1
        

        t = 0.
        while not rospy.is_shutdown():
            waypoint = (10*np.cos(0.1*t), 10*np.sin(0.1*t),6+5*np.sin(0.1*t))

            marker = Marker()
            marker.header.frame_id = "world"
            marker.type = marker.SPHERE
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = waypoint[2]
            
            
            #rospy.loginfo(drone_position.pose.position)
            force = compute_force(waypoint, drone_position) 
            t += dt
            rospy.loginfo(t)
            #rospy.loginfo(force)
            cmd_vel.linear.x = force[0]*dt
            cmd_vel.linear.y = force[1]*dt
            cmd_vel.linear.z = force[2]*dt
            pub.publish(cmd_vel)
            marker_pub.publish(marker)
            
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
