#!/usr/bin/env python
import rospy
import time
import random
import numpy as np
from hector_uav_msgs.srv import EnableMotors
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import actionlib
import hector_uav_msgs.msg
from std_msgs.msg import String


waypoints = []


for i in range(0, 100, 1):
    waypoints.append((5*np.cos(i*0.1), 5*np.sin(i*0.1), 0.5))


drone_position = PoseStamped()
def callback(data):
    
    global drone_position 
    drone_position = data


def distance(wpt, pose):
    return ((wpt[0] - pose.pose.position.x)**2 + (wpt[1] - pose.pose.position.y)**2 + (wpt[2] - pose.pose.position.z)**2)**0.5

def hector_pose_client(wpt):
     # First enable motor service
     rospy.wait_for_service("/enable_motors")
     enabler1 = rospy.ServiceProxy("/enable_motors", EnableMotors)
     resp1 = enabler1(True)

     #rospy.loginfo("Creating Action Client.")
     client = actionlib.SimpleActionClient('/action/pose', hector_uav_msgs.msg.PoseAction)
     #rospy.loginfo("Client created.")

     # This is where the program seems to hang even though I assumed hector would automatically run the action server
     client.wait_for_server()

     # Create a random goal
     g = hector_uav_msgs.msg.PoseGoal()
     g.target_pose.header.frame_id = 'world'
     g.target_pose.pose.position.x = wpt[0]
     g.target_pose.pose.position.y = wpt[1]
     g.target_pose.pose.position.z = wpt[2]

     """ q = quaternion_from_euler(1, 0, 1.2)

     g.target_pose.pose.orientation.x = q[0]
     g.target_pose.pose.orientation.y = q[1]
     g.target_pose.pose.orientation.z = q[2]
     g.target_pose.pose.orientation.w = q[3] """



     client.send_goal(g)
     client.wait_for_result()
     return(client.get_result())



if __name__ == "__main__":
     try:
         rospy.init_node('drone_explorer')
         #result = hector_pose_client()
         rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, callback)
         for waypoint in waypoints:
             checked = False
             while checked == False:
                 rospy.loginfo(waypoint)
                 rospy.loginfo(drone_position.pose.position)
                 #rospy.loginfo(drone_position.pose.position.x)
                 hector_pose_client(waypoint)
                 
                 rospy.loginfo(distance(waypoint, drone_position) )
                 if distance(waypoint, drone_position) < 1:
                     checked = True
                     rospy.loginfo("Waypoint valided !")
         rospy.loginfo("Client navigated")
         rospy.spin()


     except rospy.ROSInterruptException:
         print("program interrupted before completion")