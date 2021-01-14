#!/usr/bin/env python3

import rospy
from transitions import Machine, State, Transition

import numpy as np
from cone import ConeVectorField
from plane import PlaneVectorField
from geometry_msgs.msg import Point, Vector3
from geometry_msgs.msg import PoseStamped
position = np.zeros((3, 1))

class Vector_Control(object):
    def __init__(self):
        self.plane = PlaneVectorField(h=10)
        self.cone = ConeVectorField(h=12)
        self.landing_height = 0.5

    def state_plane_centering(self): rospy.loginfo("Current State : Plane Centering")
    def state_cone_drop(self): rospy.loginfo("Current State : Cone Drop")
    def state_height_hit(self): rospy.loginfo("Current State : Height Hit")
    def state_landed(self): rospy.loginfo("Current State : Successfully landed !")
    def state_take_off(self): rospy.loginfo("Current State : Take off")

    def is_centered(self):
        global position
        return np.linalg.norm(position-np.array([[0], [0], [self.plane.h]])) < 0.5

    def is_out_of_the_cone(self):
        global position
        return position not in self.cone

    def is_landing(self):
        global position
        return position in self.cone and position[2, 0] <= self.landing_height

    def is_height_hit(self):
        global position
        return position[2, 0] >= self.plane.h + 1


class State_Machine:
    def __init__(self):
        self.position_subscriber = rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, self.callback_position)
        self.vector_publisher = rospy.Publisher("/vector", Vector3, queue_size=10)
        rospy.loginfo("hello")
        self.states = [
            State(name="plane_centering", on_enter=['state_plane_centering']),
            State(name="cone_drop", on_enter=['state_cone_drop']),
            State(name="height_hit", on_enter=['state_height_hit']),
            State(name="landed", on_enter=['state_landed']),
            State(name="post_landed")
        ]

        self.vc = Vector_Control()
        self.machine = Machine(model=self.vc, states=self.states, initial='plane_centering')

        self.machine.add_transition("next", "plane_centering", "cone_drop", conditions="is_centered")
        self.machine.add_transition("next", "cone_drop", "height_hit", conditions="is_out_of_the_cone")
        self.machine.add_transition("next", "height_hit", "plane_centering", conditions="is_height_hit")
        self.machine.add_transition("next", "cone_drop", "landed", conditions="is_landing")
        self.machine.add_transition("next", "landed", "post_landed")
        self.machine.add_transition("next", "post_landed", "post_landed")

    def callback_position(self, msg):
        global position
        # Storage of the current position
        position[0, 0] = msg.pose.position.x
        position[1, 0] = msg.pose.position.y
        position[2, 0] = msg.pose.position.z

        #Computing the next state
        self.vc.next()

        # Creating a message
        V = Vector3()

        # Computing the state vector depending on the state
        if self.vc.is_plane_centering():
            v = self.vc.plane.vector(position)
        elif self.vc.is_cone_drop():
            v = self.vc.cone.vector(position)
        elif self.vc.is_height_hit():
            v = [0, 0, self.vc.plane.h + 1 - position[2, 0]]
        else:
            v = [0, 0, 0]
        
        V.x = v[0]
        V.y = v[1]
        V.z = v[2]

        rospy.loginfo(V)

        self.vector_publisher.publish(V)
        

if __name__ == '__main__':
    rospy.init_node('state_machine')
    s = State_Machine()
    
    rospy.spin()