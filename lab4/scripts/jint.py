#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from lab4.srv import Jint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import rospkg
import json


freq = 50
flag = 0

start = [0, 0, 0]

def interpolate(data):
    if(data.time < 0):
        return ("Czas musi być większy od zera")

    if(data.j1 < restrictions[0]['backward'] or data.j1 > restrictions[0]['forward']):
        return ("Pierwszy argument poza ograniczeniami robota")

    if(data.j2 < restrictions[1]['backward'] or data.j2 > restrictions[1]['forward']):
        return ("Drugi argument poza ograniczeniami robota")

    if(data.j3 < restrictions[2]['backward'] or data.j3 > restrictions[2]['forward']):
        return ("Trzeci argument poza ograniczeniami robota")
    
    global start
    end = [data.j1, data.j2, data.j3]

    change = start
    step=[(end[0]-start[0])/(freq*data.time),(end[1]-start[1])/(freq*data.time),(end[2]-start[2])/(freq*data.time)]

    for k in range(0, int(freq*data.time)+1):
	    for i in range(0, 3):
	        change[i]=change[i]+step[i]
	        
	        rate = rospy.Rate(freq) 
            pose_str = JointState()
            pose_str.header.stamp = rospy.Time.now()
            pose_str.name = ['base_to_link1', 'link1_to_link2', 'link2_to_link3'] #wskazanie jointow
            pose_str.position = [change[0], change[1], change[2]] #ruch jointow
            pub.publish(pose_str)
            rate.sleep()

    start = end
    return (str(data.j1)+" "+str(data.j2)+" "+str(data.j3))


if __name__ == "__main__":

    rospack = rospkg.RosPack()

    with open(rospack.get_path('lab4') + '/restrictions.json', 'r') as file:
        restrictions = json.loads(file.read())

    rospy.init_node('jint_srv')
    pub = rospy.Publisher('joint_states',JointState,queue_size=10)
    s = rospy.Service('jint', Jint, interpolate)

    rospy.spin()
