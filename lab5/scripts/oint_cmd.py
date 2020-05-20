#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import rospy
import rospkg
from lab5.srv import Oint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from geometry_msgs.msg import PoseStamped
from math import atan, sin, cos, pi

from params import get_parameters




# frequency declaration
f = 50.0

global prev_theta2
global prev_theta4
def callback(data):
    error=0
    global prev_theta
    global prev_theta2
    global prev_theta4
    rate = rospy.Rate(f)

    r14 = float(data.pose.position.x)
    r24 = float(data.pose.position.y)
    r34 = float(data.pose.position.z)
    
    theta1 = atan(r24/r14)

    alpha = r14*cos(theta1) + r24*sin(theta1)
    
    if (- a1**4 + 2*a1**2*a2**2 + 2*a1**2*alpha**2 + 2*a1**2*r34**2 - a2**4 + 2*a2**2*alpha**2 + 2*a2**2*r34**2 - alpha**4 - 2*alpha**2*r34**2 - r34**4)<0:
        theta2=prev_theta2
    else:
        theta2 = -2*atan((2*a1*r34 + (- a1**4 + 2*a1**2*a2**2 + 2*a1**2*alpha**2 + 2*a1**2*r34**2 - a2**4 + 2*a2**2*alpha**2 + 2*a2**2*r34**2 - alpha**4 - 2*alpha**2*r34**2 - r34**4)**(0.5))/(a1**2 + 2*a1*alpha - a2**2 + alpha**2 + r34**2))
        prev_theta2=theta2+pi/2
    theta2 = theta2 + pi/2
    
    if (- a1**2 + 2*a1*a2 - a2**2 + alpha**2 + r34**2)*(a1**2 + 2*a1*a2 + a2**2 - alpha**2 - r34**2)<0:
        theta4=prev_theta4
    else:  
        theta4 = -2*atan((2*a2*r34 - ((- a1**2 + 2*a1*a2 - a2**2 + alpha**2 + r34**2)*(a1**2 + 2*a1*a2 + a2**2 - alpha**2 - r34**2))**(0.5))/(- a1**2 + a2**2 + 2*a2*alpha + alpha**2 + r34**2))
        prev_theta4=theta4
 
    theta3 = theta4 - theta2
    
    theta_vec=[theta1, theta2, theta3]
    
    i=0
    for joint in restrictions:
        
        if restrictions[i]['backward']>theta_vec[i]:
            theta_vec[i]=restrictions[i]['backward']
            warn_string="Przekroczono ograniczenie dolne stawu o numerze: "+str(i+1)
            error=1
            rospy.logerr(warn_string)
        elif restrictions[i]['forward']<theta_vec[i]:
            theta_vec[i]=restrictions[i]['forward']
            warn_string="Przekroczono ograniczenie gorne stawu o numerze: "+str(i+1)
            rospy.logerr(warn_string)
            error=1
        i=i+1
    
    if error==1:
        theta_vec=prev_theta
    else:
        prev_theta=theta_vec
        
    # create a new JoinState
    newJS = JointState()
    newJS.header = Header()
    newJS.header.stamp = rospy.Time.now()
    newJS.header.frame_id = 'base_link'
    newJS.name = ['base_to_link1', 'link1_to_link2', 'link2_to_link3'] #wskazanie jointow
    newJS.position = theta_vec

    pub.publish(newJS)
    rate.sleep()
    #print(data.pose.position)
def init(x,y, z):
    trajectory(x,y,z,5)
    
def kwadrat(x0,y0, z0, x):
    trajectory(x0,y0+x,z0+x, 5)
    trajectory(x0,y0-x,z0+x, 5)
    trajectory(x0,y0-x,z0, 5)
    trajectory(x0,y0+x,z0, 5)
    trajectory(x0,y0+x,z0+x, 5)
    
 
 
def trajectory(x, y, z, time):
    rospy.wait_for_service('oint')
    try:
        trajektoria=rospy.ServiceProxy('oint', Oint)
        resp1=trajektoria(x,y,z,0,0,0,time)
        return resp1
    except rospy.ServiceException, e:
        print ("Service call failed: ",e)
    

if __name__ == '__main__':
    
    a1 = 0.7
    a2 = 1
    if get_parameters('dlugosc1'):
        a1 = rospy.get_param("/dlugosc1")

    if get_parameters('dlugosc2'):
        a2 = rospy.get_param("/dlugosc2")
    init(a2, 0, a1)
    while not rospy.is_shutdown():
        kwadrat(a2, 0, a1, 0.4)
    '''
    a1 = 0.7
    a2 = 1
    rospack = rospkg.RosPack()
    with open(rospack.get_path('lab5') + '/restrictions.json', 'r') as file:
        restrictions= json.loads(file.read())

    if get_parameters('dlugosc1'):
        a1 = rospy.get_param("/dlugosc1")

    if get_parameters('dlugosc2'):
        a2 = rospy.get_param("/dlugosc2")

    rospy.init_node('IKIN', anonymous=False)
    rospy.Subscriber("OintAxes", PoseStamped, callback)
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    #s = rospy.Service('initialize', Jint, interpolate)
    rospy.spin()
    '''
