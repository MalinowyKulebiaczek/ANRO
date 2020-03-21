#!/usr/bin/env python

import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

move_up = 'w'
move_down = 's'
move_left = 'a'
move_right = 'd'

def printInstruction():
    print("Let's move your turtle. Use WASD buttons to steer\n-------------------")

#Funkcja wzieta z teleop_twist_keyboard. Dzieki niej nie trzeba zatwierdzac enterem. Nie pytajcie jak dziala, bo nie wiem
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def move():
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    
    #wydaje mi sie, ze jak wczesniej zamiast funkcji getKey uzywalem zwyklego raw_input() i nie bylo tej linijki, to zolw sie
    #nie zatrzymywal - nie wiem dlaczego w tej wersji zolw 
    #zatrzymuje sie bez tej linijki i bez linijki rate.sleep(), ktora jest nizej   
    #rate = rospy.Rate(10)  
    
    while not rospy.is_shutdown():
        
        buf=getKey()
        if buf == move_up:
            vel_msg.linear.x = 2.0
        elif buf == move_down:
            vel_msg.linear.x = -2.0
        elif buf == move_left:
            vel_msg.angular.z = 2.0        
        elif buf == move_right:
            vel_msg.angular.z = -2.0
        #Break if ctrl+C (pozyczone z teleop_twist_keyboard)    
        elif (buf == '\x03'):
            break
        else:
            continue

         #Publish the velocity
        velocity_publisher.publish(vel_msg)        
    
        #rate.sleep()  
        #Zeruje wartosci predkosci, zeby w przyszlej petli na pewno obie poczatkowe predkosci byly zerowe 
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
    
    

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)     #POZYCZONE Z teleop_twist_keyboard - bez tego nie dziala
    try:
        #Testing our function
        if rospy.has_param('/move_up'):
            move_up = rospy.get_param('/move_up')
        if rospy.has_param('/move_down'):
            move_down = rospy.get_param('/move_down')
        if rospy.has_param('/move_left'):
            move_left = rospy.get_param('/move_left')
        if rospy.has_param('/move_right'):
            move_right = rospy.get_param('/move_right')
        
        printInstruction()
        move()
    except rospy.ROSInterruptException:
        pass 
        #termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)    #ROWNIEZ POZYCZONE - myslalem, ze to istotna                                          #linijka, ale okazalo sie, ze dziala tez ze zwyklym 'pass'
