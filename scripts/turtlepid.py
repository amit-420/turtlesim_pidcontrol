#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from rospy.core import is_shutdown
from rospy.msg import deserialize_messages
from rospy.topics import Publisher
from turtlesim.msg import Pose
import numpy as np
from math import atan2


class turtle:
    def __init__(self):
        rospy.init_node('turtle_control', anonymous=True)
        self.Kp = 0.4
        self.Kd = 0.2
        self.Ki = 0.01
        self.pose = Pose()
        self.pose.linear_velocity = 0
        self.vel_msg = Twist()
        self.e_lin = 0
        self.e_ang = 0
        self.e_lin_old = 0
        self.e_ang_old = 0
        self.elin_dt = 0
        self.eang_dt = 0 
        self.goal_reached = False
        self.pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        self.listen = rospy.Subscriber('/turtle1/pose',Pose,self.callback)
    
    def user_in(self):
        print("Moving turtle forward")
        self.posn_x = float(input("Input the desired x coordinate:"))
        self.posn_y = float(input("Input the desired y coordinate:"))
        self.posn_theta = float(input("Input the desired theta to rotate: "))
        self.theta = atan2((self.posn_y -self.pose.y),(self.posn_x - self.pose.x )) - self.pose.theta
        
    def callback(self, data):
        self.rate = rospy.Rate(10)
        self.pose = data  
        self.pose.x = data.x
        self.pose.y = data.y
        # rospy.loginfo(rospy.get_caller_id() + 'Pos: %f', data.x)
    
    # def new_target(self):
    #     x = np.dot(np.array([[-np.sin(self.pose.theta),np.cos(self.pose.theta)],[np.cos(self.pose.theta),np.sin(self.pose.theta)]]) ,np.array([self.posn_x,self.posn_y]))
    #     rospy.loginfo('new target position: %f, %f',x[0],x[1])
    #     return x


    def give_e_lin(self):
        e_lin = np.sqrt((self.posn_x - self.pose.x )**2 + (self.posn_y - self.pose.y)**2)
        self.e_lin_x = self.posn_x - self.pose.x
        self.e_lin_y = self.posn_y - self.pose.y
        # rospy.loginfo('err: %f',e_lin)
        # cosine = (self.posn_x - self.pose.x )/e_lin
        # sine = (self.posn_y - self.pose.y)/e_lin
        return e_lin
    
    def give_e_ang(self):
        self.theta = atan2((self.posn_y -self.pose.y),(self.posn_x - self.pose.x )) 
        # rospy.loginfo('theta: %f',self.theta)
        self.e_ang = self.theta - self.pose.theta

    def give_de(self):
        self.diff_e_lin = (self.e_lin - self.e_lin_old)/0.1
        self.diff_e_ang = (self.e_ang - self.e_ang_old)/0.1
        self.e_lin_old = self.e_lin
        self.e_ang_old = self.e_ang
        


    def give_inte(self):
        self.elin_dt += self.e_lin
        self.eang_dt += self.e_ang
    

    def move(self):
        self.vel_msg.linear.x = self.Kp * self.e_lin + self.Kd * self.diff_e_lin + self.Ki * self.elin_dt
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 1 * self.e_ang + 0.4 * self.diff_e_ang  + self.Ki * self.eang_dt
        rospy.loginfo(' pos_x: %f pos_y: %f err: %f err_ang: %f theta: %f angle: %f', self.pose.x ,self.pose.y,self.e_lin,self.e_ang,self.theta,self.pose.theta)
        self.pub.publish(self.vel_msg)
        
    def stop(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.angular.z = 0
        self.pub.publish(self.vel_msg)
        self.goal_reached = True
        rospy.loginfo('In else pos_x: %f pos_y: %f err: %f', self.pose.x ,self.pose.y,self.e_lin)

    def inti_turtle(self):
        self.user_in()
        self.e_lin = self.give_e_lin()
        self.give_e_ang()
        while self.e_lin  > 0.1:
            #self.posn_x, self.posn_y = self.new_target()
            self.give_de()
            self.give_inte()
            self.give_e_ang()
            self.e_lin = self.give_e_lin()
            self.move()
            self.rate.sleep()
        self.stop()

    

if __name__ == '__main__':
    
    try:
        turtle1 = turtle()
        while not rospy.is_shutdown():

            if not turtle1.goal_reached:
                turtle1.inti_turtle()
            else:
                break

    except rospy.ROSInterruptException:
        pass