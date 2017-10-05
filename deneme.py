#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
import tf
import math
import time

class RobotManager:
    def __init__(self):
        
        self.odom = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)
        self.pose2d = rospy.Publisher("/pose2d", Pose2D, queue_size=1)
        self.target = rospy.Subscriber("/target", Pose2D, self.target_callback, queue_size=1)
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.position = None
        self.target = None
        self.rate.sleep()

    def odom_callback(self, msg):
        #This blok uses for transform(euler to quarnitation)
        pose = msg.pose.pose
        quat = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        orientation = tf.transformations.euler_from_quaternion(quat)
        self.position = (
            pose.position.x,
            pose.position.y,
            orientation[2]
        )
        
        #This blok uses for publish pose2d when then after odom data transformed
        
        quar_odom_msg = Pose2D()
        quar_odom_msg.x = self.position[0]
        quar_odom_msg.y = self.position[1]
        quar_odom_msg.theta = self.position[2]
        self.pose2d.publish(quar_odom_msg)
        #rospy.logerr("odom => pose: %s"%(self.position[2]))
        
    def target_callback(self, msg):
        self.target = (
            msg.x,
            msg.y,
            msg.theta
        )
        
        #self.GoToPointLine()
        self.GoToPointCurve()
        
    def movement(self, ang_z, lin_x):   
        movement_msg = Twist()
        
        movement_msg.linear.x = lin_x
        movement_msg.angular.z = ang_z
        
        self.cmd_vel.publish(movement_msg)

    def breakes(self):
        self.movement(0, 0) #break

    def find_distance(self):
        #sqrt(sqr(x1-x2) + sqr(y1-y2))
        return math.sqrt((self.position[0] - self.target[0])**2 + (self.position[1] - self.target[1])**2)

    def am_I_close(self, radius):
        #(x-a)^2+(y-b)^2 = radius^2
        
        t_x = self.target[0]
        t_y = self.target[1]
    
        if (((t_x-self.position[0])**2)+((t_y-self.position[1])**2)) < radius**2:
            print ((((t_x-self.position[0])**2)+((t_y-self.position[1])**2)))
            print (radius**2)
            time.sleep(0.1)
            return True
        else:
            print ((((t_x-self.position[0])**2)+((t_y-self.position[1])**2)))
            print (radius**2)
            time.sleep(0.1)
            return False

    def find_target_direction(self):
        norm_x =  self.target[0] - self.position[0]
        norm_y = self.target[1] - self.position[1]
        
        target_direction = math.atan2(norm_y, norm_x)
        
        return target_direction
        
    def GoToPointLine(self):
        my_x = self.position[0]
        my_y = self.position[1]
        
        t_x = self.target[0]
        t_y = self.target[1]
        target_direction = self.find_target_direction()
        
        while math.fabs(self.position[2] - target_direction) > 0.1:
            time.sleep(0.1)
            self.movement(math.radians(20), 0)
        self.breakes()
        while not(self.am_I_close(0.3)):
            self.movement(0, 0.1)
        self.breakes()
    
    def GoToPointCurve(self):
        my_x = self.position[0]
        my_y = self.position[1]
        
        t_x = self.target[0]
        t_y = self.target[1]
        target_direction = self.find_target_direction()
        
        while not(self.am_I_close(0.3)):
            k_for_velLin = self.find_distance()
            k_for_velAng = self.find_distance()
            target_direction = self.find_target_direction()
            if (target_direction < 3.14/2) and (target_direction > -3.14/2) or (self.position[0] < 0):
                k_for_velAng = -k_for_velAng                
            print("t_d = %s" % target_direction)
            print("lin = %s" % k_for_velLin)
            print("ang = %s" % k_for_velAng)
            self.movement(0, 0.1*k_for_velLin)
            if math.fabs(self.position[2] - target_direction) > 0.3:
                self.movement(math.radians(5)*k_for_velAng, 0.1*k_for_velLin)   
        self.breakes()

def main():
    rospy.init_node("python_carrier", anonymous=True)
    robot = RobotManager()
    robot.rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    main()
