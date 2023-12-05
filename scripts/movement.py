#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2,radians
from tetraotonom.msg import robotstatusmsg
import math

class RobotMovement:
    
    def __init__ ( self,max_linear_speed,max_angular_speed,linear_acceleration,breaking_acceleration,angular_acceleration,angular_breaking_acceleration):
        self.__max_linear_speed = max_linear_speed
        self.__max_angular_speed = max_angular_speed
        self.__linear_acceleration = linear_acceleration
        self.__breaking_acceleration = breaking_acceleration
        self.__angular_acceleration = angular_acceleration
        self.__angular_breaking_acceleration = angular_breaking_acceleration
        
        self.__x = 0.0
        self.__y = 0.0
        self.__ang = 0.0
        self.__odom_update = False
        
        rospy.init_node("movement_nodee")
        
        self.__vel = Twist()
        
        self.__pub = rospy.Publisher("robotstatus",robotstatusmsg,queue_size=10)
        self.__cmd_vel = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
        rospy.Subscriber("odom",Odometry,self.__odom_callback)
        
    
    def __odom_callback(self,msg):
        orientation_q = msg.pose.pose.orientation
        theta = atan2(2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y),1.0 - 2.0 * (orientation_q.y**2 + orientation_q.z**2))
        
        if theta < 0:
            self.__ang = theta + (math.floor(abs(self.__ang)/(2*math.pi)) + 1) * math.pi * 2
        else:
            self.__ang = theta
        
        self.__ang %= math.pi * 2
        
        self.__x = msg.pose.pose.position.x
        self.__y = msg.pose.pose.position.y
        self.__odom_update = True
        
        #print("x:{}\ty:{}\tang:{}".format(self.__x,self.__y,self.__ang))
        
    def __odom_wait(self):
        while not self.__odom_update:
            pass
        
        
        
    def Rotate(self,ang):
        rate = rospy.Rate(100)
        
        self.__vel = Twist()
        
        self.__odom_wait()
        
        if ang < 0:
            ang = ang + (math.floor(abs(ang)/(2*math.pi)) + 1) * math.pi * 2
        
        ang %= math.pi * 2
        
        dist = lambda : min(abs(ang - self.__ang),2*math.pi + (min(ang,self.__ang)-max(ang,self.__ang)))
        
        b_dist = lambda : self.__vel.angular.z * (self.__vel.angular.z / self.__angular_breaking_acceleration) - self.__angular_breaking_acceleration * (self.__vel.angular.z / self.__angular_breaking_acceleration)**2 * 0.5
        
        def sign():
            if dist() == abs(ang - self.__ang):
                if ang > self.__ang:
                    return 1
                else:
                    return -1
            else:
                if ang > self.__ang:
                    return -1
                else:
                    return 1
                
        err_rate = 0.1
        
        d = 999999.0
        bd = 0.0
        
        rate.sleep()
        
        lst_t = rospy.Time.now().to_sec()
        dt = 0.0
        
        while d >  err_rate:
            d = dist()
            s = sign()
            
            if d <= bd:
                if abs(self.__vel.angular.z) < self.__max_angular_speed:
                    self.__vel.angular.z -= self.__angular_breaking_acceleration * dt * s
            else:
                if abs(self.__vel.angular.z) < self.__max_angular_speed:
                    self.__vel.angular.z += self.__angular_acceleration * dt * s
                bd = b_dist()
                
            self.__cmd_vel.publish(self.__vel)
            
            print("konum: {} hedef: {} hız: {} kalan mesafe: {} frenleme mesafesi: {} frenleme: {} {}".format(self.__ang,ang,self.__vel.angular.z,d,bd,d <= bd,self.__angular_breaking_acceleration * dt * s))
            msg = robotstatusmsg()
            msg.speed = math.sqrt(self.__vel.linear.x ** 2 + self.__vel.linear.y ** 2)
            msg.angular_speed = self.__vel.angular.z
            self.__pub.publish(msg)
            t = rospy.Time.now().to_sec()
            dt = t - lst_t
            lst_t =t
            
            rate.sleep()
        
        self.__cmd_vel.publish(Twist())
        print("Hedefe ulaşıldı.")
            
        
        
    def Move(self,distance):
        rate = rospy.Rate(100)
        
        self.__vel = Twist()
        
        self.__odom_wait()
        
        x0 = self.__x
        y0 = self.__y
        
        dist = lambda : math.sqrt((x0 - self.__x)**2 + (y0 - self.__y)**2)
        
        b_dist = lambda : self.__vel.linear.x * (self.__vel.linear.x / self.__breaking_acceleration) - self.__breaking_acceleration * (self.__vel.linear.x / self.__breaking_acceleration)**2 * 0.5
        
        rate.sleep()
        
        d = dist()
        bd = 0.0
        
        lst_t = rospy.Time.now().to_sec()
        dt = 0.0
        
        while d < distance:
            print(d,distance,self.__vel.linear.x,sep=" , ")
            d = dist()
            
            if (distance - d) <= bd:
                if abs(self.__vel.linear.x) < self.__max_linear_speed:
                    self.__vel.linear.x -= self.__breaking_acceleration * dt
            else:
                if abs(self.__vel.linear.x) < self.__max_linear_speed:
                    self.__vel.linear.x += self.__linear_acceleration * dt
                bd = b_dist()
            
            self.__cmd_vel.publish(self.__vel)
            msg = robotstatusmsg()
            msg.speed = math.sqrt(self.__vel.linear.x ** 2 + self.__vel.linear.y ** 2)
            msg.angular_speed = self.__vel.angular.z
            self.__pub.publish(msg)
                
            t = rospy.Time.now().to_sec()
            dt = t - lst_t
            lst_t =t
            
            rate.sleep()
            
        self.__vel = Twist()
        self.__cmd_vel.publish(self.__vel)
        
        print("Hedefe ulaşıldı.")
                
            
        
        
        

robot = RobotMovement(5,radians(120),0.3,0.3,radians(10),radians(10))

robot.Rotate(0)
robot.Move(2)
robot.Rotate(radians(120))
robot.Move(2)
robot.Rotate(radians(240))
robot.Move(2)
robot.Rotate(0)
