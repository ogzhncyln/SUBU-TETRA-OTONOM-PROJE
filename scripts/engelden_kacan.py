#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tetraotonom.msg import robotstatusmsg
from tetraotonom.msg import lidarmsg
from math import atan2,radians
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
        self.ang = 0.0
        self.__odom_update = False
        self.lidar_update = False
        self.lidar = []
        
        rospy.init_node("movement_nodee")
        
        self.__vel = Twist()
        
        self.__pub = rospy.Publisher("robotstatus",robotstatusmsg,queue_size=10)
        self.__lidar_pub = rospy.Publisher("robot_lidar",lidarmsg,queue_size=10)
        self.__cmd_vel = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
        rospy.Subscriber("odom",Odometry,self.__odom_callback)
        rospy.Subscriber("scan",LaserScan,self.__laser_callback)
        
    
    def __odom_callback(self,msg):
        orientation_q = msg.pose.pose.orientation
        theta = atan2(2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y),1.0 - 2.0 * (orientation_q.y**2 + orientation_q.z**2))
        
        if theta < 0:
            self.ang = theta + (math.floor(abs(self.ang)/(2*math.pi)) + 1) * math.pi * 2
        else:
            self.ang = theta
        
        self.ang %= math.pi * 2
        
        self.__x = msg.pose.pose.position.x
        self.__y = msg.pose.pose.position.y
        self.__odom_update = True
        
        #print("x:{}\ty:{}\tang:{}".format(self.__x,self.__y,self.__ang))
        
    def __odom_wait(self):
        while not self.__odom_update:
            pass
    
    # ön-önsol-sol-arkasol-arka-arkasağ-sağ
    def __laser_callback(self,data):
        self.lidar = list(data.ranges[0:359:45])
        self.lidar.pop(len(self.lidar)-1)
        self.lidar_update = True
        msg = lidarmsg()
        msg.on = self.lidar[0]
        msg.onsol = self.lidar[1]
        msg.sol = self.lidar[2]
        msg.arkasol = self.lidar[3]
        msg.arka = self.lidar[4]
        msg.arkasag = self.lidar[5]
        msg.sag = self.lidar[6]
        self.__lidar_pub.publish(msg)
        #print(self.lidar)
        
        
        
    def Rotate(self,ang):
        rate = rospy.Rate(100)
        
        self.__vel = Twist()
        
        self.__odom_wait()
        
        if ang < 0:
            ang = ang + (math.floor(abs(ang)/(2*math.pi)) + 1) * math.pi * 2
        
        ang %= math.pi * 2
        
        dist = lambda : min(abs(ang - self.ang),2*math.pi + (min(ang,self.ang)-max(ang,self.ang)))
        
        b_dist = lambda : self.__vel.angular.z * (self.__vel.angular.z / self.__angular_breaking_acceleration) - self.__angular_breaking_acceleration * (self.__vel.angular.z / self.__angular_breaking_acceleration)**2 * 0.5
        
        def sign():
            if dist() == abs(ang - self.ang):
                if ang > self.ang:
                    return 1
                else:
                    return -1
            else:
                if ang > self.ang:
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
            msg = robotstatusmsg()
            msg.speed = math.sqrt(self.__vel.linear.x ** 2 + self.__vel.linear.y ** 2)
            msg.angular_speed = self.__vel.angular.z
            self.__pub.publish(msg)
            
            #print("konum: {} hedef: {} hız: {} kalan mesafe: {} frenleme mesafesi: {} frenleme: {} {}".format(self.ang,ang,self.__vel.angular.z,d,bd,d <= bd,self.__angular_breaking_acceleration * dt * s))
            
            t = rospy.Time.now().to_sec()
            dt = t - lst_t
            lst_t =t
            
            rate.sleep()
        
        self.__cmd_vel.publish(Twist())
        print("Hedefe ulaşıldı.")
        
    def MoveForward(self,speed):       
        self.__vel = Twist()
        self.__vel.linear.x = speed
        self.__cmd_vel.publish(self.__vel)
        msg = robotstatusmsg()
        msg.speed = math.sqrt(self.__vel.linear.x ** 2 + self.__vel.linear.y ** 2)
        msg.angular_speed = self.__vel.angular.z
        self.__pub.publish(msg)
        
    def Stop(self):       
        self.__vel = Twist()
        self.__vel.linear.x = 0
        self.__cmd_vel.publish(self.__vel)
        msg = robotstatusmsg()
        msg.speed = math.sqrt(self.__vel.linear.x ** 2 + self.__vel.linear.y ** 2)
        msg.angular_speed = self.__vel.angular.z
        self.__pub.publish(msg)
        
        
        
        
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
            #print(d,distance,self.__vel.linear.x,sep=" , ")
            d = dist()
            
            if (distance - d) <= bd:
                if abs(self.__vel.linear.x) < self.__max_linear_speed:
                    self.__vel.linear.x -= self.__breaking_acceleration * dt
            else:
                if abs(self.__vel.linear.x) < self.__max_linear_speed:
                    self.__vel.linear.x += self.__linear_acceleration * dt
                bd = b_dist()
            
            self.__cmd_vel.publish(self.__vel)
                
            t = rospy.Time.now().to_sec()
            dt = t - lst_t
            lst_t =t
            
            rate.sleep()
            
        self.__vel = Twist()
        self.__cmd_vel.publish(self.__vel)
        msg = robotstatusmsg()
        msg.speed = math.sqrt(self.__vel.linear.x ** 2 + self.__vel.linear.y ** 2)
        msg.angular_speed = self.__vel.angular.z
        self.__pub.publish(msg)
        
        print("Hedefe ulaşıldı.")
                
            
        
        
        

robot = RobotMovement(5,radians(120),0.3,0.3,radians(10),radians(10))

def Scan():
    min_val = max(robot.lidar)
    indx = robot.lidar.index(min_val)
    ret = robot.ang - indx * radians(45)
    
    return ret


while not robot.lidar_update:
    print("lidar bekleniyor")

while True:
    if robot.lidar[0] > 0.5:
        robot.MoveForward(0.2)
    else:
        robot.Stop()
        r = Scan()
        print(robot.lidar[0])
        robot.Rotate(r)
    

input()

