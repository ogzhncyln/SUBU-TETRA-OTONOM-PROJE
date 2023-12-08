#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from find_object_2d.msg import ObjectsStamped
import rospy

class ObjectDetection:
    def __init__(self):
        rospy.init_node("objd_node")
        rospy.Subscriber("objectsStamped",ObjectsStamped,self.Detect)
        rospy.spin()
    
    def Detect(self,data):
        try:
            self.id = data.objects.data[0]
            print(self.id)
            if self.id == 1:
                print("Obje bulundu")
        except IndexError:
            print("Obje bulunamadı")
        
ObjectDetection()
