#!/usr/bin/env python3
#-*- coding:utf-8 -*-

'''
File: towline_control_node
Author: 김시원 (Si Won Kim)
Date: 2023-10-25
Version: v1.0
Description: 
''' 

import rospy
from util import *
from sensor_msgs.msg import LaserScan,PointField, PointCloud2
from sensor_msgs import point_cloud2
import sensor_msgs.point_cloud2 as pc2
import math, time
from math import * 
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header,Float64MultiArray,Float32,Float64
from geometry_msgs.msg import Point, Vector3
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from microstrain_inertial_msgs.msg import FilterHeading
# from os import stat_result
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
# from pyproj import Proj,Transformer,transform,CRS
from functools import partial
import numpy as np
import pandas  as pds
from functools import partial
import sys
import sensor_msgs.point_cloud2 as pc2
from atm_msg.msg import GoalInfo



class towline_control:
    def __init__(self):
        rospy.init_node('towline_control_node', anonymous=True)


        imuTopic  = rospy.get_param('/imu_topic_name', '/imu_data')
        lidarTopic= rospy.get_param('lidar_topic_name', '/navi/ray_cloud')  

        self.subLaser = rospy.Subscriber( lidarTopic, LaserScan, self.Callback_lidar)
        self.subImu = rospy.Subscriber(imuTopic, FilterHeading, self.Callback_imu)

        while not rospy.is_shutdown():
            if self.ray is None:
                pass
            else:
                self.process()

    def Callback_imu(self, float32):
        self.yaw = (float32.heading_rad) 

    def Callback_lidar(self, msg):
        self.length = len(msg.ranges)
        self.dot=msg.ranges
    
    def process(self):
        
        #* 데이터 확인
        if (self.dot is None):
            return
        
        if self.prevTime is None:
            self.prevTime = rospy.get_rostime()

            lineC_Hz = 0
        else:
            currentTime = rospy.get_rostime()
            dt = ( currentTime-self.prevTime).to_sec()
            self.prevTime = currentTime
            lineC_Hz = round(1/dt,2)

        cloud = []
        object = []
        object_group = []
        
        for i in range(0, len(self.dot)):
        
            self.angle = ((i)*(360.0/(self.length))-180)

        if 180 <= self.angle <= 360 :
            self.angle = self.angle - 2*math.pi
        else:
            pass
        
        if self.dot[i] != 0.0:
            x = self.dot[i]*math.cos(self.angle*(math.pi/180))
            y = self.dot[i]*math.sin(self.angle*(math.pi/180))
            self.all_points.append((x,y))
            how_long_distance = math.sqrt((x-0)**2 + (y-0)**2)
            
            if how_long_distance > 5:
                pass
            elif how_long_distance < 0.25:
                pass
            else:
                
                if x < 0:
                    pass
                else:
                    cloud.append((x,y))
        
        init = True
        for point in cloud:
            if init == True:
                init = False
                prev_point = point
            else:
                point_dist = distance(point, prev_point)

                if point_dist <= 1:
                    object.append(prev_point)

                    if point == cloud[-1]:
                        object_group.append(object)
                    else:
                        pass

                elif point_dist > 1:

                    object.append(prev_point)
                    object_group.append(object)
                    object = [point]

                prev_point = point