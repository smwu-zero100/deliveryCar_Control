#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import Float64,Int16,Float32MultiArray, Int32, Float32
#import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi
from math import *
import math
import time
import serial

class pathReader :
    def __init__(self,pkg_name):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)

    def read_txt(self,file_name):
        full_file_name=self.file_path+"/path/"+'poffice_utm.txt'
        openFile = open(full_file_name, 'r')
        out_path=Path()

        out_path.header.frame_id='/map'
        line=openFile.readlines()
        for i in line :
            tmp=i.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            #read_pose.pose.position.z=float(tmp[2])
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=1
            out_path.poses.append(read_pose)
        openFile.close()
        return out_path


def findLocalPath(ref_path, pose_msg, ref_index):
    out_path=Path()
    current_waypoint = 0
    min_dis=float('inf')

    for i in range(0, len(ref_path.poses)) : #함수 콜 하면 다시 0으로
    #조건문 상관 없이 끝까지 탐색, i는 조건문에 맞는 마지막을  //
        dx = pose_msg.pose.pose.position.x - ref_path.poses[i].pose.position.x
        dy = pose_msg.pose.pose.position.y - ref_path.poses[i].pose.position.y
        dis = sqrt(dx*dx + dy*dy)

        if dis < min_dis :
            min_dis = dis
            current_waypoint = i

        # else :
        #     rospy.loginfo("else문 in")
        #     break

    rospy.loginfo("LocalPath start waypoint : {0} min_dis : {1}\n".format(current_waypoint, min_dis))


    if current_waypoint+15 > len(ref_path.poses) :
        last_local_waypoint= len(ref_path.poses)

    else :
        last_local_waypoint=current_waypoint+15

    out_path.header.frame_id='map'
    for i in range(current_waypoint,last_local_waypoint) :
        tmp_pose=PoseStamped()
        tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
        #tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x=0
        tmp_pose.pose.orientation.y=0
        tmp_pose.pose.orientation.z=0
        tmp_pose.pose.orientation.w=1
        out_path.poses.append(tmp_pose)

    rospy.loginfo("LocalPath end waypoint : {0}\n".format(last_local_waypoint))
    return out_path, current_waypoint


class purePursuit :
    def __init__(self):
        self.forward_point=Point()
        self.current_position=Point()
        self.is_look_forward_point=False
        self.ld=2 #2
        self.min_ld=0.5 #2
        self.max_ld=5 #30
        self.vehicle_length=0.3
        self.r_vehicle_length=0.45
        self.steering=0
        self.index = 0
        self.path = Path()

    def getPath(self,msg):
        self.path=msg  #nav_ms5gs/Path

    def getPoseStatus(self,msg):
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        #self.current_postion.z=msg.position_z

    def getYawStatus(self,msg):
        self.gps_radian = msg.heading/180*pi #msg.orientation.z #/180*pi #rad
        self.gps_degree = msg.heading

#######Local Path#######
    def steering_angle(self, ref_index):
        target_waypoint=ref_index-1
        rotated_point=Point()
        path_point=Point()
        alpha = 0
        #self.ld = self.current_vel * 0.0237 + 5.29
        rospy.loginfo("gps radian : {0}".format(self.gps_radian))

        for i in self.path.poses: #i는 pose메시지 형태
            target_waypoint += 1
            #print("steering angle for문 i = {}".format(current_waypoint))
            path_point = i.pose.position
            dy = path_point.x - self.current_position.x
            dx = path_point.y - self.current_position.y

            target_enu = atan(dy/dx)*180/pi  #degree
            rotated_point.x = cos(self.gps_radian)*dx + sin(self.gps_radian)*dy
            # rotated_point.y = -sin(self.gps_yaw)*dx + cos(self.gps_yaw)*dy

            # ENU <-> NEU (UP)
            if rotated_point.x < 0 :
                dis=sqrt(pow(dx,2)+pow(dy,2))
                #dis=sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
                #rospy.loginfo("gps degree : {0}".format(self.gps_degree))
                if dis >= self.ld :
                    self.forward_point = path_point
                    rospy.loginfo("Steering target point: {0}번째 waypoint     {1}  {2}".format(target_waypoint, path_point.x, path_point.y))

                    if (self.gps_degree >= 0 and self.gps_degree <90):
                        self.target_yaw = target_enu
                    elif (self.gps_degree > 90 and self.gps_degree <=180):
                        self.target_yaw = target_enu + 180
                    elif (self.gps_degree < 0 and self.gps_degree >= -90):
                        self.target_yaw = target_enu
                    elif (self.gps_degree < -90 and self.gps_degree >= -180):
                        self.target_yaw = target_enu -180
                   
                    alpha = -(self.gps_degree - self.target_yaw)

                    if (alpha <= -180):
                        alpha = alpha + 360
                    elif (90 <= alpha < 180):
                        alpha = alpha - 180
                    
                    if (alpha >= 180) :
                        alpha = alpha - 360
                    elif (-90 >= alpha > -180):
                        alpha = alpha + 180

                    alpha = alpha*pi/180 #radian
                    theta = 2 * self.vehicle_length * sin(alpha)/self.ld
                    self.steering=atan(theta)*180/pi #deg
                    rospy.loginfo("target steering angle: {0}   look-ahead distance : {1}".format(self.steering, self.ld))
                    rospy.loginfo("alpha : {0}  gps_degree : {1}   target_yaw : {2}".format(alpha*180/pi, self.gps_degree, self.target_yaw))

                    # right : +, left : -
                    return self.steering, self.forward_point, target_waypoint

        #is_look_forward_point = false
        rospy.loginfo("no found forward point")
        self.steering = 0
        return self.steering, self.forward_point, target_waypoint


##
