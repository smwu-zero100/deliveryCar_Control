#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sys

from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import PoseStamped,Point,Pose
from morai_msgs.msg import Poffice
from std_msgs.msg import Int32
from ublox_msgs.msg import NavPVT
from lib.utils_final import pathReader,findLocalPath,purePursuit

class PofficePlanner():
    def __init__(self):
        rospy.init_node('Poffice_planner')
        arg = rospy.myargv(argv = sys.argv)
        self.path_name = 'poffice_utm' #.txt file name
        rate = rospy.Rate(5)

        self.pose_msg = Odometry()    #gps
        self.yaw_msg = NavPVT()       #gps에서 heading 받아옴 
        pure_pursuit = purePursuit()
        look_steering_point = Point() #steering 계산에 기준이 되는 포인트

        #publisher
        #poffice_pub = rospy.Publisher("/poffice_cmd", Poffice, queue_size=10)
        pub = rospy.Publisher("/poffice_cmd", Int32, queue_size=10)
        #subscriber
        rospy.Subscriber("/odom", Odometry, self.pose_callback)
        rospy.Subscriber("/ublox/navpvt", NavPVT, self.yaw_callback)
        
       #Path 생성에 대한 객체, 변수
        path_reader = pathReader('poffice_control')    #package name
        self.global_path=path_reader.read_txt(self.path_name+".txt")
        self.current_waypoint = 0
        self.target_angle_index = 0
        self.steering_angle = 0

        while not rospy.is_shutdown():
            local_path, self.current_waypoint=findLocalPath(self.global_path, self.pose_msg, self.current_waypoint)
            
            #pure_pursuit
            pure_pursuit.getPath(local_path)
            pure_pursuit.getPoseStatus(self.pose_msg)
            pure_pursuit.getYawStatus(self.yaw_msg)
            
            self.steering_angle, look_steering_point, self.target_angle_index = pure_pursuit.steering_angle(self.current_waypoint)
            #poffice_msg.steering = (int)(self.steering_angle + 80)
            steering = (int)(80-self.steering_angle)
            rospy.loginfo("steering :{0}".format(steering))
            pub.publish(steering)

            rospy.loginfo("pub.get_num_connections() : {0}".format(pub.get_num_connections()))
            

            rate.sleep()


    def pose_callback(self,data):
        self.pose_msg=Odometry()
        self.pose_msg=data

    def yaw_callback(self,yaw_data):
        self.yaw_msg=NavPVT()
        self.yaw_msg=yaw_data
    

    
if __name__ == '__main__':
    try:
        pathtracking= PofficePlanner()
    except rospy.ROSInterruptException:
        pass
