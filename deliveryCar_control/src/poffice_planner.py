#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sys

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import Int32
from zero100_msgs.msg import Heading
from lib.utils_final import pathReader,findLocalPath,purePursuit

class DeliveryCarPlanner():
    def __init__(self):
        rospy.init_node('deliveryCar_planner')
        arg = rospy.myargv(argv = sys.argv)
        self.path_name = 'poffice_utm' #.txt file name
        rate = rospy.Rate(5)

        self.pose_msg = Odometry()
        self.yaw_msg = Heading()
        pure_pursuit = purePursuit()
        look_steering_point = Point()

        #publisher
        pub = rospy.Publisher("/deliveryCar_cmd", Int32, queue_size=1)
        
        #subscriber
        rospy.Subscriber("/odom", Odometry, self.pose_callback)
        rospy.Subscriber("/ipad/heading", Heading, self.heading_callback)
        
       #Path 생성에 대한 객체, 변수
        path_reader = pathReader('deliveryCar_control')
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
            steering = (int)(80-self.steering_angle)
            rospy.loginfo("steering :{0}".format(steering))
            pub.publish(steering)

            rospy.loginfo("pub.get_num_connections() : {0}".format(pub.get_num_connections()))
            

            rate.sleep()


    def pose_callback(self,data):
        self.pose_msg=Odometry()
        self.pose_msg=data

    def yaw_callback(self,data):
        self.yaw_msg=Heading()
        self.yaw_msg=data
    

    
if __name__ == '__main__':
    try:
        pathtracking= DeliveryCarPlanner()
        
    except rospy.ROSInterruptException:
        pass
