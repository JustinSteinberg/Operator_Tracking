#!/usr/bin/env python
 
##Justin Steinberg 

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point  
import math

CONST_VALUE = .6096/3  #1ft in meters 

class Operator_Tracking(object):

    def __init__ (self):
        self.sub = rospy.Subscriber('scan_filtered', LaserScan, self.tracking_callback)
	self.pub = rospy.Publisher('visualization_marker', Marker, queue_size = 10)  
       # self.goal_pub = ropsy.Publisher('move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size = 10)
       # self.pub_points_timer = rospy.Timer(rospy.Duration(0.05), self.PubPoints)
        self.avg_pub = rospy.Publisher('operator_position', Point, queue_size = 10)         

        self.avg_point = Point()

        self.marker = Marker()  
        self.list_x = []
        self.list_y = []

        self.count = 0.0
        self.sum_sqr_dist_x = 0
        self.sum_sqr_dist_y = 0

        self.prev_std_dev_x = 0   
        self.prev_std_dev_y = 0
    
        self.std_dev_x = .3048
        self.std_dev_y = .3048

        self.average_x = .9144    #x coordinate Distance from robot 
        self.average_y = 0.0      #y coordinate Distance from robot

        self.prev_avg_x = 0.0
        self.prev_avg_y = 0.0         

    def create_box(self, average_x, average_y, std_dev_x, std_dev_y):

        self.marker.header.frame_id = "laser"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD

        #marker scale
        self.marker.scale.x = .03
        self.marker.scale.y = .03
        self.marker.scale.z = .03
   
        #marker color
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
    
        #marker orientation
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        #marker position
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
  
        #marker line points
        self.marker.points = []
		
        #first point
        first_line_point = Point()
        first_line_point.x = self.average_x + CONST_VALUE
        first_line_point.y = self.average_y + CONST_VALUE
        first_line_point.z = 0.0
        self.marker.points.append(first_line_point)
        #second point
        second_line_point = Point()
        second_line_point.x = self.average_x + CONST_VALUE
        second_line_point.y = self.average_y - CONST_VALUE
        second_line_point.z = 0.0
        self.marker.points.append(second_line_point)
        #third point
        third_line_point = Point()
        third_line_point.x = self.average_x - CONST_VALUE
        third_line_point.y = self.average_y - CONST_VALUE
        third_line_point.z = 0.0
        self.marker.points.append(third_line_point)
        #fourth point
        fourth_line_point = Point()
        fourth_line_point.x = self.average_x - CONST_VALUE
        fourth_line_point.y = self.average_y + CONST_VALUE
        fourth_line_point.z = 0.0
        self.marker.points.append(fourth_line_point)
        #fifth point
        fifth_line_point = Point()
        fifth_line_point.x = self.average_x + CONST_VALUE
        fifth_line_point.y = self.average_y + CONST_VALUE
        fifth_line_point.z = 0.0
        self.marker.points.append(fifth_line_point)

    #The callback creates the initial bounds for the box, the box containing the only
    #points we care to analyize. The average of all the points in the box is then computed.
    #Using that average, it will be the center of the next box, changing the bounds of the
    #initial box, thus tracking those points. 

    def tracking_callback(self, msg):

        self.count = 0
        self.sum_dist_x = 0
        self.sum_dist_y = 0
        self.sum_sqr_dist_x = 0        
        self.sum_sqr_dist_y= 0
        self.list_x = []
        self.list_y = []        

        for i, value in enumerate(msg.ranges):    
        
            if value*math.cos(msg.angle_min+msg.angle_increment*i) < self.average_x + CONST_VALUE and  value*math.cos(msg.angle_min+msg.angle_increment*i) > self.average_x - CONST_VALUE:  
 
                if value*math.sin(msg.angle_min+msg.angle_increment*i) >= self.average_y - CONST_VALUE and value*math.sin(msg.angle_min+msg.angle_increment*i) <= self.average_y + CONST_VALUE:       
                
                    self.list_x.append(value*math.cos(msg.angle_min+msg.angle_increment*i))
                    self.sum_sqr_dist_x += ((value*math.cos(msg.angle_min+msg.angle_increment*i))-self.average_x)**2
                    self.list_y.append(value*math.sin(msg.angle_min+msg.angle_increment*i))
                    self.sum_sqr_dist_y += ((value*math.sin(msg.angle_min+msg.angle_increment*i))-self.average_y)**2
                   
        self.count = len(self.list_x)
        print "Count:", self.count
        
        if self.count == 0:
            if self.prev_avg_x == 0:
                self.average_x = .9144
                self.average_y = 0

                self.std_dev_x = .3048
                self.std_dev_y = .3048
                print "No particles in area"

            else:
                self.average_x = self.prev_avg_x
                self.average_y = self.prev_avg_y
 
                self.std_dev_x = .3048 
                self.std_dev_y = .3048

                print "The average x was:", self.average_x, "The average y was:", self.average_y
        else:
            self.prev_avg_x = self.average_x
            self.prev_avg_y = self.average_y
          
            self.prev_std_dev_x = self.std_dev_x     
            self.prev_std_dev_y = self.std_dev_y
            
            
            self.std_dev_x = (math.sqrt(self.sum_sqr_dist_x/self.count)*2 + self.prev_std_dev_x)/2 
            self.std_dev_y = (math.sqrt(self.sum_sqr_dist_y/self.count)*2 + self.prev_std_dev_y)/2

            self.average_x = sum(self.list_x)/len(self.list_x)
            self.average_y = sum(self.list_y)/len(self.list_y)
            print "M,x Dist:", self.average_x, "| M,y Dist:", self.average_y, "Std,x:", self.std_dev_x, "Std,y:", self.std_dev_y

        self.avg_point.x = self.average_x
        self.avg_point.y = self.average_y
        self.create_box(self.average_x, self.average_y, self.std_dev_x, self.std_dev_y)
        self.pub.publish(self.marker)
        self.avg_pub.publish(self.avg_point)

if __name__ == "__main__":
	rospy.init_node('scan_values')
        Tracker1 = Operator_Tracking()
	rospy.spin()
