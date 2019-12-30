#!/usr/bin/env python
 
##Author: Justin Steinberg 
##Description: This tracking script received scanning laser data from the Lidar mounted on the robot
##             and trackes a certain set of points that falls within the tracking parameters. Using the 
##             the data points within the parameters or "box", the data is then sent to the plan_points
##             code for further processing.

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped, PoseStamped
import math
import tf
import numpy as np

CONST_VALUE = .3  #1ft in meters 

class Operator_Tracking(object):

    def __init__ (self):
        self.sub = rospy.Subscriber('scan_filtered', LaserScan, self.tracking_callback)
    #    self.pose_sub = rospy.Subscriber('/slam_out_pose', PoseStamped, self._poseCallback)

	self.pub = rospy.Publisher('visualization_marker', Marker, queue_size = 10)  
    #    self.waypoint_pub = rospy.Publisher('/interactive_waypoint_markers/add_waypoint', PoseStamped, queue_size=10) 
        self.avg_pub = rospy.Publisher('operator_position', PointStamped, queue_size = 10)
  
        self.listener = tf.TransformListener()

        self.avg_point = PointStamped()
        self.avg_point.header.frame_id = 'base_link'
        self.avg_point.header.stamp = rospy.Time(0)

        self.tracking = False
        self.start_val = 0  
        self.end_val = 0

        self.marker = Marker() 
        self.marker2 = Marker() 
        self.list_x = []
        self.list_y = []
        self.total_val_x = []
        self.total_val_y = []
        self.total2x = []
        self.total2y = []
        self.diff_x = []
        self.diff_y = []

        self.count = 0.0
        self.sum_sqr_dist_x = 0
        self.sum_sqr_dist_y = 0

        self.prev_std_dev_x = 0   
        self.prev_std_dev_y = 0
    
        self.std_dev_x = .3048
        self.std_dev_y = .3048

        self.average_x = .9144    #x coordinate Distance from robot 
        self.average_y = 0      #y coordinate Distance from robot

        self.prev_avg_x = 0.0
        self.prev_avg_y = 0.0         

        self.cov_sum = 0.0
        self.cov_xy = 0.0
        self.corr_xy = 0.0

        self.lil_throt = 31

#def create_box(self, average_x, average_y, std_dev_x, std_dev_y):
#    this function creates a box around a set of points that the 
#    tracker is actually tracking for visualization purposes in Rviz. I prevous would change the bounds 
#    of the box using the standard deviation of the data within the box,
#    however the bounds are currenly static from the center.  


    def create_box(self, average_x, average_y, std_dev_x, std_dev_y):

        self.marker.header.frame_id = "laser"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD
        self.marker.id = 0

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
        first_line_point.x = self.average_x + CONST_VALUE #self.std_dev_x
        first_line_point.y = self.average_y + CONST_VALUE #self.std_dev_y
        first_line_point.z = 0.0
        self.marker.points.append(first_line_point)
        #second point
        second_line_point = Point()
        second_line_point.x = self.average_x + CONST_VALUE #self.std_dev_x
        second_line_point.y = self.average_y - CONST_VALUE #self.std_dev_y
        second_line_point.z = 0.0
        self.marker.points.append(second_line_point)
        #third point
        third_line_point = Point()
        third_line_point.x = self.average_x - CONST_VALUE #self.std_dev_x
        third_line_point.y = self.average_y - CONST_VALUE #self.std_dev_y
        third_line_point.z = 0.0
        self.marker.points.append(third_line_point)
        #fourth point
        fourth_line_point = Point()
        fourth_line_point.x = self.average_x - CONST_VALUE #self.std_dev_x
        fourth_line_point.y = self.average_y + CONST_VALUE #self.std_dev_y
        fourth_line_point.z = 0.0
        self.marker.points.append(fourth_line_point)
        #fifth point
        fifth_line_point = Point()
        fifth_line_point.x = self.average_x + CONST_VALUE #self.std_dev_x
        fifth_line_point.y = self.average_y + CONST_VALUE #self.std_dev_y
        fifth_line_point.z = 0.0
        self.marker.points.append(fifth_line_point)

#def create_cylinder(self):
#    This function creates a circle around the robot so you can easily view its
#    position in rviz.

    def create_cylinder(self): #, average_x, average_y, std_dev_x, std_dev_y):

        self.marker2.header.frame_id = "laser"
        self.marker2.type = self.marker.CYLINDER
        self.marker2.action = self.marker.ADD
        self.marker2.id = 1

        #marker scale
        self.marker2.scale.x = .03
        self.marker2.scale.y = .03
        self.marker2.scale.z = .03
   
        #marker color
        self.marker2.color.a = 1.0
        self.marker2.color.r = 1.0
        self.marker2.color.g = 1.0
        self.marker2.color.b = 0.0
    
        #marker orientation
        self.marker2.pose.orientation.x = 0.0
        self.marker2.pose.orientation.y = 0.0
        self.marker2.pose.orientation.z = 0.0
        self.marker2.pose.orientation.w = 1.0

        #marker position
        self.marker2.pose.position.x = 0.0
        self.marker2.pose.position.y = 0.0
        self.marker2.pose.position.z = 0.0

        #scale
        self.marker2.scale.x = .4
        self.marker2.scale.y = .4
        self.marker2.scale.z = 0

#def rest_vars(self):
#    This function simply resets all the variables to their null-values
#    because we want to re-calculate them every callback because each 
#    set of data received from the lidar is different  

    def rest_vars(self):

        self.count = 0.0
        self.sum_dist_x = 0.0
        self.sum_dist_y = 0.0
        self.sum_sqr_dist_x = 0.0        
        self.sum_sqr_dist_y= 0.0
        self.list_x = []
        self.list_y = []
        self.diff_x = []
        self.diff_y = []   
        self.cov_sum = 0.0
        self.cov_xy = 0.0
        self.corr_xy = 0.0



#def tracking_callback(self, msg):
#    This function is the heart of the tracker, it subscribes the the scan_filtered topic
#    and only publishes data defined in the bounds. It convert the data from polar to
#    cartesian coordinates, calculates the averages, std_devs, and covariance, and 
#    and transforms the frame of reference from laser to map. The tracking_callback creates the initial bounds for the box, 
#    the box containing the only points we care to analyize. The average of all the points in the box is then computed.
#    Using that averages, std_devs, it will be the center of the next box, changing the position of the box according to the 
#    the movement of the points, thus tracking those points. 


    def tracking_callback(self, msg):

        self.rest_vars()              

        for i, value in enumerate(msg.ranges):    
        
            if value*math.cos(msg.angle_min+msg.angle_increment*i) < self.average_x + CONST_VALUE and value*math.cos(msg.angle_min+msg.angle_increment*i) > self.average_x - CONST_VALUE:  
            
                if value*math.sin(msg.angle_min+msg.angle_increment*i) >= self.average_y - CONST_VALUE and value*math.sin(msg.angle_min+msg.angle_increment*i) <= self.average_y + CONST_VALUE :       
                
                    self.list_x.append(value*math.cos(msg.angle_min+msg.angle_increment*i))
                    self.sum_sqr_dist_x += ((value*math.cos(msg.angle_min+msg.angle_increment*i))-self.average_x)**2
                    self.list_y.append(value*math.sin(msg.angle_min+msg.angle_increment*i))
                    self.sum_sqr_dist_y += ((value*math.sin(msg.angle_min+msg.angle_increment*i))-self.average_y)**2

        self.count = len(self.list_x)
        print "Count:", self.count
         
        temp_x = []         
        temp_y = []

        for x,y in zip(self.list_x, self.list_y):
            if (x >= .22 or x <= -.22) or (y >=.22 or y<=-.22):
        
                temp_x.append(x)
                temp_y.append(y)

        self.list_x = temp_x 
        self.list_y = temp_y
      
        #self.list_x = [x for x in self.list_x if x >= .1 or x <= -.1] 
        #self.list_y = [y for y in self.list_y if y >= .1 or y <= -.1]      

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
                          
                self.tracking = False

                print "The average x was :", self.average_x, "The average y was :", self.average_y, "\n"
        else:

            self.tracking = True         
          #  rospy.loginfo("Tracking is True")           
 
            self.prev_avg_x = self.average_x
            self.prev_avg_y = self.average_y
          
            self.prev_std_dev_x = self.std_dev_x     
            self.prev_std_dev_y = self.std_dev_y
            
            
            self.std_dev_x = (math.sqrt(self.sum_sqr_dist_x/self.count)*2 + self.prev_std_dev_x)/2 
            self.std_dev_y = (math.sqrt(self.sum_sqr_dist_y/self.count)*2 + self.prev_std_dev_y)/2

            self.average_x = sum(self.list_x)/len(self.list_x)
            self.average_y = sum(self.list_y)/len(self.list_y)
             
            for i in range(len(self.list_x)):
                self.cov_sum += (self.list_x[i]-self.average_x)*(self.list_y[i] - self.average_y)
            
            if self.count >= 2:
                self.cov_xy = self.cov_sum/(self.count - 1)
      
                self.corr_xy = self.cov_xy/((self.std_dev_x)*(self.std_dev_y))
            else:
                print "Not enough particles \n" 
           
        #    print "M,x Dist:", self.average_x, "| M,y Dist:", self.average_y, "\n" "Std,x:", self.std_dev_x, "Std,y:", self.std_dev_y
        #    print "CoVariance:", self.cov_xy, "Corr:", self.corr_xy

        self.avg_point.point.x = self.average_x
        self.avg_point.point.y = self.average_y
        self.avg_point.point.z = 0
	  
        self.create_box(self.average_x, self.average_y, self.std_dev_x, self.std_dev_y)
        self.create_cylinder()
        self.pub.publish(self.marker)
        self.pub.publish(self.marker2)

	self.listener.waitForTransform("map","base_link",rospy.Time(0),rospy.Duration(0.4))
	self.avg_point.header.stamp = rospy.Time(0)
        self.p = self.listener.transformPoint('map', self.avg_point)
	
        if self.tracking == True:
            rospy.loginfo("Publishing Point")
            rospy.loginfo("Point Positions: [%f, %f, %f,]"%(self.p.point.x, self.p.point.y, self.p.point.z)) 
            self.avg_pub.publish(self.p)

#---------------This portion of the code is un-used but I did get to work in conjunction with the rest of the code------------------
#****However due to errors occuring after the implemention of the part of the code, which may or may not have actually caused 
#    said errors, I was forced to roll-back the code. This is what is left. I also created a function that created boxes around the clusters 
#    based on how many clusters there were so we were able to view them in Rviz.*****

#    What the cluster function did was it clustered laser_data points based on euclidean distance. If there was a large enough cluster of 
#    points then it would save them in a dictionary. Once going through the entire data set received from the laser data, using recusiion, which placed a 
#    large burden on processing power, the line_check function would determine if each cluster of data has a linear relationship past a certain threshold. If they did
#    have a linear relationship, then that cluster of data would most likely represent a wall. 

#    Being able to identify walls in the lider data would greatly improve this current tracking system so that when an operator would be in
#    a tight space or get too close to a wall, the tracking would simply ignore those points because they would be taken out of the data set,
#    leaving the tracker to only focus on the operator. It could also be used to make sure the operator does not lead the robot into a wall.
  
#""" ""
#    def cluster(self, msg, j):
#        x_value = np.array([])
#        y_value = np.array([])

#        self.dict = {}
#        cluster_id = 1
#        cluster = np.array([])

#        for i, value in enumerate(msg.ranges):
#            x_value = np.append(x_value,value*math.cos(msg.angle_min+msg.angle_increment*i))
#            y_value = np.append(y_value,value*math.sin(msg.angle_min+msg.angle_increment*i))

#        for i in range(j,x_value.size-1):
#            if abs(x_value[i]-x_value[i+1]) < .07 and abs(y_value[i]-y_value[i+1]) < .07:
#               cluster = np.append(cluster,zip(x_value[i],y_value[i])) 
               
#            else:
#                if cluster.size() > 5:
#                    j = i
#                    self.dict[cluster_id] = cluster
#                    self.cluster_id += 1  
#                    self.line_check(msg, j) 

#    def line_check(self, dict): 
#        x_value = np.array([])
#        y_value = np.array([])
#         for key, value in self.dict.iteritems():
#              x_value = np.array([])
#             y_value = np.array([])

#             for x,y in self.dict[key]:
#                 x_value = np.append(x_value,x)
#                 y_value = np.append(y_value,y)

                
#                m = (((mean(x_value)*mean(y_value)- mean(x_value*y_value))/ ((mean(x_value)*mean(x_value)-mean(x_value*x_value)))
#                b = mean(y_value)-m*mean(x_value)
       
#                regression_line = [(m*x)+b for x in x_value]

#                y_mean_line = [mean(y_value) for y in y_value]
#                squared_error_regr = sum((regression_line-y_value)*(regression_line-y_value))
#                squared_error_y_mean = sum((y_mean_line - y_value)*(y_mean_line - y_value))
#                r_squared = 1 - (squared_error_regr/squared_error_mean)
#                self.dict2['r_sqr'+ key] = r_squared
#---------------------------------------------------------------------------------------------------------------------------------


#------def tracking_filter(self, msg) is also un-used----------------------
#The goal of this function was to compare a new set of data to the previous set, and cancel out the static points so that
#the only points we would need to worry about would be the moving points, aka the operator. However the complexity that comes with
# this is that the operator can stand still for an unknown amount of time, but I'm sure this caveat could be overcome. 

    def tracking_filter(self, msg):

        if self.lil_throt > 30:

            self.total_x_val = []
            self.total_y_val = []

            self.total2x = []
            self.total2y = []

            index = 0         

            for i, value in enumerate(msg.ranges):
                self.total_x_val.append(value*math.cos(msg.angle_min+msg.angle_increment*i))
                self.total_y_val.append(value*math.sin(msg.angle_min+msg.angle_increment*i))
  

            for j in range(len(self.total_x_val)):

                if j < self.start_val and j > self.end_val:

                    self.total2x.append(self.total_x_val[j])
                    self.total2y.append(self.total_y_val[j])

                else:

                    self.total2x.append(0)      
                    self.total2y.append(0)
                    index += 1
            
            self.lil_throt = 0

        self.lil_throt += 1    
   
if __name__ == "__main__":
	rospy.init_node('scan_values')
        Tracker1 = Operator_Tracking()
	rospy.spin()
