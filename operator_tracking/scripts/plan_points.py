#!/usr/bin/env python

#Author: Justin Steinberg
#Description: This script creates the planning_points Node that processes specified Lidar 
#             laser data from the tracking_V2.py Node. It creates the path and goal points
#             for the robot to follow and sends it to the DWA Planner. There have also been
#             Updates made to Interactive_waypoints package and operator_gui package.

import rospy
import geometry_msgs.msg
from std_msgs.msg import String, Bool 
import math
import tf
import numpy as np
import interactive_waypoints.msg
import sys
import dynamic_reconfigure.client

class Waypoints(object):


    def __init__(self):

        self.start = True
        self.tracking = True  
        self.plan_sent = False

       #Changing the navigation speed of the robot

        self.dwaConfigCli = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS",timeout=3,config_callback=None)
        self.navSpeed = .4
        self.navSpeed = self.dwaConfigCli.update_configuration({"max_vel_x":self.navSpeed})["max_vel_x"]

        self.size = 0
        self.distance = 0  
        self.distance2 = 0
        self.dist = None

        self.robot_position = None

        self.Goalpoint = geometry_msgs.msg.PoseStamped()
        self.current_poses = geometry_msgs.msg.PoseArray()
        self.current_poses.poses.append([])
        self.avg_x = 0
        self.avg_y = 0

        self.prevPose = geometry_msgs.msg.Pose()
        self.prevPose.position.x = .3048
        self.prevPose.position.y = 0

        self.processThrottle = 0 
        self.processThrottle2 = 0

        self.orientation_2 = None
        self.orientation_3 = None

        self.orientation = math.pi/4
        self.Goalpoint.pose.orientation.x = 0 
        self.Goalpoint.pose.orientation.y = 0 
        self.Goalpoint.pose.orientation.z = math.sin(math.pi/4)
        self.Goalpoint.pose.orientation.w = math.cos(math.pi/4)
    
        self.sub_point = rospy.Subscriber('operator_position', geometry_msgs.msg.PointStamped, self.OperatorCallback)
        self.robot_pos_sub = rospy.Subscriber('slam_out_pose', geometry_msgs.msg.PoseStamped, self.RobotCallback)
        self.points_pub = rospy.Publisher('plan_points', geometry_msgs.msg.PoseArray, queue_size=10)
        self.pub_points_timer = rospy.Timer(rospy.Duration(0.2), self.PubPoints)
        self.pub_goal = rospy.Publisher('move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size = 10)
        self.pause_robot = rospy.Publisher('cmd_vel_mux/input/teleop', geometry_msgs.msg.Twist, queue_size = 10)
        
        self.pub_to_way = rospy.Publisher('tracking_path', interactive_waypoints.msg.Path, queue_size=2) 

#def OperatorCallback(self.msg):
#   Here the input msg is the averaged scan data received from the Lidar. The data is converted into
#   a Pose and is stored in a Pose array. The Pose array represents the points the robot will drive
#   along. The newest received value will be the goal, it will always be the head of the path. 

    def OperatorCallback(self, msg):

        if self.processThrottle > 1:

            self.processThrottle = 0
            
            self.prevPose.position.x = self.avg_x      
            self.prevPose.position.y = self.avg_y  

            prev_p = self.prevPose.position
            prev_o = self.prevPose.orientation

            new_var = geometry_msgs.msg.Pose()
           # new_var.header.stamp = rospy.Time.now()
           # new_var.header.frame_id = "map"
            new_var.position.x = self.prevPose.position.x
            new_var.position.y = self.prevPose.position.y            

           # rospy.loginfo("Prev Pos: [%f %f %f]"%(prev_p.x, prev_p.y, prev_p.z))
           # rospy.loginfo("Prev Quat: [%f %f %f %f]"%(prev_o.x, prev_o.y, prev_o.z, prev_o.w))
         
            if len(self.current_poses.poses) == 0:

                self.current_poses.poses.append(new_var)

            else:
                      

                self.Path_Refiner(new_var)
            
            # rospy.loginfo("Appending Prev Pose to Current Pose")
            # rospy.loginfo("Current Poses Length: " + str(len(self.current_poses.poses)))

            self.avg_x = msg.point.x
            self.avg_y = msg.point.y        

            if msg.point.y != 0 or msg.point.x != .9144: #Start tracking initiaed when tracking box has moved from its initial position 
                self.tracking = True            

               # rospy.loginfo_throttle(10,"Tracking is True")

               # rospy.loginfo("Tracking is True")


            self.orientation = math.atan2(msg.point.y-self.robot_position.y, msg.point.x-self.robot_position.x)
            q = tf.transformations.quaternion_from_euler(0,0,self.orientation) #pitch/roll/yaw

            self.Goalpoint.pose.orientation = geometry_msgs.msg.Quaternion(q[0],q[1],q[2],q[3])
            self.Goalpoint.header.frame_id = "map"
            self.Goalpoint.header.stamp = rospy.Time.now()
            self.Goalpoint.pose.position.x = msg.point.x
            self.Goalpoint.pose.position.y = msg.point.y
            self.Goalpoint.pose.position.z = 0

            if self.tracking == True:
                position = self.Goalpoint.pose.position
                quat = self.Goalpoint.pose.orientation
#                rospy.loginfo("Publishing Goal Msg!")
#                rospy.loginfo("Goal Position: [%f, %f, %f] "%(position.x, position.y, position.z))
#                rospy.loginfo("Goal Quat: [%f, %f, %f, %f] "%(quat.x, quat.y, quat.z, quat.w))
                if self.plan_sent == True:
                    
                    if len(self.current_poses.poses) <= 1: 
                        self.pub_goal.publish(self.Goalpoint) 
                        self.plan_sent = False                     
                    else:
                        self.Goal_Refiner()  

        self.processThrottle += 1

#def Path_Refiner(self, new_var):    
#   The path refiner function ensures that the incoming data will not be overloading the DWA planner with too many points to update, as well as make 
#   a smoother path for the robot to follow. It makes sure the points are evenly spaced out along the path and that 
#   their orientations are in the correct direction. 

    def Path_Refiner(self, new_var):

        self.size = len(self.current_poses.poses)

        smooth = []

        if self.size > 0:

            self.distance = math.sqrt((new_var.position.x-self.current_poses.poses[self.size-1].position.x)**2 + (new_var.position.y-self.current_poses.poses[self.size-1].position.y)**2)        
            if self.distance >= .2:

                self.orientation_2 = math.atan2(new_var.position.y-self.current_poses.poses[self.size-1].position.y, new_var.position.x-self.current_poses.poses[self.size-1].position.x)
                q2 = tf.transformations.quaternion_from_euler(0,0,self.orientation_2)
                self.current_poses.poses[self.size-1].orientation = geometry_msgs.msg.Quaternion(q2[0],q2[1],q2[2],q2[3])

                self.current_poses.poses.append(new_var)

# ----------------Unused Section of the code that I'm leaving in for others to view. --------------------
#This commented out section of the code along with the un-used SmoothListGaussian/ weightedSmooth
#functions were failed attempts to smooth out the path so it would not be as choppy due to errors
#in the measurement of the position of an object (in our case a human). The path would appear quite 
#angular instead of properly smooth, which would improve robot movement. 

      #  self.weightedSmooth()

        #for i in range(len(self.current_poses.poses)):

         #   w.append(self.current_poses.poses[i].orientation.w) 
                
        #smooth = self.smoothListGaussian(w)

      #  print "COMP: ", len(smooth), len(self.current_poses.poses)

       # if self.size > 1:

        #    for i in range(self.size):  

         #       self.current_poses.poses[i].orientation.w = smooth[i]

    
    def smoothListGaussian(self, list, rippedXs=False,degree=1):  

         window=degree*2-1  

         weight=np.array([1.0]*window)  

         weightGauss=[]  

         for i in range(window):  

             i=i-degree+1  

             frac=i/float(window)  

             gauss=1/(np.exp((4*(frac))**2))  

             weightGauss.append(gauss)  

         weight=np.array(weightGauss)*weight  

         smoothed=[0.0]*(len(list)*window)  

         for i in range(len(smoothed)):  

             smoothed[i]=sum(np.array(list[i:i+window])*weight)/sum(weight)  

         return smoothed  

    def weightedSmooth(self):

        if len(self.current_poses.poses) >= 3:

            array = []

            for i in range(len(self.current_poses.poses)):
                array.append(self.current_poses.poses[i].orientation.w)
 
            weights = [.15, .7, .15]

            out = np.convolve(array,np.array(weights)[::-1],'same')

            for j in range(len(out)):
                self.current_poses.poses[j].orientation.w = out[j]
#------------------------------------------------------UNUSED SECTION ENDS HERE --------------------------------------------------------------------------

#Def Goal_Refiner(self):
#   The Goal_Refiner Function ensures that the goal has the desired orientation and distance (meters) from the previous goal. If it meets
#   the specified requirements it will publish the goal to the move_base_simple/goal topic.

    def Goal_Refiner(self):

        self.size = len(self.current_poses.poses)      
 
        if self.size > 1:

            self.distance2 = math.sqrt((self.Goalpoint.pose.position.x-self.current_poses.poses[self.size-1].position.x)**2 + (self.Goalpoint.pose.position.y-self.current_poses.poses[self.size-1].position.y)**2) 

            if self.distance2 >= .2:

                self.orientation_3 = math.atan2(self.Goalpoint.pose.position.y-self.current_poses.poses[self.size-1].position.y, self.Goalpoint.pose.position.x-self.current_poses.poses[self.size-1].position.x)      
                q3 = tf.transformations.quaternion_from_euler(0,0,self.orientation_3)
                self.Goalpoint.pose.orientation = geometry_msgs.msg.Quaternion(q3[0],q3[1],q3[2],q3[3])

                self.pub_goal.publish(self.Goalpoint)

#Def RobotCallback(self,msg):
#    The RobotCallback receives the position of the robot and makes sure that the robot does not come too close to 
#    the position of the operator. If the robot comes too close it ruins tracking, and we do not want the robot 
#    running into the operator when they stop moving. When the path is sent to interative_waypoints when the code
#    is shutdown the robot will run along the entire path.  

    def RobotCallback(self, msg):    
        
        self.robot_position = msg.pose.position

        if self.start == True:
            self.current_poses.poses[0] = msg.pose
            self.start = False

        self.dist = math.sqrt((self.Goalpoint.pose.position.x-self.robot_position.x)**2 + (self.Goalpoint.pose.position.y-self.robot_position.y)**2)
        
        if self.dist < .7:
          
            self.pub_goal.publish(msg)

#def PubPoints(self):
#   The PubPoints function publishes the path on a timer to the plan_points topic.
#   It makes sure that we are tracking an operator, and if we have enough points
#   to make a suitable path, then we send it to the DWA Planner. If there are not
#   enough points for sime reason it sends an empty poseArray which allows for
#   the DWA planner to create/optimize its own path.

    def PubPoints(self):
        msg = geometry_msgs.msg.PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"     

        if len(self.current_poses.poses) > 2:
            if self.tracking == True:
                msg.poses = self.current_poses.poses
                self.points_pub.publish(msg)
                self.plan_sent = True     
      
        else:   
            if self.tracking == True:
                msg.poses = []
                self.points_pub.publish(msg)
                self.plan_sent = True

#-----The def stop_tracking(self, string) function is obviously un-used but left in for record dexterity purposes-----
#This was the first attempt at shutting down the nodes and sending the path/goal to
#the interactive_waypoints through user input. It converts the PoseArray/ PoseStamped
#messages into a custom waypoint message that is used in the interacive_waypoints package.

    #def stop_tracking(self, string):
     #   if string == "y" or string == "yes":
      #      path = interactive_waypoints.msg.Path()
       #     path.waypoints.append(interactive_waypoints.msg.Waypoint(String("Start"),Bool(False),self.current_poses.poses[0],[])
        #    if len(self.current_poses.poses) > 1:
         #       path.waypoints.append(interactive_waypoints.msg.Waypoint(String("Start"),Bool(False),self.current_poses.poses[1],[])
          #  if len(self.current_poses.poses) > 4: 
           #     path_tail = interactive_waypoints.msg.Waypoint(String("null"),Bool(True),self.current_poses.poses[len(self.current_poses.poses)-1],self.current_poses.poses[2:len(self.current_poses.poses)-2])
            #else:
             #   path_tail = interactive_waypoints.msg.Waypoint(String("null"),Bool(True),self.current_poses.poses[len(self.current_poses.poses)-1],self.current_poses.poses[3])

           # path.waypoints.append(path_tail)
           # self.pub_to_way.publish(path)                

#def shutdown(self):
#    The shutdown function is what is run on shutdown of the nodes. The shutdown "medium" is through the operator_gui, thus would
#    not work through user input on the keyboard that I previously implemented. On shutdown it sends the Path() message to interactive_waypoints
#    package where it can save the desired path for the robot to move along without tracking. It splices the PoseArray so that
#    each part is sent in correct order. If no path was established before shutdown it does not publish.

    def shutdown(self):
        if len(self.current_poses.poses) > 2:      
      
            path = interactive_waypoints.msg.Path()
            path.waypoints.append(interactive_waypoints.msg.Waypoint(String("Start"),Bool(False),self.current_poses.poses[0],[]))
            path.waypoints.append(interactive_waypoints.msg.Waypoint(String("Start"),Bool(False),self.current_poses.poses[1],[]))

            if len(self.current_poses.poses) > 4:
                path_tail = interactive_waypoints.msg.Waypoint(String("null"),Bool(True),self.current_poses.poses[len(self.current_poses.poses)-1],self.current_poses.poses[2:len(self.current_poses.poses)-1])

            else:
                path_tail = interactive_waypoints.msg.Waypoint(String("null"),Bool(True),self.current_poses.poses[len(self.current_poses.poses)-1],self.current_poses.poses[2])

            path.waypoints.append(path_tail)
            self.pub_to_way.publish(path)

        else:
            rospy.loginfo("No Path Was Established") 

if __name__=="__main__":
    rospy.init_node("planning_points", disable_signals = True)
    iMarkers = Waypoints()
  
#    print "Do you want to end tracking mode? (y/yes)" 
#    arg = raw_input()
#    if arg == "y" or arg == "yes":
#        iMarkers.stop_tracking(arg)
#        rospy.signal_shutdown('Quit')

    rospy.on_shutdown(iMarkers.shutdown)

    rospy.spin()


