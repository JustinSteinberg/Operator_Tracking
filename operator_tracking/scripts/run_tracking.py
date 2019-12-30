#!/usr/bin/env python

import rospy
import roslaunch

def launch():
    uuid = roslaunch.rlutil.get_or_generate_uuid(None,False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid,['/home/ubuntu/catkin_ws/src/operator_tracking/launch/my_launch.launch']) 
    launch.start()

    launch.shut_down()
#if __name__ == "__main__":

launch()
