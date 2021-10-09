#!/usr/bin/env python

from sys import stdin, version
import os

if version.startswith('3'):
    print('python 3 not supported')
    exit(1)

import rospy
import rospkg
import roslaunch
# import time

from sensor_msgs.msg import CameraInfo, Image



class SimulationController:
    def __init__(self, launch_file_path):
        self.launch_file_path = launch_file_path
        self.ros_is_running = False
        self.timer = None
        

        # full_path = getPath()
        full_path = self.launch_file_path

        # roslaunch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [full_path])

    

    def run(self):
        self.launch.start()
        rospy.loginfo("started")


    
    def wait_for_ros(self):
        rospy.loginfo("waiting for topic msg")
        rospy.init_node('sim_controller', anonymous=False, disable_signals=True)
        
            
 
        
        print('waiting for message')
        msg = rospy.wait_for_message("/camera2/image_raw/camera_info", CameraInfo, timeout=None)
        print('wait is over')
        self.timer = rospy.get_time()


        r = rospy.Rate(10) # 10hz
        
        while not rospy.is_shutdown():
            # if not self.timer is None:
            if rospy.get_time() - self.timer > 15:
                print('rospy.get_time() - self.timer > 15')
                print(rospy.get_time(), self.timer)
                break
            r.sleep()


        rospy.loginfo('timeout: shutting down')
        # rospy.signal_shutdown('timeout')
        self.launch.shutdown()
        # self.process.terminate()




def main():



    controller = SimulationController('/home/ilan/catkin_ws/src/stonefish_scene_generator/launch/audo_generated.launch')
    controller.run()
    controller.wait_for_ros()


if __name__=='__main__':
    main()