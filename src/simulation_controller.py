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

from sensor_msgs.msg import CameraInfo


class SimulationController:
    def __init__(self):
        self.ros_is_running = False
        self.timer = None
        


        def getPath():
            # get path
            # get an instance of RosPack with the default search paths
            rospack = rospkg.RosPack()

            # list all packages, equivalent to rospack list
            # rospack.list() 

            # get the file path for rospy_tutorials
            package_path = rospack.get_path('stonefish_scene_generator')


            full_path = os.path.join(package_path, 'launch','sparus2_haifa_deepersense_simulation.launch')
            return full_path

        full_path = getPath()
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [full_path])

    def run(self):
        self.launch.start()
        rospy.loginfo("started")

    
    def wait_for_ros(self):
        rospy.init_node('sim_controller', anonymous=True)
        

        def callback(data):
            if self.timer is None:
                rospy.loginfo('got ROS topic (camera info). starting timer')
                self.timer = rospy.get_time()
                return

            
            
        
        rospy.Subscriber("/camera2/image_raw/camera_info", CameraInfo, callback, queue_size=1)



        r = rospy.Rate(10) # 10hz
        
        while not rospy.is_shutdown():
            if not self.timer is None:
                if rospy.get_time() - self.timer > 15:
                    break
            r.sleep()



        rospy.signal_shutdown('timeout')
        self.launch.shutdown()




def main():



    controller = SimulationController()
    controller.run()
    controller.wait_for_ros()


if __name__=='__main__':
    main()