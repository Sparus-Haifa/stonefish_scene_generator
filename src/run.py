import os
# import sys
import rospy
import rosbag
import rospkg
# import glob
import rospy
from collections import defaultdict
import cv2
import numpy as np
from cv_bridge import CvBridge

from simulation_controller import SimulationController
from scene_generator import SceneGenerator, ColorMap
from launchfile_maker import LaunchFileMaker


def make_a_scene():
    seabed_depth = 15
    gen = SceneGenerator(seabed_depth)
    gen.add_empty_frame(ColorMap.jet)
    # gen.add_boxes()
    gen.add_reefs()
    scene = gen.generate()
    return scene


def save_scene_to_file(scene, scene_file_full_path):
    f = open(scene_file_full_path + '.scn', "w")
    f.write(scene)
    f.close()


def create_folder(path):
    dir_exists = os.path.isdir(path)
    if not dir_exists:
        os.mkdir(path)
        print('folder created: {}'.format(path))

def imshow(win, img):
    cv2.imshow(win, img)
    cv2.waitKey(10)

def process_image(msg, topic):
    bridge = CvBridge()
    encoding = 'bgr8'
    img = np.asarray(bridge.imgmsg_to_cv2(msg, encoding))
    return img

def process_depth(msg, topic):
    bridge = CvBridge()
    encoding = '32FC1'
    img = np.asarray(bridge.imgmsg_to_cv2(msg, encoding))
    return img

def normaliaze_depth(msg):
    cv_image_array = np.array(msg, dtype = np.dtype('f8'))
    slice1Copy = cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
    slice1Copy = np.uint8(slice1Copy*255)
    return(slice1Copy)

def save_image(img, filename):
    # print(file_name)
    cv2.imwrite(filename, img)


def main():
    scene = make_a_scene()
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('stonefish_scene_generator')
    # scene_folder_path = os.path.join(package_path, 'scene')
    scene_file_full_path = os.path.join(package_path, 'scene', 'basic')
    save_scene_to_file(scene, scene_file_full_path)
    data_folder_path = '/home/data/'
    # create_folder(data_folder_path)
    launch_file_name = 'auto_generated.launch'
    launch_file_path = os.path.join(package_path, 'launch', launch_file_name)
    print(launch_file_path)
    rate = 300.0
    resolution = (1200, 800)
    # data_folder_path = '/home/data'
    bagfile_path = os.path.join(data_folder_path, 'basic' + '.bag')
    maker = LaunchFileMaker(scene_file_full_path + '.scn', rate, resolution, bagfile_path)
    maker.save_to_file(launch_file_path)
    controller = SimulationController(launch_file_path)
    controller.run()
    controller.wait_for_ros()
    bag = rosbag.Bag(bagfile_path, 'r')
    image_folder_path = data_folder_path





    iterator = bag.read_messages()
    counter = defaultdict(lambda: 0)
    image_num = 0
    for topic, msg, time in iterator:
        msg_time = rospy.Time.to_sec(time) # .to_time(time)
        msg_type = str(type(msg)).split('__')[1].split("'")[0]
        print(msg_time, msg_type, topic)
        counter[topic]+=1
        if msg_type == 'CameraInfo':
            continue
        image_num += 1

        if 'image_color' in topic or 'display' in topic:
            bridged_image = process_image(msg, str(topic))
            imshow(topic, bridged_image)
            file_name = os.path.join(image_folder_path, str(image_num) + '.tif')
            save_image(bridged_image, file_name)



        elif 'image_depth2/image_depth' in topic:
            bridged_image = process_depth(msg, str(topic))
            norm_image = normaliaze_depth(bridged_image)
            imshow(topic, norm_image)
            file_name = os.path.join(image_folder_path, str(image_num) + '.tif')
            save_image(norm_image, file_name)
        else:
            print(topic)

    cv2.destroyAllWindows()
# def main():
#     print('hello')


#     seabed_depth = 15
#     gen = SceneGenerator(seabed_depth)
#     gen.add_empty_frame(ColorMap.jet)
#     gen.add_reefs()
#     scene = gen.generate()
#     print(scene)


if __name__=='__main__':
    main()