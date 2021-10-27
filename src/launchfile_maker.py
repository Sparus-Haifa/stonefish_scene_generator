class LaunchFileMaker:
    def __init__(self, scn_file_path, rate, resolution, output_bagfile_path):
        self.scn_file_path = scn_file_path
        self.rate = rate
        self.resolution = resolution
        self.output_bagfile_path = output_bagfile_path
        self.arr = [
"""
<!--Autogenerated file-->
<launch>
    <!-- Configurable arguments -->
    <arg name="robot_name" default="sparus2"/>
    <arg name="enable_keyboard" default="true"/>
    <arg name="enable_joystick" default="true"/>
    <arg name="joystick_device" default="/dev/input/js0"/>""",
"""<arg name="enable_gui" default="{}"/>""".format("true"),
"""
<arg name="enable_rviz" default="true"/>
<arg name="is_simulation" default="true"/>
    
    
    <!-- Run Simulator -->
    <include file="$(find stonefish_ros)/launch/simulator.launch">
        <arg name="simulation_data" value="/home/data/assets"/>""",
"""<arg name="scenario_description" value="{}"/>""".format(self.scn_file_path),
"""<arg name="simulation_rate" value="{}"/>""".format(self.rate),
"""<arg name="graphics_resolution" value="{} {}"/>""".format(self.resolution[0], self.resolution[1]),
"""
        <arg name="graphics_quality" value="high"/>
    </include>
    



    <node pkg="rosbag" type="record" name="rosbag_record_cam"
       args="record -O """,
       """{} """.format(self.output_bagfile_path),
       """/camera2/image_raw/camera_info /camera2/image_raw/image_color /image_depth2/camera_info /image_depth2/image_depth /sparus2/Blue_view_M900_FLS2/display /sparus2/Blue_view_M900_FLS3/display /sparus2/Blue_view_M900_FLS4/display"
        />



</launch>

                    """]

    # <!-- RViz -->
    # <group if="$(arg enable_rviz)">
    #     <node pkg="rviz" type="rviz" name="rviz" args="-d $(find cola2_sparus2)/scene/sparus2.rviz"/>
    # </group>

    
    def generate_xml(self, arr):
        return "\n".join(arr)


    def save_to_file(self, launch_file_path):
        f = open(launch_file_path, "w")
        xml = self.generate_xml(self.arr)
        f.write(xml)
        f.close()




def main():
    scene_file_path = '/home/ilan/catkin_ws/src/stonefish_scene_generator/data'
    maker = LaunchFileMaker(scene_file_path, 300, (1920, 1080))
    launch_file_path = '/home/ilan/catkin_ws/src/stonefish_scene_generator/launch/test.launch'
    maker.save_to_file(launch_file_path)


if __name__=="__main__":
    main()