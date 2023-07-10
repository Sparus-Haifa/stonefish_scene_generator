

# from xml.etree.ElementTree import Element
# from re import S
from random import seed
import xml.etree.ElementTree as xml
from xml.dom import minidom

from enum import Enum

import rospkg


class ColorMap(Enum):
    hot = 'hot'
    jet = 'jet'
    perula = 'perula'
    greenblue = 'greenblue'
    coldblue = 'coldblue'
    orangecopper = 'orangecopper'


class StoneFishSceneElement(object):
    def __init__(self, header):
        # self.tag = tag
        self.xml_element = xml.Element(header)
        # self.children = []

    def set_attribute(self, attrib_name, attrib_value):
        self.xml_element.set(attrib_name, str(attrib_value))

    def get_attribute(self, attrib_name):
        return self.xml_element.get(attrib_name)

    def toString(self):
        """Return a pretty-printed XML string for the Element.
        """
        rough_string = xml.tostring(self.xml_element, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ")

    def append(self, sf_element):
        # self.children.append(sf_element)
        self.xml_element.append(sf_element.xml_element)


class Sparus2(StoneFishSceneElement):
    def __init__(self, position=(0.0, 0.0, 0.0)):
        super(Sparus2, self).__init__("include")
        self.set_attribute("file", "$(find cola2_stonefish)/scenarios/sparus2_haifa.scn")
        self.append(Argument('position', str(position[0]) + ' ' + str(position[1]) + ' ' + str(position[2])))


class Argument(StoneFishSceneElement):
    def __init__(self, name, value):
        super(Argument, self).__init__('arg')
        self.set_attribute('name', name)
        self.set_attribute('value', value)


class WorldTransform(StoneFishSceneElement):
    def __init__(self, rpy = (0.0, 0.0, 0.0), xyz = (0.0, 0.0, 0.0)):
        super(WorldTransform, self).__init__('world_transform')
        self.set_world_transform(rpy, xyz)
    
    def set_world_transform(self, rpy, xyz):
        self.rpy = rpy
        self.xyz = xyz
        super(WorldTransform, self).set_attribute('rpy', '{} {} {}'.format(self.rpy[0], self.rpy[1], self.rpy[2]))
        super(WorldTransform, self).set_attribute('xyz', '{} {} {}'.format(self.xyz[0], self.xyz[1], self.xyz[2]))


class Origin(StoneFishSceneElement):
    def __init__(self, rpy=(0.0, 0.0, 0.0), xyz=(0.0, 0.0, 0.0)):
        super(Origin, self).__init__('origin')
        r, p, y1 = rpy
        x, y2, z = xyz
        self.set_attribute('rpy', '{} {} {}'.format(r, p, y1))
        self.set_attribute('xyz', '{} {} {}'.format(x, y2, z))


class Scene(StoneFishSceneElement):
    def __init__(self):
        super(Scene, self).__init__('scenario')
        self.materials = StoneFishSceneElement('materials')
        self.append(self.materials)
        self.looks = StoneFishSceneElement('looks')
        self.append(self.looks)
        self.environment = Environment()
        self.append(self.environment)


    def add_material(self, name, density, restitution):
        met = Material(name, density, restitution)
        self.materials.append(met)

    def add_look(self, name, gray, rgb, roughness, texture=None):
        look = Look(name, gray, rgb, roughness, texture)
        self.looks.append(look)


class Environment(StoneFishSceneElement):
    def __init__(self, latitude="32.8141543637297", 
        longitude="34.94752031090906", azimuth="20.0", elevation="50.0"):
        super(Environment, self).__init__('environment')
        
        # ned
        ned = StoneFishSceneElement('ned')
        ned.set_attribute('latitude',latitude)
        ned.set_attribute('longitude',longitude)

        # ocean
        ocean = StoneFishSceneElement('ocean')

        # atmosphere
        atmosphere = StoneFishSceneElement('atmosphere')
        sun = StoneFishSceneElement('sun')
        sun.set_attribute('azimuth',azimuth)
        sun.set_attribute('elevation',elevation)
        atmosphere.append(sun)

        self.append(ned)
        self.append(ocean)
        self.append(atmosphere)


class Light(StoneFishSceneElement):
    def __init__(self, name, radius, illuminance, rgb,
        xyz, rpy):
        super(Light, self).__init__('light')
        # name
        self.set_attribute('name', name)
        # attributes:
        # specs
        specs = StoneFishSceneElement('specs')
        specs.set_attribute('radius', radius)
        specs.set_attribute('illuminance', illuminance)
        self.append(specs)

        # color
        color = StoneFishSceneElement('color')
        color.set_attribute('rgb', '{} {} {}'.format(rgb[0], rgb[1], rgb[2]))
        self.append(color)

        # World_transform
        world_transform = WorldTransform(rpy, xyz)
        self.append(world_transform)


class Look(StoneFishSceneElement):
    def __init__(self, name, gray, rgb, roughness, texture=None):
        super(Look, self).__init__('look')
        self.set_attribute('name', name)
        if gray is not None:
            self.set_attribute('gray',gray)
        if rgb is not None:
            self.set_attribute('rgb','{} {} {}'.format(rgb[0], rgb[1], rgb[2]))
        if roughness is not None:
            self.set_attribute('roughness',roughness)
        if texture is not None:
            self.set_attribute('texture', texture)


class Material(StoneFishSceneElement):
    def __init__(self, name , density, restitution):
        super(Material, self).__init__('material')
        self.set_attribute('name', name)
        self.set_attribute('density',density)
        self.set_attribute('restitution',restitution)






class StoneFishModel(StoneFishSceneElement):
    def __init__(self, header, name, type, material, look, 
    world_transform):
        super(StoneFishModel, self).__init__(header)
        self.set_attribute('name',name)
        self.set_attribute('type', type)
        self.type = type
        # material
        self.material = StoneFishSceneElement('material')
        self.material.set_attribute('name', material)
        self.append(self.material)

        # look
        self.look = StoneFishSceneElement('look')
        self.look.set_attribute('name', look)
        self.append(self.look)


        self.world_transform = world_transform
        self.append(world_transform)

    def get_type(self):
        return self.type




class StaticModel(StoneFishModel):
    def __init__(self, name, type, material, look, world_transform):
        super(StaticModel, self).__init__('static', name, type, material, look, world_transform)


        

class AnimatedModel(StoneFishSceneElement):
    def __init__(self, name, model, keypoints):
        super(AnimatedModel, self).__init__('animated')

        self.set_attribute('name', name)
        self.set_attribute('type', model.get_type())
        self.set_attribute('collisions', 'flase')

        self.append(model.material)
        self.append(model.look)
        if model.dimensions is not None:
            self.append(model.dimensions)

        origin = Origin(keypoints[0][2], keypoints[0][1])

        self.append(origin)

        trajectory = Trajectory(keypoints)

        self.append(trajectory)








class Plane(StaticModel):
    def __init__(self, name, material, look, world_transform):
        super(Plane, self).__init__(name, 'plane', material, look, world_transform)


class Dimentions(StoneFishSceneElement):
    def __init__(self):
        super(Dimentions, self).__init__('dimensions')


class SphereDimentions(Dimentions):
    def __init__(self, radius):
        super(SphereDimentions, self).__init__()
        super(SphereDimentions, self).set_attribute('radius', radius)


class BoxDimensions(Dimentions):
    def __init__(self, x, y, z):
        super(BoxDimensions, self).__init__()
        super(BoxDimensions, self).set_attribute('xyz', '{} {} {}'.format(x, y, z))

class CylinderDimentions(Dimentions):
    def __init__(self, radius, height):
        super(CylinderDimentions, self).__init__()
        super(CylinderDimentions, self).set_attribute('radius', radius)
        super(CylinderDimentions, self).set_attribute('height', height)





#   <static name="docking_station" type="model">
#     <material name="Aluminum"/>
#     <look name="docking_station"/>
#     <world_transform rpy="0 0 0" xyz="20 10 -0.5"/>
#     <physical>
#       <mesh filename="docking_station/docking_station.png" scale="0.001"/>
#       <origin rpy="3.14 0.0 0.0" xyz="0.0 0.0 0.0"/>
#     </physical>
#   </static>








# <static name="Docking_station" type="model">
# 	<physical>
# 		<mesh filename="docking_station/docking_station.obj" scale="0.001"/>
# 		<origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
# 	</physical>
# 	<material name="Aluminium"/>
# 	<look name="docking_station"/>
# 	<world_transform rpy="-1.57 0.0 1.57" xyz="20.0 10.0 -0.5"/>
# </static>
# class DockingStation(StaticModel):
#     def __init__(self, name, type, material, look, world_transform):
#         super(DockingStation, self).__init__(name, type, material, look, world_transform)

#         physical = StoneFishSceneElement('physical')
#         mesh = StoneFishSceneElement('mesh')
#         mesh.set_attribute("filename", "docking_station/docking_station.obj")
#         mesh.set_attribute("scale", "0.001")
#         physical.append(mesh)
#         self.append(physical)




class Sphere(StaticModel):
    def __init__(self, name, radius, material, look, world_transform):
        super(Sphere, self).__init__(name, 'sphere', material, look, world_transform)
        super(Sphere, self).append(SphereDimentions(radius))


class Box(StaticModel):
    def __init__(self, name, xyz,
    material, look, world_transform):
        super(Box, self).__init__(name, 'box', material, look, world_transform)
        x, y, z = xyz
        self.dimensions = BoxDimensions(x, y, z)
        super(Box, self).append(BoxDimensions(x, y, z))

class Cylinder(StaticModel):
    def __init__(self, name, radius, height, material, look, world_transform):
        super(Cylinder, self).__init__(name, 'cylinder', material, look, world_transform)
        self.dimensions = CylinderDimentions(radius, height)
        super(Cylinder, self).append(self.dimensions)


class MeshFileModel(StaticModel):
    def __init__(self, name, filename, scale, material, look, world_transform):
        super(MeshFileModel, self).__init__(name, 'model', material, look, world_transform)

        # physical
        physical = StoneFishSceneElement('physical')
        super(MeshFileModel, self).append(physical)
        
        # mesh
        mesh = StoneFishSceneElement('mesh')
        physical.append(mesh)
        mesh.set_attribute('filename', filename)
        mesh.set_attribute('scale', scale)

        # origin
        origin = StoneFishSceneElement('origin')
        rpy = (0.0, 0.0, 0.0)
        xyz = (0.0, 0.0, 0.0)
        origin = Origin(rpy, xyz)
        physical.append(origin)




class Sensor(StoneFishSceneElement):
    def __init__(self, name, type, origin, topic):
        super(Sensor, self).__init__('sensor')
        self.set_attribute('name', name)
        self.set_attribute('type', type)

        ros_publisher = StoneFishSceneElement('ros_publisher')
        self.append(ros_publisher)
        ros_publisher.set_attribute('topic', topic)
        # self.set_attribute('topic', topic)
        self.append(origin)
        link = StoneFishSceneElement('link')
        self.append(link)
        link.set_attribute('name', 'Vehicle')


class DepthCam(Sensor):
    def __init__(self, name, origin, topic, rate,
    resolution=(1280,720), horizontal_fov=55.5, depth_min=0.2, depth_max=60.0):
        super(DepthCam, self).__init__(name, 'depthcamera', origin, topic)
        # rate
        self.set_attribute('rate', rate)
        # specs
        specs = StoneFishSceneElement('specs')
        specs.set_attribute('resolution_x', resolution[0])
        specs.set_attribute('resolution_y', resolution[1])
        specs.set_attribute('horizontal_fov', horizontal_fov)
        specs.set_attribute('depth_min', depth_min)
        specs.set_attribute('depth_max', depth_max)
        self.append(specs)
        # noise
        noise = StoneFishSceneElement('noise')
        noise.set_attribute('depth', 0.00000)
        self.append(noise)
  


class FLC(Sensor):
    def __init__(self, name, origin, topic, rate,
    resolution=(1280,720), horizontal_fov=55.5):
        super(FLC, self).__init__(name, 'camera', origin, topic)
        # rate
        self.set_attribute('rate', rate)
        # specs
        specs = StoneFishSceneElement('specs')
        specs.set_attribute('resolution_x', resolution[0])
        specs.set_attribute('resolution_y', resolution[1])
        specs.set_attribute('horizontal_fov', horizontal_fov)
        self.append(specs)
        # rendering
        rendering = StoneFishSceneElement('rendering')
        rendering.set_attribute('spp', 4)
        self.append(rendering)



class FLS(Sensor):
    def __init__(self, name, origin, topic, fls_colormap, beams = 512,
    bins = 500, horizontal_fov = 130.0, vertical_fov = 20.0, range_min = 0.05,
    range_max = 60.0, gain = 3.1):
        super(FLS, self).__init__(name, 'fls', origin, topic)
        # specs
        specs = StoneFishSceneElement('specs')
        specs.set_attribute('beams', beams)
        specs.set_attribute('bins', bins)
        specs.set_attribute('horizontal_fov', horizontal_fov)
        specs.set_attribute('vertical_fov', vertical_fov)
        self.append(specs)
        # settings
        settings = StoneFishSceneElement('settings')
        settings.set_attribute('range_min', range_min)
        settings.set_attribute('range_max', range_max)
        settings.set_attribute('gain', gain)
        self.append(settings)
        # rendering
        rendering = StoneFishSceneElement('rendering')
        rendering.set_attribute('spp', 4)
        self.append(rendering)
        # display
        display = StoneFishSceneElement('display')
        display.set_attribute('colormap', fls_colormap)  # fls_colormap
        self.append(display)
        # noise
        # <noise multiplicative="0.01" additive="0.02"/>
        noise = StoneFishSceneElement('noise')
        noise.set_attribute('multiplicative', 0.01)
        noise.set_attribute('additive', 0.02)
        self.append(noise)




class KeyPoint(StoneFishSceneElement):
    def __init__(self, time, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
        super(KeyPoint, self).__init__('keypoint')
        self.set_attribute('time', time)
        x, y1, z = xyz
        r, p, y2 = rpy
        self.set_attribute('xyz', '{} {} {}'.format(x, y1, z))
        self.set_attribute('rpy', '{} {} {}'.format(r, p, y2))



class Trajectory(StoneFishSceneElement):
    def __init__(self, keypoints):
        super(Trajectory, self).__init__('trajectory')
        self.set_attribute('type', 'spline')
        # self.set_attribute('playback', 'repeat')
        self.set_attribute('playback', 'boomerang')
        for time, xyz, rpy in keypoints:
            keypoint = KeyPoint(time, xyz, rpy)
            self.append(keypoint)




class AnimatedFrame(StoneFishSceneElement):
    def __init__(self, name, keypoints, colormap):
        super(AnimatedFrame, self).__init__('animated')
        self.set_attribute('name', name)
        self.set_attribute('type', 'empty')

        depth_cam = DepthCam('Dcam', Origin((1.57, 0.0, 1.57), (0.9,  0, -0.052)), "/image_depth2", 30.0, (1280,720), 55.0)
        flc = FLC('Prosilica', Origin((1.57, 0.0, 1.57), (0.9,  0, -0.052)), "/camera2/image_raw", 30.0, (1280,720), 55.5)
        # print(str(ColorMap.hot))
        fls = FLS('Blue_view_M900', Origin((1.57, 0.0, 1.57), (1.2,  0, 0.052)), "/sparus2/Blue_view_M900_FLS2", colormap)

        self.append(depth_cam)
        self.append(flc)
        self.append(fls)

        trajectory = Trajectory(keypoints)
        self.append(trajectory)



def create_marker(name):
    marker = Box(name, (0.02, 0.3, 0.3), 'Steel', name, WorldTransform((0, 0, 0), (0, 0, 0)))
    return marker

def create_keypoints(position, qaurter, time_per_step, distance_from_center):
    start_time = 0.0
    x_center, y_center, z_center = position[0], position[1], position[2]
    pattern = [(0, 1, 1), (1, 0, 0), (0, -1, -1), (-1, 0, 2)]
    shifted_pattern = pattern[qaurter:] + pattern[:qaurter]
    keypoints = []
    for i in range(4):
        x, y, yaw = shifted_pattern[i]
        keypoints.append((start_time + i * time_per_step, (x_center + x * distance_from_center, y_center + y * distance_from_center, z_center), (0.0, 0.0, 1.57 * yaw)))
    return keypoints

def create_anim_markers(position, time_per_step, distance_from_center):
    anim_merkers = []
    for i in range(4):
        keypoints = create_keypoints(position, i, time_per_step, distance_from_center)
        number = str(i)
        marker_name = "marker" + number
        marker = create_marker(marker_name)
        look = Look(marker_name, 1.0, None, 0.9, "aruco/aruco-" + number + "_3d_border.png")
        anim_model = AnimatedModel("anim_model" + number, marker, keypoints)
        anim_merkers.append((anim_model, look))
    return anim_merkers



def main():

    scene = Scene()

    scene.add_material('Natural', 1000.0, 0.5)
    scene.add_look('white', 1.0, None, 0.2)

    # box
    # wt1 = WorldTransform((1.0, 2.0, 3.0), (4.0, 5.0, 6.0))
    # box = Box("box1", (1, 2, 3), 'Steel', 'Red', wt1)
    # scene.append(box)
    
    # # sphere
    # wt2 = WorldTransform((6.0, 7.0, 8.0), (9.0, 8.0, 7.0))
    # ball = Sphere("ball1", 5.0, 'Steel', 'Red', wt2)
    # scene.append(ball)

    # # cylinder
    # wt3 = WorldTransform((16.0, 17.0, 0.0), (-19.0, -18.0, 0.0))
    # cylinder = Cylinder("cylinder1", 10, 10, 'Steel', 'Red', wt3)
    # scene.append(cylinder)


    # seabed
    scene.add_material('Rock', 3000.0, 0.8)
    # <look name="seabed" rgb="0.7 0.7 0.5" roughness="0.9" texture="sand_normal.png"/>
    scene.add_look('seabed', None, (0.7, 0.7, 0.5), 0.9, texture='sand_normal.png')
    bottom = Plane('Bottom', 'Rock', 'seabed', WorldTransform((0,0,0),(0,0,15)))
    scene.append(bottom)

    # acuro
    # scene.add_look('marker0', 1.0, None, 0.9, "aruco1.png")
    # marker = Box('Marker0', (0.02, 0.3, 0.3), 'Neutral', 'marker0', WorldTransform((3.14,0,3.1416/2),(5,5,1)))
    # scene.append(marker)

    # reef model
    # scene.add_look('reef3', 1.0, None, 0.3, 'reef3/2019_sat.jpg')
    # reef = MeshFileModel('reef', 'reef3/2019_up.obj', 1.0, 'Rock', 'reef3', WorldTransform((0,0,0),(-35,-5,6.1)))
    # scene.append(reef)

    # animated frame
    # keypoints = []
    # keypoints.append((0.0, (15.0, 15.0, 2.0), (0.0, 0.0, -1.57)))
    # keypoints.append((5.0, (15.0, 20.0, 2.0), (0.0, 0.0, -1.57)))
    # keypoints.append((10.0, (20.0, 20.0, 2.0), (0.0, 0.0, -1.57)))
    # keypoints.append((15.0, (20.0, 15.0, 2.0), (0.0, 0.0, -1.57)))
    # keypoints.append((20.0, (15.0, 15.0, 2.0), (0.0, 0.0, -1.57)))
    # frame = AnimatedFrame("frame1", keypoints, ColorMap.hot)
    # scene.append(frame)

    # <look name="hull" gray="1.0" roughness="0.3" texture="sparus2/hull_tex.png"/>
    scene.add_look('hull', "1.0", None, "0.3", "sparus2/hull_tex.png")
    position = (0, 0, 0.0)
    sparus2 = Sparus2(position)
    scene.append(sparus2)


    # Dock station
    def create_dockstation(position):
        # position = (40.0, 30, -0.5)
        scene.add_material("Aluminium", "2710.0", "0.7")
        scene.add_look("docking_station", "1.0", None, "0.3", "docking_station/docking_station.png")
        dockstation = MeshFileModel("docking_station", "docking_station/docking_station.obj", "0.001", "Aluminum", "docking_station", WorldTransform((-1.57, 0.0, 1.57),position))
        dockstation.dimensions = None
        # dockstation_animation = AnimatedModel("docking_station_animation", dockstation, create_keypoints(position, 0, 5.0, 0.0))
        # scene.append(dockstation_animation)
        scene.append(dockstation)
        # wt3 = WorldTransform((1.57, 0.0, 0.0), position)
        # cylinder = Cylinder("cylinder1", 0.4, 2, 'Steel', 'Red', wt3)

        # start_time = 0.0
        # keypoints = []
        # for i in range(4):
        #     keypoints.append((start_time + i * 5.0, (position[0], position[1], position[2] + 1) , (1.57, 0.0, 1.57 * i)))


        # dockstation_animation = AnimatedModel("docking_station_animation", cylinder, keypoints)



        # scene.append(dockstation_animation)


        # Markers

        # scene.add_material("Neutral", "1000.0", "0.5")
        scene.add_material("Steel", "1000.0", "0.9")



        anim_markers = create_anim_markers((position[0],position[1],position[2] + 3), 5.0, 0.15)
        for anim_marker, look in anim_markers:
            scene.looks.append(look)
            scene.append(anim_marker)

    

        name = "Omni"
        radius = 0.1
        illuminance = 1000.0
        rgb = (0.0, 0.3, 0.0)
        xyz = (position[0], position[1], position[2] + 3)
        rpy = (0.0, 0.0, 0.0)
        light = Light(name, radius, illuminance, rgb, xyz, rpy)
        scene.append(light)





    create_dockstation((35.0, 30, -0.5))
    # create_dockstation((10.0, 10, -0.5))




    # create_dockstation((30.0, 35, -0.5))
    # create_dockstation((30.0, 25, -0.5))
    # create_dockstation((25.0, 30, -0.5))










    # Boat
    # scene.add_look('white', 1.0, None, 0.2)
    # boat1 = MeshFileModel('boat', 'models/Tug_boat_simple1.obj', 0.0010, 'Aluminium', 'white', WorldTransform((-1.57, 0.0, 1.57),(40 ,20 ,0.8)))
    # scene.append(boat1)

    # boat2 = MeshFileModel('boat', 'models/Tug_boat_simple1.obj', 0.0005, 'Aluminium', 'white', WorldTransform((-1.57, 0.0, 0.0),(40 ,40 ,0.8)))
    # scene.append(boat2)


    # boat3 = MeshFileModel('boat', 'models/Tug_boat_simple1.obj', 0.0005, 'Aluminium', 'white', WorldTransform((-1.57, 0.0, -1.57),(40 ,10 ,0.8)))
    # scene.append(boat3)




    scene.add_look('Red'  , None, (1.0, 0.0, 0.0), 0.3)
    scene.add_look('Green', None, (0.0, 1.0, 0.0), 0.3)
    scene.add_material('None', 0.0, 0.0)


    for i in range(10, 100, 10):
        red_ball = Sphere("ball1", 0.5, 'None', 'Red', WorldTransform((0.0, 0.0, 0.0), (i, 0.0, -1.0)))
        green_ball = Sphere("ball1", 0.5, 'None', 'Green', WorldTransform((0.0, 0.0, 0.0), (0.0, i, -1.0)))
        scene.append(red_ball)
        scene.append(green_ball)







    xml = scene.toString()
    print(xml)

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    # get the file path for cola2_stonefish/scenarios/sparus2_generated_scene.scn
    scn_path = rospack.get_path('cola2_stonefish') + '/scenarios/sparus2_generated_scene.scn'

    # filename = "/home/ygutnik/catkin_ws/src/cola2_stonefish/scenarios/sparus2_generated_scene.scn"
    with open(scn_path, "w") as f:
        f.write(xml)



if __name__=="__main__":
    main()



