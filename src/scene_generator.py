from pystonefish import Scene, WorldTransform, Box, Sphere, Cylinder, Plane, MeshFileModel, AnimatedFrame, ColorMap
import random

random.seed(0)


class SceneGenerator:
    def __init__(self, seabed_depth = 5.0):
        self.scene = Scene()
        # seabed
        self.seabed_depth = seabed_depth
        self.scene.add_material('Rock', 3000.0, 0.8)
        bottom = Plane('Bottom', 'Rock', 'seabed', WorldTransform((0,0,0),(0,0,self.seabed_depth)))
        self.scene.append(bottom)

        # # box
        self.scene.add_look('Red', None, (1.0, 0.1, 0.1), 0.3)
        self.scene.add_look('seabed',None, (0.7, 0.7, 0.5), 0.9, 'sand_normal.png') # <look name="seabed" rgb="0.7 0.7 0.5" roughness="0.9" texture="sand_normal.png"/>

        # wt1 = WorldTransform((1.0, 2.0, 3.0), (4.0, 5.0, 0.0))
        # box = Box("box1", (1, 2, 3), 'Steel', 'Red', wt1)
        # self.scene.append(box)

        # # box2
        # # self.scene.add_look('Red', None, (1.0, 0.1, 0.1), 0.3)
        # wt1 = WorldTransform((1.0, 2.0, 3.0), (10.0, 5.0, 0.0))
        # box = Box("box2", (1, 2, 3), 'Steel', 'Red', wt1)
        # self.scene.append(box)

    def add_empty_frame(self, colormap):
        # animated frame
        keypoints = []
        t = 0.0
        for x in range(-10,110,10):
            keypoints.append((t, (x, 0.0, self.seabed_depth-2), (0.0, 0.0, 0.0)))
            t += 5.0



        frame = AnimatedFrame("frame1", keypoints, colormap.value)
        self.scene.append(frame)       


    def generate(self):
        """returns a scn file in str format"""
        return self.scene.toString()

    def add_boxes(self):
        """add lots of boxes"""
        box_height = 1
        for x in range (0,100,10):
            for y in range(-100,100,10):
                depth = random.uniform(0,self.seabed_depth - box_height/2)
                wt = WorldTransform((0.0, 0.0, 0.0), (x, y, depth))
                box = Box("box{}.{}".format(x,y), (1,1,1), 'Rock', 'Red', wt)
                self.scene.append(box)


    def add_reefs(self):
        """add lots of reefs"""
        self.scene.add_look('reef3', 1.0, None, 0.3, 'reef3/2019.jpg')
        error = 2
        for x in range (0,120,10):
            for y in range(-40,40,10):
                # depth = random.uniform(0,self.seabed_depth - box_height/2)
                allow = random.randint(1,5) == 1
                if not allow:
                    continue
                angle = random.uniform(-3.14,3.14)
                x = random.uniform(x - error,x + error)
                y = random.uniform(y - error,y + error)
                wt = WorldTransform((-0.2, 0.15, angle), (x, y, self.seabed_depth+0.05))
                # reef model
                reef = MeshFileModel('reef', 'reef3/2019.obj', 1.0, 'Rock', 'reef3',wt)
                self.scene.append(reef)



def simple_scene():
    seabed_depth = 15.0
    gen = SceneGenerator(seabed_depth)
    # gen.add_empty_frame(ColorMap.hot)
    gen.add_empty_frame(ColorMap.jet)
    # gen.add_empty_frame(ColorMap.perula)
    # gen.add_empty_frame(ColorMap.greenblue)
    # gen.add_empty_frame(ColorMap.coldblue)
    # gen.add_empty_frame(ColorMap.orangecopper)




    # gen.add_boxes()
    gen.add_reefs()

    scene = gen.generate()
    print(scene)

    f = open("/home/ilan/catkin_ws/src/cola2_stonefish/scenarios/sparus2_haifa_deepersense.scn", "w")
    f.write(scene)
    f.close()



def main():

    scene = Scene()

    scene.add_material('Natural', 1000.0, 0.5)
    scene.add_look('white', 1.0, None, 0.2)
    scene.add_look('Red', None, (1.0, 0.1, 0.1), 0.3)

    # box
    wt1 = WorldTransform((1.0, 2.0, 3.0), (4.0, 5.0, 6.0))
    box = Box("box1", (1, 2, 3), 'Steel', 'Red', wt1)
    scene.append(box)
    
    # sphere
    wt2 = WorldTransform((6.0, 7.0, 8.0), (9.0, 8.0, 7.0))
    ball = Sphere("ball1", 5.0, 'Steel', 'Red', wt2)
    scene.append(ball)

    # cylinder
    wt3 = WorldTransform((16.0, 17.0, 0.0), (-19.0, -18.0, 0.0))
    cylinder = Cylinder("cylinder1", 10, 10, 'Steel', 'Red', wt3)
    scene.append(cylinder)


    # seabed
    scene.add_material('Rock', 3000.0, 0.8)
		
    scene.add_look('seabed',None, (0.7, 0.7, 0.5), 0.9, 'sand_normal.png') # <look name="seabed" rgb="0.7 0.7 0.5" roughness="0.9" texture="sand_normal.png"/>
    bottom = Plane('Bottom', 'Rock', 'seabed', WorldTransform((0,0,0),(0,0,5)))
    scene.append(bottom)

    # acuro
    scene.add_look('marker0', 1.0, None, 0.9, "aruco1.png")
    marker = Box('Marker0', (0.02, 0.3, 0.3), 'Neutral', 'marker0', WorldTransform((3.14,0,3.1416/2),(5,5,1)))
    scene.append(marker)

    # reef model
    scene.add_look('reef3', 1.0, None, 0.3, 'reef3/2019_sat.jpg')
    reef = MeshFileModel('reef', 'reef3/2019.obj', 1.0, 'Rock', 'reef3', WorldTransform((0,0,0),(-35,-5,6.1)))
    scene.append(reef)

    # animated frame
    keypoints = []
    keypoints.append((0.0, (15.0, 15.0, 2.0), (0.0, 0.0, -1.57)))
    keypoints.append((5.0, (15.0, 20.0, 2.0), (0.0, 0.0, -1.57)))
    keypoints.append((10.0, (20.0, 20.0, 2.0), (0.0, 0.0, -1.57)))
    keypoints.append((15.0, (20.0, 15.0, 2.0), (0.0, 0.0, -1.57)))
    keypoints.append((20.0, (15.0, 15.0, 2.0), (0.0, 0.0, -1.57)))
    frame = AnimatedFrame("frame1", keypoints)
    scene.append(frame)

    xml = scene.toString()
    print(xml)

if __name__=="__main__":
    # main()
    scene = simple_scene()