from simulation_controller import SimulationController
from scene_generator import SceneGenerator, ColorMap




def main():
    print('hello')


    seabed_depth = 15
    gen = SceneGenerator(seabed_depth)
    gen.add_empty_frame(ColorMap.jet)
    gen.add_reefs()
    scene = gen.generate()
    print(scene)


if __name__=='__main__':
    main()