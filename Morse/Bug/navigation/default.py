#! /usr/bin/env morseexec

"""
Hier wird eine morse Szene erstellt
"""

from morse.builder import *
import pytoml as toml
import os

# control if a file exists
def findFileInPath(filename, extension, paths):
    for path in paths:
        if os.path.exists(os.path.join(path, filename)):
            return os.path.join(path, filename)
        elif os.path.exists(os.path.join(path, filename + '.' + extension)):
            return os.path.join(path, filename + '.' + extension)
    raise FileNotFoundError('Die Datei ' + filename + ' konnte nicht gefunden werden!')


def addEduMorseDefaultCollisionToRobot(robot, s):
    eduMorse_default_collision_sensor = Collision()
    eduMorse_default_collision_sensor.translate(0, 0, s[0])
    eduMorse_default_collision_sensor.scale = (s[1], s[2], s[3])
    eduMorse_default_collision_sensor.frequency(3)
    eduMorse_default_collision_sensor.add_interface("socket")
    robot.append(eduMorse_default_collision_sensor)


def main():

    ############################################################
    # IMPORTANT PATHS
    ############################################################

    PWD = os.path.dirname(os.path.abspath(__file__))
    EDUMORSEPATH = os.environ.get("EDUMORSEPATH")
    GAMESPATH = os.path.join(EDUMORSEPATH, "games")
    MAPSPATH = os.path.join(GAMESPATH, "maps")
    OBJECTSPATH = os.path.join(GAMESPATH, "objects")
    ROBOTSPATH = os.path.join(GAMESPATH, "robots")

    ############################################################
    # GAME CONFIGURATION
    ############################################################

    # Open and load the local file containing the configuration of the simulation
    with open(os.path.join(PWD, "simulation.toml"), 'r') as simulation_file:
        simulation = toml.loads(simulation_file.read())

        # Check if rules file exists and load it
        rules_name = simulation['simulation']['name']
        try:
            rules_name = findFileInPath(rules_name, 'toml', [PWD, GAMESPATH])
        except:
            raise

        with open(rules_name) as rules_file:
            rules = toml.loads(rules_file.read())


            # Control if any robot file exists and collect them in a list
            num_robot = rules['simulation']['numrobot']
            robot_file_list = []
            for robot_file in simulation['simulation']['robot']:
                try:
                    rf = findFileInPath(robot_file['file'], 'toml', [PWD, ROBOTSPATH])
                except:
                    raise
                robot_file_list.append((robot_file['name'], rf))

            # Open and load any robot file and collect them in a list
            config = []
            for robot_file in robot_file_list:
                with open(robot_file[1]) as conffile:
                    config.append((robot_file[0], toml.loads(conffile.read())))

            # Check if the number of robots is higher than the admitted one
            if len(config) > num_robot:
                raise Exception('Too many robots')

            # Check if the robots are used once
            if len(set([name[0] for name in robot_file_list])) < len([name[0] for name in robot_file_list]):
                raise Exception('Robots with the same name')


            ############################################################
            # ROBOT CONFIGURATION
            ############################################################

            positions = list(rules['simulation']['robot_position'])
            display = False

            for robot_config in config:
                rob = robot_config[1]['robot']
                robot = eval(rob['type'] + '()')
                robot.name = robot_config[0]
                for cam in simulation['simulation']['camera_position']:
                    if robot.name == cam['robot']:
                        camera = VideoCamera()
                        camera.translate(cam['x_cam'], cam['y_cam'], cam['z_cam'])
                        camera.rotate(cam['p_cam'], cam['q_cam'], cam['r_cam'])
                        camera.properties(Vertical_Flip=False)
                        robot.append(camera)
                        display = True

                aes = []  # actuators and sensors
                aes.append('eduMorse_default_collision_sensor')

                for act in rob['actuators']:
                    if act['id'] in aes:
                        raise Exception('Error: actuator id is not unique')
                    if act['type'] in rules['simulation']['actuators']:
                        actuator = eval(act['type'] + '()')
                        actuator.name = act['id']
                        p = act.get('properties', None)
                        if p:
                            actuator.properties(**p)
                        i = act.get('interface', None)
                        if i:
                            iprop = i.get('properties', None)
                            itype = i['type']
                            if iprop:
                                actuator.add_interface(itype, **iprop)
                            actuator.add_interface(itype)
                        actuator.translate(act.get('x', 0.0), act.get('y', 0.0), act.get('z', 0.0))
                        actuator.rotate(act.get('p', 0.0), act.get('q', 0.0), act.get('r', 0.0))

                        robot.append(actuator)
                        aes.append(act['id'])
                    else:
                        raise Exception('Actuator type not allowed in this game')

                for sens in rob['sensors']:
                    if sens['id'] in aes:
                        raise Exception('Error: sensor id is not unique')
                    if sens['type'] in rules['simulation']['sensors']:
                        sensor = eval(sens['type'] + '()')
                        sensor.name = sens['id']
                        p = sens.get('properties', None)
                        if p:
                            sensor.properties(**p)
                        i = sens.get('interface', None)
                        if i:
                            iprop = i.get('properties', None)
                            itype = i['type']
                            if iprop:
                                sensor.add_interface(itype, **iprop)
                            sensor.add_interface(itype)
                        sensor.translate(sens.get('x', 0.0), sens.get('y', 0.0), sens.get('z', 0.0))
                        sensor.rotate(sens.get('p', 0.0), sens.get('q', 0.0), sens.get('r', 0.0))

                        robot.append(sensor)
                        aes.append(sens['id'])
                    else:
                        raise Exception('Sensor type not allowed in this game')

                for c in rob['eduMorseCollision']:
                    addEduMorseDefaultCollisionToRobot(robot, [c['zTran'], c['x'], c['y'], c['z']])

                pos = positions.pop(0)
                robot.translate(pos['x'], pos['y'], pos['z'])
                robot.rotate(pos['p'], pos['q'], pos['r'])


            ############################################################
            # OBJECT CONFIGURATION
            ############################################################

            num_object = rules['simulation']['numobject']
            objects = []
            for o in rules['simulation']['objects']:
                try:
                    object_file = findFileInPath(o['file'], 'blend', [PWD, OBJECTSPATH])
                except:
                    raise
                objects.append((object_file, o))

            if len(objects) > num_object:
                raise Exception('Too many objects')

            object_name = []
            for i in objects:
                if i[1]['name'] in object_name:
                    raise Exception('Error: object name is not unique')
                obj = PassiveObject(i[0])
                obj.name = i[1]['name']
                obj.translate(i[1].get('x', 0.0), i[1].get('y', 0.0), i[1].get('z', 0.0))
                obj.rotate(i[1].get('p', 0.0), i[1].get('q', 0.0), i[1].get('r', 0.0))
                p = i[1].get('properties', None)
                if p:
                    obj.properties(**p)
                object_name.append(i[1]['name'])


            ############################################################
            # ENVIRONMENT CONFIGURATION
            ############################################################

            # Check if map file exists
            game_map = rules['simulation']['map']
            try:
                game_map = findFileInPath(game_map, 'blend', [PWD, MAPSPATH])
            except:
                raise

            fmode = simulation['simulation']['fastmode']

            env = Environment(game_map, fastmode = fmode)

            for cam in rules['simulation']['camera_position']:
                env.set_camera_location([cam['x_cam'], cam['y_cam'], cam['z_cam']])
                env.set_camera_rotation([cam['p_cam'], cam['q_cam'], cam['r_cam']])

            if display:
                env.select_display_camera(camera)

if __name__ == "__main__":
    main()
