from pymorse import Morse
from numpy import random

from crash_generator import configuration
from crash_generator.motion_control.boundary_following import BoundaryFollower
from crash_generator.motion_control.bug2_exit_condition import Bug2ExitCondition, Bug2WithNoTargetExitCondition
from crash_generator.motion_control.motion_to_goal import MotionToGoal
from crash_generator.motion_control.simple_motion import ConstantMotion
from crash_generator.robot.robot import Robot
from crash_generator.trilateration.planar_geometry import Point2D
from crash_generator.trilateration.trilateration import Trilaterator
from crash_generator.trilateration.trilateration_solution import TrilaterationSolver


with Morse() as sim:
    robot = Robot(sim.robot, sim)

    # make sure to be close enough to the target
    while robot.distance_to_target >= configuration.PROXIMITY_RANGE:
        print("Das Zeil konnte nicht gefunden werden!")
        goal = Point2D(random.uniform(-10, 10), random.uniform(-10, 10))
        print("Fiktives Ziel wurde gesetzt {}... !".format(goal))
        _motion = MotionToGoal(robot, goal)
        _boundary = BoundaryFollower(robot, Bug2WithNoTargetExitCondition(robot, goal))

        while robot.distance_to_target >= configuration.PROXIMITY_RANGE:

            _motion.go_until_target_or_obstacle()

            if robot.position.distance_from(goal) <= robot.SENSORS_RANGE * 1.3:
                print("Fiktives Ziel ist verloren")
                break

            if robot.free_space_ahead < robot.SENSORS_RANGE:
                _boundary.follow_till_exit()

    print('Ziel gefunden!')
    robot.stop()

    # trilaterate the goal
    trilaterator = Trilaterator(robot, TrilaterationSolver())
    motion = ConstantMotion(robot,
                            configuration.CONSTANT_MOTION_LINEAR_SPEED,
                            configuration.CONSTANT_MOTION_ANGULAR_VELOCITY,
                            trilaterator)
    boundary = BoundaryFollower(robot, trilaterator)

    motion.go_until_exit()
    print('Trilaterating oder crashing!')

    if trilaterator.goal is None:
        print('Wand wurde erkannt. Hindernisse werden gefolgt bis zum trilaterating!')
        boundary.follow_till_exit()

    robot.stop()
    goal = trilaterator.goal
    print("Target am {}...!".format(goal))
    '''end tmp'''

    # proceed to the target in a bug-2 fashion
    motion = MotionToGoal(robot, goal)
    boundary = BoundaryFollower(robot, Bug2ExitCondition(robot, goal))

    while True:

        motion.go_until_target_or_obstacle()

        if robot.distance_to_target <= robot.SENSORS_RANGE * 1.3:
            print("Näher am ziel!")
            break

        if robot.free_space_ahead < robot.SENSORS_RANGE:
            print("Die wand wird gefolgt!")
            boundary.follow_till_exit()
            print("Die Wand wird verlassen")

    robot.stop()
