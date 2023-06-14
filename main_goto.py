"""
    Go To Control System

    Subject:
        -   Robotics

    Team:
        -   1.  Abner Vasconcelos Graboski
        -   2.  Felipe dos Santos Ferreira
        -   3.  Thiago Estradioto dos Santos
        -   4. João Pedro Verona Turra

    Teacher:
        -   Dr. Leandro dos Santos Coelho
"""


import time

# import keyboard
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
import math

# matplotlib.use("QtCairo")

import sim
from robot import Robot

client_id: int = -1
left_speed: float = 0.0
right_speed: float = 0.0


def parseResponseMessage(response: int) -> str:
    if (response == sim.simx_return_ok):
        return "Success"

    msg = ""
    if (response & sim.simx_return_illegal_opmode_flag != 0):
        msg += "Illegal Opmode Error;\n"
    if (response & sim.simx_return_initialize_error_flag != 0):
        msg += "Initialize Error;\n"
    if (response & sim.simx_return_local_error_flag != 0):
        msg += "Local Error;\n"
    if (response & sim.simx_return_novalue_flag != 0):
        msg += "No Value Error;\n"
    if (response & sim.simx_return_remote_error_flag != 0):
        msg += "Remote Error;\n"
    if (response & sim.simx_return_split_progress_flag != 0):
        msg += "Split Progress Error;\n"
    if (response & sim.simx_return_timeout_flag != 0):
        msg += "Timeout Error;\n"
    return msg


def connectToCoppeliaSim():

    sim.simxFinish(-1)  # just in case, close all opened connections
    return sim.simxStart(
        '127.0.0.1', 19999, True, True, 5000, 5
    )  # Connect to CoppeliaSim


def moveRobot(robot: Robot, left_speed_motor: float, right_speed_motor: float):
    global left_speed, right_speed

    left_speed = left_speed_motor
    right_speed = right_speed_motor

    robot.move(left_speed, right_speed)


def main():
    global client_id, left_speed, right_speed
    print('Program started')

    client_id = connectToCoppeliaSim()

    parseResponseMessage(client_id)
    if client_id != sim.simx_error_noerror:
        return -1

    wheel_radius = 0.15 / 2
    wheel_dist = 0.35
    time_sampling = 5e-3

    robot = Robot(client_id, 'Simple', wheel_radius, wheel_dist, time_sampling)
    result = robot.getMotorHandlers('left_wheel_joint', 'right_wheel_joint')
    result2 = robot.getUltrasonicHandlers(
        front_sensor_name="front_us_sensor",
        left_sensor_name="left_us_sensor",
        right_sensor_name="right_us_sensor"
    )
    print("Motor Setup: ", parseResponseMessage(result))
    print("Ultrasonic Setup: ", parseResponseMessage(result2))

    if result != sim.simx_return_ok or result2 != sim.simx_return_ok:
        stop = input("Do you want to stop?[Y/n]")
        if stop and (stop.lower() == 'n' or stop.lower() == 'no'):
            return

    front_us_trigger = 0.5
    left_us_trigger = 0.4
    right_us_trigger = 0.4

    STOP = 0
    GOTO = 1
    REACHED = 2

    state = GOTO

    front_us_value = 1e5
    left_us_value = 1e5
    right_us_value = 1e5

    time_step = 0

    left_speed = 0
    right_speed = 0

    while sim.simxGetConnectionId(client_id):
        try:
            # robot.updateUltrasonicMeasure()

            # front_us_value_aux = robot.ultrasonic_measures[0]
            # if type(front_us_value_aux) != int:
            #     front_us_value = front_us_value_aux

            # left_us_value_aux = robot.ultrasonic_measures[1]
            # if type(left_us_value_aux) != int:
            #     left_us_value = left_us_value_aux

            # right_us_value_aux = robot.ultrasonic_measures[2]
            # if type(right_us_value_aux) != int:
            #     right_us_value = right_us_value_aux

            # print("Front ", front_us_value)
            # print("Left ", left_us_value)
            # print("Right ", right_us_value)

            # state = STOP

            if state == STOP:
                left_speed = 0
                right_speed = 0
                # if time_step % 1e3 == 0:
                #     print("1ms")
                #     state = GOTO

            elif state == GOTO:
                x = robot.local["x"][-1]
                y = robot.local["y"][-1]
                phi = robot.local["phi"][-1]

                # print("Before X %.2f, Y %.2f" % (x, y))
                factor = 1 / 1.46
                x_target = 3 * factor
                y_target = 3 * -factor

                if (round(x, 3) >= (x_target - 1e-2))  \
                    and (round(y, 3) >= (y_target - 1e-2)):
                    print("STOP")
                    state = STOP
                else:
                    left_speed, right_speed = robot.goto(x_target, y_target)

            robot.move(left_speed, right_speed, 1)
            robot.updatePosition(left_speed * 1, right_speed * 1)

            time.sleep(time_sampling)
            time_step += 1
        except KeyboardInterrupt:
            print("Keyboard Interrup Active!")
            break

    # print("X: ", robot.local["x"])
    # print("Y: ", robot.local["y"])

    plt.figure(figsize=(10, 10))
    plt.subplot(3, 1, 1)
    plt.plot(robot.local["x"], color="blue", label="X coord")

    plt.subplot(3, 1, 2)
    plt.plot(robot.local["y"], color="red", label="Y coord")

    plt.subplot(3, 1, 3)
    plt.plot(
        robot.local["x"], robot.local["y"], color="orange", label="XY coord"
    )

    plt.show()


if __name__ == '__main__':
    main()