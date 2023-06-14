"""
    Main file for the project

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

import matplotlib.pyplot as plt

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
    time_sampling = 1e-6

    robot = Robot(client_id, 'Simple', wheel_radius, wheel_dist, time_sampling)
    result = robot.getMotorHandlers('left_wheel_joint', 'right_wheel_joint')
    print("Motor Setup: ", parseResponseMessage(result))

    # result = motor.getUltrasonicHandlers(
    #     'front_us_sensor', 'left_us_sensor', 'right_us_sensor'
    # )
    # print("Ultrasonic Setup: ", parseResponseMessage(result))

    throttle = 0
    yaw = 0
    count = 0
    last_time = time.time()

    x_hist = []
    y_hist = []
    phi_hist = []

    while sim.simxGetConnectionId(client_id):
        try:
            now = time.time()

            # TODO: Add local pose tracking here
            if (now - last_time) >= time_sampling:
                last_time = now
                count += 1

            if count % 10 == 0:
                print("Local X: ", robot.local_x)
                print("Local Y: ", robot.local_y)
                print("Local PHI: ", robot.local_phi)
                x_hist.append(robot.local_x)
                y_hist.append(robot.local_y)
                phi_hist.append(robot.local_phi)

            if count >= 12e6:
                left_speed = 0
                right_speed = 0
                count = 0
            elif count >= 8e6:
                left_speed = -1
                right_speed = -1
            elif count >= 6e6:
                left_speed = 0
                right_speed = 1
            elif count >= 4e6:
                left_speed = 1
                right_speed = 0
            elif count >= 1e6:
                left_speed = 1
                right_speed = 1

            # TODO: Add state change here (Manual | Go To | Follow Wall .....)

            # TODO: Add keyboard control setup here

            # TODO: Add speed logic here

            if count % 500e3 == 0:
                print("Setting: ", left_speed, right_speed)
                robot.move(left_speed, right_speed)
                robot.updateLocalPosition(left_speed, right_speed)
        except KeyboardInterrupt:
            print("Keyboard Interrup Active!")
            break

    plt.plot(y_hist, x_hist)
    plt.show()


if __name__ == '__main__':
    main()