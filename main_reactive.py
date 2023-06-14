"""
    Reactive control for a Pioneer robot in CoppeliaSim.

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

    # Simulation parameters
    time_sampling = 50e-3

    # Robot parameters
    wheel_radius = 0.15 / 2
    wheel_dist = 0.35
    robot_names = ['Simple', 'Pioneer']
    robot_joint_names = [['left_wheel_joint', 'right_wheel_joint'], 
                         ['leftMotor', 'rightMotor']]
    robot_us_names = [["front_us_sensor", "left_us_sensor", "right_us_sensor"], 
                      ["ultrasonicSensor_front", "ultrasonicSensor_left", "ultrasonicSensor_right"]]

    # Robot setup
    robot = Robot(client_id, robot_names[1], wheel_radius, wheel_dist, time_sampling)
    result = robot.getMotorHandlers(robot_joint_names[1][0], robot_joint_names[1][1])
    result2 = robot.getUltrasonicHandlers(
        front_sensor_name=robot_us_names[1][0],
        left_sensor_name=robot_us_names[1][1],
        right_sensor_name=robot_us_names[1][2]
    )
    print("Motor Setup: ", parseResponseMessage(result))
    print("Ultrasonic Setup: ", parseResponseMessage(result2))

    if result != sim.simx_return_ok or result2 != sim.simx_return_ok:
        stop = input("Do you want to stop?[Y/n]")
        if stop and (stop.lower() == 'n' or stop.lower() == 'no'):
            return

    # Ultrassonic sensor limits
    front_us_trigger = 0.3
    left_us_trigger = 0.3
    right_us_trigger = 0.3

    # Simulation time sampling
    Ts = 25e-3

    # Robot state machine
    STOP = 0
    GO_AHEAD = 1
    CHECK_TURNIN = 2
    TURN_LEFT = 3
    TURN_RIGHT = 4
    state = 1

    # Initial values
    front_us_value = 1e5
    left_us_value = 1e5
    right_us_value = 1e5
    left_speed = 0
    right_speed = 0

    while sim.simxGetConnectionId(client_id):
        try:
            robot.updateUltrasonicMeasure()

            # Get ultrasonic measures (only if it is a valid value)
            front_us_value_aux = robot.ultrasonic_measures[0]
            if type(front_us_value_aux) != int:
                front_us_value = front_us_value_aux

            left_us_value_aux = robot.ultrasonic_measures[1]
            if type(left_us_value_aux) != int:
                left_us_value = left_us_value_aux

            right_us_value_aux = robot.ultrasonic_measures[2]
            if type(right_us_value_aux) != int:
                right_us_value = right_us_value_aux

            print("Front ", front_us_value)
            print("Left ", left_us_value)
            print("Right ", right_us_value)
            
            if state == GO_AHEAD:
                left_speed = 3
                right_speed = 3
                kp = 0.5

                # If front ultrasonic sensor detects an obstacle
                if front_us_value < front_us_trigger:
                    state = CHECK_TURNIN
                # If left ultrasonic sensor detects an wall, apply correction
                elif left_us_value < left_us_trigger:
                    error = (left_us_trigger - left_us_value) / left_us_trigger
                    right_speed -= error * kp
                # If right ultrasonic sensor detects an wall, apply correction
                elif right_us_value < right_us_trigger:
                    error = (
                        right_us_trigger - right_us_value
                    ) / right_us_trigger
                    left_speed -= error * kp

            elif state == CHECK_TURNIN:
                # If left has more space to turn
                if left_us_value > right_us_value:
                    state = TURN_LEFT
                # If right has more space to turn
                elif left_us_value < right_us_value:
                    state = TURN_RIGHT
                # Turn randmonly
                else:
                    right = int(time.time()) % 2 == 0

                    if right:
                        state = TURN_LEFT
                    else:
                        state = TURN_RIGHT

            elif state == TURN_LEFT:
                left_speed = -1
                right_speed = 1
                if front_us_value > (
                    front_us_trigger + front_us_trigger * 0.1
                ):
                    state = GO_AHEAD

            elif state == TURN_RIGHT:
                left_speed = 1
                right_speed = -1
                if front_us_value > (
                    front_us_trigger + front_us_trigger * 0.1
                ):
                    state = GO_AHEAD

            else:
                left_speed = 0
                right_speed = 0

            robot.move(left_speed, right_speed)

            time.sleep(Ts)
        except KeyboardInterrupt:
            print("Keyboard Interrup Active!")
            break

    print("X: ", robot.local["x"])
    print("Y: ", robot.local["y"])

    # plt.figure(figsize=(10, 10))
    # plt.subplot(3, 1, 1)
    # plt.plot(robot.local["x"], color="blue", label="X coord")

    # plt.subplot(3, 1, 2)
    # plt.plot(robot.local["y"], color="red", label="Y coord")

    # plt.subplot(3, 1, 3)
    # plt.plot(
    #     robot.local["x"], robot.local["y"], color="orange", label="XY coord"
    # )

    # plt.show()


if __name__ == '__main__':
    main()