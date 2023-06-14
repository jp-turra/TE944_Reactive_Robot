import math

import numpy as np

import sim

np.set_printoptions(precision=4)

# Initial Pose Control Class
# TODO: Fix this class to work properly
class Positioning():

    time_sampling: float = 0.0

    wheel_dist: float = 0.0  # Distance between left and right Wheel (Axis)
    wheel_radius: float = 0.0  # Wheel radius

    local = {
        "x": [0.0],
        "y": [0.0],
        "phi": [0.0],
    }

    def __init__(
        self, wheel_radius: float, wheel_dist: float, time_sampling: float
    ) -> None:
        self.wheel_dist = wheel_dist
        self.wheel_radius = wheel_radius
        self.time_sampling = time_sampling

    def updatePosition(
        self, left_speed: float, right_speed: float, Ts: float = None
    ):
        w = (right_speed - left_speed) / self.wheel_dist
        v = (right_speed + left_speed) / 2

        time_sampling = self.time_sampling
        if Ts != None:
            time_sampling = Ts

        x = self.local["x"][-1] + v * time_sampling * math.cos(
            self.local["phi"][-1]
        )
        y = self.local["y"][-1] + v * time_sampling * math.sin(
            self.local["phi"][-1]
        )
        phi = self.local["phi"][-1] + w * time_sampling

        # print("X ", round(x, 2))
        # print("Y ", round(y, 2))
        # print("phi ", round(phi, 2))

        self.local["x"].append(x)
        self.local["y"].append(y)
        self.local["phi"].append(phi)

# Abstraction Robot Class of CoppeliaSim API
class Robot(Positioning):

    client_id: int = 0
    robot_name: str = ""
    ultrasonic_handler = [0, 0, 0]
    ultrasonic_measures = [1e5, 1e5, 1e5]
    ultrasonic_status = [False, False, False]
    first: bool = True

    def __init__(
        self, client_id: int, robot_name: str, wheel_radius: float,
        wheel_dist: float, time_sampling: float
    ) -> None:
        super().__init__(wheel_radius, wheel_dist, time_sampling)
        self.client_id = client_id
        self.robot_name = robot_name

    def getMotorHandlers(self, left_motor_name: str, right_motor_name: str):

        abs_path = "/%s/"%(self.robot_name)

        return_code, self.left_motor_handle = sim.simxGetObjectHandle(
            self.client_id, abs_path+left_motor_name, sim.simx_opmode_oneshot_wait
        )
        if return_code != sim.simx_error_noerror:
            return return_code

        return_code, self.right_motor_handle = sim.simxGetObjectHandle(
            self.client_id, abs_path+right_motor_name, sim.simx_opmode_oneshot_wait
        )
        if return_code != sim.simx_error_noerror:
            return return_code

        return return_code

    def getUltrasonicHandlers(
        self,
        front_sensor_name: str = "ultrasonicSensor_front",
        left_sensor_name: str = "ultrasonicSensor_left",
        right_sensor_name: str = "ultrasonicSensor_right"
    ) -> int:

        # abs_path = "/%s/"%(self.robot_name)

        return_code, self.ultrasonic_handler[0] = sim.simxGetObjectHandle(
            self.client_id, front_sensor_name, sim.simx_opmode_oneshot_wait
        )
        if return_code != sim.simx_error_noerror:
            return return_code

        return_code, self.ultrasonic_handler[1] = sim.simxGetObjectHandle(
            self.client_id, left_sensor_name, sim.simx_opmode_oneshot_wait
        )
        if return_code != sim.simx_error_noerror:
            return return_code

        return_code, self.ultrasonic_handler[2] = sim.simxGetObjectHandle(
            self.client_id, right_sensor_name, sim.simx_opmode_oneshot_wait
        )

        return return_code

    def updateUltrasonicMeasure(self) -> None:

        mode = sim.simx_opmode_streaming if self.first else sim.simx_opmode_buffer
        self.first = False

        _, self.ultrasonic_status[
            0], ultrasonic_front_vector, _, _ = sim.simxReadProximitySensor(
                self.client_id, self.ultrasonic_handler[0], mode
            )
        _, self.ultrasonic_status[
            1], ultrasonic_left_vector, _, _ = sim.simxReadProximitySensor(
                self.client_id, self.ultrasonic_handler[1], mode
            )
        _, self.ultrasonic_status[
            2], ultrasonic_right_vector, _, _ = sim.simxReadProximitySensor(
                self.client_id, self.ultrasonic_handler[2], mode
            )

        if not self.ultrasonic_status[0]:
            self.ultrasonic_measures[0] = 2
        else:
            self.ultrasonic_measures[0] = round(
                math.sqrt(
                    math.pow(ultrasonic_front_vector[0], 2)
                    + math.pow(ultrasonic_front_vector[1], 2)
                    + math.pow(ultrasonic_front_vector[2], 2)
                ), 2
            )
        if not self.ultrasonic_status[1]:
            self.ultrasonic_measures[1] = 2
        else:
            self.ultrasonic_measures[1] = round(
                math.sqrt(
                    math.pow(ultrasonic_left_vector[0], 2)
                    + math.pow(ultrasonic_left_vector[1], 2)
                    + math.pow(ultrasonic_left_vector[2], 2)
                ), 2
            )
        if not self.ultrasonic_status[1]:
            self.ultrasonic_measures[2] = 2
        else:
            self.ultrasonic_measures[2] = round(
                math.sqrt(
                    math.pow(ultrasonic_right_vector[0], 2)
                    + math.pow(ultrasonic_right_vector[1], 2)
                    + math.pow(ultrasonic_right_vector[2], 2)
                ), 2
            )

    def move(
        self,
        left_motor_speed: float,
        right_motor_speed: float,
        scaller: float = 1
    ) -> None:

        sim.simxSetJointTargetVelocity(
            self.client_id, self.left_motor_handle, left_motor_speed * scaller,
            sim.simx_opmode_oneshot_wait
        )
        sim.simxSetJointTargetVelocity(
            self.client_id, self.right_motor_handle,
            right_motor_speed * scaller, sim.simx_opmode_oneshot_wait
        )

    # TODO: Fix this to work properly
    def goto(self, x_ref: float, y_ref: float):
        return self.proportionalControl(x_ref, y_ref)

    def _sign(self, x: float | int) -> int:
        if x >= 0:
            return 1
        else:
            return -1

    # TODO: Correct to work with Pose Control
    def proportionalControl(self, x_ref: float, y_ref: float):
        y_error = (y_ref - self.local["y"][-1])
        x_error = (x_ref - self.local["x"][-1])

        phi_ref = 0
        if x_error != 0:
            phi_ref = math.atan(y_error / x_error)
        phi_error = phi_ref - self.local["phi"][-1]

        if abs(phi_error) > (math.pi):
            phi_error = (math.pi) * self._sign(phi_error)

        print("\nx \t\t", self.local["x"][-1])
        print("y \t\t", self.local["y"][-1])
        print("x_error \t", x_error)
        print("y_error \t", y_error)
        print("phi_error\t", phi_error)

        # Default proportional control law
        # K = 1
        # w = K * (phi_error)
        # v = K * (math.sqrt(y_error**2 + x_error**2))

        # Better proportional control law
        Kw = 8
        Kv = 2
        w = Kw * math.atan(math.tan(phi_error))
        v = Kv * (math.sqrt(y_error**2
                            + x_error**2)) * self._sign(math.cos(phi_error))

        if abs(v) > 4:
            v = 4 * self._sign(v)

        if abs(w) > math.pi:
            w = math.pi * self._sign(w)

        print("w \t\t", w)
        print("v \t\t", v)

        left_speed = (2 * v - self.wheel_dist * w) / 2
        right_speed = (2 * v + self.wheel_dist * w) / 2

        print("left_speed \t", left_speed)
        print("right_speed\t", right_speed)

        return left_speed, right_speed