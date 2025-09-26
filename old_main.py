from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Axis, Color
from pybricks.tools import wait, StopWatch
import umath

STEER_DEFAULT_SPEED = 150
DRIVE_DEFAULT_SPEED = 200


class Robot:
    def __init__(
        self,
        hub: InventorHub,
        color_sensor: ColorSensor,
        steer_motor: Motor,
        drive_motor: Motor,
    ):
        self.hub = hub
        self.color_sensor = color_sensor
        self.steer_motor = steer_motor
        self.drive_motor = drive_motor
        self.drive_velocity = 0.0
        self.steer_velocity = 0.0
        self.set_drive_velocity(0.0)
        self.set_steer_velocity(0.0)

        self.steer_motor.run_target(STEER_DEFAULT_SPEED, 0.0)
        self.lost_look_side = "right"
        self.lost_new_side = True

        self.steer_left_limit = -55.0
        self.steer_right_limit = 55.0

        self.state = "follow"  # follow | lost
        self.prev_state = "follow"

        self.t = 0.0
        self.state_time = 0.0

        self.initial = True

    def get_imu(self, axis: Axis):
        return self.hub.imu.rotation(axis)

    def set_drive_velocity(self, vel: float):
        if abs(vel - self.drive_velocity) < 0.1:
            return
        self.drive_velocity = vel
        self.drive_motor.run(self.drive_velocity)

    def set_steer_velocity(self, vel: float):
        if abs(vel - self.steer_velocity) < 0.1:
            return
        self.steer_velocity = vel
        self.steer_motor.run(self.steer_velocity)

    def get_color(self):
        color = self.color_sensor.color()
        if color == Color.NONE:
            color = Color.GREEN

        return color

    def get_steer_orientation(self):
        return self.steer_motor.angle()

    def state_follow(self):
        if self.initial:
            self.set_drive_velocity(DRIVE_DEFAULT_SPEED)
            self.set_steer_velocity(0)

        if self.get_color() != Color.GREEN:
            self.state = "lost"

    def state_lost(self):
        if self.initial:
            self.set_drive_velocity(0)
            self.lost_new_side = True

        if self.lost_look_side == "right":
            if self.lost_new_side:
                self.set_steer_velocity(STEER_DEFAULT_SPEED)
                self.lost_new_side = False

            if self.get_steer_orientation() > self.steer_right_limit:
                self.lost_look_side = "left"
                self.lost_new_side = True
        if self.lost_look_side == "left":
            if self.lost_new_side:
                self.set_steer_velocity(-STEER_DEFAULT_SPEED)
                self.lost_new_side = False

            if self.get_steer_orientation() < self.steer_left_limit:
                self.lost_look_side = "right"
                self.lost_new_side = True

        if self.get_color() == Color.GREEN:
            self.state = "follow"

    def step(self, dt: float):
        self.t += dt / 1000
        self.state_time += dt / 1000

        if self.prev_state != self.state:
            self.state_time = 0.0
            self.prev_state = self.state
            self.initial = True

        print("STATE:", self.state)
        if self.state == "follow":
            self.state_follow()
        elif self.state == "lost":
            self.state_lost()
        else:
            print("Unknown state!!!")

        self.initial = False


robot = Robot(
    hub=InventorHub(),
    color_sensor=ColorSensor(Port.A),
    steer_motor=Motor(Port.B),
    drive_motor=Motor(Port.C),
)

stopwatch = StopWatch()
prev_time = stopwatch.time()
while True:
    print("\033[2J\033[H", end="")
    curr_time = stopwatch.time()
    dt = curr_time - prev_time
    prev_time = curr_time

    robot.step(dt)
    print("Drive velocity:", robot.drive_velocity)
    print("Steer velocity:", robot.steer_velocity)
    print("Detected color:", robot.color_sensor.color())
    print("Lost look side:", robot.lost_look_side)
    print("Steer orientation:", robot.get_steer_orientation())
    print("Steer stalled:", robot.steer_motor.stalled())

    # wait(1000)
