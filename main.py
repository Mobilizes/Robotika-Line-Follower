from pybricks.hubs import InventorHub
from pybricks.robotics import DriveBase
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.tools import StopWatch
from pybricks.parameters import Port, Color, Direction, Stop


def map(
    value: float, src_min: float, src_max: float, dest_min: float, dest_max: float
) -> float:
    src_val = min(value, max(src_min, src_max))
    src_val = max(src_val, min(src_min, src_max))
    return dest_min + (src_val - src_min) / (src_max - src_min) * (dest_max - dest_min)


# TODO: need more tuning, but the program logic is not finished yet.
# Probably also add more configuration in the future
class RobotConfig:
    def __init__(
        self,
        wheel_diameter=56,
        axle_track=112,
        drive_velocity=200,
        drive_change_delay=0.100,
        turn_velocity=100,
        turn_left_limit=-55,
        turn_right_limit=55,
        yaw_velocity=100,
        yaw_left_limit=-110,
        yaw_right_limit=-50,
        pitch_velocity=100,
        pitch_up_limit=25,
        pitch_down_limit=-25,
        lost_line_time=2.0,
    ):
        # Hardware Configuration, to adjust for hardware
        self.wheel_diameter = wheel_diameter
        self.axle_track = axle_track

        self.drive_velocity = drive_velocity
        self.drive_change_delay = drive_change_delay

        self.turn_velocity = turn_velocity
        self.turn_left_limit = turn_left_limit
        self.turn_right_limit = turn_right_limit

        self.yaw_velocity = yaw_velocity
        self.yaw_left_limit = yaw_left_limit
        self.yaw_right_limit = yaw_right_limit

        self.pitch_velocity = pitch_velocity
        self.pitch_up_limit = pitch_up_limit
        self.pitch_down_limit = pitch_down_limit

        # Program Configuration, for program tuning
        self.lost_line_time = lost_line_time


class Robot(DriveBase):
    def __init__(
        self,
        hub: InventorHub,
        left_motor: Motor,
        right_motor: Motor,
        pitch_motor: Motor,
        yaw_motor: Motor,
        color_sensor: ColorSensor,
        ultrasonic_sensor: UltrasonicSensor,
        config: RobotConfig = RobotConfig(),
    ):
        self.config = config

        super().__init__(
            left_motor,
            right_motor,
            self.config.wheel_diameter,
            self.config.axle_track,
        )

        self.hub = hub

        self.pitch_motor = pitch_motor
        self.yaw_motor = yaw_motor
        self.color_sensor = color_sensor
        self.ultrasonic_sensor = ultrasonic_sensor

        self.step_state = "follow"  # follow | lost
        self.prev_step_state = self.step_state

        self.stopwatch = StopWatch()

        self.t = 0.0
        self.dt = 0.0
        self.state_time = 0.0
        self.lost_line_time = 0.0

        self.initial = True

        self.lost_look_side = "right"
        self.lost_new_side = True

        self.yaw_detect_line_angle = 0.0

        self.settings(straight_speed=self.config.drive_velocity)
        self.settings(turn_rate=self.config.turn_velocity)

    def is_detecting_line(self):
        # TODO: For some reason, the color sensor could detect black line as green
        # totally fixable but I don't feel like tuning the HSV for black.
        return self.get_color() in [Color.BLACK, Color.GREEN]

    def get_color(self):
        if self.color_sensor.color() == Color.NONE:
            return Color.BLACK
        return self.color_sensor.color()

    def sensor_move_to_target(
        self, yaw_target: float = None, pitch_target: float = None
    ):
        if yaw_target is not None:
            if yaw_target < self.config.yaw_left_limit:
                yaw_target = self.config.yaw_left_limit
            if yaw_target > self.config.yaw_right_limit:
                yaw_target = self.config.yaw_right_limit
            self.yaw_motor.run_target(self.config.yaw_velocity, yaw_target, wait=False)

        if pitch_target is not None:
            if pitch_target < self.config.pitch_down_limit:
                pitch_target = self.config.pitch_down_limit
            if pitch_target > self.config.pitch_up_limit:
                pitch_target = self.config.pitch_up_limit
            self.pitch_motor.run_target(
                self.config.yaw_velocity, pitch_target, wait=False
            )

    # this method name kinda sucks, idk a better one tho.
    # submit an issue for this lol
    def set_drive(self, drive_speed, turn_rate, force=False):
        if self.state_time > self.drive_delay or force:
            self.drive_delay += self.config.drive_change_delay

            if drive_speed != 0:
                self.drive(drive_speed, turn_rate)
            elif turn_rate != 0:
                self.turn(1 if turn_rate > 0 else -1, Stop.NONE, False)

    def process_scan(self):
        if self.lost_look_side == "right":
            if self.lost_new_side:
                self.sensor_move_to_target(yaw_target=self.config.yaw_right_limit)
                self.lost_new_side = False

            if self.yaw_motor.done():
                self.lost_look_side = "left"
                self.lost_new_side = True
        elif self.lost_look_side == "left":
            if self.lost_new_side:
                self.sensor_move_to_target(yaw_target=self.config.yaw_left_limit)
                self.lost_new_side = False

            if self.yaw_motor.done():
                self.lost_look_side = "right"
                self.lost_new_side = True

        if self.is_detecting_line():
            self.yaw_detect_line_angle = self.yaw_motor.angle()
            self.lost_line_time = 0.0
        else:
            self.lost_line_time += self.dt

    def process_state(self):
        if self.prev_step_state != self.step_state:
            self.initial = True
            self.state_time = 0.0
            self.prev_step_state = self.step_state

        print("STATE:", self.step_state)
        if self.step_state == "follow":
            self.state_follow()
        elif self.step_state == "lost":
            self.state_lost()
        else:
            print("Unknown state!!!")

        self.initial = False

    def state_follow(self):
        if self.initial:
            self.drive_delay = 0.0

        turn_rate = map(
            self.yaw_detect_line_angle,
            self.config.yaw_left_limit,
            self.config.yaw_right_limit,
            -self.config.turn_velocity,
            self.config.turn_velocity,
        )
        drive_speed = map(
            abs(turn_rate),
            0.0,
            self.config.turn_velocity,
            self.config.drive_velocity,
            self.config.drive_velocity * 0.5,
        )
        drive_speed -= map(
            self.lost_line_time,
            max(self.config.lost_line_time - 1.0, 0.0),
            self.config.lost_line_time,
            0.0,
            self.config.drive_velocity * 0.5,
        )  # To slow down if the robot don't find the line.

        self.set_drive(drive_speed, turn_rate)

        if self.lost_line_time > self.config.lost_line_time:
            self.step_state = "lost"

    def state_lost(self):
        if self.initial:
            self.set_drive(0.0, 0.0, force=True)

        # Basically t = s / v. I could just set a fixed value, but it's a pain
        # to set up a configuration for this one. I also just wanna try to
        # use this formula, maybe I don't need to tune this after all.
        if (
            self.state_time
            > (self.config.yaw_left_limit + self.config.yaw_right_limit)
            * 2
            / self.config.yaw_velocity
        ):
            self.set_drive(
                0.0,
                -1 if self.yaw_detect_line_angle < 0 else 1
            )

        # TODO: Assuming that color sensor is flawless and will not
        # detect random things as line. If not perfect, just add a timer.
        if self.is_detecting_line():
            self.step_state = "follow"

    def step(self):
        self.dt = self.stopwatch.time() / 1000
        self.stopwatch.reset()

        self.t += self.dt
        self.state_time += self.dt

        self.process_scan()
        self.process_state()


robot_config = RobotConfig()
hub = InventorHub()
left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)
pitch_motor = Motor(Port.C)
yaw_motor = Motor(Port.D)
color_sensor = ColorSensor(Port.E)
ultrasonic_sensor = UltrasonicSensor(Port.A)

robot = Robot(
    hub,
    left_motor,
    right_motor,
    pitch_motor,
    yaw_motor,
    color_sensor,
    ultrasonic_sensor,
    robot_config,
)

while True:
    robot.step()
