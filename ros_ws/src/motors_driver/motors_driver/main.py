import pigpio  # http://abyz.co.uk/rpi/pigpio/python.html

import rclpy
from rclpy.node import Node
from interfaces.msg import WheelsVelocities


class Motor():
    def __init__(self, pi, pid_period, pwm_pin, direction_pin, encoder_A_pin, encoder_B_pin, P, I, D):
        self.pi = pi
        self.pid_period = pid_period
        self.pwm_pin = pwm_pin
        self.direction_pin = direction_pin
        self.encoder_A_pin = encoder_A_pin
        self.encoder_B_pin = encoder_B_pin

        self.P = P
        self.I = I
        self.D = D

        self.requested_velocity = 0
        self.actual_velocity = 0
        self.current_tick = 0
        self.last_tick = 0
        self.error_integrated = 0
        self.error_previous = 0

        self.pwm_frequency = 1000
        self.pi.set_mode(direction_pin, pigpio.OUTPUT)
        self.pi.set_mode(encoder_A_pin, pigpio.INPUT)
        self.pi.set_mode(encoder_B_pin, pigpio.INPUT)

        self.max_error_integrated = 1000

        pi.callback(encoder_A_pin, RISING_EDGE, self.encoder_callback)

    def encoder_callback(self, gpio, level, tick):
        if self.pi.read(self.encoder_B_pin):
            self.current_tick += 1
        else:
            self.current_tick -= 1

    def calculate_actual_velocity(self):
        self.actual_velocity = (self.current_tick - self.last_tick) / self.pid_period
        self.last_tick = self.current_tick

    def calculate_pid_effort(self):
        error = (self.requested_velocity - self.actual_velocity)

        self.error_integrated += error
        if self.error_integrated > self.max_error_integrated:
            self.error_integrated = self.max_error_integrated
        if self.error_integrated < -self.max_error_integrated:
            self.error_integrated = -self.max_error_integrated

        error_derivative = error - self.error_previous
        self.error_previous = error

        effort = self.P * error + self.I * self.error_integrated + self.D * error_derivative

        if effort > 1:
            effort = 1
        if effort < -1:
            effort = -1

        return effort

    def set_motor_effort(self, effort):
        if effort > 0:
            self.pi.write(self.direction_pin, 1)
        else:
            self.pi.write(self.direction_pin, 0)

        dutycycle = abs(effort) * 1000000
        self.pi.hardware_PWM(self.pwm_pin, self.pwm_frequency, dutycycle)


class MotorsDriver(Node):

    def __init__(self):
        super().__init__('motors_driver')
        pi = pigpio.pi()
        pid_period = 0.5  # seconds
        P = 1
        I = 0
        D = 0

        right_pwm_pin = 0
        right_direction_pin = 0
        right_encoder_forward_pin = 0
        right_encoder_back_pin = 0

        left_pwm_pin = 0
        left_direction_pin = 0
        left_encoder_forward_pin = 0
        left_encoder_back_pin = 0

        self.right_motor = Motor(pi, pid_period, right_pwm_pin, right_direction_pin, right_encoder_forward_pin, right_encoder_back_pin, P, I, D)
        self.left_motor = Motor(pi, pid_period, left_pwm_pin, left_direction_pin, left_encoder_forward_pin, left_encoder_back_pin, P, I, D)

        self.subscription = self.create_subscription(WheelsVelocities, 'requested_velocity', self.set_requested_speed_callback, 10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(pid_period, self.calculate_pid_callback)
        self.actual_velocity_publisher = self.create_publisher(WheelsVelocities, 'actual_velocity', 10)

    def set_requested_speed_callback(self, msg):
        self.right_motor.requested_velocity = msg.right_wheel
        self.left_motor.requested_velocity = msg.left_wheel

    def calculate_pid_callback(self):
        self.right_motor.calculate_actual_velocity()
        effort = self.right_motor.calculate_pid_effort()
        self.right_motor.set_motor_effort(effort)

        self.left_motor.calculate_actual_velocity()
        effort = self.left_motor.calculate_pid_effort()
        self.left_motor.set_motor_effort(effort)

        msg = WheelsVelocities()
        msg.right_wheel = self.right_motor.actual_velocity
        msg.left_wheel = self.left_motor.actual_velocity
        self.actual_velocity_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    motors_driver = MotorsDriver()

    rclpy.spin(motors_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motors_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
