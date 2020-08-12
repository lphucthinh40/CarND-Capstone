import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,wheel_radius,
                       wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        self.steering_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0.
        mx = 0.2
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5
        ts = 0.02
        self.vel_lowpass_filter = LowPassFilter(tau, ts) 

        self.prev_time = rospy.get_time()

    
    def control(self, target_linear_vel, target_angular_vel, current_vel, dbw_enabled):

        if not dbw_enabled:
        	self.throttle_controller.reset()
        	return 0., 0., 0.

        current_vel = self.vel_lowpass_filter.filt(current_vel)

        steering = self.steering_controller.get_steering(target_linear_vel, target_angular_vel, current_vel)

        vel_error = target_linear_vel - current_vel
        curr_time = rospy.get_time()
        sample_time = curr_time - self.prev_time
        self.prev_time = curr_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if target_linear_vel == 0. and current_vel < 0.1:
        	throttle = 0.
        	brake = 700

        elif throttle < 0.1 and vel_error < 0:
        	decel = max(vel_error, self.decel_limit)
        	brake = abs(decel)*self.vehicle_mass*self.wheel_radius

        return throttle, brake, steering