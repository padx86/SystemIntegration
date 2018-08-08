# Rospy imports
import rospy

# Controller and filter imports
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

DEBUG_ON = True

STATE_STOPPED = 1
STATE_DRIVING = 2

class Controller(object):
    def __init__(self, vehicle_mass, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        
        self.accelaration_controller = PID(kp=0.3, ki=0.1, kd=0.1, mn=-1.0, mx=0.5)
        self.yaw_controller = YawController(wheel_base=wheel_base, steer_ratio=steer_ratio, min_speed=0.05, max_lat_accel=max_lat_accel, max_steer_angle=max_steer_angle)
        self.lpfilter = LowPassFilter(tau=0.5,ts=0.02)
        
        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius
        self.vehicle_mass = vehicle_mass

        self.last_state = STATE_STOPPED
        self.last_time = rospy.get_time()
        if DEBUG_ON:
            rospy.logwarn('Controller Initiated')

    def control(self, cur_longitudinal_velocity, cur_angular_velocity, longitudinal_velocity, angular_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        # Initialize output
        throttle = 0.0
        braking = 0.0
        steering = 0.0

        # Get sample time
        cur_time = rospy.get_time()
        sample_time = cur_time - self.last_time
        self.last_time = cur_time

        # Filter velocity
        cur_velocity = self.lpfilter.filt(cur_longitudinal_velocity)

        # Get steering from yaw controller
        steering = self.yaw_controller.get_steering(longitudinal_velocity,angular_velocity,cur_velocity)

        # Get throttle from PID Controller
        pid_err = longitudinal_velocity - cur_velocity
        throttle = self.accelaration_controller.step(pid_err, sample_time)

        #Map throtlle to brake
        if throttle < 0:
            braking = 0.33* abs(throttle) * self.vehicle_mass * self.wheel_radius
            throttle = 0

        # Renew State
        if abs(cur_velocity) < 1:
            state = STATE_STOPPED
        else:
            state = STATE_DRIVING

        # State Transition 
        if state != self.last_state:
            if state == STATE_STOPPED:
                if DEBUG_ON:
                    rospy.logwarn('State transition ---- new state: STOPPED' )
            elif state == STATE_DRIVING:
                if DEBUG_ON:
                    rospy.logwarn('State transition ---- new state: DRIVING' )

        
        #Store current state
        self.last_state = state
        
        return throttle, braking, steering
