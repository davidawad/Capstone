
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, wheel_radius, decel_limit, accel_limit):
        
        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

    def control(self, current_velocity, target_velocity):

    	# Convert to MPH
    	target_velocity = ONE_MPH * target_velocity

    	# Braking Torque 
        force = self.vehicle_mass * self.decel_limit
        torque = force * self.wheel_radius

        # Difference in Target and Actual Velocities
        del_vel = (target_velocity - current_velocity)


        # If Positive difference throttle = nonzero, brake = 0
        # Else throttle = 0, brake = nonzero
        if(del_vel > 0):
        	throttle = 1.0 - (current_velocity/target_velocity)
        	brake = 0
        else:
        	throttle = 0
        	brake = torque

        return throttle, brake
