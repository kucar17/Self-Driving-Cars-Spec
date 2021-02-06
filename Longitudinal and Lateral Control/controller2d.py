#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.

THE CODE IS NOT COMPLETE, IT SHALL NOT YIELD THE DESIRED RESULT FOR THE LATERAL CONTROL PART:
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """

        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('error_integral', 0.0)
        self.vars.create_var('error_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('lateral_error_previous', 0.0)
        self.vars.create_var('steer', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            Kp = 4
            Ki = 0.22
            Kd = 0.2
            
            dt = t - self.vars.t_previous
            error = v_desired - v
            print("longitudinal error is: ", error)
            self.vars.error_integral += error * dt
            acceleration = Kp * error + (Ki * self.vars.error_integral) + (Kd * (error - self.vars.error_previous) / dt)

            throttle_output = acceleration
            brake_output    = 0

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change the steer output with the lateral controller. 
            current_index = int(t / dt)
            steer_output = 0
            k = 0.3

            xc = x + (1.5 * np.cos(yaw))
            yc = y + (1.5 * np.sin(yaw))

            ref_path_x = waypoints[current_index][0] - waypoints[current_index-1][0]
            ref_path_y = waypoints[current_index][1] - waypoints[current_index-1][1]

            slope = ref_path_y / ref_path_x
            constant = waypoints[current_index][1] - (slope * waypoints[current_index][0])

            a = -1 * slope
            b = 1
            c = -1 * constant

            slope_angle = np.arctan2(waypoints[current_index][1]-waypoints[current_index-1][1], waypoints[current_index][0]-waypoints[current_index-1][0])

            #if slope_angle > np.pi:
            #    slope_angle = -2 * np.pi + slope_angle
            #if slope_angle < - np.pi:
            #    slope_angle = 2 * np.pi - slope_angle

            e_cross = (a*xc) + (b*yc) + c
            e_cross = e_cross / (np.sqrt(a**2 + b**2))

            current_xy = np.array([x, y])
            e_cross = np.min(np.sum((current_xy - np.array(waypoints)[:, :2])**2, axis=1))

            cross_steering = np.arctan(k*abs(e_cross) / (v+0.001))

            theta = slope_angle - yaw
            if theta > np.pi:
                theta -= 2 * np.pi
            if theta < - np.pi:
                theta += 2 * np.pi

            #if theta > np.pi:
            #    theta = np.pi - theta
            print("yaw is: ", theta)

            e_heading = np.arctan2(-a, b) - theta

            steer_prime = e_heading + cross_steering

            if steer_prime > 1.22:
                steer_output = 1.22
            elif steer_prime < -1.22:
                steer_output = -1.22
            else:
                steer_output = steer_prime 
            

            #error_dot = -v * np.sin(yaw - self.vars.steer)
            #lat_error = self.vars.lateral_error_previous + (error_dot * dt)
            print("lateral error is: ", e_cross)
            print("steer is: ", steer_output)
            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.error_previous = error
        self.vars.t_previous = t
