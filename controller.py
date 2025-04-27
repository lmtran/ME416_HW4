#!/usr/bin/env python 3
""" A first template for a PID controller """

class PID():
    """ Computes proportional, derivative, and integral terms for a PID controller """
    def __init__(self, gain_kp=1.0, gain_kd=1.0, gain_ki=1.0):
        """Initializes gains and internal state (previous error and integral error)"""
        self.gain_kp = gain_kp
        self.gain_kd = gain_kd
        self.gain_ki = gain_ki

        self.error_signal_previous = None
        self.error_signal_integral = 0

    def proportional(self, error_signal):
        """ Compute proportional term (with gain) """
        # proportional control u = -kp * e
        control_p = -self.gain_kp * error_signal
        return control_p

    def integral(self, error_signal, time_delay):
        """ Compute integral term (with gain) """
        # implement Euler's method to solve simple ODE
        self.error_signal_integral = self.error_signal_integral + time_delay * error_signal
        control_i = -self.gain_ki * self.error_signal_integral
        return control_i

    def derivative(self, error_signal, time_delay):
        """ Compute derivative term (with gain) """
        # derivative control u = -kd * e_dot (approximate e_dot as (e(t) - e(t - h)/h )
        # h is the delay between two measurements fo the error - in our case, 2 frames
        if self.error_signal_previous is None:
            control_d = 0.
        else:
            control_d = -self.gain_kd * (error_signal - self.error_signal_previous) / time_delay
        self.error_signal_previous = error_signal
        return control_d
