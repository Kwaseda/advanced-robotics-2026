"""The task 4 circle as a function of time. No ROS, so it can be tested on its own."""

import math


class CircleTrajectory:

    def __init__(self, radius, centre_x, centre_y, lap_period, laps, start_delay):
        if radius <= 0.0 or lap_period <= 0.0 or laps <= 0:
            raise ValueError('radius, lap_period and laps must all be positive')
        self.radius = radius
        self.centre_x = centre_x
        self.centre_y = centre_y
        self.lap_period = lap_period
        self.laps = laps
        self.start_delay = max(0.0, start_delay)

    def point_at(self, elapsed):
        """Setpoint (x, y, finished) at `elapsed` seconds after the node started.

        Holds the start point during start_delay so the robot can drive onto the circle,
        then walks it at constant speed, then holds the final point.
        """
        running = min(max(elapsed - self.start_delay, 0.0), self.laps * self.lap_period)
        finished = elapsed >= self.start_delay + self.laps * self.lap_period
        phase = 2.0 * math.pi * running / self.lap_period
        return (self.centre_x + self.radius * math.cos(phase),
                self.centre_y + self.radius * math.sin(phase),
                finished)
