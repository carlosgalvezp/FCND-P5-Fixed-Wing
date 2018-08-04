# -*- coding: utf-8 -*-
import numpy as np

def normalize_angle(x):
    """Normalize angle to the range [pi, -pi]."""
    x = (x + np.pi) % (2.0*np.pi)

    if x < 0:
        x += 2.0*np.pi

    return x - np.pi

class PIDController(object):
    def __init__(self, k_p=0.0, k_d=0.0, k_i=0.0, cmd_min=np.finfo(np.float64).min, cmd_max=np.finfo(np.float64).max):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        self.cmd_max = cmd_max
        self.cmd_min = cmd_min

        self.cum_error = 0.0

    def run(self, error, error_dot=0.0, dt=1.0, feedforward=0.0):
        p = self.k_p * error
        d = self.k_d * error_dot

        cmd = p + d + feedforward

        if self.cmd_min < cmd < self.cmd_max:
            # Prevent integrator windup
            self.cum_error += error * dt

            i = self.k_i * self.cum_error
            cmd += i
        else:
            # Make sure the command is in its boundaries
            cmd = min(max(cmd, self.cmd_min), self.cmd_max)

        return cmd

class LongitudinalAutoPilot(object):
    def __init__(self):
        self.max_throttle_rpm = 2500
        self.max_elevator = np.radians(30.0)

        self.min_throttle = 0.0
        self.max_throttle = 1.0
        self.max_pitch_cmd = np.radians(30.0)
        self.max_pitch_cmd2 = np.radians(45.0)

        self.speed_int = 0.0
        self.alt_int = 0.0
        self.climb_speed_int = 0.0

        # Controllers
        self.pitch_controller = PIDController(k_p = 4.0, k_d = 0.5,
                                              cmd_min = -self.max_elevator,
                                              cmd_max = self.max_elevator)
        self.altitude_controller = PIDController(k_p = 0.02, k_d = 0.0, k_i = 0.0025,
                                                 cmd_min = -self.max_pitch_cmd,
                                                 cmd_max =  self.max_pitch_cmd)

        self.airspeed_controller = PIDController(k_p = 0.1, k_d = 0.0, k_i = 0.01,
                                                 cmd_min = self.min_throttle,
                                                 cmd_max = self.max_throttle)

        self.airspeed_pitch_controller = PIDController(k_p = 0.1, k_d = 0.0, k_i = 0.05,
                                                       cmd_min = -self.max_pitch_cmd2,
                                                       cmd_max =  self.max_pitch_cmd2)

    def pitch_loop(self, pitch, pitch_rate, pitch_cmd):
        """Used to calculate the elevator command required to acheive the target
        pitch

            Args:
                pitch: in radians
                pitch_rate: in radians/sec
                pitch_cmd: in radians

            Returns:
                elevator_cmd: in percentage elevator [-1,1]
        """
        elevator_cmd = 0.0
        # STUDENT CODE HERE
        pitch_rate_cmd = 0.0
        error = pitch_cmd - pitch
        error_dot = pitch_rate_cmd - pitch_rate

        elevator_cmd = self.pitch_controller.run(error=error, error_dot=error_dot)

        return elevator_cmd

    """Used to calculate the pitch command required to maintain the commanded
    altitude

        Args:
            altitude: in meters (positive up)
            altitude_cmd: in meters (positive up)
            dt: timestep in seconds

        Returns:
            pitch_cmd: in radians
    """
    def altitude_loop(self, altitude, altitude_cmd, dt):
        pitch_cmd = 0.0
        # STUDENT CODE HERE
        error = altitude_cmd - altitude
        pitch_cmd = self.altitude_controller.run(error=error, dt=dt)
        return pitch_cmd


    """Used to calculate the throttle command required command the target
    airspeed

        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
            dt: timestep in seconds

        Returns:
            throttle_command: in percent throttle [0,1]
    """
    def airspeed_loop(self, airspeed, airspeed_cmd, dt):
        throttle_cmd = 0.0
        # STUDENT CODE HERE
        throttle_trim = 0.68  # From Scenario 1
        error = airspeed_cmd - airspeed
        throttle_cmd = self.airspeed_controller.run(error=error, dt=dt, feedforward=throttle_trim)

        return throttle_cmd
    """Used to calculate the pitch command required to maintain the commanded
    airspeed

        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
            dt: timestep in seconds

        Returns:
            pitch_cmd: in radians
    """
    def airspeed_pitch_loop(self, airspeed, airspeed_cmd, dt):
        pitch_cmd = 0.0
        # STUDENT CODE HERE
        error = airspeed_cmd - airspeed
        # NOTE: more airspeed means less pitch, so we need to invert the error
        pitch_cmd = self.airspeed_pitch_controller.run(error=-error, dt=dt)

        return pitch_cmd

    """Used to calculate the pitch command and throttle command based on the
    aicraft altitude error

        Args:
            airspeed: in meter/sec
            altitude: in meters (positive up)
            airspeed_cmd: in meters/sec
            altitude_cmd: in meters/sec (positive up)
            dt: timestep in seconds

        Returns:
            pitch_cmd: in radians
            throttle_cmd: in in percent throttle [0,1]
    """
    def longitudinal_loop(self, airspeed, altitude, airspeed_cmd, altitude_cmd,
                          dt):
        pitch_cmd = 0.0
        throttle_cmd = 0.0

        # STUDENT CODE HERE
        # Threshold after which to switch from climb/descend to altitude hold
        altitude_threshold = 20  # [m]

        if abs(altitude_cmd - altitude) > altitude_threshold:  # Climb/descend
            pitch_cmd = self.airspeed_pitch_loop(airspeed, airspeed_cmd, dt)

            if altitude_cmd > altitude:
                print('Climbing')
                throttle_cmd = self.max_throttle  # Climb
            else:
                print('Descending')
                throttle_cmd = self.min_throttle  # Descend
        else:  # Altitude hold
            print('Altitude hold')
            pitch_cmd = self.altitude_loop(altitude, altitude_cmd, dt)
            throttle_cmd = self.airspeed_loop(airspeed, airspeed_cmd, dt)

        return [pitch_cmd, throttle_cmd]



class LateralAutoPilot:

    def __init__(self):
        self.g = 9.81
        self.integrator_yaw = 0.0
        self.gate = 1
        self.max_roll = np.radians(60.0)
        self.state = 1

        self.roll_controller = PIDController(k_p = 10.0, k_d = 2.0,
                                             cmd_min = -self.max_roll,
                                             cmd_max = self.max_roll)

        self.sideslip_controller = PIDController(k_p = 10.0, k_i = 0.5)

        self.yaw_controller = PIDController(k_p = 1.5, k_i = 0.0,
                                            cmd_min = -self.max_roll,
                                            cmd_max = self.max_roll)

        self.straight_line_controller = PIDController(k_p=0.005)
        self.orbit_controller = PIDController(k_p=0.0075)


    """Used to calculate the commanded aileron based on the roll error

        Args:
            phi_cmd: commanded roll in radians
            phi: roll angle in radians
            roll_rate: in radians/sec
            dt: timestep in sec

        Returns:
            aileron: in percent full aileron [-1,1]
    """
    def roll_attitude_hold_loop(self,
                                phi_cmd,  # commanded roll
                                phi,    # actual roll
                                roll_rate,
                                dt = 0.0):
        aileron = 0
        # STUDENT CODE HERE
        roll_rate_cmd = 0.0
        error = phi_cmd - phi
        error_dot = roll_rate_cmd - roll_rate

        aileron = self.roll_controller.run(error, error_dot)

        return aileron

    """Used to calculate the commanded roll angle from the course/yaw angle

        Args:
            yaw_cmd: commanded yaw in radians
            yaw: roll angle in radians
            roll_rate: in radians/sec
            dt: timestep in sec

        Returns:
            roll_cmd: commanded roll in radians
    """
    def yaw_hold_loop(self,
                      yaw_cmd,  # desired heading
                      yaw,     # actual heading
                      dt,
                      roll_ff=0):
        roll_cmd = 0

        # STUDENT CODE HERE
        error = normalize_angle(yaw_cmd - yaw)
        roll_cmd = self.yaw_controller.run(error=error, dt=dt, feedforward=roll_ff)

        return roll_cmd


    """Used to calculate the commanded rudder based on the sideslip

        Args:
            beta: sideslip angle in radians
            dt: timestep in sec

        Returns:
            rudder: in percent full rudder [-1,1]
    """
    def sideslip_hold_loop(self,
                           beta, # sideslip angle
                           dt):
        rudder = 0
        # STUDENT CODE HERE
        error = beta
        rudder = self.sideslip_controller.run(error, dt=dt)

        return rudder

    """Used to calculate the desired course angle based on cross-track error
    from a desired line

        Args:
            line_origin: point on the desired line in meters [N, E, D]
            line_course: heading of the line in radians
            local_position: vehicle position in meters [N, E, D]

        Returns:
            course_cmd: course/yaw cmd for the vehicle in radians
    """
    def straight_line_guidance(self, line_origin, line_course,
                               local_position):
        course_cmd = 0
        # STUDENT CODE HERE
        # https://en.wikipedia.org/w/index.php?title=Distance_from_a_point_to_a_line
        # Create 2 points on the line
        p1 = np.array([line_origin[0], line_origin[1]])
        p2 = p1 + np.array([np.cos(line_course), np.sin(line_course)])

        # Apply formula to compute perpendicular distance to line (CTE)
        p0 = np.array([local_position[0], local_position[1]])
        cte = ((p2[1] - p1[1])*p0[0] - (p2[0]-p1[0])*p0[1] + p2[0]*p1[1] - p2[1]*p1[0]) / np.linalg.norm(p2 - p1)

        print(cte)

        course_cmd = self.straight_line_controller.run(error=cte, feedforward=line_course)
        return course_cmd

    """Used to calculate the desired course angle based on radius error from
    a specified orbit center

        Args:
            orbit_center: in meters [N, E, D]
            orbit_radius: desired radius in meters
            local_position: vehicle position in meters [N, E, D]
            yaw: vehicle heading in radians
            clockwise: specifies whether to fly clockwise (increasing yaw)

        Returns:
            course_cmd: course/yaw cmd for the vehicle in radians
    """
    def orbit_guidance(self, orbit_center, orbit_radius, local_position, yaw,
                       clockwise = True):
        course_cmd = 0
        # STUDENT CODE HERE
        # Compute distance to orbit
        p1 = np.array(orbit_center[0:2])
        p2 = np.array(local_position[0:2])
        distance = np.linalg.norm(p2 - p1)

        # Compute radius error
        error = distance - orbit_radius

        # Compute command
        course_cmd = self.orbit_controller.run(error=error, feedforward=yaw)

        # Increasing yaw means clockwise rotation
        if not clockwise:
            course_cmd = -course_cmd

        return course_cmd

    """Used to calculate the feedforward roll angle for a constant radius
    coordinated turn

        Args:
            speed: the aircraft speed during the turn in meters/sec
            radius: turning radius in meters
            cw: true=clockwise turn, false = counter-clockwise turn

        Returns:
            roll_ff: feed-forward roll in radians
    """
    def coordinated_turn_ff(self, speed, radius, cw):

        roll_ff = 0
        # STUDENT CODE HERE
        roll_ff = np.arctan2(speed**2, radius * self.g)

        if not cw:
            roll_ff = -roll_ff

        return roll_ff

    """Used to calculate the desired course angle and feed-forward roll
    depending on which phase of lateral flight (orbit or line following) the
    aicraft is in

        Args:
            local_position: vehicle position in meters [N, E, D]
            yaw: vehicle heading in radians
            airspeed_cmd: in meters/sec

        Returns:
            roll_ff: feed-forward roll in radians
            yaw_cmd: commanded yaw/course in radians
    """
    def path_manager(self, local_position, yaw, airspeed_cmd):

        roll_ff = 0
        yaw_cmd = 0
        # STUDENT CODE HERE


        return(roll_ff,yaw_cmd)


    """Used to calculate the desired course angle and feed-forward roll
    depending on which phase of lateral flight (orbit or line following) the
    aicraft is in

        Args:
            waypoint_tuple: 3 waypoints, (prev_waypoint, curr_waypoint, next_waypoint), waypoints are in meters [N, E, D]
            local_position: vehicle position in meters [N, E, D]
            yaw: vehicle heading in radians
            airspeed_cmd: in meters/sec

        Returns:
            roll_ff: feed-forward roll in radians
            yaw_cmd: commanded yaw/course in radians
            cycle: True=cycle waypoints (at the end of orbit segment)
    """
    def waypoint_follower(self, waypoint_tuple, local_position, yaw, airspeed_cmd):
        roll_ff = 0.0
        yaw_cmd = 0.0
        cycle = False

        # STUDENT CODE HERE



        return(roll_ff, yaw_cmd, cycle)



def euler2RM(roll,pitch,yaw):
    R = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
    cr = np.cos(roll)
    sr = np.sin(roll)

    cp = np.cos(pitch)
    sp = np.sin(pitch)

    cy = np.cos(yaw)
    sy = np.sin(yaw)

    R[0,0] = cp*cy
    R[1,0] = -cr*sy+sr*sp*cy
    R[2,0] = sr*sy+cr*sp*cy

    R[0,1] = cp*sy
    R[1,1] = cr*cy+sr*sp*sy
    R[2,1] = -sr*cy+cr*sp*sy

    R[0,2] = -sp
    R[1,2] = sr*cp
    R[2,2] = cr*cp

    return R.transpose()
