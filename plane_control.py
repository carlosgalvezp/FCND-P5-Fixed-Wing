# -*- coding: utf-8 -*-
import numpy as np
PI = 3.14159
class PlaneControl(object):
    def __init__(self):
        self.max_throttle_rpm = 2500
        self.max_elevator = 30.0*PI/180.0
        
        self.speed_int = 0.0
        self.alt_int = 0.0
        self.climb_speed_int = 0.0
        
        return
    
    
    """Used to calculate the throttle command required command the target 
    airspeed
        
        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
        
        Returns:
            throttle_command: in percent throttle [0,1]
    """
    def airspeed_loop(self, airspeed, airspeed_cmd, 
                      dt = 0.0):        
        throttle_cmd = 0.0
        # STUDENT CODE HERE
        
        # START SOLUTION
        throttle_ff = 0.67
        gain_p_speed = 0.2
        gain_i_speed = 0.1
        max_speed_int = 0.25
        speed_error = (airspeed_cmd-airspeed)
        self.speed_int = self.speed_int + speed_error*dt
        if(gain_i_speed*abs(self.speed_int) > max_speed_int):
            self.speed_int = np.sign(self.speed_int)*max_speed_int/gain_i_speed
        

        throttle_cmd = gain_p_speed*speed_error + gain_i_speed * self.speed_int
        throttle_cmd = throttle_cmd + throttle_ff
        # END SOLUTION
        return throttle_cmd
    
    """Used to calculate the elevator command required to acheive the target
    pitch
    
        Args:
            pitch: in radians
            pitch_rate: in radians/sec
            pitch_cmd: in radians
        
        Returns:
            elevator_cmd: in percentage elevator [-1,1]
    """
    def pitch_loop(self, pitch, pitch_rate, pitch_cmd):
        elevator_cmd = 0.0
        # STUDENT CODE HERE
        
        # START SOLUTION
        gain_p_pitch = 20.0
        gain_p_q = 10.0
        elevator_cmd = gain_p_pitch*(pitch_cmd - pitch) - gain_p_q*pitch_rate
        # END SOLUTION
        return elevator_cmd
    
    """Used to calculate the pitch command required to maintain the commanded
    altitude
    
        Args:
            altitude: in meters (positive up)
            altitude_cmd: in meters (positive up)
        
        Returns:
            pitch_cmd: in radians
    """
    def altitude_loop(self, altitude, altitude_cmd, dt = 0.0):
        pitch_cmd = 0.0
        # STUDENT CODE HERE
        
        # START SOLUTION
        gain_p_alt = 0.03
        gain_i_alt = 0.005
        max_alt_int = 0.1
        max_pitch_cmd = 30.0*np.pi/180.0
        
        altitude_error = altitude_cmd-altitude
        self.alt_int = self.alt_int + altitude_error*dt
        if(abs(self.alt_int) > max_alt_int):
            self.alt_int = np.sign(self.alt_int)*max_alt_int
            
        pitch_cmd = gain_p_alt*altitude_error + gain_i_alt*self.alt_int
        
        if(abs(pitch_cmd)>max_pitch_cmd):
            pitch_cmd = np.sign(pitch_cmd)*max_pitch_cmd
        # END SOLUTION
        
        return pitch_cmd
    
    """Used to calculate the pitch command required to maintain the commanded
    airspeed
    
        Args:
            airspeed: in meters/sec
            airspeed_cmd: in meters/sec
        
        Returns:
            pitch_cmd: in radians
    """
    def airspeed_pitch_loop(self, airspeed, airspeed_cmd,
                            dt = 0.0, pitch_ff = 0.0):
        pitch_cmd = 0.0
        # STUDENT CODE HERE
        
        # START SOLUTION
        gain_p_airspeed = 0.2
        gain_i_airspeed = 0.02
        max_airspeed_int = 50.0
        
        airspeed_error = airspeed_cmd-airspeed
        self.climb_speed_int = self.climb_speed_int + airspeed_error*dt
        if(abs(self.climb_speed_int) > max_airspeed_int):
            self.climb_speed_int = np.sign(self.climb_speed_int)*max_airspeed_int
        pitch_cmd = -1.0*(gain_p_airspeed*airspeed_error +
                          gain_i_airspeed*self.climb_speed_int)
        pitch_cmd = pitch_cmd + pitch_ff
        #print(gain_i_airspeed*self.climb_speed_int)

        # END SOLUTION
        return pitch_cmd
    
    """Used to calculate the pitch command and throttle command based on the
    aicraft altitude error
    
        Args:
            airspeed: in meter/sec
            altitude: in meters (positive up)
            airspeed_cmd: in meters/sec
            altitude_cmd: in meters/sec (positive up)
            
        Returns:
            pitch_cmd: in radians
            throttle_cmd: in in percent throttle [0,1]
    """
    def longitudinal_loop(self, airspeed, altitude, airspeed_cmd, altitude_cmd,
                          dt = 0.0):
        pitch_cmd = 0.0
        throttle_cmd = 0.0
        # STUDENT CODE HERE
        
        # START SOLUTION
        max_altitude_diff = 25.0
        if(altitude_cmd-altitude > max_altitude_diff):
            throttle_cmd = 1.0
            pitch_cmd = self.airspeed_pitch_loop(airspeed, airspeed_cmd, dt)
        elif(altitude - altitude_cmd > max_altitude_diff):
            throttle_cmd = 0.1
            pitch_cmd = self.airspeed_pitch_loop(airspeed, airspeed_cmd, dt)
        else:
            throttle_cmd = self.airspeed_loop(airspeed, airspeed_cmd, dt)
            pitch_cmd = self.altitude_loop(altitude, altitude_cmd, dt)
        # END SOLUTION
        return[pitch_cmd, throttle_cmd]


# Vladimir's Autopilot Code


class DigitalContoroller:
    
    def __init__(self,kp,ki,kd,tau,t_s,u_max):
        '''
        Args:
            kp: proportionality coefficient 
            ki: integral coefficient
            kd: differential coefficient
            tau: time constant for controlling filter 
            t_s: time difference between samples
            u_max: maximum controls input 
        '''
        
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.tau = tau
        self.t_s = t_s 
        
        # initial values 
        self.d = 0.0
        self.e = 0.0
        self.i = 0.0
        self.previous_e = 0.0
        
        # maximum control input
        self.u = 0.0 
        self.u_max = u_max


    def proportional(self,commanded_value,true_value):
        '''
        Calculates the factor of the control input which is responsible for the proportionality element
        
        Args:
            commanded_value: comanded value which it needs to achieve 
            true_value: true value which needs to be changed 
        
        Returns:
            proportionality coefficient times the difference between commanded and true values 
        '''
        # please note that the differentation part is using the previous error value 
        self.previous_e = self.e
        self.e = commanded_value - true_value
        
        prop_factor = self.kp * self.e
        
        return prop_factor


    def differential(self):
        '''
        Calculates the factor of the control input which is responsible for the differentiation element
        '''
        
        # This step calculates and updates the D[n] on the same time 
        
        self.d = (2*self.tau - self.t_s)/(2*self.tau + self.t_s)*self.d \
                 + (2*self.tau)/(2*self.tau + self.t_s)*(self.e-self.previous_e)
            
        diff_factor =self.kd * self.d
        
        return diff_factor

    def integrator(self):
        '''
        calculates the integrator portion of the control input. 
        '''
        
        self.i = self.i + (self.t_s)/(2)*(self.e+self.previous_e)
        
        int_factor = self.ki * self.i
        
        return int_factor 

    def anti_wind_up(self):
        '''
        Performs anti-wind-up prosedure to keep the integrator withing levels that the controls will not over saturate 
        '''
        if self.ki !=0.0:
            u_unsat= self.kp * self.e + self.kd * self.d + self.ki * self.i

            self.i = self.i +self.t_s / self.ki * (self.u-u_unsat)


    def control_execution(self,commanded_value,true_value):
        '''
        Putting all the nessesary functions together to perform the controls operation
        '''
        prop_factor = self.proportional(commanded_value,true_value)
        diff_factor = self.differential()
        int_factor = self.integrator()
        
        u = prop_factor + diff_factor + int_factor 
        
        if abs(u) > self.u_max:
            self.u = np.sign(u) * self.u_max
        else:
            self.u = u 
        
        self.anti_wind_up()


class LateralAutoPilot:
    
    def __init__(self):
        self.g = 9.81
        self.integrator_phi = 0.0
        self.differentiation_phi = 0.0
        self.integrator_yaw = 0.0 
        self.integrator_beta = 0.0
        self.integrator_theta = 0.0 
        self.integrator_h = 0.0 
        self.error_x_dl = 0.0
        self.error_phi_dl = 0.0 
        self.error_beta_dl = 0.0 
        self.error_theta_dl = 0.0
        self.error_h_dl = 0.0 
        self.delta_phi_max = 60/180*np.pi




    def roll_attitude_hold_loop(self,
                                phi_c,  # commanded roll
                                phi,    # actual roll 
                                roll_rate, 
                                T_s = 0.0,
                                phi_ff = 0.0):
        
        

        gain_p_phi = 40.0
        gain_d_phi = 1.0
        #k_i_phi = 0.5

        #T_s = s
        #tau = 5*T_s

        #controlerX= DigitalContoroller(k_p_phi,k_i_phi,k_d_phi,tau,T_s,self.delta_phi_max)
        #controlerX.control_execution(phi_c,phi)
        #delta_a = controlerX.u
        aileron = gain_p_phi*(phi_c-phi) - gain_d_phi*roll_rate
        return aileron


    def yaw_hold_loop(self,
                         yaw_cmd,  # desired heading
                         yaw,     # actual heading 
                         T_s
                         ):
        
        
        gain_p_yaw = 2.0
        gain_i_yaw = 0.01

        #T_s = s 
        #tau = 5*T_s

        #courceX = DigitalContoroller(k_p_x,k_i_x,0.0,tau,T_s,np.pi)
        #courceX.control_execution(chi_c,chi)
        #phi_c = courceX.u
        
        self.integrator_yaw = self.integrator_yaw + (yaw_cmd-yaw)*T_s
        roll_cmd = gain_p_yaw*(yaw_cmd-yaw)+gain_i_yaw*self.integrator_yaw
        return roll_cmd



    def sideslip_hold_loop(self,
                           beta, # sideslip angle 
                           T_s):
        
        gain_p_beta = 1.0
        gain_i_beta = 1.0

        #T_s = s 
        #tau = 5*T_s

        #sideslipX = DigitalContoroller(k_p_beta,k_i_beta,0.0,tau,T_s,np.pi)
        #sideslipX.control_execution(0.0,beta)
        #delta_r= sideslipX.u 
        self.integrator_beta = self.integrator_beta+(0.0-beta)*T_s
        rudder = -1.0*(gain_p_beta*(0.0-beta)+gain_i_beta*self.integrator_beta)
        return rudder
    
    def straight_line_guidance(self, line_origin, line_course, 
                               local_position):
        gain_p_xtrack = 0.002
        xtrack_error = np.cos(line_course)*(local_position[1]-line_origin[1])+\
            -np.sin(line_course) * (local_position[0]-line_origin[0])
        course_cmd = -np.pi/2*np.arctan(gain_p_xtrack * xtrack_error) +\
            line_course;
        return course_cmd
    
    def orbit_guidance(self, orbit_center, orbit_radius, local_position, yaw,
                       clockwise = True):
        gain_orbit = 2.5
        radius = np.linalg.norm(orbit_center[0:2]-local_position[0:2])
        course_cmd = np.pi / 2 + np.arctan(
                gain_orbit * (radius - orbit_radius) / orbit_radius);
        if (clockwise == False):
            course_cmd = -course_cmd;

        addon = np.arctan2(local_position[1] - orbit_center[1],
                           local_position[0] - orbit_center[0]);
        if (addon-yaw < -np.pi):
            while (addon-yaw < -np.pi):
                addon = addon + np.pi * 2;
        elif (addon - yaw > np.pi):
            while (addon - yaw > np.pi):
                addon = addon - np.pi * 2;
        course_cmd = course_cmd + addon;
        
        
        return course_cmd





class LongitudinalAutoPilot:
    
    def __init__(self):
        self.g= 9.81
        self.integrator_phi = 0.0
        self.differentiation_phi = 0.0
        self.integrator_x = 0.0 
        self.integrator_beta = 0.0
        self.integrator_theta = 0.0 
        self.integrator_h = 0.0 
        self.error_x_dl = 0.0
        self.error_phi_dl = 0.0 
        self.error_beta_dl = 0.0 
        self.error_theta_dl = 0.0
        self.error_h_dl = 0.0 
        self.delta_phi_max = 60/180*np.pi
        self.delta_e_max = 60/180*np.pi
        



    def pitch_attitude_hold_loop(self,
                                 theta_c, 
                                 theta, 
                                 q,
                                 s
                                 ):
        
        T_s = s 
        
        k_p_theta = -1.0 
        k_i_theta = 1.0
        k_d_theta = 1.0

        tau = 5*T_s
        pitchX= DigitalContoroller(k_p_theta,k_i_theta,k_d_theta,tau,T_s,np.pi)
        pitchX.control_execution(theta_c,theta)

        delta_e = pitchX.u

        return delta_e 


    def attitude_hold_using_pitch(self,
                                  v_a, # velocity 
                                  h_c, # commanded height
                                  h,   # current height
                                  s
                                  ):
        

        # This is not being used 

        T_s = s
        k_i_h = 0.001
        k_d_h = 0.001
        k_p_h = 1.0

        error_h = h_c - h

        self.integrator_h = self.integrator_h + T_s/2*(error_h - self.error_h_dl)
        self.error_h_dl = error_h
        theta_c = k_i_h * error_h + k_d_h * (error_h - self.integrator_h) 

        theta_c_max = 5/180*np.pi
        if abs(theta_c) > theta_c_max:
          theta_c = np.sign(theta_c) * theta_c_max 
        
        return theta_c 


    def airspeed_hold_using_pitch(self,
                                  v_a_c,
                                  v_a,
                                  s
                                  ):
        
        # This is not being used

        W_v2 = 100.0
        self.zeta_v2 = 0.1
        
        omega_n_v2 = self.omega_n_theta / W_v2

        k_p_v2 = (self.a_v1 - 2.0 * self.zeta_v2 * omega_n_v2)/(self.K_theta_DC * self.g)   # eq. 6.23
        k_i_v2 =  - omega_n_v2**2 / (self.K_theta_DC * self.g)                         # eq. 6.25
        theta_c = k_p_v2 * (v_a_c - v_a) + k_i_v2/s * (v_a_c - v_a)
        
        return theta_c


    def airspeed_hold_using_throttle(self,
                                     delta_t_prime,
                                     v_a_c,
                                     v_a,
                                     s):
        
        self.zeta_v= 1.0
        self.a_v1 = 10.0
        self.a_v2 = 1000.0 
        
        omega_n_v = 1.0 
        
        k_p_v = (2.0 * self.zeta_v * omega_n_v - self.a_v1)/self.a_v2   # eq. 6.28
        k_i_v = omega_n_v**2 / self.a_v2                                # eq. 6.27 

        T_s = s 
        tau = 5*T_s

        throtleX = DigitalContoroller(k_p_v,k_i_v,0,tau,T_s,10)
        throtleX.control_execution(v_a_c,v_a)
        delta_t = delta_t_prime + throtleX.u

        
        return delta_t
    
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