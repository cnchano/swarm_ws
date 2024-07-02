
import numpy as np
#/////////////////////////////////////////////////////////////////// DELETE ME
# DRONE_SPAWN_2 = {
#     'drone_1_pos': ['-2.0', '-2.0', '1.0', '0.0'], #x, y, z [meters] , yaw 

#     'drone_2_pos': ['0.0', '1.0', '1.0', '0.0'], #x, y, z [meters] , yaw 

#     'drone_3_pos': ['0.0', '3.0', '1.0', '0.0'], #x, y, z [meters] , yaw 

# }

# DRONE_SPAWN = {
#     'Drone1_pos': np.array([[1.0], [1.0], [1.0]]), #x, y, z [meters] #must be column vector
#     'Drone1_yaw': 0.0 #yaw
# }
#///////////////////////////////////////////////////////////////////
FORMATION_CONST = {
    'formation_names': np.array(['single_file', 'single_row', 'arrow', 'circle'])
}

FILTER_CONSTANT = {
    'imu_accel_moving_avg_window': 25,
    'velocity_moving_avg_window': 100,#was 10 9:26pm 4/20/23
    'pid_integral_window': 100, #NOTE: HOW LARGE SHOULD IT BE? CONSIDER WIND-UP
    'pid_accel_avg_window': 10
}

DRONE_CONSTANT = {
    #Mass moment of inertia about the center of mass of the quadrotor
    'Ixx': 0.19978,# kg*m^2
    'Iyy': 0.19979,# kg*m^2
    'Izz': 0.35833,# kg*m^2
    #mass moment of inertia of each propeller's rotating mass
    'Jtp': 0.001543074, # kg*m^2
    #mass of the quadrotor
    'mass': 6.3, #kg
    #gravitational acceleration
    'g': 9.80665, #m/sec^2

    #quadrotor arm length
    'arm_length': 0.4475, # meters
    #initial quadrotor propeller motor speed [rad/sec]
    'init_omega_prop': 110, #[rad/sec]
    
    #Air Density - reference excel sheet calcs for propeller coefficients
    'air_density': 1.2041, #kg/m^3
    'prop_radius': 0.266701, #meters
  


}

POLES = {
    'P1': -2, #previous -5
    'P2': -2, #previous -7

    'pos_alt_Kp': 3.0, 
    'pos_alt_Kd': 2.0,
    'ss_pos_alt_Kp': 16.0,
    'ss_pos_alt_Ki': 3.0,
    'ss_pos_alt_Kd': 12.0,

    'pos_xy_Kp': 0.0,
    'pos_xy_Ki': 0.0,
    'pos_xy_Kd': 1.0,

    'yaw_Kp': 20,
    'yaw_Ki': 10,
    'yaw_Kd': 1,

    'roll_Kp': 12,
    'roll_Ki': 0.01,
    'roll_Kd': 280,

    'pitch_Kp': 12,
    'pitch_Ki': 0.01,
    'pitch_Kd': 280,
}

POS_CONTROL = {
    #The poles (aka lambda values) for the controller gains [constants]
    'lambda_x': np.array([POLES['P1'],POLES['P2']]),
    'lambda_y': np.array([POLES['P1'],POLES['P2']]),
    'lambda_z': np.array([POLES['P1'],POLES['P2']]),
    'K2': POLES['P1'] + POLES['P2'],
    'K1': (POLES['P1'] - ((POLES['P1'] + POLES['P2'])/2))**(2) - ((POLES['P1'] + POLES['P2'])**2)/4
} 


MPC_CONTROL = {
    'Q': np.matrix('10 0 0; 0 10 0; 0 0 10'), # weights for outputs (all samples, except the last one)
    'S': np.matrix('20 0 0; 0 20 0; 0 0 20'), # weights for the final horizon period outputs
    'R': np.matrix('10 0 0; 0 10 0; 0 0 10'), # weights for inputs
    'horizon': 4,
}

CTRL_DT = {     
    'Ts': 0.1, #time step of the drone controller conductor
    'vel_Ts': 0.01 #time step of velocity calculation
}

SWARM_FAILURE = {
    'fail_alt_bound': 2.0, #meter
    'min_fail_alt_bound': 0.125, #meter
}

BEHAVIORAL_CTRL = {
    #'controller_time_step': 0.4, #sec
 
    #goal/waypoint weight
    'w_goal': 0.75, #0.25 used with form at 1.0 #0.5 with form 0.5 good #1.0 with form 0.75 good but slightly disorganized
    # 'w_waypoint': 1, #goal and waypoint treated the same
    #goal/waypoint boundary to start slowing down
    'bound_goal': 10, # meter radius
    'min_bound_goal': 0.05, #meter radius
    
    'bound_waypt': 0.25, #mater radius
    'min_waypoint_bound': 0.5, #% of the maximum speed allowed when passing through a waypoint.
    #==========================================================
    #climbing and descending behavior
    'w_climb': 1.0,
    'bound_climb': 10,
    'min_bound_climb': 0.5,
    #==========================================================
    # #obstacle avoidance weight #NOTE: NO OBSTACLE AVOIDANCE
    # 'w_obj': 1,
    # #obstacle avoidance boundary around object (outer boundary/upper limit)
    # 'bound_obj_ul': 2, #meter radius
    # #obstacle avoidance boundary around object (inner boundary/lower limit)
    # 'bound_obj_ll': 1, #meter radius
    # #Angle of escape relative to nearest object for the right side of the quadrotor
    # 'beta_right': np.pi/2,
    # #Angle of escape relative to nearest object for the left side of the quadrotor
    # 'beta_left': -np.pi/2,

    #==========================================================
    #formation maintenance weight
    'w_form': 0.0, #0.85 , NOTE: WORKED WELL FOR REFORMATION #0.5 with goal 0.5 good
    'bound_form': 10, #meter radius
    #Formation maintenance position error maximum
    'min_bound_form': 0.0, #meter radius
    #==========================================================
    #Inter-agent avoidance weight
    'w_avoid_uav': 0.2, # 1.0,

    #Inter-agent avoidance boundary
    'bound_avoid_uav': 6.0, #meter radius
    'min_bound_avoid_uav': 3.0, #meter radius

    #Drone Physical collision radius
    'collision_bound_avoid_uav': 1.0, #meters

   
    #==========================================================
    #Altitude maintenance
    'w_alt': 1.0,
    'bound_alt': 0.5, #m
    'bound_alt_pos_ctrl': 0.5, #m
    'min_bound_alt': 0.125, # prev 0.125
    #Max ascent acceleration 3m/s (arbitrary)
    'max_lin_accel_z': 1, #m/s^2
    #Max ascent velocity 6 m/s
    'max_lin_vel_z': 1.0, #0.5*6,

    #==========================================================
    #Max TRANSLATIONAL velocity (Specs max velocity = 23 m/s)
    'max_lin_vel_xy': 3.8, #NOTE: When value set to 1, everything will cancel out when goal position is zero and thus no velocity vector in any direction.

    #Max TRANSLATIONAL Acceleration
    'max_lin_accel_xy': 1, 
    #==========================================================
    #Orientation Maintenance
    #Boundary around the goal position to start implementing the weight for reduced speed near the goal
    'bound_ang_yaw': np.pi/8, 
    #weight max value
    'w_yaw': 1,
    #angle from the x-axis of the body frame to the centerline of the quadrotor counter-clockwise
    'alpha_f': np.pi/4,
    #Max yaw angular velocity from documentation 100deg/sec NOTE! set at 50% of max
    'max_yaw_ang_vel': 0.25*((100*np.pi)/180),
    #Boundary around final goal yaw position to start implementing integral controller and any change in PID gain constants
    'bound_yaw_pos_ctrl': 5*(np.pi/180), #m
    #==========================================================
    #///////////////////////////////
    #Attitude Maintenance
    'bound_ang_roll&pitch': 20*(np.pi/180),

    'max_roll&pitch_ang_vel': 100*(np.pi/180), #spec states 300deg/sec or 5.23598775 rad/sec NOTE: will set to 33% speed
    #weight max value
    'w_roll&pitch': 1,
    #//////////////////////////////
    #==========================================================

}

ATTITUDE_CTRL = {
    'bound_roll_pos_ctrl': 20*(np.pi/180), #Max and min boundary of phi angle to ensure small angle approxmation
    'bound_pitch_pos_ctrl': 20*(np.pi/180), #Max and min boundary of theta angle to ensure small angle approxmation

}
