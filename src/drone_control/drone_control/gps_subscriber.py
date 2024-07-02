

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
import numpy as np
# Import writer class from csv module
from csv import writer
import os
import pymap3d #Version 2.9.1 NOTE: Latest version has API updates
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from drone_common.srv import GPStoENU
from drone_common.srv import SimClock


class GPS_Subscriber(Node):

    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            'gps', #call 'gps' relative (without "/" i.e. '/gps') to ensure the namespace is attached
            self.gps_listener_callback, # <- This is a required argument, can not be commented out
            10
        )
        
        self.declare_parameter('drone_spawn_x')
        self.declare_parameter('drone_spawn_y')
        self.declare_parameter('drone_spawn_z')
        self.declare_parameter('drone_spawn_yaw')

        #Service to provide GPS coordinates of drone in cartesian coordinates against a reference location
        self.srv = self.create_service(GPStoENU, 'gps_WGS84_to_enu_service', self.gps_WGS84_to_enu_callback)
        
        self.subscription  # prevent unused variable warning
        
        #Get drone spawn location from the specified parameter
        drone_spawn_x = self.get_parameter('drone_spawn_x').get_parameter_value().double_value #Needs to be double value to accept the string value povided in the launch file which is then converted
        drone_spawn_y = self.get_parameter('drone_spawn_y').get_parameter_value().double_value
        drone_spawn_z = self.get_parameter('drone_spawn_z').get_parameter_value().double_value
        drone_spawn_yaw = self.get_parameter('drone_spawn_yaw').get_parameter_value().double_value

        DRONE_SPAWN_ARRAY = [(drone_spawn_x), (drone_spawn_y), (drone_spawn_z), (drone_spawn_yaw) ]
        self.get_logger().info(f'Test drone spawn coordinates passed from launch: {DRONE_SPAWN_ARRAY}')
        #for the sake of testing, the reference location will be defined here
        self.llh_ref_g = np.array([[52.211309],[21.001001],[0.00]]) #[latitude (deg), longitude (deg), altitude (meters)]
        



        # self.sim_clock_sub_node = rclpy.create_node('sim_clock_sub_node') #create sub-node for gps client
        # #Initialize sim_clock Client
        # self.sim_clock_client = self.sim_clock_sub_node.create_client(SimClock, 'sim_clock_service')
        # while not self.sim_clock_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.sim_clock_req = SimClock.Request()
        
        #Initialize iteration counter for csv write to initialize file, then later add variables
        self.iter_count = 0.0

        #///////////////NOTE: NOT USED
        # #initialize past iteration timestamp to zero
        # self.past_time = 0.0

        # #initialize past velocity as zero m/s
        # self.past_drone_vel_if = np.zeros(3)
        #////////////////

        #Initialize drone gps location to be null
        self.past_denu = None

        #Initialize drone position in WGS84 coordinates
        ell_wgs84 = pymap3d.Ellipsoid('wgs84') #SET ellipsoid to WGS84 which is consistent with gazebo GPS plugin
        lat1, lon1, h1 = pymap3d.enu2geodetic(-DRONE_SPAWN_ARRAY[0], -DRONE_SPAWN_ARRAY[1], DRONE_SPAWN_ARRAY[2], #NOTE: UNSURE WHY this outputs gps coordinates in the negative of east/north positions
                                            self.llh_ref_g[0], self.llh_ref_g[1],self.llh_ref_g[2],                                 
                                            ell=ell_wgs84, deg=True)  # use wgs86 ellisoid
        #self.get_logger().info(f'Test ENU spawn coordinates by enu2geodetic(): {lat1, lon1, h1}')
        self.llh_drone_g = np.array([[lat1],[lon1],[h1]]) #INITIALIZED DRONE POSITION
        
        # e1k, n1k, u1k = pymap3d.geodetic2enu(lat1, lon1, h1, self.llh_ref_g[0], self.llh_ref_g[1],self.llh_ref_g[2], ell=ell_wgs84, deg=True)
        # print(e1k, n1k, u1k)   # 0,0,0  OK
        # self.get_logger().info(f'Test Inversion check by `geodetic2enu()`: {e1k, n1k, u1k}')

        
        
        # #///////////////////////////////////////
        # # Your ENU system needs origin definition (lat0, lon0, h0) +
        # # and also needs a reference ellipsoid: let's use `ell_wgs84` defined above
        # lat0, lon0, h0 = -90, 45, 0   # origin of ENU, (h is height above ellipsoid)

        # # Test ENU coordinates: (e1, n1, u1) by `enu2geodetic()`
        # e1, n1, u1     =  0.0,  0.0,  0.0  # just the origin of this ENU system
        # lat1, lon1, h1 = pymap3d.enu2geodetic(e1, n1, u1, \
        #                                     lat0, lon0, h0, \
        #                                     ell=ell_wgs84, deg=True)  # use wgs86 ellisoid
        # # this should agree with: (lat0, lon0, h0)
        # # print(lat1, lon1, h1)  # -90.0 44.99999999999999 1.313839409243646e-12  OK!
        # self.get_logger().info(f'Test ENU coordinates: (e1, n1, u1) by enu2geodetic(): {lat1, lon1, h1}')
        # # Inversion check by `geodetic2enu()`
        # # input values to convert: lat1, lon1, h1
        # e1k, n1k, u1k = pymap3d.geodetic2enu(lat1, lon1, h1, lat0, lon0, h0, ell=ell_wgs84, deg=True)
        # # print(e1k, n1k, u1k)   # 0,0,0  OK
        # self.get_logger().info(f'Test Inversion check by `geodetic2enu()`: {e1k, n1k, u1k}')
        # #///////////////////////////////////////

        # # Now arbitrary ENU to lat/long and reverse
        # lat112, lon112, h112 = pymap3d.enu2geodetic(1120, 100, 10, \
        #                                     lat0, lon0, h0, \
        #                                     ell=ell_wgs84, deg=True)
        # # print(lat112, lon112, h112)
        # self.get_logger().info(f'arbitrary ENU to lat/long: {lat112, lon112, h112}')
        # # Check
        # e112k, n112k, u112k = pymap3d.geodetic2enu(lat112, lon112, h112, lat0, lon0, h0, ell=ell_wgs84, deg=True)
        # # print(e112k, n112k, u112k)   # 1120, 100, 10 OK
        # self.get_logger().info(f'arbitrary lat/long to ENU: {e112k, n112k, u112k}')

    def gps_WGS84_to_enu_callback(self, request, response):

        drone_name = request.drone_name
        denu = self.dllh2denu(self.llh_ref_g, self.llh_drone_g)
        # self.get_logger().info(f'denu result: {denu}')
        de = denu[0]
        dn = denu[1]
        du = denu[2]
        response.de = float(de)
        response.dn = float(dn)
        response.du = float(du) 

        #response.drone_lin_vel = [float(drone_vel_if[0]), float(drone_vel_if[1]), float(drone_vel_if[2])]
        # #/////////////////////////////////////////////////////////////////////////////////////
        # #List that we want to add as a new row for data collection
        # csv_list = [
        #     self.llh_ref_g[0], self.llh_ref_g[1], self.llh_ref_g[2], 
        #     self.llh_drone_g[0],self.llh_drone_g[1], self.llh_drone_g[2], 
        #     de,  dn, du,
        #     # drone_vel_if[0], drone_vel_if[1], drone_vel_if[2],
        #     # self.time_now, self.past_time
        #     ]

        # csv_list_header = [
        #     "Ref_Frame_Lat_Pos [deg]", "Ref_Frame_Long_Pos [deg]", "Ref_Frame_Alt_Pos [m]", 
        #     "Drone_Lat_Pos [deg]", "Drone_Long_Pos [deg]", "Drone_Alt_Pos [m]", 
        #     "de_X_pos [m]", "dn_Y_pos [m]", "du_Z_pos [m]",
        #     # "drone_vel_if_X[m/s]", "drone_vel_if_Y[m/s]", "drone_vel_if_Z[m/s]",
        #     # "time_now", "past_time"
        #     ]


        # directory_csv = os.path.join(os.getcwd(), 'src/drone_control/gps_sub_data.csv')
        # # self.get_logger().info(f'Getting csv file path: {directory_csv}...')
        # if (self.iter_count == 0.0): #self.prev_ctrl_loop_timestamp is initialized at zero in the initialization function, therefore the first run of the script should have this value equal to zero
        #     self.get_logger().info(f'creating GPS csv file...')
        #     with open(directory_csv, 'w', newline='') as f_object:
        #         writer_object = writer(f_object)
        #         writer_object.writerow(csv_list_header)
        #         writer_object.writerow(csv_list)
        #         # Close the file object
        #         f_object.close()
        #         self.iter_count += 1
        #     self.get_logger().info(f'Finished first write to csv file...')
        # else:
        #     # Open our existing CSV file in append mode
        #     # Create a file object for this file
        #     with open(directory_csv, 'a') as f_object:
            
        #         # Pass this file object to csv.writer()
        #         # and get a writer object
        #         writer_object = writer(f_object)
            
        #         # Pass the list as an argument into
        #         # the writerow()
        #         writer_object.writerow(csv_list)
            
        #         # Close the file object
        #         f_object.close()
        #/////////////////////////////////////////////////////////////////////////////////////
        
        return response
        
    def gps_listener_callback(self, msg):
        
        #self.get_logger().info('Timestamp: "%s"' % msg.header.stamp)
        #self.get_logger().info('Altitude: "%s"' % msg.altitude)
        #self.get_logger().info('Longitude: "%s"' % msg.longitude)
        #self.get_logger().info('Latitude: "%s"' % msg.latitude)
        
        
        #take gps message data
        self.llh_drone_g = np.array([[msg.latitude],[msg.longitude],[msg.altitude]]) #[latitude (deg), longitude (deg), altitude (meters)]
        
        # time = self.get_clock().now().to_msg() #rclpy.time.Time() #self.get_clock().now().to_msg()
        # self.get_logger().info(f'Getting time now: {time}')
        # self.time_now = float(time.sec) + float(time.nanosec*10**(-9))
        # self.get_logger().info(f'Getting processed time now: {self.time_now}')
        

        

    def dllh2denu(self, llh_ref , llh_drone):
        # #Initialize drone position in WGS84 coordinates
        # ell_wgs84 = pymap3d.Ellipsoid('wgs84') #SET ellipsoid to WGS84 which is consistent with gazebo GPS plugin
        # de, dn, du = pymap3d.geodetic2enu(llh_drone[0], llh_drone[1], llh_drone[2], llh_ref[0], llh_ref[1],llh_ref[2], ell=ell_wgs84, deg=True)
        
        #////////////////////////////////////////////
        #Constants (function relevant only)
        
        a = 6378137 #meters
        b = 6356752.3142 #meters
        e2 = 1-(b/a)**2
          
        #convert reference frame coordinate to radians [except for altitude]
        phi_ref = (llh_ref[0,0]*np.pi)/180 #latitude
        lam_ref = (llh_ref[1,0]*np.pi)/180 #longitude
        alt_ref = llh_ref[2,0] #altitude
        # Convert current location of drone from degree to radians [except for altitude]. 
        # Then take differential distance from reference
        dphi = (llh_drone[0,0]*np.pi)/180 - phi_ref #latitude
        dlam = (llh_drone[1,0]*np.pi)/180 - lam_ref #longitude
        dalt = llh_drone[2,0] - alt_ref #altitude
        #pre-process some calculations
        tmp1 = np.sqrt(1- e2*np.sin(phi_ref)**2)

        cp = np.cos(phi_ref)
        sp = np.sin(phi_ref)
        #transformation from WGS84 to ENU coordinates from the reference
        #East coordinate NOTE: Values appear to be the opposite sign, so -1 is multiplied by the equation. The results are consistent when testing different spawn points
        de = -1*(a/tmp1+alt_ref)*cp*dlam - (a*(1-e2)/(tmp1**3)+alt_ref)*sp*dphi*dlam + cp*dlam*dalt
        #North coordinate NOTE: Values appear to be the opposite sign, so -1 is multiplied by the equation. The results are consistent when testing different spawn points
        dn = -1*(a*(1-e2)/tmp1**3 + alt_ref)*dphi + 1.5*cp*sp*a*e2*dphi**2 + sp**2 * dalt * dphi + 0.5*sp*cp*(a/tmp1 - alt_ref)*dlam**2
        #Up coordinate
        du = dalt - 0.5*(a - 1.5*a*e2*cp**2 + 0.5*a*e2 + alt_ref)*dphi**2 - 0.5*cp**2 *(a/tmp1 - alt_ref)*dlam**2

        denu = np.array([de,dn,du])


        
        return denu

    # def call_sim_clock_service(self):

    #     self.sim_clock_future = self.sim_clock_client.call_async(self.sim_clock_req)
    #     rclpy.spin_until_future_complete(self.sim_clock_sub_node, self.sim_clock_future)
    #     return self.sim_clock_future.result()
    

def main(args=None):
    rclpy.init(args=args)

    gps_subscriber = GPS_Subscriber()

    rclpy.spin(gps_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()