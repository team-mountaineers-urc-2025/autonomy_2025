import rclpy
import math
import time
from rclpy.node import Node
from autonomy_2025.autonomy_targets import AutonomyTargets as Targets
from autonomy_2025.decision_status import DecisionStatus as Status

from std_msgs.msg import String, Float32, Float64, Int32, Empty, Bool
from robot_interfaces.msg import Pose2DArray, ObjectPoint
from geometry_msgs.msg import Pose2D, Vector3
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

EARTH_RADIUS_METERS = 6_378_137

class Brian(Node):

    def __init__(self):
        super().__init__('brian')

        self.declare_parameter("danger_timeout", 2.0)
        self.declare_parameter("roll_threshold", 30)
        self.declare_parameter("pitch_threshold", 60)
        self.declare_parameter('stopping_threshold', 1.5) 

        self.danger_timeout = self.get_parameter("danger_timeout").get_parameter_value().double_value
        self.roll_threshold = self.get_parameter("roll_threshold").get_parameter_value().double_value
        self.pitch_threshold = self.get_parameter("pitch_threshold").get_parameter_value().double_value
        self.stopping_threshold = self.get_parameter("stopping_threshold").get_parameter_value().double_value

        # QOS Profiles
        self.mavros_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        ) 

        # Publishers
        self.status_pub = self.create_publisher(Int32, "decision_status", 10)
        self.local_pub  = self.create_publisher(Pose2D, "next_local", 10)
        self.led_pub    = self.create_publisher(Bool, 'goal_alert', 10)
        self.speed_pub  = self.create_publisher(Float32, "/drivetrain/speed_multiplier", 10)

        # Subscriptions
        self.next_point_sub     = self.create_subscription(Pose2D, "next_waypoint", self.next_point_callback, 10)
        self.seen_objects_sub   = self.create_subscription(ObjectPoint, "object_locale", self.seen_objects_callback, 10)
        self.orientation_sub    = self.create_subscription(Vector3, "/health_monitor/chassis_orientation", self.danger_callback, qos_profile_sensor_data)
        self.current_gps_sub    = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.update_location_callback, self.mavros_qos_profile)

        self.current_pose = Pose2D()
        self.seen_objects = []

        self.pitch_value    = 0.0
        self.last_pitch     = 0.0
        self.roll_value     = 0.0
        self.last_roll      = 0.0

        self.speed_pub.publish(Float32(data=0.8))

    def seen_objects_callback(self, msg):
        #Check if each object matches the object id we are looking for
        for object in self.seen_objects:
            #If we find an identical object already in the list
            if(object.object_id == msg.object_id):
                #update the list
                self.seen_objects.remove(object)
                self.seen_objects.append(msg)
                return
            
        #Add the object to the list if not found
        self.seen_objects.append(msg)

    def update_location_callback(self, msg: NavSatFix):
        self.current_pose.x = msg.latitude
        self.current_pose.y = msg.longitude

    def gps_analysis(self, x, y, type):

        # Check the distance between where we are and where we want to go
        goal_pose = Pose2D(x=x, y=y)
        dist = abs(self.distanceGPS2d(goal_pose, self.current_pose))
        theta = self.gps_To_Ang(self.current_pose, goal_pose)
        
        local_x = dist * math.cos(theta)
        local_y = dist * math.sin(theta)

        if dist < self.stopping_threshold:

            if type == Targets.GPS.value:
                # Flash LEDS
                self.signal_arrival()

            else:
                self.get_logger().info("next point plz")
                self.status_pub.publish(Int32(data=Status.NEXT_POINT.value))
                self.seen_objects = []

                # Stop moving
                self.local_pub.publish(Pose2D(x=0.0, y=0.0))
    
        else:
            # Send the position to the local path planner
            self.local_pub.publish(Pose2D(x=local_x, y=local_y))
    
    def object_analysis(self, latitude, longitude, object):
        
        # check the distance between where we are and where we want to go
        local_x = 0.0
        local_y = 0.0
        object_seen = False

        match object:

            case Targets.ARUCO_0.value:

                #Check if Aruco 0 has been seen
                for obj in self.seen_objects:
                    #If we find an identical object already in the list
                    # self.get_logger().info(f'Objet ID: {obj.object_id}  vs Target ID: {Targets.ARUCO_1.value}')
                    if(obj.object_id == Targets.ARUCO_0.value):
                        #update the local x and y
                        local_x = float(obj.point.point.x)
                        local_y = float(obj.point.point.y)
                        
                        object_seen = True

                        pass

                pass

            case Targets.ARUCO_1.value:

                 #Check if Aruco 1 has been seen
                for obj in self.seen_objects:
                    #If we find an identical object already in the list
                    if(obj.object_id == Targets.ARUCO_1.value):
                        #update the local x and y
                        local_x = float(obj.point.point.x)
                        local_y = float(obj.point.point.y)
                        
                        object_seen = True

                        pass
              
                pass

            case Targets.ARUCO_2.value:

                #Check if Aruco 2 has been seen
                for obj in self.seen_objects:
                    #If we find an identical object already in the list
                    if(obj.object_id == Targets.ARUCO_2.value):
                        #update the local x and y
                        local_x = float(obj.point.point.x)
                        local_y = float(obj.point.point.y)
                        
                        object_seen = True

                        pass
                
                pass

            case Targets.ARUCO_3.value:

                #Check if Aruco 3 has been seen
                for obj in self.seen_objects:
                    #If we find an identical object already in the list
                    if(obj.object_id == Targets.ARUCO_3.value):
                        #update the local x and y
                        local_x = float(obj.point.point.x)
                        local_y = float(obj.point.point.y)
                        
                        object_seen = True

                        pass

                pass
        
            case Targets.BOTTLE.value:

                #Check if Bottle has been seen
                for obj in self.seen_objects:
                    #If we find an identical object already in the list
                    if(obj.object_id == Targets.BOTTLE.value):
                        #update the local x and y
                        local_x = float(obj.point.point.x)
                        local_y = float(obj.point.point.y)
                        
                        object_seen = True

                        pass

                pass
        
            case Targets.HAMMER.value:

                 #Check if Hammer has been seen
                for obj in self.seen_objects:
                    #If we find an identical object already in the list
                    if(obj.object_id == Targets.HAMMER.value):
                        #update the local x and y
                        local_x = float(obj.point.point.x)
                        local_y = float(obj.point.point.y)
                        
                        object_seen = True

                        pass
                pass

            case _:
                self.get_logger().warn("Incorrect Object ID")
                object_seen = False

        # If we haven't seen anything yet, pretend it is a GPS point
        if not object_seen:
            self.gps_analysis(latitude, longitude, Targets.WAYPOINT.value)
            self.get_logger().warn("NO OBJECT DETECTED")

        # If we have seen something check the distance
        elif math.sqrt(local_x**2 + local_y**2) > self.stopping_threshold:
            self.get_logger().info(f"OBJECT DETECT: X:{local_x}, Y:{local_y}")
            self.local_pub.publish(Pose2D(x=local_x, y=local_y))
            self.speed_pub.publish(Float32(data=0.3))


        else:
            self.signal_arrival()

        return

    def signal_arrival(self):
        # self.get_logger().warn("ARRIVED AT LOCATION")

        # Flash LEDS
        self.led_pub.publish(Bool(data=True))

        # Publish Arrival to Waypoint Manager
        self.status_pub.publish(Int32(data=Status.ARRIVED.value))
        self.seen_objects = []

        # Stop moving
        self.local_pub.publish(Pose2D(x=0.0, y=0.0))
        self.speed_pub.publish(Float32(data=0.8))

    def danger_callback(self, msg : Vector3):
        self.roll_value  = msg.x
        self.pitch_value = msg.y

        uncorrected_theta = msg.z
        corrected_theta = (uncorrected_theta - math.pi/2.0)

        while corrected_theta > math.pi:
            corrected_theta -= 2*math.pi

        while corrected_theta < -math.pi:
            corrected_theta += 2*math.pi

        self.current_pose.theta = corrected_theta

        self.last_pitch = self.last_roll = time.time()

    def next_point_callback(self, msg : Pose2D):

        self.gnss_lat       = msg.x
        self.gnss_long      = msg.y
        self.search_object  = int(msg.theta)

        # Are we stopped or not?
        if self.search_object == Targets.STOP.value:
            self.local_pub.publish(Pose2D())
            return
        
        elif self.search_object == Targets.EMPTY.value:
            self.signal_arrival()
            return
        
        # If we aren't stopped, then make sure the lights are red (we're moving)
        self.led_pub.publish(Bool(data=False))

        # Check to see if the place we are is good
        curr_time = time.time()

        # If we aren't looking for anything, go to the next point
        if self.search_object == Targets.GPS.value or self.search_object == Targets.WAYPOINT.value:
            self.gps_analysis(self.gnss_lat, self.gnss_long, self.search_object)

        # If we are looking for something, check if we have seen ig
        else:
            self.object_analysis(self.gnss_lat, self.gnss_long, self.search_object)

    
    # Get the distance between two GPS points
    def distanceGPS2d(self, pose1 : Pose2D, pose2 : Pose2D):
        
        phi1 = math.radians(pose1.x)
        phi2 = math.radians(pose2.x)
        delta_phi = math.radians(pose2.x - pose1.x)
        delta_lambda = math.radians(pose2.y - pose1.y)

        a = math.sin(delta_phi / 2.0) ** 2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda / 2.0) ** 2
        
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return EARTH_RADIUS_METERS * c     
            
    def gps_To_Ang(self, current: Pose2D, goal: Pose2D):

        #Creating a gps cooridinate with the same y as current to obtain the distance purely in the x direction
        modified_y_goal = Pose2D()
        modified_y_goal.x = goal.x
        modified_y_goal.y = current.y

        delta_x = self.distanceGPS2d(current, modified_y_goal)

        #Delta x will always be positive initially. To get an accurate angle I need to determine whether the goal is north (delta x is positive) or south (negative) of the current location
        if current.x > goal.x:
            delta_x = -delta_x

        #Creating a gps with the same x as current to obtain the distance purely in the y direction
        modified_x_goal = Pose2D()
        modified_x_goal.y = goal.y
        modified_x_goal.x = current.x

        delta_y = self.distanceGPS2d(current, modified_x_goal)

        #Assuming positive x is north, then positive y would be west, however, longitude is the opposite of that, hence the current < goal for negative
        if current.y < goal.y: 
            delta_y = -delta_y

        theta_coords = math.atan2(delta_y, delta_x)

        #this is from the pixhawk and it's given in degrees
        theta_orientation = current.theta

        # The angle of the point in Heimdall's frame relative to the x axis and positive being towards positive y
        theta_local = theta_coords - theta_orientation

        while(theta_local >math.pi):
            theta_local -= 2*math.pi

        while(theta_local < -math.pi):
            theta_local += 2*math.pi

        return theta_local #in radians

def main(args=None):
    rclpy.init(args=args)

    brian = Brian()

    rclpy.spin(brian)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    brian.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
