import rclpy
import math
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from lidar_utils.laser_utils import get_index_from_angle

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[1;32m"
YELLOW = "\033[93m"
NC = "\033[0m"

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Wall Follower Node Ready ....')

        # Default state ...
        self.state = 'WALL_FOLLOW'
        
    # laserscan_callback to process incoming LaserScan messages ...
    
    def laserscan_callback(self, msg):
        
        # Store LaserScan data for processing ...
        self.scan_ranges = msg.ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_max = msg.angle_max
        self.scan_angle_increment = msg.angle_increment
        self.scan_range_min = msg.range_min
        self.scan_range_max = msg.range_max
        self.number_of_ranges = len(msg.ranges)

        # Start calling functions now ...
        sector_distances = self.detect_obstacles_by_sector()
        self.obstacle_avoidance(sector_distances)
    
    def detect_obstacles_by_sector(self):

        """
        Detect minimum obstacle distance in each predefined angular sector.

        Returns:
            dict: sector_name -> min distance (float) or None
        """

        # Define sector angles in radians, this will be dynamic based on the robot's laser scanner configuration., you have to check the FOV using sector_utils.py
        # Using angles instead of indices for better adaptability. 
       
        SECTORS = {

            "Front":       (-0.35, 0.35),    # approx -10 to 10 degrees
            "Right":       (4.451, 4.974)   # approx 255 to 285 degrees            
        }

        sector_distances = {}

        # for each sector-one at a time(whole for loop for each dictionary key-word pair), get the respective min and max indices ...
        for sector_name, (angle_start, angle_end) in SECTORS.items():
            start_idx = get_index_from_angle(angle_start, self.scan_angle_min, self.scan_angle_max, self.scan_angle_increment, self.number_of_ranges)
            end_idx = get_index_from_angle(angle_end, self.scan_angle_min, self.scan_angle_max, self.scan_angle_increment, self.number_of_ranges)

            if start_idx is None or end_idx is None:
                sector_distances[sector_name] = None
                continue
                   
            
            if start_idx < self.number_of_ranges and end_idx < self.number_of_ranges:
                # ✅ Correct wrap-aware slicing
                if start_idx <= end_idx:
                    # Normal case (no wrap)
                    sector_ranges = self.scan_ranges[start_idx:end_idx + 1]
                else:
                    sector_ranges = (
                        self.scan_ranges[start_idx:] +
                        self.scan_ranges[:end_idx + 1]
                    )

                valid_ranges = [
                    r for r in sector_ranges         # return r if r falls in between scan_range_min to scan_range max values.
                    if self.scan_range_min <= r <= self.scan_range_max
                ]

                if valid_ranges:
                    sector_distances[sector_name] = min(valid_ranges)
                else:
                    print(f"{RED}Can't find any Obstacles, within the LIDAR range{NC}")
                    sector_distances[sector_name] = float('inf')

        return sector_distances

    # Approach: Obstacle Avoidance logic using FSM + Hystersis + Propotional Controller...
    def obstacle_avoidance(self, sector_distances):

        action = Twist()

        front = sector_distances.get("Front", float('inf'))
        right = sector_distances.get("Right",  float('inf'))

        # Use for debugging ...
        #self.get_logger().info(f"Front distance: {front:.3f}")
        #self.get_logger().info(f"Left distance: {left:.3f}")
    
        # Distance thresholds
        FRONT_ENTER = 0.50   # enter front-avoid
        FRONT_EXIT  = 0.55   # exit front-avoid (hysteresis)
        FRONT_TURN_SPEED = 0.5

        RIGHT_MIN = 0.20
        RIGHT_MAX = 0.29

        FORWARD_SPEED = 0.10
        WALL_TURN_SPEED = 0.2


        # STEP 1 : ---- DEFINE STATE TRANSITIONS ----

        # If currently NOT avoiding front wall
        if self.state != 'AVOID_FRONT_WALL':    

            # Enter front avoidance mode
            if front < FRONT_ENTER:

                self.state = 'AVOID_FRONT_WALL'
                self.get_logger().warn("FRONT obstacle detected → switching to AVOID_FRONT_WALL")

        # If already avoiding front wall
        else:

            # Exit front avoidance mode
            if front > FRONT_EXIT:

                self.state = 'WALL_FOLLOW'
                self.get_logger().info("FRONT clear → switching to WALL_FOLLOW")

        
        # STEP 2 : ---- EXECUTE STATE TRANSITIONS ----
        
        # IF STATE: ==== AVOID_FRONT_WALL ====

        if self.state == 'AVOID_FRONT_WALL':

            # Move slowly while turning
            action.linear.x = FORWARD_SPEED * 0.7
            action.angular.z = FRONT_TURN_SPEED

            self.get_logger().warn(f"AVOID_FRONT_WALL → front={front:.2f}")

        # IF STATE: ==== WALL_FOLLOW ====

        elif self.state == 'WALL_FOLLOW':

            action.linear.x = FORWARD_SPEED

            # ---------- WALL FOLLOW BEHAVIOR ----------
            action.linear.x = FORWARD_SPEED  # Keep moving forward

            if right >= RIGHT_MAX:
                action.angular.z = -WALL_TURN_SPEED
                self.get_logger().info(" Too far from wall → turn RIGHT")

            elif right <= RIGHT_MIN:
                action.angular.z = WALL_TURN_SPEED
                self.get_logger().info(" Too close to wall → turn LEFT")

            else:
                action.angular.z = 0.0
                self.get_logger().info(" Wall aligned → moving straight")

        self.publisher.publish(action)
        return

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()