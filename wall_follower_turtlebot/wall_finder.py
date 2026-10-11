import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from lidar_utils.laser_utils import get_index_from_angle
from custom_interfaces.srv import FindWall
import time

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[1;32m"
YELLOW = "\033[93m"
NC = "\033[0m"
CYAN = "\033[96m"
MAGENTA = "\033[95m"
LIME    = "\033[38;5;118m"
ORANGE = "\033[38;5;208m"
TEAL    = "\033[38;5;44m"


class WallFinderNode(Node):
    def __init__(self):
        super().__init__('find_wall')
        
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        name_service = '/find_wall'
        self.srv = self.create_service(FindWall, name_service, self.find_wall_callback)
        
        # --- Variables ----
        self.mode = 'FIND_NEAREST_WALL'
        self.target_wall = None
        self.find_wall_done = False
        self.find_wall_active = False
        self.indices_initialized = False

        # ---Logging helpers ---
        self.previous_mode = None
        self.last_log = None

        self.get_logger().info(name_service + 'Service Server Ready ....')
        
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
    
        # Initialize indices ONCE(not again and again, this reduced resource usage.)
        if not self.indices_initialized:
            self.initialize_indices()
        
        sector_distances = self.detect_obstacles_by_sector()
        
        # start finding wall once the find wall service is called. 
        if self.find_wall_active:   
            self.find_nearest_wall(sector_distances)

    def initialize_indices(self):
        
        # Define boundaries for symmetry.(For ensuring proper Wall detection alignment.)
        self.front_left_idx = get_index_from_angle(0.262, self.scan_angle_min, self.scan_angle_max, self.scan_angle_increment, self.number_of_ranges)
        self.front_right_idx = get_index_from_angle(-0.262, self.scan_angle_min, self.scan_angle_max, self.scan_angle_increment, self.number_of_ranges)
        
        self.left_start_idx = get_index_from_angle(1.309, self.scan_angle_min, self.scan_angle_max, self.scan_angle_increment, self.number_of_ranges)
        self.left_end_idx = get_index_from_angle(1.833, self.scan_angle_min, self.scan_angle_max, self.scan_angle_increment, self.number_of_ranges)

        self.right_start_idx = get_index_from_angle(4.451, self.scan_angle_min, self.scan_angle_max, self.scan_angle_increment, self.number_of_ranges)
        self.right_end_idx = get_index_from_angle(4.974, self.scan_angle_min, self.scan_angle_max, self.scan_angle_increment, self.number_of_ranges)

        # Right side rays for wall alignment
        self.rf_idx = get_index_from_angle(5.236,self.scan_angle_min,self.scan_angle_max,self.scan_angle_increment,self.number_of_ranges)
        self.rr_idx = get_index_from_angle(4.189,self.scan_angle_min,self.scan_angle_max,self.scan_angle_increment,self.number_of_ranges)
        
        self.indices_initialized = True

    # Logging Helper fucntion to help reduce log spam 
    def log_once(self, key, message):

        if self.last_log != key:
            self.get_logger().info(message)
            self.last_log = key
    
    # SECTOR DETECTION
    def detect_obstacles_by_sector(self):

        """
        Detect minimum obstacle distance in each predefined angular sector.
        """
        
        SECTORS = {

            "Front_Left":       (0.0, 0.262),   # approx 0 to 15 degrees   
            "Front_Right":      (-0.262, 0.0),    # approx -15 to 0 degrees
            "Left":             (1.309, 1.833),   # 75° to 105°
            #"Rear":             (2.879, 3.404),   # 165° to 195°           
            "Right":             (4.451, 4.974)   # approx 255 to 285 degrees           
        }

        sector_distances = {}

        
        for sector_name, (angle_start, angle_end) in SECTORS.items():
            start_idx = get_index_from_angle(angle_start, self.scan_angle_min, self.scan_angle_max, self.scan_angle_increment, self.number_of_ranges)
            end_idx = get_index_from_angle(angle_end, self.scan_angle_min, self.scan_angle_max, self.scan_angle_increment, self.number_of_ranges)

            if start_idx is None or end_idx is None:
                sector_distances[sector_name] = None
                continue
                   
            
            if start_idx < self.number_of_ranges and end_idx < self.number_of_ranges:
                # Correct wrap-aware slicing
                if start_idx <= end_idx:
                    # Normal case (no wrap)
                    sector_ranges = self.scan_ranges[start_idx:end_idx + 1]
                else:
                    sector_ranges = (
                        self.scan_ranges[start_idx:] +
                        self.scan_ranges[:end_idx + 1]
                    )

                valid_ranges = [
                    r for r in sector_ranges         
                    if self.scan_range_min <= r <= self.scan_range_max
                ]

                if valid_ranges:
                    sector_distances[sector_name] = min(valid_ranges)
                else:
                    (f"{RED}Can't find any Obstacles, within the LIDAR range{NC}")
                    sector_distances[sector_name] = float('inf')

        return sector_distances
    
    # Service server callback.
    def find_wall_callback(self, request, response):
            
        # START wall finding if not already active ..
        if not self.find_wall_active:
            self.get_logger().info(f"{TEAL}FindWall request received → starting wall alignment{NC}")
            self.find_wall_active = True

        # Check if wall was found
        if self.find_wall_done:
            response.wallfound = True
        else:
            response.wallfound = False
        
        return response
    
    def find_nearest_wall(self, sector_distances):
        """
        Rotate until the sector with minimum distance becomes the front.
        """

        if self.find_wall_done:     # switch to detect when wall is found. If found, stop the function.
            return
                            
        # Variables
        action = Twist()
        SYMM_TOL  = 0.03
        TARGET_FRONT_DIST = 0.3
        TURN_SPEED = 0.2
        PARALLEL_TOL = 0.04   
        
        # Sector distances(for finding nearest wall)
        front_left  = sector_distances.get('Front_Left', float('inf'))
        front_right = sector_distances.get('Front_Right', float('inf'))
        left        = sector_distances.get('Left', float('inf'))
        right       = sector_distances.get('Right', float('inf'))
        #rear       = sector_distances.get('Rear', float('inf'))
        
        front = min(front_left, front_right)
        
        # Symmetry conditions(for wall detection)
        front_symmetry_error = abs(self.scan_ranges[self.front_left_idx] - self.scan_ranges[self.front_right_idx])
        left_symmetry_error = abs(self.scan_ranges[self.left_start_idx] - self.scan_ranges[self.left_end_idx])
        right_symmetry_error = abs(self.scan_ranges[self.right_start_idx] - self.scan_ranges[self.right_end_idx])
        #rear_symmetry_error = abs(self.scan_ranges[self.rear_start_idx] - self.scan_ranges[self.rear_end_idx])

        # Side : distance mapping
        side_distances = {
            "Front": front,
            "Left":  left,
            #"Rear":  rear,
            "Right": right
        }

        nearest_side = min(side_distances, key=side_distances.get)  # side with minimum value i.e Front, Left, Right
        minimum = side_distances[nearest_side]    # the actual minimum value of the corresponding side.

    # debugger ...
        #self.get_logger().info(f"Front={front:.2f}, "f"Left={left:.2f}, Right={right:.2f}, Rear={rear:.2f}, ")
        #self.get_logger().info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        #self.get_logger().info(f"Front_Left={front_left:.2f}, Front_Right={front_right:.2f}, ")

        if self.mode == 'FIND_NEAREST_WALL':
            
            if nearest_side == 'Left' and left_symmetry_error < SYMM_TOL:  # check which side is giving minimum value and is symmetric ...
                self.get_logger().info(f"{GREEN}Detected nearest wall on {nearest_side} side → starting alignment{NC}")
                self.target_wall = 'Left'
                self.mode = 'ALIGN_TO_WALL'
                
            elif nearest_side == 'Right' and right_symmetry_error < SYMM_TOL:
                self.get_logger().info(f"{GREEN}Detected nearest wall on {nearest_side} side → starting alignment{NC}")
                self.target_wall = 'Right'
                self.mode = 'ALIGN_TO_WALL'
                
            # elif nearest_side == 'Rear' and rear_symmetry_error < SYMM_TOL:
            #     self.get_logger().info(f"{RED}Detected nearest wall on {nearest_side} side → starting alignment{NC}")
            #     self.target_wall = 'Rear'
            #     self.mode = 'ALIGN_TO_WALL'
                
            elif nearest_side == 'Front':
                self.get_logger().info(f"{GREEN}Detected nearest wall on {nearest_side} side → starting alignment{NC}")
                self.mode = 'ALIGN_TO_WALL'
                self.target_wall = 'Front'     

            else:
                # Keep searching until wall at some side is detected.
                self.get_logger().info(f"{ORANGE}Searching for Nearest Wall{NC}")
                action.angular.z = TURN_SPEED
                
        elif self.mode == 'ALIGN_TO_WALL':
            
            self.log_once("ALIGN TO WALL", f"{ORANGE}Aligning to wall on {self.target_wall} side{NC}") 

            # STEP 1: Rotate until target wall is in FRONT 
            if front > minimum:          # if robot is not facing front but other side, then rotate until robot's front is aligned with wall
                if self.target_wall == 'Left':         # if wall is on left side, turn left
                    action.angular.z = TURN_SPEED  
                                                
                elif self.target_wall == 'Right':
                        action.angular.z = -TURN_SPEED

                # elif self.target_wall == 'Rear':
                #     action.angular.z = TURN_SPEED

                elif self.target_wall == 'Front':
                    # Already facing wall
                    pass
                
            else:         # if robot is already facing front
                 
                self.log_once("CORRECTING_SYMMETRY", f"{ORANGE}Wall in front, fine-tuning symmetry!{NC}")
                
                if front_symmetry_error > SYMM_TOL:
                    if front_left > front_right:
                        action.angular.z = -TURN_SPEED  
                    else:
                        action.angular.z = TURN_SPEED
                
                else:
                    self.get_logger().info("Successfully faced Wall")
                    self.mode = 'MOVE_WALL'

        # Move towards the wall if front distance is more than TARGET_FRONT_DIST
        elif self.mode == 'MOVE_WALL':

            self.log_once("MOVE WALL", f"{ORANGE}Approaching wall Now!{NC}")
            action.linear.x = 0.08
            action.angular.z = 0.0
            if front < TARGET_FRONT_DIST:
                (f"{GREEN}Reached target distance from wall{NC}")
                action.linear.x = 0.0
                self.mode = 'ALIGN_RIGHT'

        elif self.mode == 'ALIGN_RIGHT':

            self.log_once("ALIGN RIGHT", f"{LIME}Aligning right side with wall{NC}")

            rf = self.scan_ranges[self.rf_idx]
            rr = self.scan_ranges[self.rr_idx]

            parallel_error = rf - rr

            # DEBUGGER ..
            #self.get_logger().info(f"{LIME}Parallel Error: {parallel_error:.3f}{NC}")

            if abs(parallel_error) < PARALLEL_TOL:
                action.angular.z = 0.0
                self.get_logger().info(f"{GREEN}Aligned Successfully, Ready to start Wall Following{NC}")
                self.mode = 'DONE'
            
            else:
            
                action.angular.z = TURN_SPEED

        elif self.mode == 'DONE':
            
            action.linear.x = 0.0
            action.angular.z = 0.0
            self.find_wall_done = True    # It's like a switch.
            self.find_wall_active = False
        
        # Detect State Changes(to reduce too many logs..)
        if self.mode != self.previous_mode:
            self.get_logger().info(f"{CYAN}State changed from {self.previous_mode} to {self.mode}{NC}")
            self.previous_mode = self.mode
        
        # PUBLISH Finally
        self.publisher.publish(action)
        return
    
def main(args=None):
    rclpy.init(args=args)
    node = WallFinderNode()
    
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
