import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from lidar_utils.laser_utils import get_index_from_angle
from custom_interfaces.srv import FindWall
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from custom_interfaces.action import OdomRecord

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[1;32m"
YELLOW  = "\033[93m"
NC = "\033[0m"
CYAN = "\033[96m"
MAGENTA = "\033[95m"
ORANGE = "\033[38;5;208m"
GOLD    = "\033[38;5;220m"

# Service Client .......
class FindWallClient(Node):
    def __init__(self):
        super().__init__('find_wall_client')
        
        # Create client for calling FindWall service 
        self.find_wall_client = self.create_client(FindWall, '/find_wall')
        
        # Wait for the service to be available (checks every second)
        while not self.find_wall_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /find_wall...')
        
        # Create a request
        self.request = FindWall.Request()
        self.future = None

    def send_request(self):
            self.future = self.find_wall_client.call_async(self.request)

# Action client ...
class RecordOdomClient(Node):
    def __init__(self):
        super().__init__('record_odom_client')

        self.action_client = ActionClient(
            self,
            OdomRecord,
            '/record_odom'
        )

        self.get_logger().info("Waiting for /record_odom action server...")
        self.action_client.wait_for_server()   # "Wait until action server becomes available."

        self.goal_future = None

    def send_goal(self):
        goal_msg = OdomRecord.Goal()

        self.goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self.goal_future.add_done_callback(self.goal_response_callback)

    # ---------- ACTION CALLBACKS ----------

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Record odom goal rejected")
            return

        self.get_logger().info(f"{GREEN}Odom recording started.{NC}")

        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"{CYAN}[ODOM FEEDBACK] Distance: {feedback.current_total:.2f} m{NC}"
        )

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f"{GREEN}Odom recording finished. Points: {result.list_of_odoms}{NC}"
        )
        
# Main class ... 
class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---Logging helpers ---
        
        self.previous_state = None
        self.state = 'WALL_FOLLOW'
        self.motion = "MOVING_STRAIGHT"
        self.previous_motion = None
        
        self.get_logger().info('Wall Follower Node Ready ....') 

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

    # Detect minimum obstacle distance in each predefined angular sector ...
    def detect_obstacles_by_sector(self):

        """
        Detect minimum obstacle distance in each predefined angular sector.

        Returns:
            dict: sector_name -> min distance (float) or None
        """

        # Define sector angles in radians, this will be dynamic based on the robot's laser scanner configuration., you have to check the FOV using sector_utils.py
        # Using angles instead of indices for better adaptability. 
       
        SECTORS = {

            "Front":       (-0.35, 0.35),   # approx -20 to 20 degrees
            "Right":       (4.451, 4.974),   # approx 255 to 285 degrees    
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
                    r for r in sector_ranges         # return r if r falls in between scan_range_min to scan_range max values.
                    if self.scan_range_min <= r <= self.scan_range_max
                ]

                if valid_ranges:
                    sector_distances[sector_name] = min(valid_ranges)
                else:
                    self.get_logger().info(f"{RED}Can't find any Obstacles, within the LIDAR range{NC}")
                    sector_distances[sector_name] = float('inf')

        return sector_distances

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

        RIGHT_MIN = 0.20
        RIGHT_MAX = 0.29

        FORWARD_SPEED = 0.10
        WALL_TURN_SPEED = 0.2

        # PROPOTIONAL CONTROLLER VARIABLES FOR TURNING
        KP_FRONT = 2.0

        # STEP 1 : ---- DEFINE STATE TRANSITIONS ----

        # If currently NOT avoiding front wall
        if self.state != 'AVOID_FRONT_WALL':    

            # Enter front avoidance mode
            if front < FRONT_ENTER:
                self.state = 'AVOID_FRONT_WALL'

        # If already avoiding front wall
        else:

            # Exit front avoidance mode
            if front > FRONT_EXIT:
                self.state = 'WALL_FOLLOW'
 
        # STEP 2 : ---- EXECUTE STATE TRANSITIONS ----
        
        # IF STATE: ==== AVOID_FRONT_WALL ====

        if self.state == 'AVOID_FRONT_WALL':

            # Move slowly while turning
            action.linear.x = FORWARD_SPEED * 0.7

            # Proportional turning
            front_error = FRONT_EXIT - front
            action.angular.z = KP_FRONT * front_error
            
            # debug
            #self.get_logger().warn(f"AVOID_FRONT_WALL → front={front:.2f}")

        # IF STATE: ==== WALL_FOLLOW ====

        elif self.state == 'WALL_FOLLOW':

            action.linear.x = FORWARD_SPEED

            # ---------- WALL FOLLOW BEHAVIOR ----------
            action.linear.x = FORWARD_SPEED  # Keep moving forward

            if right >= RIGHT_MAX:
                action.angular.z = -WALL_TURN_SPEED
                self.motion = "MOVE TOWARDS THE WALL"

            elif right <= RIGHT_MIN:
                action.angular.z = WALL_TURN_SPEED
                self.motion = "MOVE AWAY FROM WALL"

            else:
                action.angular.z = 0.0
                self.motion = "MOVING_STRAIGHT"
                
        # ---------- MOTION CHANGE DETECTION ----------
        if self.motion != self.previous_motion:
            self.get_logger().info(f"{RED}Motion changed from {self.previous_motion} to {self.motion}{NC}\n")
            self.previous_motion = self.motion
    
        # ---------- STATE CHANGE DETECTION ----------
        if self.state != self.previous_state:
            self.get_logger().info(f"{ORANGE}State changed from {self.previous_state} to {self.state}{NC}\n")
            self.previous_state = self.state

        self.publisher.publish(action)
        return   

def main(args=None):

    rclpy.init(args=args)

    # ---- PHASE 1: FIND WALL(Service client) ----

    find_wall_client = FindWallClient()
    find_wall_client.send_request()

    start_time = time.time()
    timeout = 30.0

    # ---Logging helpers ---
    waiting_logged = False

    while rclpy.ok():

        rclpy.spin_once(find_wall_client)

        # ----- IF CLIENT GETS A RESPONSE -----
        if find_wall_client.future.done():

            try:
                response = find_wall_client.future.result()
            except Exception as e:
                find_wall_client.get_logger().info(f'{RED}Service call failed: {e}{NC}')
              
            # If wall is found ...
            if response.wallfound:
                find_wall_client.get_logger().info(f"{GREEN}Wall found successfully.{NC}")
                waiting_logged = False
                break
            else:

                if not waiting_logged:

                    find_wall_client.get_logger().info(f"{GOLD}Waiting for alignment...{NC}")
                    waiting_logged = True

                time.sleep(0.5)
                find_wall_client.send_request()

        # Stop if timeout is reached i.e wall isn't found withon 30 seconds
        if time.time() - start_time > timeout:

            find_wall_client.get_logger().error("Timeout while finding wall.")
            find_wall_client.destroy_node()
            rclpy.shutdown()
            return

    # Destroy find wall client as it is no longer needed once wall is found
    find_wall_client.destroy_node()

    # ---- PHASE 2: START ODOM RECORDING (ACTION CLIENT) ----

    odom_client = RecordOdomClient()
    odom_client.send_goal()

    # ---- PHASE 3: START WALL FOLLOWER ----

    node = WallFollowerNode()

    # Create a MultiThreadedExecutor to independently run the nodes and avoid blockage.
    executor = MultiThreadedExecutor()

    executor.add_node(node)
    executor.add_node(odom_client)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        odom_client.destroy_node()
        if rclpy.ok():
            executor.shutdown()
            rclpy.shutdown()
        
if __name__ == '__main__':
    main()
