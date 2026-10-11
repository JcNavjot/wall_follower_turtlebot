import rclpy
import math
import tf_transformations
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.action import ActionServer
from custom_interfaces.action import OdomRecord

class RecordOdometryNode(Node):
    def __init__(self):
        super().__init__('Record_odometry_node')
        
        self.subscriber_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            )
        
        self.action_server = ActionServer(self, OdomRecord, '/record_odom', self.execute_callback)
        
        # Variables
        self.first_odom = Point()
        self.last_odom = Point()
        self.odom_record = []
        self.total_distance = 0.0
        self.recording_active = False
        self.record_timer = None
        self.last_x = 0.0
        self.last_y = 0.0

        self.get_logger().info("Record Odometry Node has been started.")

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)
        
        self.last_odom.x = x
        self.last_odom.y = y

    # This one will be executed when action server is called. It will be executed only once per call.
    def execute_callback(self, goal_handle):
        
        self.get_logger().info("Received request to record odometry.")

        if self.recording_active:
            self.get_logger().warn("Already recording. Rejecting new goal.")
            goal_handle.abort()
            return OdomRecord.Result()

        self.goal_handle = goal_handle
        
        # Reset variables for new recording session
        self.odom_record = []      # reset list
        self.total_distance = 0.0  # reset counters
        
        # Initialize first odometry data, it will be updated in the odom_callback when the first message is received and will not change.
        self.first_odom.x = self.last_odom.x
        self.first_odom.y = self.last_odom.y

        # Initialize last position's for distance calculation, this would avoid incorrect distance calculation/ direct jump from 0 in the first step of recording. The last_odom will be updated in the odom_callback and we want to start distance calculation from the first recorded position.
        self.last_x = self.last_odom.x
        self.last_y = self.last_odom.y

        # Set recording to true so that the record_step function can start recording odometry data
        self.recording_active = True

        # Start a timer to record odometry data every second ...
        self.record_timer = self.create_timer(1.0, self.record_step)

        while self.recording_active:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Results Section-Finally, logging the results.
        result = OdomRecord.Result()
        result.list_of_odoms = self.odom_record
        goal_handle.succeed()
        return result

    # Define the timer function. This function will be called every second to record odometry data and publish feedback.
    def record_step(self):

        if not self.recording_active:
            return
        
        # Create point for adding to the record list
        current_point = Point()

        # Update current point
        current_point.x = self.last_odom.x
        current_point.y = self.last_odom.y
        
        # Record and add Odometry data every second
        self.odom_record.append(current_point)

        # STEP-1- Compute total distance covered by the robot during the recording session.
        
        # Calculate distance between the current live position(last_odom) and the last recorded position, and add it to the total distance. This is done every second to keep track of the total distance traveled during the recording session.
        distance = math.hypot((self.last_odom.x - self.last_x), (self.last_odom.y - self.last_y))
        
        # Filter out small movements that are likely due to noise, this is important to avoid overestimating the distance traveled due to sensor noise or minor fluctuations in the robot's position.
        if distance > 0.001:  # Only consider movements greater than 1 mm
            self.total_distance += distance
            self.last_x = self.last_odom.x  # update last recorded position
            self.last_y = self.last_odom.y  

        # Feedback section-Generate feedback 
        self.feedback_msg = OdomRecord.Feedback()

        # Publish Feedback message with current total distance
        self.feedback_msg.current_total = self.total_distance
        self.goal_handle.publish_feedback(self.feedback_msg)

        # STEP-2- Perform lap detection by checking if the robot has returned close to the starting point (first recorded odometry) after covering a certain distance threshold. This is done to determine if the robot has completed a lap around the track.

        dist_to_start = math.hypot((self.last_odom.x - self.first_odom.x), (self.last_odom.y - self.first_odom.y))
        
        if self.total_distance > 0.1 and dist_to_start < 0.10:  # Check if the distance to the starting point is less than 10 cm
            
            self.recording_active = False
            self.record_timer.cancel()
            self.record_timer = None
            self.get_logger().info("Lap completed! Stopping recording.")
            self.destroy_node()
          
def main(args=None):
    rclpy.init(args=args)
    node = RecordOdometryNode()
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







        
        