import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger  # You can use a custom service if needed



class LabNavigator(Node):

    def __init__(self):
        super().__init__('supermarket_navigator') #when i launch this node, it will be called supermarket_navigator
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.srv = self.create_service(Trigger, 'navigate_to_aisle', self.navigate_to_lab_callback) #other node can call this service(navigate_to_aisle) to navigate to the aisle
        self.goal_pose = PoseStamped() #this is the message type that will be published to the topic 'goal_pose'

    def send_goal(self, x, y):
        self.goal_pose.header.frame_id = "map"  # Set the frame of reference
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()

        # Set the desired position (x, y) and orientation (z, w)
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.position.z = 0.0  # Assuming flat surface navigation

        # Assuming no rotation, i.e., facing forward in the direction of the goal
        #This determines the final direction of the robot. It is the direction the robot will face when it gets to the goal
        self.goal_pose.pose.orientation.z = 0.0
        self.goal_pose.pose.orientation.w = 1.0

        # Publish the goal pose to the navigation stack
        self.publisher.publish(self.goal_pose)
        self.get_logger().info(f"Navigating to ({x}, {y})")
        
    def navigate_to_lab_callback(self, request, response):
        # In a real application, you might pass the lab name as part of the service request
        #So here is where the flutter server will send the aisle name/number to the robot
        aisle_name = "SuperMarket"  # For now, hardcoded; you will replace this with actual data

        # Define waypoints based on aisle names
        waypoints = {
            "Diary": (4.10, 1.89),
            "Snacks": (-3.06, 1.96),
            "Drinks": (9.86, 0.08),
            "Fruits": (6.73, -3.66),
            "Seasonings & Sauces": (10.82, -8.55),
            "Meat and Seafood": (2.66, -1.56)
        }

        if aisle_name in waypoints:
            x, y = waypoints[aisle_name]
            self.send_goal(x, y)
            response.success = True
            response.message = f"Navigating to {aisle_name} at coordinates ({x}, {y})."
        else:
            response.success = False
            response.message = f"Lab '{aisle_name}' is not defined in waypoints."

        return response


def main(args=None):
    rclpy.init(args=args)
    navigator = LabNavigator()

    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()