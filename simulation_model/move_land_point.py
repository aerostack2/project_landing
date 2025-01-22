import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from mocap4r2_msgs.msg import RigidBodies
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

class PoseSubscriber(Node):
    def __init__(self, model_name):
        super().__init__('pose_subscriber')
        self.model_name = model_name

        # Create a subscriber to the /rigid_bodies topic
        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.listener_callback,
            10  # Queue size
        )
        self.subscription  # Prevent unused variable warning

        # Initialize Gazebo transport node
        self.gz_node = GzNode()
        
        self.get_logger().info(f"Subscribed to /rigid_bodies topic and working with model {self.model_name}")

    def listener_callback(self, msg: RigidBodies):
        # Assume msg contains a list of rigid bodies; process the first one
        if len(msg.rigidbodies) > 0:
            rigid_body = msg.rigidbodies[0].pose

            # Extract position and orientation
            position = [rigid_body.position.x, rigid_body.position.y, rigid_body.position.z]
            orientation = [rigid_body.orientation.x, rigid_body.orientation.y, 
                           rigid_body.orientation.z, rigid_body.orientation.w]

            # Call Gazebo service to update model pose
            self.set_model_pose(position, orientation)

    def set_model_pose(self, position, orientation):
        # Define the service to call
        service = "/world/empty/set_pose"

        # Create the request message
        pose_msg = Pose()
        pose_msg.name = self.model_name
        pose_msg.position.x = position[0]
        pose_msg.position.y = position[1]
        pose_msg.position.z = position[2]
        pose_msg.orientation.x = orientation[0]
        pose_msg.orientation.y = orientation[1]
        pose_msg.orientation.z = orientation[2]
        pose_msg.orientation.w = orientation[3]

        # Define the request and response message types
        request_type = Pose
        response_type = Boolean

        # Call the service with a timeout
        timeout = 1000  # milliseconds
        result, response = self.gz_node.request(service, pose_msg, request_type, response_type, timeout)

        # if result:
        #     self.get_logger().info(f"Result: {result}, Response: {response}")
        #     self.get_logger().info(f"Model {self.model_name} move to position {position}")
        # else:
        if not result:
            self.get_logger().error(f"Failed. Result: {result}, Response: {response}")

def main(args=None):
    rclpy.init(args=args)
    model_name = "land_plane"  # Replace with your model name
    pose_subscriber = PoseSubscriber(model_name)

    try:
        rclpy.spin(pose_subscriber)
    except KeyboardInterrupt:
        pose_subscriber.get_logger().info('Shutting down...')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
