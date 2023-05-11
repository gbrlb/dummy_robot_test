import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

class OneJointState(Node):

    def __init__(self):
        super().__init__('OneJointState')
        # create subscriber
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        
        # create publisher
        self.publisher_ = self.create_publisher(
            JointState, 
            'one_joint_state', 
            10)

        self.msg = JointState()
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.msg.header = msg.header
        self.msg.name = [msg.name[0]]
        self.msg.position = [msg.position[0]]
        # self.msg.velocity = [msg.velocity[0]]
        # self.msg.effort = [msg.effort[0]]
        self.publisher_.publish(self.msg)
        self.i += 1
        pass

    def timer_callback(self):
        self.get_logger().info(f"{self.i}: {self.msg.position[0]}")
        pass

def main(args=None):
    rclpy.init(args=args)

    one_joint_state = OneJointState()
    rclpy.spin(one_joint_state)

    one_joint_state.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
