import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
import numpy as np

class DummyJointStates(Node):

    def __init__(self):
        super().__init__('DummyJointStates')      
        # create joint_state  publisher
        self.publisher_ = self.create_publisher(
            JointState, 
            'joint_states', 
            10)

        self.msg = JointState()
        self.msg.name.append("single_rrbot_joint1")
        self.msg.name.append("single_rrbot_joint2")
        self.msg.position = [.0,.0]
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.i += 1
        self.msg.position[0] = np.sin(self.i/100)*np.pi/2
        self.msg.position[1] = np.cos(self.i/10)*np.pi
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.msg)
        self.get_logger().info(f"{self.i}: {self.msg.header.stamp}")
        self.get_logger().info(f"{self.i}: {self.msg.name}")
        self.get_logger().info(f"{self.i}: {self.msg.position}")
        pass

def main(args=None):
    rclpy.init(args=args)

    dummy_joint_states = DummyJointStates()
    rclpy.spin(dummy_joint_states)

    dummy_joint_states.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
