#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ForkElevatorController(Node):
    def __init__(self):
        super().__init__('fork_elevator_controller')
        
        self.sub = self.create_subscription(
            Float64, 
            '/cmd_fork_global_height', 
            self.height_callback, 
            10)
            
        self.mast_pub = self.create_publisher(Float64, '/mast_cmd', 10)
        self.fork_pub = self.create_publisher(Float64, '/fork_cmd', 10)
        
        # KINEMATICS (URDF limits: mast [-1.25, 0], fork [-0.15, 1.25])
        # Logical height h = elevation above the lowest point (tine 0.03 m above the floor).
        # The first 0.15 m are free lift: only the fork moves (from -0.15 to 0).
        # Above that, symmetric mast/fork split like a real duplex mast.
        self.FREE_LIFT = 0.15   # range covered by fork_joint only
        self.MAX_LIFT = 2.65    # FREE_LIFT + 2*1.25 (total travel)

        self.get_logger().info('Node de control telescòpic Actiu. h=0 deixa la pua a 0.03 m del terra.')

    def height_to_joints(self, h):
        """Logical height [0, MAX_LIFT] -> (mast_cmd, fork_cmd)."""
        if h <= self.FREE_LIFT:
            return 0.0, h - self.FREE_LIFT
        half = (h - self.FREE_LIFT) / 2.0
        return -half, half  # mast axis is inverted: negative = up

    def height_callback(self, msg):
        requested_height = max(0.0, min(msg.data, self.MAX_LIFT))
        mast_pos, fork_pos = self.height_to_joints(requested_height)

        mast_msg = Float64()
        mast_msg.data = mast_pos
        fork_msg = Float64()
        fork_msg.data = fork_pos

        self.mast_pub.publish(mast_msg)
        self.fork_pub.publish(fork_msg)

        self.get_logger().info(
            f'Usuari: {requested_height:.2f}m | '
            f'Màstil: {mast_msg.data:.3f}m | Forquilla: {fork_msg.data:.3f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ForkElevatorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()