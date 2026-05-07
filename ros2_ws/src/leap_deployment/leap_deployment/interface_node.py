import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

class InterfaceNode(Node):
    def __init__(self):
        super().__init__('interface_node')
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/hand/joint_commands',
            self.command_callback,
            10
        )
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        # Match the joint names in the URDF exactly in the expected order (16 joints)
        self.joint_names = [
            'if_mcp', 'if_rot', 'if_pip', 'if_dip',
            'mf_mcp', 'mf_rot', 'mf_pip', 'mf_dip',
            'rf_mcp', 'rf_rot', 'rf_pip', 'rf_dip',
            'th_cmc', 'th_axl', 'th_mcp', 'th_ipl'
        ]
        
        # Physical limits from right_hand.xml
        self.joint_limits = {
            'mcp': [-0.314, 2.23],
            'rot': [-1.047, 1.047],
            'pip': [-0.506, 1.885],
            'dip': [-0.366, 2.042],
            'thumb_cmc': [-0.349, 2.094],
            'thumb_axl': [-0.349, 2.094],
            'thumb_mcp': [-0.47, 2.443],
            'thumb_ipl': [-1.34, 1.88]
        }

        # Mapping our 16 joints to their respective classes
        self.joint_types = [
            'mcp', 'rot', 'pip', 'dip',          # Index
            'mcp', 'rot', 'pip', 'dip',          # Middle
            'mcp', 'rot', 'pip', 'dip',          # Ring
            'thumb_cmc', 'thumb_axl', 'thumb_mcp', 'thumb_ipl' # Thumb
        ]

        # Initialize kinematics state
        self.current_positions = [0.0] * 16
        self.target_positions = [0.0] * 16
        
        # 100 Hz kinematic interpolator timer
        self.timer = self.create_timer(0.01, self.publish_callback)

    def command_callback(self, msg):
        if len(msg.data) != 16:
            self.get_logger().warn(f'Expected 16 joint commands, got {len(msg.data)}')
            return
            
        # Delta control: action in [-1, 1] scaled by max_delta (0.1 rad per step)
        max_delta = 0.1
        for i in range(16):
            jtype = self.joint_types[i]
            low, high = self.joint_limits[jtype]
            action = msg.data[i]
            
            # Apply delta
            new_target = self.target_positions[i] + action * max_delta
            
            # Clip to physical joint limits
            self.target_positions[i] = max(low, min(new_target, high))

    def publish_callback(self):
        # Linear interpolation smoothing (exponential moving average)
        # alpha = 0.15 means it closes 15% of the gap every 10ms
        alpha = 0.15 
        for i in range(16):
            self.current_positions[i] += alpha * (self.target_positions[i] - self.current_positions[i])
            
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.current_positions
        
        self.publisher_.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    interface_node = InterfaceNode()
    try:
        rclpy.spin(interface_node)
    except KeyboardInterrupt:
        pass
    finally:
        interface_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
