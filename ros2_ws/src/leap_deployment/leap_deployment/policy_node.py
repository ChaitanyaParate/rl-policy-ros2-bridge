import sys
import os


_THIS_FILE = os.path.abspath(__file__)
_PROJECT_ROOT = os.path.normpath(os.path.join(os.path.dirname(_THIS_FILE), '..', '..', '..', '..'))


venv_site_packages = os.path.join(_PROJECT_ROOT, 'leap_ros_venv', 'lib', 'python3.12', 'site-packages')
if os.path.exists(venv_site_packages):
    sys.path.insert(0, venv_site_packages)

import site
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import math

try:
    from stable_baselines3 import PPO
    import numpy as np
    SB3_AVAILABLE = True
except ImportError:
    SB3_AVAILABLE = False

class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/hand/joint_commands', 10)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.state_callback,
            10
        )
        self.current_joint_positions = [0.0] * 16
        
        self.declare_parameter('use_sine_wave', False)
        self.use_sine_wave = self.get_parameter('use_sine_wave').get_parameter_value().bool_value
        
        # Locate best_model.zip relative to the project root (works on any clone)
        self.model_path = os.path.join(_PROJECT_ROOT, 'best_model.zip')
        
        self.model = None
        if not self.use_sine_wave:
            if SB3_AVAILABLE and os.path.exists(self.model_path):
                self.get_logger().info(f'Loading SB3 model from {self.model_path}')
                try:
                    self.model = PPO.load(self.model_path)
                    self.get_logger().info('Model loaded successfully.')
                except Exception as e:
                    self.get_logger().error(f'Failed to load model: {e}')
                    self.get_logger().info('Falling back to sine wave.')
                    self.use_sine_wave = True
            else:
                self.get_logger().warn('SB3 not available or model not found. Falling back to sine wave.')
                self.use_sine_wave = True
                
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.t = 0.0

    def state_callback(self, msg):
        # Update current joint positions from interface node feedback
        if len(msg.position) == 16:
            self.current_joint_positions = list(msg.position)

    def timer_callback(self):
        msg = Float32MultiArray()
        
        if self.use_sine_wave:
            # Generate a simple sine wave for all 16 joints
            # Let's map it to [-1, 1] which is standard action space
            val = math.sin(self.t)
            msg.data = [val] * 16
            self.t += self.timer_period
        else:
            # Inference mode
            # Construct a 39-dim observation:
            # [0:16] = real joint positions feedback
            obs = np.zeros(39, dtype=np.float32)
            obs[0:16] = self.current_joint_positions
            
            # [16:32] = joint velocities (keep 0)
            
            # [32:35] = object position (use spawn pos from env to prevent wild grasping)
            obs[32:35] = [0.0, 0.04, 0.085]
            
            # [35:39] = object quaternion [w, x, y, z]
            obs[35] = 1.0
            
            action, _states = self.model.predict(obs, deterministic=False)
            self.get_logger().info(f'Action max: {max(action):.2f}, min: {min(action):.2f}')
            msg.data = [float(x) for x in action]
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    policy_node = PolicyNode()
    try:
        rclpy.spin(policy_node)
    except KeyboardInterrupt:
        pass
    finally:
        policy_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
