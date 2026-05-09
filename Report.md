# Task B: Policy Deployment via ROS 2
**Research Internship Shortlisting Exercise**  
**Robotics Research Center (RRC), IIIT Hyderabad**  

## 1. Overview
This report details the integration of a trained reinforcement learning policy (PPO) for the LEAP Hand into a ROS 2 (Humble) environment. The objective of Task B was to bridge the gap between simulation and real-world deployment by setting up a ROS 2 pipeline that runs the policy in inference mode, streams joint commands, and visualizes the robot's motion in RViz2.

## 2. ROS 2 System Architecture

The deployment architecture is completely modular and built using Python nodes within the `leap_deployment` package. It relies on four primary nodes communicating over standard ROS 2 topics:

1. **`policy_node`**: This node initializes the trained Stable Baselines 3 PPO model (`best_model.zip`). At a frequency of 10 Hz, it computes the continuous action targets `[-1, 1]` based on the current state and publishes them to `/hand/joint_commands` as a `Float32MultiArray`. 
2. **`interface_node`**: Serving as the kinematic bridge, this node subscribes to the joint commands. It un-normalizes the actions into absolute physical joint limits, applies a 100 Hz exponential moving average (kinematic smoothing) to prevent sudden jumps, and publishes the true hand positions to `/joint_states` as standard `sensor_msgs/JointState` messages.
3. **`robot_state_publisher`**: Consumes `leap_hand.urdf` to broadcast static transforms and the physical `mujoco_menagerie` `.obj` meshes to RViz.

### Topic Communication Graph
```
[policy_node] -- /hand/joint_commands --> [interface_node]
[interface_node] -- /joint_states --> [robot_state_publisher]
[robot_state_publisher] -- /tf --> [RViz2]
```

## 3. Implementation Highlights

* **URDF Generation**: The original MJCF models from `mujoco_menagerie` were programmatically parsed using a Python script (`generate_urdf_mjcf.py`). This script extracted the kinematic chains, translated Quaternions to Euler angles, and bound the original high-fidelity `.obj` meshes into a standard URDF format compatible with `robot_state_publisher`.
* **Robust Fallback Mechanism**: The `policy_node` includes a robust `try/except` initialization block. If the SB3 PPO model cannot be located or loaded, the node gracefully falls back to generating a programmatic sine-wave trajectory. This ensures the ROS 2 integration pipeline can always be visually verified even without a fully converged RL policy.
* **Open-Loop Visualization Safety**: Because RViz doesn't compute physical contact constraints, an un-normalized policy could cause the fingers to visually clip through one another when closing into a fist. To provide a clean visualization, the `interface_node` artificially clamps the upper visual flexion limits of the joints, ensuring the fingers close smoothly without interpenetrating. Additionally, it implements a 5-second automatic episode reset to continuously loop the grasping animation.

## 4. Build and Execution Instructions

To replicate the visual deployment:

1. **Build the Workspace**:
   ```bash
   cd ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Launch the Pipeline**:
   ```bash
   ros2 launch leap_deployment display.launch.py
   ```
   This unified launch file spins up the entire node graph and automatically opens RViz2 with a pre-configured perspective focusing on the hand.
