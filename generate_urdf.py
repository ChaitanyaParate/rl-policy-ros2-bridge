import os

urdf = """<?xml version="1.0"?>
<robot name="leap_hand">
  <link name="palm">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.1 0.1 0.02"/></geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>
"""

fingers = {
    'if': {'x': -0.01, 'y': 0.04},
    'mf': {'x': -0.01, 'y': 0.00},
    'rf': {'x': -0.01, 'y': -0.04},
    'th': {'x': -0.05, 'y': -0.02}
}

joints = ['mcp', 'rot', 'pip', 'dip']
th_joints = ['cmc', 'axl', 'mcp', 'ipl']

for prefix, pos in fingers.items():
    parent = "palm"
    jnames = th_joints if prefix == 'th' else joints
    
    for i, jname in enumerate(jnames):
        j_full = f"{prefix}_{jname}"
        l_full = f"{prefix}_link_{i}"
        
        # Joint
        px = pos['x']
        py = pos['y']
        origin_xyz = f"{px} {py} 0.05" if i == 0 else "0 0 0.04"
        
        urdf += f"""
  <joint name="{j_full}" type="revolute">
    <parent link="{parent}"/>
    <child link="{l_full}"/>
    <origin xyz="{origin_xyz}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="10"/>
  </joint>
"""
        # Link
        urdf += f"""
  <link name="{l_full}">
    <visual>
      <origin xyz="0 0 0.02" rpy="0 0 0"/>
      <geometry><cylinder radius="0.008" length="0.04"/></geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>
"""
        parent = l_full

urdf += "</robot>\n"

with open("/mnt/newvolume/Programming/Python/Deep_Learning/Policy_Deployment_via_ROS2/ros2_ws/src/leap_deployment/urdf/leap_hand.urdf", "w") as f:
    f.write(urdf)
print("URDF generated!")
