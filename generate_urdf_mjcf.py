import os
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R

_HERE = os.path.dirname(os.path.abspath(__file__))

mjcf_path = os.path.join(_HERE, "leap_hand_mjcf", "right_hand_grasp.xml")
scene_path = os.path.join(_HERE, "leap_hand_mjcf", "grasp_scene.xml")
mesh_dir   = os.path.join(_HERE, "leap_hand_mjcf", "assets")
urdf_out   = os.path.join(_HERE, "ros2_ws", "src", "leap_deployment", "urdf", "leap_hand.urdf")

# 1. Read right_hand_grasp.xml
tree_hand = ET.parse(mjcf_path)
hand_root = tree_hand.getroot()

# Build mesh mapping from asset section
mesh_mapping = {}
for asset in hand_root.findall('.//asset/mesh'):
    mesh_mapping[asset.get('name')] = asset.get('file')

def q_to_rpy(w, x, y, z):
    r = R.from_quat([x, y, z, w])
    return r.as_euler('xyz', degrees=False)

def build_urdf(node, parent_name="base_link", urdf_xml=""):
    if node.tag == 'body':
        body_name = node.get('name')
        if not body_name: return urdf_xml
        
        # Determine joint connecting parent to this body
        joint = node.find('joint')
        if joint is not None:
            joint_name = joint.get('name')
            joint_type = 'revolute'
            axis = "0 0 -1" # default from file
            limit = '<limit lower="-3.14" upper="3.14" effort="10" velocity="10"/>'
        else:
            joint_name = f"{parent_name}_to_{body_name}"
            joint_type = 'fixed'
            axis = "0 0 1"
            limit = ""

        # Extract origin
        pos = node.get('pos', "0 0 0")
        quat = node.get('quat', "1 0 0 0")
        w, x, y, z = map(float, quat.split())
        rpy = q_to_rpy(w, x, y, z)
        
        urdf_xml += f'''
  <joint name="{joint_name}" type="{joint_type}">
    <parent link="{parent_name}"/>
    <child link="{body_name}"/>
    <origin xyz="{pos}" rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>
    <axis xyz="{axis}"/>
    {limit}
  </joint>
  <link name="{body_name}">
'''
        # Add visuals
        for geom in node.findall('geom'):
            geom_class = geom.get('class')
            geom_type = geom.get('type', 'box' if geom.get('size') else 'mesh')
            
            geom_pos = geom.get('pos', "0 0 0")
            geom_quat = geom.get('quat', "1 0 0 0")
            gw, gx, gy, gz = map(float, geom_quat.split())
            grpy = q_to_rpy(gw, gx, gy, gz)
            
            rgba = geom.get('rgba', "0.5 0.5 0.5 1")
            
            if geom_class == 'visual' or geom.get('mesh') is not None or geom_class in ['tip', 'thumb_tip']:
                mesh_name = geom.get('mesh', geom_class)
                actual_mesh_file = mesh_mapping.get(mesh_name, mesh_name + ".obj")
                # Use package:// URI so RViz2 resolves meshes portably
                urdf_xml += f'''    <visual>
      <origin xyz="{geom_pos}" rpy="{grpy[0]:.6f} {grpy[1]:.6f} {grpy[2]:.6f}"/>
      <geometry>
        <mesh filename="package://leap_deployment/urdf/assets/{actual_mesh_file}"/>
      </geometry>
      <material name="mat_{body_name}"><color rgba="{rgba}"/></material>
    </visual>
'''
            elif geom_type == 'box' and geom_class != 'collision':
                # for table and object
                size = geom.get('size', '0.1 0.1 0.1')
                sx, sy, sz = map(float, size.split())
                urdf_xml += f'''    <visual>
      <origin xyz="{geom_pos}" rpy="{grpy[0]:.6f} {grpy[1]:.6f} {grpy[2]:.6f}"/>
      <geometry>
        <box size="{sx*2} {sy*2} {sz*2}"/>
      </geometry>
      <material name="mat_{geom.get('name', 'box')}"><color rgba="{rgba}"/></material>
    </visual>
'''
        urdf_xml += f'  </link>\n'

        # Recurse for children
        for child in node.findall('body'):
            urdf_xml = build_urdf(child, body_name, urdf_xml)
            
    return urdf_xml

hand_palm = hand_root.find('.//body[@name="palm"]')

# 2. Read grasp_scene.xml
tree_scene = ET.parse(scene_path)
scene_root = tree_scene.getroot()
worldbody = scene_root.find('worldbody')

urdf = '<?xml version="1.0"?>\n<robot name="leap_grasp_scene">\n'
urdf += '  <link name="base_link"/>\n'

# Parse all bodies in grasp_scene.xml (which might include table, object)
for body in worldbody.findall('body'):
    if body.get('name') == 'object':
        continue
    urdf = build_urdf(body, "base_link", urdf)

# And parse the hand which might not be explicitly instanced in scene worldbody directly if it uses include
# The palm is in hand_root
urdf = build_urdf(hand_palm, "base_link", urdf)

urdf += '</robot>\n'

with open(urdf_out, 'w') as f:
    f.write(urdf)

print(f"Generated {urdf_out} with scene elements and hand.")
