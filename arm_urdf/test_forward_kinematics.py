from urdfpy import URDF
import numpy as np
from scipy.spatial.transform import Rotation as R

# Load URDF file
robot = URDF.load('dummy_geom_urdf.urdf')  # Make sure this file is in the same directory

# Set joint angles in radians
joint_angles = {
    'joint_1': np.deg2rad(30),
    'joint_2': np.deg2rad(0),
    'joint_3': np.deg2rad(0),
    'joint_4': np.deg2rad(0)
}

# Compute forward kinematics
fk = robot.link_fk(cfg=joint_angles)
end_effector_link = robot.links[-1]  # Last link as end-effector

# Extract position and orientation
pose_matrix = fk[end_effector_link]
position = pose_matrix[:3, 3]
rotation = R.from_matrix(pose_matrix[:3, :3])
euler_angles = rotation.as_euler('xyz', degrees=True)

print(f"End-effector Position: {position}")
print(f"End-effector Orientation (XYZ Euler angles in degrees): {euler_angles}")

