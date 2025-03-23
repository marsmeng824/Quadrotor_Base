import numpy as np
import transform3d  # Install with `pip install transforms3d`

def quaternion_to_euler(qx, qy, qz, qw):
    roll, pitch, yaw = transform3d.quat2euler([qx, qy, qz, qw])
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

# Example quaternion values (Replace with real data)
qx, qy, qz, qw = -0.6083, 0.000039, 0.000099, 0.7937

roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")