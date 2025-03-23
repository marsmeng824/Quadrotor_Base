import numpy as np

def quat2euler(quaternion):
    """transform quaternion into euler angle (Roll, Pitch, Yaw)"""
    x, y, z, w = quaternion

    # compute Roll (X axis)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    if -0.0001<= roll <= 0.0001:
       roll = 0 

    # compute Pitch (Y axis)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2  # limit in [-1,1]
    else:
        pitch = np.arcsin(sinp)
    if -0.0001<= pitch <= 0.0001:
       pitch = 0 

    # 计算 Yaw (Z axis)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    if yaw <=0.01:
        yaw = 0

    return roll, pitch, yaw  # return (Roll, Pitch, Yaw)
