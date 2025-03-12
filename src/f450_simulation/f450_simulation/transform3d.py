import numpy as np

def quat2euler(quaternion):
    """将四元数转换为欧拉角 (Roll, Pitch, Yaw)"""
    x, y, z, w = quaternion

    # 计算 Roll (X 轴旋转)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # 计算 Pitch (Y 轴旋转)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2  # 防止超出范围 [-1,1]
    else:
        pitch = np.arcsin(sinp)

    # 计算 Yaw (Z 轴旋转)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw  # 返回 (Roll, Pitch, Yaw)
