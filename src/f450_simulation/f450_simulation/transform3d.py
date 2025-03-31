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


def acc_to_euler(acc_des, yaw_des):
    """
    acc_des: Expectation acceleration [ax, ay, az]
    yaw_des: expectation yaw (rad)
    Return: Target Euler Angle [roll_d, pitch_d, yaw_d]
    """
    g = 9.81
    norm = np.linalg.norm(acc_des)
    if norm < 1e-6:
        acc_des = np.array([0, 0, g])
        norm = g

    # thrust orientation
    z_b_des = acc_des / norm
    ax, ay, az = z_b_des
    psi = yaw_des

    ## Compute desired roll and pitch from desired acceleration and yaw
    theta_d = np.arcsin(ax * np.sin(psi) - ay * np.cos(psi))
    phi_d = np.arctan2(ax * np.cos(psi) + ay * np.sin(psi), az)

    return np.array([phi_d, theta_d, psi])

def euler_to_omega(euler, euler_dot):
        """Convert Euler angle rates to angular velocity (in body frame)"""
        phi, theta, _ = euler

        T = np.array([
            [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
            [0, np.cos(phi),              -np.sin(phi)],
            [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
        ])

        return T @ euler_dot
