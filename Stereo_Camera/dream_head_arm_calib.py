#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm, norm
from scipy.optimize import least_squares, minimize
from scipy.spatial.transform import Rotation as R

"""
We are trying to optimize on T0 and T_tag so that Total error is least
Ti is the camera -> tag transform 
T0 * T_i * T_tag = T_cam_to_tag
"""


def exp_map(a, b, c):
    # Define the generators of the Lie algebra so(3)
    # TODO: what are these?
    G_1 = np.array([[0, 0, 0], [0, 0, -1], [0, 1, 0]])

    G_2 = np.array([[0, 0, 1], [0, 0, 0], [-1, 0, 0]])

    G_3 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])

    tangent_vec = a * G_1 + b * G_2 + c * G_3
    return expm(tangent_vec)


def cost_func_exp_map(vec):
    a, b, c = vec
    R = exp_map(a, b, c)
    R_target = np.eye(3)
    error = norm(R - R_target, "fro")
    return error


def to_se3(pose):
    # x,y,z, wx, wy, wz, w, T0, Ttag
    translation = np.array(pose[:3])
    # quaternion = np.array(pose[3:])
    # Convert quaternion to rotation matrix
    so3_vec = np.array(pose[3:])
    rotation_matrix = exp_map(*so3_vec)
    # rotation_matrix = R.from_quat(quaternion).as_matrix()

    # Construct the SE(3) matrix
    SE3 = np.eye(4)
    SE3[:3, :3] = rotation_matrix
    SE3[:3, 3] = translation

    return SE3


def rotation_matrix_to_so3(R):
    assert R.shape == (3, 3), "R must be a 3x3 matrix"

    theta = np.arccos((np.trace(R) - 1) / 2.0)

    if theta < 1e-6:
        # For very small angles, we can approximate the axis
        return np.zeros(3)

    u = (1 / (2 * np.sin(theta))) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]
    )

    return theta * u


def to_pose_vec(SE3):
    translation = SE3[:3, 3]
    rotation_matrix = SE3[:3, :3]
    # quaternion = R.from_matrix(rotation_matrix).as_quat()
    so3_vec = rotation_matrix_to_so3(rotation_matrix)
    pose_vec = np.concatenate((translation, so3_vec))
    return pose_vec


def pose_error(params, Tis, T_tag_to_cams):
    total_error = 0
    T0 = to_se3(params[:6])
    T_tag = to_se3(params[6:])
    for Ti, T_tag_to_cam in zip(Tis, T_tag_to_cams):
        T_err = T0 @ Ti @ T_tag @ T_tag_to_cam
        vec_err = to_pose_vec(T_err)
        total_error += vec_err @ vec_err
    return total_error


def residual_error(params, Tis, T_tag_to_cams):
    T0 = to_se3(params[:6])
    T_tag = to_se3(params[6:])
    residuals = []
    for Ti, T_tag_to_cam in zip(Tis, T_tag_to_cams):
        T_err = T0 @ Ti @ T_tag @ T_tag_to_cam
        vec_err = to_pose_vec(T_err)
        residuals.append(vec_err @ vec_err)
    return residuals


def get_april_tag_poses():
    pass


def euler_to_rotation_matrix(roll, pitch, yaw):
    # Create a rotation object from Euler angles
    r = R.from_euler("xyz", [roll, pitch, yaw])
    # Convert to a rotation matrix
    return r.as_matrix()


def get_pose_vec(x, y, z, r, p, yaw):
    rot = euler_to_rotation_matrix(r, p, yaw)
    a, b, c = rotation_matrix_to_so3(rot)
    return [x, y, z, a, b, c]


def optimize(Tis, T_tag_to_cams, t0_estimate, t_tag_estimate):
    # x,y,z, wx, wy, wz, w, T0, Ttag
    T0_vec = np.array(t0_estimate + t_tag_estimate)
    # TODO: what is L-BFGS-8?
    # TODO: what's the jacobian of this?
    # Either the least_squares method or the minimize method could work.
    result = least_squares(
        residual_error, T0_vec, args=(Tis, T_tag_to_cams), method="lm"
    )
    # result = minimize(pose_error, T0_vec, args = (Tis, T_tag_to_cams), method='Powell')
    # These methods are not very good.
    # result = minimize(pose_error, T0_vec, args = (Tis, T_tag_to_cams), method='COBYLA')
    # result = minimize(pose_error, T0_vec, args = (Tis, T_tag_to_cams), method='Nelder-Mead')
    return result


if __name__ == "__main__":
    # Tis, T_tag_to_cams = get_april_tag_poses()
    t0_estimate = get_pose_vec(0, 0, 0, 0, 0, 0)
    t_tag_estimate = get_pose_vec(0, 0, 0, 0, 0, 0)
    # optimize()
