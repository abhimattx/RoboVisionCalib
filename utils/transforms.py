import numpy as np
import cv2

def create_transform_matrix(rvec, tvec):
    """
    Create 4x4 transformation matrix from rotation and translation vectors.
    
    Args:
        rvec: Rotation vector
        tvec: Translation vector
    
    Returns:
        4x4 transformation matrix
    """
    rot_mat, _ = cv2.Rodrigues(rvec)
    transform = np.eye(4)
    transform[:3, :3] = rot_mat
    transform[:3, 3] = tvec
    return transform

def transform_point(point, transform_matrix):
    """
    Transform a 3D point using a 4x4 transformation matrix.
    
    Args:
        point: 3D point as [x, y, z]
        transform_matrix: 4x4 transformation matrix
    
    Returns:
        Transformed 3D point
    """
    point_homogeneous = np.append(point, 1.0)
    transformed = transform_matrix @ point_homogeneous
    return transformed[:3]

def invert_transform(transform):
    """Invert a 4x4 transformation matrix."""
    inverted = np.eye(4)
    R = transform[:3, :3]
    t = transform[:3, 3]
    
    inverted[:3, :3] = R.T
    inverted[:3, 3] = -R.T @ t
    
    return inverted