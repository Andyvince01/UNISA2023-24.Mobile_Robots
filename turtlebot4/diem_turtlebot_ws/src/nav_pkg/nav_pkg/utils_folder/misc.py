import numpy as np

from ._pose import Pose

# =============================================================================
# Add Gaussian noise to a Pose object.
# =============================================================================
def add_gaussian_noise(pose: Pose, mean : float = 0.0, std_dev : float = 0.0) -> Pose:
    '''
    Add Gaussian noise to the position of the pose.
    
    args:
        - pose: Pose object
        - mean: mean of the Gaussian distribution
        - std_dev: standard deviation of the Gaussian distribution
        
    return:
        - Pose: Pose object with noise added to the position
    '''
    pose.position.x += np.random.normal(mean, std_dev)
    pose.position.y += np.random.normal(mean, std_dev)
    # pose.position.z += np.random.normal(mean, std_dev)
    
    return pose

# =============================================================================
# Distance between poses.
# =============================================================================
def distance_between_poses(*poses: Pose) -> float:
    '''
    Calculate the distance between two poses.
    
    args:
        - poses: a list of Pose objects
        
    return:
        - float: distance between the two poses
    '''
    if len(poses) != 2:
        raise ValueError('Two poses are required to calculate distance.')
    
    x = np.array([pose.position.x for pose in poses])
    y = np.array([pose.position.y for pose in poses])
    z = np.array([pose.position.z for pose in poses])           # z coordinate is not used
    return np.linalg.norm([np.diff(x), np.diff(y)])