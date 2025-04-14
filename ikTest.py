import math

def IK(x, y, z, L1, L2, constraints):
    """
    Inverse kinematics for a 3DOF robotic arm:
    
    Parameters:
        x, y, z       - Target end-effector coordinates
        L1, L2        - Lengths of the upper and lower arm segments
        constraints   - Dictionary with joint angle limits (in degrees), e.g.
                        {
                            'base': (-90, 90),
                            'shoulder': (0, 135),
                            'elbow': (0, 150)
                        }
    Returns:
        A dictionary with joint angles in degrees, or None if unreachable
    """
    # Calculate base angle (yaw)
    base_angle = math.degrees(math.atan2(y, x))

    # Convert to planar (2D) coordinates in the base frame
    r = math.hypot(x, y)  # distance in XY plane
    d = math.hypot(r, z)  # distance to target in 3D

    # Check reachability
    if d > (L1 + L2) or d < abs(L1 - L2):
        return None  # Unreachable

    # Law of cosines for elbow angle
    cos_elbow = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    elbow_angle = math.acos(cos_elbow)
    
    # Law of cosines for shoulder angle
    angle_a = math.atan2(z, r)
    angle_b = math.acos((d**2 + L1**2 - L2**2) / (2 * L1 * d))
    shoulder_angle = angle_a + angle_b

    # Convert to degrees
    base_deg = base_angle
    shoulder_deg = math.degrees(shoulder_angle)
    elbow_deg = math.degrees(elbow_angle)

    # Enforce constraints
    if not (constraints['base'][0] <= base_deg <= constraints['base'][1]):
        return None
    if not (constraints['shoulder'][0] <= shoulder_deg <= constraints['shoulder'][1]):
        return None
    if not (constraints['elbow'][0] <= elbow_deg <= constraints['elbow'][1]):
        return None

    return {
        'base': base_deg,
        'shoulder': shoulder_deg,
        'elbow': elbow_deg
    }

constraints = {
    'base': (-90, 90),
    'shoulder': (0, 135),
    'elbow': (0, 150)
}

def SafeIK(x, y, z, L1, L2, constraints):

    angles = IK(x=x, y=y, z=z, L1=L1, L2=L2, constraints=constraints)

    if angles:
        print("Joint Angles:", angles)
        return angles
    else:
        print("Target position is unreachable or violates constraints.")
        return False


