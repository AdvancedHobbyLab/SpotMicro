import math

def InverseKinematics(x, y, z):
    """Main Inverse Kinematic function.

    Calculates the angles (deg.) for each servo (foot, leg, shoulder) on a single leg given
    an offset in mm from the shoulder joint to the foot. The axis (x, y, z) are oriented
    so that the X axis is forward/backward, the Y axis is down/up, and the Z is the distance 
    righ/left. For the Z axis, the direction is flipped on the left side so that positive
    is always away from the center.

    Parameters
    ----------
    x : float
        Distance of the foot along the X axis in mm
    y : float
        Distance of the foot along the Y axis in mm
    z : float
        Distance of the foot along the Z axis in mm

    Returns
    -------
    float: Angle (deg) to set the servo controlling the foot
    float: Angle (deg) to set the servo controlling the leg
    float: Angle (deg) to set the servo controlling the shoulder
    """
    
    # Constant lengths in mm
    upper_leg = 120
    lower_leg = 120
    shoulder_offset = 40

    y1 = math.sqrt(y*y + z*z - shoulder_offset*shoulder_offset)
    
    distance = math.sqrt(x*x + y1*y1)

    foot = math.acos((distance*distance - upper_leg*upper_leg - lower_leg*lower_leg)/(-2*upper_leg*lower_leg))
    
    leg = math.asin((lower_leg*math.sin(foot))/distance) - (math.atan(x/y) if y!=0 else 0)

    shoulder = math.atan(distance/shoulder_offset) + math.atan(z/y)
    
    # Convert radians to degrees
    foot = foot/math.pi * 180
    leg = leg/math.pi * 180
    shoulder = shoulder/math.pi * 180
    
    return foot, leg, shoulder

def interpolate(x1, x2, ratio):
    """Helper function to interpolate between two values.

    Parameters
    ----------
    x1 : float
        Value from which to interpolate
    x2 : float
        Value to which to interpolate
    ratio : float
        Amount to interpolate. Should be in the range of 0.0 - 1.0

    Returns
    -------
    float: Interpolated value
    """
    ratio = max(0.0, min(ratio, 1.0))
    return x1 + (x2-x1)*ratio

def KeyframeKinematics(key1, key2, ratio):
    """Helper function to calculate the inverse kinematics interpolated between two keyframes.

    This function is useful for creating smooth transition between two keyframes. Given
    an amount of time to transition from one keyframe to the next, you can divide the
    amount of time that has elapsed by the total transition time to get the ratio.

    Parameters
    ----------
    key1: list
        First keyframe
    key2: list
        Second keyframe
    ratio: float
        Amount to interpolate. Should be in the range of 0.0 - 1.0

    Returns
    -------
    float: Angle (deg) to set the servo controlling the foot
    float: Angle (deg) to set the servo controlling the leg
    float: Angle (deg) to set the servo controlling the shoulder
    """
    return InverseKinematics(
        interpolate(key1[0], key2[0], ratio),
        interpolate(key1[1], key2[1], ratio),
        interpolate(key1[2], key2[2], ratio))