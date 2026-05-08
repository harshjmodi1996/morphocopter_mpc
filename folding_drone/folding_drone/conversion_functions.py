import numpy as np
import math

def q2rpy(q):
    # convert quaternion to roll pitch yaw (USING ZYX convention)
    # reference: https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
    roll  = math.atan2(2.0 * (q[3] * q[3] + q[0] * q[1]) , 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))
    pitch = math.asin(2.0 * (q[2] * q[0] - q[3] * q[1]))
    yaw = math.atan2(2.0 * (q[3] * q[0] + q[1] * q[2]) , - 1.0 + 2.0 * (q[0] * q[0] + q[1] * q[1]))
    return([roll,pitch,yaw])

def rpy2q(r,p,y):
    # convert roll pitch yaw to quaternions(USING ZYX convention)
    R = rpy2rotmat(r,p,y)
    return(rotmat2q(R))

# Old function error giving:
# def rotmat2q(R):
#     # convert rotation matrix to quaternion
#     # reference: https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/ or https://intra.ece.ucr.edu/~farrell/AidedNavigation/D_App_Quaternions/Rot2Quat.pdf
#     w = ((1+R[0,0]+R[1,1]+R[2,2])**0.5)/2.0
#     x = (R[2,1] - R[1,2])/( 4.0 *w)
#     y = (R[0,2] - R[2,0])/( 4.0 *w)
#     z = (R[1,0] - R[0,1])/( 4.0 *w)
#     return([w,x,y,z])


def rotmat2q(R):
    # Reference https://github.com/harshjmodi1996/CAL_UB_harsh_src/blob/main/simulation_nonlinear_controller_gap_passing_trajectory.py
    q = np.empty((4,))
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        q[0] = 0.25 * S
        q[1] = (R[2, 1] - R[1, 2]) / S
        q[2] = (R[0, 2] - R[2, 0]) / S
        q[3] = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q[0] = (R[2, 1] - R[1, 2]) / S
        q[1] = 0.25 * S
        q[2] = (R[0, 1] + R[1, 0]) / S
        q[3] = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q[0] = (R[0, 2] - R[2, 0]) / S
        q[1] = (R[0, 1] + R[1, 0]) / S
        q[2] = 0.25 * S
        q[3] = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q[0] = (R[1, 0] - R[0, 1]) / S
        q[1] = (R[0, 2] + R[2, 0]) / S
        q[2] = (R[1, 2] + R[2, 1]) / S
        q[3] = 0.25 * S
    return [float(q[0]),float(q[1]),float(q[2]),float(q[3])]

def q2rotmat(q):
    # convert quaternion to rotation matrix
    # First row of the rotation matrix
    r00 = 2.0 * (q[0] * q[0] + q[1] * q[1]) - 1.0
    r01 = 2.0 * (q[1] * q[2] - q[0] * q[3])
    r02 = 2.0 * (q[1] * q[3] + q[0] * q[2])
    # Second row of the rotation matrix
    r10 = 2.0 * (q[1] * q[2] + q[0] * q[3])
    r11 = 2.0 * (q[0] * q[0] + q[2] * q[2]) - 1.0
    r12 = 2.0 * (q[2] * q[3] - q[0] * q[1])
    # Third row of the rotation matrix
    r20 = 2.0 * (q[1] * q[3] - q[0] * q[2])
    r21 = 2.0 * (q[2] * q[3] + q[0] * q[1])
    r22 = 2.0 * (q[0] * q[0] + q[3] * q[3]) - 1.0
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
    return(rot_matrix)

def rotmat2rpy(R):
    # conver rotaiton matrix to roll, pitch, yaw (using ZYX convention)
    # reference: https://stackoverflow.com/questions/11514063/extract-yaw-pitch-and-roll-from-a-rotationmatrix
    yaw = math.atan2(R[1,0],R[0,0])
    pitch = math.atan2(-R[2,0],(R[2,1]**2.0+R[2,2]**2.0)**0.5)
    roll = math.atan2(R[2,1],R[2,2])
    return([roll,pitch,yaw])

# def rotmat2rpy_XYZ(R):
#     # conver rotaiton matrix to roll, pitch, yaw (using XYZ convention)
#     # reference: https://stackoverflow.com/questions/11514063/extract-yaw-pitch-and-roll-from-a-rotationmatrix
#     roll = math.atan2(-R[1,2],R[2,2])
#     pitch = math.atan2(R[0,2],(R[0,0]**2.0+R[0,1]**2.0)**0.5)
#     yaw = math.atan2(-R[0,1],R[0,0])
#     return([roll,pitch,yaw])

def rpy2rotmat(r,p,y):
    # convert roll/pitch/yaw to rotation matrix (using ZYX convention)
    about_yaw = np.array([[math.cos(y),-math.sin(y),0.0],
                          [math.sin(y),math.cos(y),0.0],
                          [0.0,0.0,1.0]])
    about_pitch = np.array([[math.cos(p),0.0,math.sin(p)],
                            [0.0,1.0,0.0],
                            [-math.sin(p),0.0,math.cos(p)]])
    about_roll = np.array([[1.0,0.0,0.0],
                           [0.0,math.cos(r),-math.sin(r)],
                           [0.0,math.sin(r),math.cos(r)]])
    rotmat = np.matmul(np.matmul(about_yaw,about_pitch),about_roll).copy()
    return(rotmat)


def convert_rotation(joint_angle,q):
    # Convert Measured rotation quaternion in Pixhawk FRD coordinate frame into UAV configuration FRD frame (due to upper arm rotation)
    # joint_angle rotation in radians, q is quaternions
    yaw_rotation = np.array([[math.cos(-joint_angle/2.0),-math.sin(-joint_angle/2.0),0.0],
                         [math.sin(-joint_angle/2.0),math.cos(-joint_angle/2.0),0.0],
                         [0.0,0.0,1.0]])
    rotation_matrix = q2rotmat(q)
    rotation_matrix_FRD_Joint = np.matmul(rotation_matrix,yaw_rotation).copy()  # order changed on Jul 3, 2024, before this, only yaw was coming correctly
    return(rotmat2q(rotation_matrix_FRD_Joint))

def convert_rotation_back(joint_angle,q):
    # Convert UAV configuration FRD frame quaternion commands to Pixhawk FRD coordinate frame quaternion commands (due to upper arm rotation)
    yaw_rotation = np.array([[math.cos(joint_angle/2.0),-math.sin(joint_angle/2.0),0.0],
                         [math.sin(joint_angle/2.0),math.cos(joint_angle/2.0),0.0],
                         [0.0,0.0,1.0]])
    rotation_matrix = q2rotmat(q)
    rotation_matrix_FRD_Joint_Pixhawk = np.matmul(rotation_matrix,yaw_rotation).copy()
    return(rotmat2q(rotation_matrix_FRD_Joint_Pixhawk))

def convert_axes(joint_angle,xyz_np_array):
    # Convert Measured rotation speed/torques in between Pixhawk FRD coordinate frame and UAV configuration FRD frame (due to upper arm rotation)
    # positive joint angle for converting from Pixhawk FRD to Joint FRD
    # negative joint angel for converting from Joint FRD to Pixhawk FRD
    # joint_angle rotation in radians, q is quaternions
    yaw_rotation = np.array([[math.cos(joint_angle/2.0),-math.sin(joint_angle/2.0),0.0],
                         [math.sin(joint_angle/2.0),math.cos(joint_angle/2.0),0.0],
                         [0.0,0.0,1.0]])
    return(np.matmul(yaw_rotation, xyz_np_array))

def euclidean_distance(p1, p2):
    ''' Calculate the Euclidean distance between two points p1 and p2
        p1, p2 : 3D points in the form of numpy arrays or lists
        returns : Euclidean distance as a float'''
    
    return np.linalg.norm(np.array(p1) - np.array(p2))


