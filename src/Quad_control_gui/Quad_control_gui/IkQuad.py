import numpy as np
from scipy.optimize import minimize

class Kinematics():
    def __init__(self, base = np.eye(4),dh=[]):
        self.dh_params = dh
        self.base = base
    
    def changeBase(self,base):
        self.base = base
    def dh_transformation_matrix(self, a, alpha, d, theta):
        """
        Compute the DH transformation matrix for the given parameters.
        a, alpha, d, theta are the DH parameters for a joint.
        """
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    def forward_kinematics(self, joint_angles):
        """
        Compute the forward kinematics for the robot using the given joint angles.
        joint_angles is a list of joint angles [theta1, theta2, theta3, theta4, theta5].
        """
        # Start with the identity matrix
        T = self.base
        
        # Multiply each DH transformation matrix with corresponding joint angle
        for i, (a, alpha, d, theta_offset) in enumerate(self.dh_params):
            theta = joint_angles[i] + theta_offset  # Add the joint angle to the offset (theta_i + theta_offset)
            T_i = self.dh_transformation_matrix(a, alpha, d, theta)
            T = np.dot(T, T_i)
        
        # The position of the end-effector is the last column of the transformation matrix T
        position = T[:3, 3]
        # The orientation of the end-effector is the upper-left 3x3 submatrix of T
        orientation_matrix = T[:3, :3]

        # Optionally, convert the orientation matrix to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = self.rotation_matrix_to_euler_angles(orientation_matrix)
        orientation = np.array([roll, pitch, yaw])
        return position,orientation
    def rotation_matrix_to_euler_angles(self, R):
        """
        Convert a rotation matrix to Euler angles (roll, pitch, yaw).
        The rotation matrix R is a 3x3 numpy array.
        """
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0

        return roll, pitch, yaw
    def create_transformation_matrix(self,x, y, z, roll, pitch, yaw):
        """
        Create a homogeneous transformation matrix from position (x, y, z) and 
        orientation (roll, pitch, yaw) defined in Euler angles.
        
        Parameters:
        x, y, z: Translation in the global frame
        roll, pitch, yaw: Rotation angles (in radians) for the orientation
        
        Returns:
        A 4x4 homogeneous transformation matrix.
        """
        # Rotation matrix around the x-axis (roll)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        # Rotation matrix around the y-axis (pitch)
        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # Rotation matrix around the z-axis (yaw)
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation matrix
        R = np.dot(R_z, np.dot(R_y, R_x))  # R = Rz * Ry * Rx
        
        # Translation vector
        t = np.array([x, y, z])
        
        # Create the homogeneous transformation matrix
        T = np.eye(4)
        T[:3, :3] = R  # Top-left 3x3 part is the rotation matrix
        T[:3, 3] = t   # Top-right 3x1 part is the translation vector
        
        return T

class FiveDOFArm():
    def __init__(self,params,joint_angles=[0.0,0.0,0.0,0.0],base=[0.0,0.0,0.0,0.0,0.0,0.0]):
        self.dhParams = params
        self.JointAngles = joint_angles

        self.cogPose = base[0:3]
        self.cogRot = base[3:6]
        base = np.append(self.cogPose,self.cogRot)
        self.ArmKinematics = Kinematics(dh=self.dhParams)
        base = self.ArmKinematics.create_transformation_matrix(base[0],base[1],base[2],base[3],base[4],base[5])
        self.ArmKinematics.changeBase(base)
    def changeArmBase(self,Yaw):
        base_Yaw = self.ArmKinematics.create_transformation_matrix(Yaw[0],Yaw[1],Yaw[2]+0.0710,Yaw[3],Yaw[4],Yaw[5])
        self.ArmKinematics.changeBase(base_Yaw)

    def ArmPose(self):
        current_position , orientation= self.ArmKinematics.forward_kinematics(self.JointAngles)
        return current_position,orientation
    def getJoint(self,joint):
        return float(self.JointAngles[joint])
    def setJoint(self,joint,angle):
        print(self.JointAngles)
        print(joint)
        self.JointAngles[joint]=angle
        print(angle)
        print(self.JointAngles)
        
    
    def objective_function(self, joint_angles, target_position):
        """
        Objective function to minimize the error between the target position and the FK result.
        """
        
        current_position, _ = self.ArmKinematics.forward_kinematics(joint_angles)
        error = np.linalg.norm(target_position - current_position)  # Euclidean distance
        return error
    
    def inverse_kinematics(self, target_position, initial_guess):
        """
        Solve the inverse kinematics using optimization.
        target_position is the desired end-effector position (x, y, z).
        initial_guess is the initial guess for the joint angles.
        """
        result = minimize(self.objective_function, initial_guess, args=(target_position,), method='BFGS')
        if result.success:
            return result.x  # Optimized joint angles
        else:
            raise ValueError("Optimization failed")


class ForwardKinematicsLeg():
    def __init__(self,params,base=[0,0,0,0,0,0],edge=[1,1],joints=[0,0,0,0,0],):
        self.dhParams = params
        self.cogPose = base[0:3]
        self.cogRot = base[3:6]
        self.edge = edge
        base = np.append(self.cogPose,self.cogRot)
        self.LegKinematics = Kinematics(dh=self.dhParams[:])
        base = self.LegKinematics.create_transformation_matrix(base[0],base[1],base[2],base[3],base[4],base[5])
        self.LegKinematics.changeBase(base)
        self.LegJoint = joints
        
        

    def FootPose(self):
        current_position,_ = self.LegKinematics.forward_kinematics(self.LegJoint)
        return current_position
    def getJoint(self,joint):
        return float(self.LegJoint[joint])
    def getJoints(self):
        return float(self.LegJoint)
    def setJoint(self,joint,angle):
        self.LegJoint[joint]=angle
    def changePoseLeg(self,Yaw):
        base_Yaw = self.LegKinematics.create_transformation_matrix(Yaw[0],Yaw[1],Yaw[2],Yaw[3],Yaw[4],Yaw[5])
        self.edge = self.LegKinematics.create_transformation_matrix(self.edge[0],self.edge[1],0,0,0,0)
        base_adjust_90_y= self.LegKinematics.create_transformation_matrix(0,0,0,0,np.pi/2,0)
        
        self.base = np.dot(np.dot(base_Yaw,self.edge),base_adjust_90_y)
        self.LegKinematics.changeBase(self.base)
        