import sys
import os

# Add root folder to sys.path to resolve local imports if run directly
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    # Try importing compiled C++ pybind11 module from root directory
    import robot_kinematics as rk
except ImportError:
    rk = None
    print("[WARNING] Khong the load thu vien robot_kinematics.pyd.")
    print("Vui long chay file 'build_kinematics.py' de bien dich thu vien truoc.")

class Kinematics:
    def __init__(self, config_manager):
        self.config = config_manager
        self.initialized = False
        self.dh_params = {}
        self.initialize_kinematics()

    def initialize_kinematics(self):
        """Initializes robot kinematics with configuration limits and DH parameters."""
        self.dh_params = self.config.get_dh_parameters()
        self.initialized = True
        
        if rk is None:
            # Return True since we can compute using Python fallback
            return True

        try:
            # 1. Reset and set base robot parameters
            rk.robot_data_reset()
            rk.robot_set()

            # 2. Extract and apply DH parameters from config
            dh = self.dh_params
            rk.set_dh_parameters_explicit(
                dh["theta"][0], dh["theta"][1], dh["theta"][2], dh["theta"][3], dh["theta"][4], dh["theta"][5],
                dh["alpha"][0], dh["alpha"][1], dh["alpha"][2], dh["alpha"][3], dh["alpha"][4], dh["alpha"][5],
                dh["a"][0], dh["a"][1], dh["a"][2], dh["a"][3], dh["a"][4], dh["a"][5],
                dh["d"][0], dh["d"][1], dh["d"][2], dh["d"][3], dh["d"][4], dh["d"][5]
            )

            # 3. Extract and apply Joint limits
            pos_limits = [
                float(self.config.get("J1PosLim", 170)),
                float(self.config.get("J2PosLim", 90)),
                float(self.config.get("J3PosLim", 52)),
                float(self.config.get("J4PosLim", 180)),
                float(self.config.get("J5PosLim", 105)),
                float(self.config.get("J6PosLim", 180))
            ]
            neg_limits = [
                float(self.config.get("J1NegLim", 170)),
                float(self.config.get("J2NegLim", 42)),
                float(self.config.get("J3NegLim", 89)),
                float(self.config.get("J4NegLim", 180)),
                float(self.config.get("J5NegLim", 105)),
                float(self.config.get("J6NegLim", 180))
            ]
            rk.set_joint_limits(pos_limits, neg_limits)
            
            return True
        except Exception as e:
            print(f"Error initializing C++ kinematics: {e}")
            return False

    def forward_py(self, joints):
        """Native Python implementation of Craig's DHM Forward Kinematics (J1-J6)"""
        import math
        
        # 1. Base frame (default to identity)
        T = [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        ]
        
        def matrix_multiply(A, B):
            out = [0.0] * 16
            out[0] = A[0]*B[0] + A[4]*B[1] + A[8]*B[2]
            out[1] = A[1]*B[0] + A[5]*B[1] + A[9]*B[2]
            out[2] = A[2]*B[0] + A[6]*B[1] + A[10]*B[2]
            out[3] = 0.0
            
            out[4] = A[0]*B[4] + A[4]*B[5] + A[8]*B[6]
            out[5] = A[1]*B[4] + A[5]*B[5] + A[9]*B[6]
            out[6] = A[2]*B[4] + A[6]*B[5] + A[10]*B[6]
            out[7] = 0.0
            
            out[8] = A[0]*B[8] + A[4]*B[9] + A[8]*B[10]
            out[9] = A[1]*B[8] + A[5]*B[9] + A[9]*B[10]
            out[10] = A[2]*B[8] + A[6]*B[9] + A[10]*B[10]
            out[11] = 0.0
            
            out[12] = A[0]*B[12] + A[4]*B[13] + A[8]*B[14] + A[12]
            out[13] = A[1]*B[12] + A[5]*B[13] + A[9]*B[14] + A[13]
            out[14] = A[2]*B[12] + A[6]*B[13] + A[10]*B[14] + A[14]
            out[15] = 1.0
            return out

        def DHM_2_pose(alpha, a, theta, d):
            crx = math.cos(alpha)
            srx = math.sin(alpha)
            crz = math.cos(theta)
            srz = math.sin(theta)
            
            pose = [0.0] * 16
            pose[0] = crz
            pose[4] = -srz
            pose[8] = 0.0
            pose[12] = a
            
            pose[1] = crx * srz
            pose[5] = crx * crz
            pose[9] = -srx
            pose[13] = -d * srx
            
            pose[2] = srx * srz
            pose[6] = crz * srx
            pose[10] = crx
            pose[14] = d * crx
            
            pose[3] = 0.0
            pose[7] = 0.0
            pose[11] = 0.0
            pose[15] = 1.0
            return pose

        # Loop through the 6 joints
        for i in range(6):
            alpha = self.dh_params["alpha"][i] * math.pi / 180.0
            a = self.dh_params["a"][i]
            theta_offset = self.dh_params["theta"][i] * math.pi / 180.0
            d = self.dh_params["d"][i]
            
            ji_rad = joints[i] * math.pi / 180.0
            
            hi = DHM_2_pose(alpha, a, theta_offset + ji_rad, d)
            T = matrix_multiply(T, hi)
            
        # Extracted pose translation
        x = T[12]
        y = T[13]
        z = T[14]
        
        # Calculate Rz, Ry, Rx (axis-angle representation in degrees)
        sin_angle = (((T[0] + T[5]) + T[10]) - 1.0) * 0.5
        sin_angle = max(-1.0, min(1.0, sin_angle))
        
        angle = math.acos(sin_angle)
        if angle < 1e-6:
            rx = 0.0
            ry = 0.0
            rz = 0.0
        else:
            sin_angle_val = math.sin(angle)
            if abs(sin_angle_val) < 1e-6:
                iidx = 0
                max_diag = T[0]
                if T[5] > max_diag:
                    max_diag = T[5]
                    iidx = 1
                if T[10] > max_diag:
                    max_diag = T[10]
                    iidx = 2
                    
                b_I = [0] * 9
                b_I[0] = 1
                b_I[4] = 1
                b_I[8] = 1
                
                denom = 2.0 * (1.0 + max_diag)
                denom = math.sqrt(denom) if denom > 0.0 else 0.0
                
                vector = [0.0] * 3
                vector_tmp = iidx * 4
                vector[0] = (T[vector_tmp] + b_I[3 * iidx]) / denom if denom > 0 else 0
                vector[1] = (T[1 + vector_tmp] + b_I[1 + 3 * iidx]) / denom if denom > 0 else 0
                vector[2] = (T[2 + vector_tmp] + b_I[2 + 3 * iidx]) / denom if denom > 0 else 0
                angle_deg = 180.0
            else:
                denom = 1.0 / (2.0 * sin_angle_val)
                vector = [
                    (T[6] - T[9]) * denom,
                    (T[8] - T[2]) * denom,
                    (T[1] - T[4]) * denom
                ]
                angle_deg = angle * 180.0 / math.pi
                
            rx = vector[2] * angle_deg
            ry = vector[1] * angle_deg
            rz = vector[0] * angle_deg
            
        return [x, y, z, rz, ry, rx]

    def forward(self, joints):
        """Computes Forward Kinematics (FK)
        Input: list of 6 joint angles in degrees [J1, J2, J3, J4, J5, J6]
        Output: list of 6 Cartesian values [X, Y, Z, Rz, Ry, Rx] in mm/degrees
        """
        if rk is None:
            return self.forward_py(joints)
        try:
            return rk.forward_kinematics(joints)
        except Exception as e:
            print(f"Error in C++ forward kinematics: {e}")
            return self.forward_py(joints)

    def inverse(self, target_pose, current_joints_estimate):
        """Computes Inverse Kinematics (IK)
        Input: 
            target_pose: [X, Y, Z, Rz, Ry, Rx]
            current_joints_estimate: [J1, J2, J3, J4, J5, J6] (initial seed)
        Output:
            list of 6 joint angles [J1, J2, J3, J4, J5, J6] in degrees
        """
        if rk is None:
            return current_joints_estimate
        try:
            return rk.inverse_kinematics(target_pose, current_joints_estimate)
        except Exception as e:
            print(f"Error in C++ inverse kinematics: {e}")
            return current_joints_estimate
