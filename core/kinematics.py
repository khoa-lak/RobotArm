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
        self.initialize_kinematics()

    def initialize_kinematics(self):
        """Initializes robot kinematics with configuration limits and DH parameters."""
        if rk is None:
            return False

        try:
            # 1. Reset and set base robot parameters
            rk.robot_data_reset()
            rk.robot_set()

            # 2. Extract and apply DH parameters from config
            dh = self.config.get_dh_parameters()
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
            
            self.initialized = True
            return True
        except Exception as e:
            print(f"Error initializing kinematics: {e}")
            return False

    def forward(self, joints):
        """Computes Forward Kinematics (FK)
        Input: list of 6 joint angles in degrees [J1, J2, J3, J4, J5, J6]
        Output: list of 6 Cartesian values [X, Y, Z, Rz, Ry, Rx] in mm/degrees
        """
        if not self.initialized or rk is None:
            # Fallback mock values for testing
            return [267.0, -43.0, 412.0, 9.0, 121.0, -2.0]
        try:
            return rk.forward_kinematics(joints)
        except Exception as e:
            print(f"Error in forward kinematics: {e}")
            return [0.0]*6

    def inverse(self, target_pose, current_joints_estimate):
        """Computes Inverse Kinematics (IK)
        Input: 
            target_pose: [X, Y, Z, Rz, Ry, Rx]
            current_joints_estimate: [J1, J2, J3, J4, J5, J6] (initial seed)
        Output:
            list of 6 joint angles [J1, J2, J3, J4, J5, J6] in degrees
        """
        if not self.initialized or rk is None:
            return current_joints_estimate
        try:
            return rk.inverse_kinematics(target_pose, current_joints_estimate)
        except Exception as e:
            print(f"Error in inverse kinematics: {e}")
            return current_joints_estimate
