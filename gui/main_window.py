import sys
import os
import customtkinter as ctk
from serial.tools import list_ports

# Add root folder to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.config_manager import ConfigManager
from core.kinematics import Kinematics
from core.serial_driver import SerialDriver
from gui.vtk_viewer import VTKViewer

# Set CTK styles
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class MainWindow(ctk.CTk):
    def __init__(self):
        super().__init__()
        
        self.title("Modern Robot Control HMI")
        self.geometry("1024x720")
        
        # 1. Initialize logic components
        self.config = ConfigManager()
        self.kinematics = Kinematics(self.config)
        self.serial = SerialDriver(feedback_callback=self._on_serial_feedback)
        
        # 2. Joint Angles State [J1, J2, J3, J4, J5, J6]
        self.joint_angles = [
            float(self.config.get("J1AngCur", 0.0)),
            float(self.config.get("J2AngCur", 0.0)),
            float(self.config.get("J3AngCur", 0.0)),
            float(self.config.get("J4AngCur", 0.0)),
            float(self.config.get("J5AngCur", 90.0)),
            float(self.config.get("J6AngCur", 0.0))
        ]
        
        # 3. Initialize 3D Viewer
        self.viewer = VTKViewer(self, self.config)
        
        # 4. Construct Layout
        self._create_widgets()
        self._update_coordinates()

    def _create_widgets(self):
        # Configure layout grids
        self.grid_columnconfigure(0, weight=1) # Control Panel
        self.grid_columnconfigure(1, weight=1) # Position Monitors
        self.grid_rowconfigure(0, weight=1)
        
        # Left Panel (Connection & Jog Controls)
        self.left_panel = ctk.CTkFrame(self, corner_radius=10)
        self.left_panel.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        self.left_panel.grid_columnconfigure(0, weight=1)
        
        self._create_connection_frame()
        self._create_jog_frame()
        
        # Right Panel (XYZ Coordinates & Viewer Launcher)
        self.right_panel = ctk.CTkFrame(self, corner_radius=10)
        self.right_panel.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        self.right_panel.grid_columnconfigure(0, weight=1)
        
        self._create_coordinate_frame()
        self._create_viewer_frame()

    def _create_connection_frame(self):
        conn_frame = ctk.CTkLabelFrame(self.left_panel, text="Serial Connection")
        conn_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        conn_frame.grid_columnconfigure(0, weight=1)
        conn_frame.grid_columnconfigure(1, weight=1)
        
        # List serial ports
        ports = [p.device for p in list_ports.comports()]
        if not ports:
            ports = ["None"]
            
        self.port_menu = ctk.CTkOptionMenu(conn_frame, values=ports)
        self.port_menu.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        
        self.connect_btn = ctk.CTkButton(conn_frame, text="Connect", command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=1, padx=10, pady=10, sticky="ew")

    def _create_jog_frame(self):
        jog_frame = ctk.CTkLabelFrame(self.left_panel, text="Joint Jog Controls")
        jog_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        jog_frame.grid_columnconfigure(0, weight=1)
        
        self.sliders = []
        self.angle_labels = []
        
        joint_names = ["J1 Base", "J2 Shoulder", "J3 Elbow", "J4 Wrist", "J5 Pitch", "J6 Roll"]
        mins = [-170.0, -42.0, -89.0, -180.0, -105.0, -180.0]
        maxs = [170.0, 90.0, 52.0, 180.0, 105.0, 180.0]
        
        for i in range(6):
            j_row = ctk.CTkFrame(jog_frame)
            j_row.grid(row=i, column=0, padx=5, pady=5, sticky="ew")
            j_row.grid_columnconfigure(1, weight=1)
            
            lbl = ctk.CTkLabel(j_row, text=joint_names[i], width=100)
            lbl.grid(row=0, column=0, padx=5)
            
            slider = ctk.CTkSlider(
                j_row, 
                from_=mins[i], 
                to=maxs[i], 
                command=lambda val, idx=i: self._on_slider_move(idx, val)
            )
            slider.set(self.joint_angles[i])
            slider.grid(row=0, column=1, padx=5, sticky="ew")
            self.sliders.append(slider)
            
            val_lbl = ctk.CTkLabel(j_row, text=f"{self.joint_angles[i]:.2f}°", width=60)
            val_lbl.grid(row=0, column=2, padx=5)
            self.angle_labels.append(val_lbl)

    def _create_coordinate_frame(self):
        self.coord_frame = ctk.CTkLabelFrame(self.right_panel, text="Cartesian Position (TCP)")
        self.coord_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        self.coord_frame.grid_columnconfigure((0, 1), weight=1)
        
        self.xyz_labels = {}
        axes = ["X", "Y", "Z", "Rz", "Ry", "Rx"]
        for idx, axis in enumerate(axes):
            row = idx % 3
            col = idx // 3
            
            lbl_text = f"{axis}: --"
            if axis in ["X", "Y", "Z"]:
                lbl_text += " mm"
            else:
                lbl_text += "°"
                
            lbl = ctk.CTkLabel(self.coord_frame, text=lbl_text, font=("Arial", 16, "bold"))
            lbl.grid(row=row, column=col, padx=20, pady=15, sticky="w")
            self.xyz_labels[axis] = lbl

    def _create_viewer_frame(self):
        viewer_frame = ctk.CTkLabelFrame(self.right_panel, text="3D Visualizer")
        viewer_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        viewer_frame.grid_columnconfigure(0, weight=1)
        
        btn = ctk.CTkButton(
            viewer_frame, 
            text="Launch 3D Robot Viewer", 
            height=60, 
            command=self._launch_viewer
        )
        btn.grid(row=0, column=0, padx=20, pady=40, sticky="ew")

    def _toggle_connection(self):
        if not self.serial.running:
            port = self.port_menu.get()
            if self.serial.connect(port):
                self.connect_btn.configure(text="Disconnect", fg_color="red")
        else:
            self.serial.disconnect()
            self.connect_btn.configure(text="Connect", fg_color=["#3B8ED0", "#1F538D"])

    def _on_slider_move(self, idx, value):
        # Update state angle
        self.joint_angles[idx] = float(value)
        self.angle_labels[idx].configure(text=f"{value:.2f}°")
        
        # Save angle to config (save current position)
        self.config.set(f"J{idx+1}AngCur", f"{value:.4f}")
        
        # Trigger 3D model update
        if self.viewer.vtk_running:
            self.viewer.update_joints(self.joint_angles)
            
        # Update XYZ math calculations
        self._update_coordinates()
        
        # Send serial updates (or print online mode mock packet)
        self.serial.send_move_command(self.joint_angles)

    def _update_coordinates(self):
        """Calculates XYZ coordinates using FK from joint angles and updates the UI labels."""
        coords = self.kinematics.forward(self.joint_angles)
        axes = ["X", "Y", "Z", "Rz", "Ry", "Rx"]
        for idx, axis in enumerate(axes):
            val = coords[idx]
            unit = " mm" if axis in ["X", "Y", "Z"] else "°"
            self.xyz_labels[axis].configure(text=f"{axis}: {val:.2f}{unit}")

    def _launch_viewer(self):
        self.viewer.launch()
        # Apply initial joint rotation values to the VTK assembly
        self.viewer.update_joints(self.joint_angles)

    def _on_serial_feedback(self, data):
        """Callback run when receiving feedback strings from the serial port."""
        print(f"[UI Serial Feedback]: {data}")

    def on_closing(self):
        # Graceful cleanup
        self.serial.disconnect()
        self.viewer.vtk_running = False
        self.destroy()

# CTK Label Frame custom class to display with title border
class CTKCTkLabelFrame(ctk.CTkFrame):
    def __init__(self, master, text="", **kwargs):
        super().__init__(master, **kwargs)
        self.label = ctk.CTkLabel(self, text=text, font=("Arial", 12, "bold"))
        self.label.grid(row=0, column=0, padx=10, pady=(5, 0), sticky="w")
        
    def grid(self, **kwargs):
        super().grid(**kwargs)
        
ctk.CTkLabelFrame = CTKCTkLabelFrame
