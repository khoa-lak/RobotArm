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
        self.geometry("1200x800")
        
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
        
        # Force Tkinter to update and map widgets to obtain valid HWND and layout sizes
        self.update()
        
        # 5. Launch 3D Viewer directly in the right panel
        self._launch_viewer()

    def _create_widgets(self):
        import tkinter as tk
        
        # Create PanedWindow for horizontal split
        self.main_pane = tk.PanedWindow(self, orient=tk.HORIZONTAL, bg="#1a1a1a", bd=0, sashwidth=6, sashpad=2, relief="flat")
        self.main_pane.pack(fill="both", expand=True)
        
        # Left Panel (Control Panel)
        self.left_panel = ctk.CTkFrame(self.main_pane, corner_radius=0, fg_color="transparent")
        self.left_panel.grid_columnconfigure(0, weight=1)
        self.main_pane.add(self.left_panel, width=420, minsize=350)
        
        # Right Panel (3D Viewer Container)
        self.right_panel = ctk.CTkFrame(self.main_pane, corner_radius=0, fg_color="transparent")
        self.right_panel.grid_columnconfigure(0, weight=1)
        self.right_panel.grid_rowconfigure(0, weight=1)
        self.main_pane.add(self.right_panel, minsize=400)
        
        # Create Tabview inside left_panel
        self.tabview = ctk.CTkTabview(self.left_panel, corner_radius=10)
        self.tabview.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.tab_control = self.tabview.add("Control")
        self.tab_config = self.tabview.add("Config")
        
        self.tab_control.grid_columnconfigure(0, weight=1)
        self.tab_config.grid_columnconfigure(0, weight=1)
        
        # Create control elements inside the Control tab
        self._create_connection_frame()
        self._create_coordinate_frame()
        self._create_jog_frame()
        
        # Create configuration elements inside the Config tab
        self._create_config_tab_widgets()

    def _create_connection_frame(self):
        conn_frame = ctk.CTkLabelFrame(self.tab_control, text="Serial Connection")
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
        jog_frame = ctk.CTkLabelFrame(self.tab_control, text="Joint Jog Controls")
        jog_frame.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
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
            
        # Add Go to Home button at the bottom of the Jog frame
        home_btn = ctk.CTkButton(
            jog_frame,
            text="Go to Home (Về Home)",
            height=40,
            command=self._go_to_home
        )
        home_btn.grid(row=6, column=0, padx=10, pady=15, sticky="ew")

    def _create_coordinate_frame(self):
        self.coord_frame = ctk.CTkLabelFrame(self.tab_control, text="Cartesian Position (TCP)")
        self.coord_frame.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
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
        self.viewer.launch(self.right_panel)
        # Apply initial joint rotation values to the VTK assembly
        self.viewer.update_joints(self.joint_angles)

    def _on_serial_feedback(self, data):
        """Callback run when receiving feedback strings from the serial port."""
        print(f"[UI Serial Feedback]: {data}")

    def _create_config_tab_widgets(self):
        # 1. DH Parameters grid frame
        dh_frame = ctk.CTkLabelFrame(self.tab_config, text="DH Parameters Editor (J1-J6)")
        dh_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        
        dh_frame.grid_columnconfigure(0, weight=1) # Joint label column
        dh_frame.grid_columnconfigure((1, 2, 3, 4), weight=2)
        
        # Header Labels
        headers = ["Joint", "Theta (θ)", "Alpha (α)", "d (mm)", "a (mm)"]
        for col_idx, header in enumerate(headers):
            lbl = ctk.CTkLabel(dh_frame, text=header, font=("Arial", 11, "bold"))
            lbl.grid(row=0, column=col_idx, padx=2, pady=5)
            
        # 24 Entry fields for DH parameters
        self.dh_entries = {}
        param_keys = ["Θ", "α", "d", "a"]
        
        for row_idx in range(6):
            j_lbl = ctk.CTkLabel(dh_frame, text=f"J{row_idx+1}", font=("Arial", 11, "bold"))
            j_lbl.grid(row=row_idx+1, column=0, padx=2, pady=3)
            
            for col_idx, param in enumerate(param_keys):
                cfg_key = f"J{row_idx+1}{param}DHpar"
                val = self.config.get(cfg_key, "0.0")
                
                entry = ctk.CTkEntry(dh_frame, width=60, justify="center")
                entry.insert(0, str(val))
                entry.grid(row=row_idx+1, column=col_idx+1, padx=2, pady=3)
                self.dh_entries[(row_idx+1, param)] = entry
                
        # 2. STL Path configuration frame
        path_frame = ctk.CTkLabelFrame(self.tab_config, text="Robot 3D Model STL Directory")
        path_frame.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        path_frame.grid_columnconfigure(0, weight=1)
        
        self.stl_path_entry = ctk.CTkEntry(path_frame, placeholder_text="Select folder containing STL files")
        self.stl_path_entry.insert(0, self.viewer.stl_dir)
        self.stl_path_entry.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        
        browse_btn = ctk.CTkButton(path_frame, text="Browse", width=60, command=self._browse_stl_dir)
        browse_btn.grid(row=0, column=1, padx=5, pady=10)
        
        # 3. Apply Button
        apply_btn = ctk.CTkButton(self.tab_config, text="Apply & Save Config", height=40, command=self._apply_and_save_config)
        apply_btn.grid(row=2, column=0, padx=10, pady=15, sticky="ew")

    def _browse_stl_dir(self):
        from tkinter import filedialog
        path = filedialog.askdirectory(initialdir=self.viewer.stl_dir)
        if path:
            self.stl_path_entry.delete(0, "end")
            self.stl_path_entry.insert(0, path)

    def _apply_and_save_config(self):
        import os
        from tkinter import messagebox
        
        # 1. Validate and read DH entries
        param_keys = ["Θ", "α", "d", "a"]
        temp_data = {}
        for row_idx in range(6):
            for param in param_keys:
                entry = self.dh_entries[(row_idx+1, param)]
                val = entry.get().strip()
                try:
                    float(val)
                except ValueError:
                    messagebox.showerror("Validation Error", f"Invalid numeric value '{val}' for J{row_idx+1} {param}")
                    return
                temp_data[f"J{row_idx+1}{param}DHpar"] = val
                
        # Validate STL path
        stl_dir = self.stl_path_entry.get().strip()
        if not os.path.exists(stl_dir):
            messagebox.showerror("Validation Error", f"STL directory does not exist: {stl_dir}")
            return
            
        # 2. Save values in ConfigManager
        for k, v in temp_data.items():
            self.config.config_data[k] = v
        self.config.save_config()
        
        # 3. Re-initialize Kinematics
        self.kinematics.initialize_kinematics()
        
        # 4. Reload VTK Viewer model
        self.viewer.reload_robot(new_stl_dir=stl_dir)
        
        # 5. Recompute current position labels
        self._update_coordinates()
        
        messagebox.showinfo("Success", "Configuration applied and saved successfully!")

    def _go_to_home(self):
        # 1. Reset state joint angles to 0.0
        self.joint_angles = [0.0] * 6
        
        # 2. Update sliders, labels and config values
        for i in range(6):
            self.sliders[i].set(0.0)
            self.angle_labels[i].configure(text="0.00°")
            self.config.set(f"J{i+1}AngCur", "0.0000")
            
        # 3. Trigger 3D model update
        if self.viewer.vtk_running:
            self.viewer.update_joints(self.joint_angles)
            
        # 4. Update XYZ math coordinates display
        self._update_coordinates()
        
        # 5. Send serial updates
        self.serial.send_move_command(self.joint_angles)

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
