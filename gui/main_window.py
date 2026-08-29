import sys
import os
import time
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
        
        # 3. Initialize 3D Viewer & Custom Objects
        self.viewer = VTKViewer(self, self.config)
        self.selected_obj_id = None
        self._updating_obj_ui = False
        
        # 3b. Robot Link Alignment & Calibration State
        self.selected_link_key = "Base"
        self._updating_link_ui = False
        self.link_display_map = {
            "Base (Khớp đế)": "Base",
            "Khớp 1 (J1 - Trục quay Đế)": "Link 1",
            "Khớp 2 (J2 - Trục Vai)": "Link 2",
            "Khớp 3 (J3 - Trục Khuỷu)": "Link 3",
            "Khớp 4 (J4 - Trục Xoay Cổ tay)": "Link 4",
            "Khớp 5 (J5 - Trục Gập Cổ tay)": "Link 5",
            "Khớp 6 (J6 - Mặt Bích / Flange)": "Link 6"
        }
        self.link_key_to_display = {v: k for k, v in self.link_display_map.items()}
        
        self._load_saved_custom_objects()
        
        # State variables for Program Editor & Simulator
        self.program_steps = []
        self.simulating = False
        self.executing_real = False
        self.captured_joints = list(self.joint_angles)
        self.captured_coords = self.kinematics.forward(self.joint_angles)
        
        # 4. Construct Layout
        self._create_widgets()
        self._update_coordinates()
        
        # Force Tkinter to update and map widgets to obtain valid HWND and layout sizes
        self.update()
        
        # 5. Launch 3D Viewer directly in the right panel
        self._launch_viewer()

    def _create_widgets(self):
        import tkinter as tk
        
        # Create PanedWindow for 3-horizontal splits
        self.main_pane = tk.PanedWindow(self, orient=tk.HORIZONTAL, bg="#1a1a1a", bd=0, sashwidth=6, sashpad=2, relief="flat")
        self.main_pane.pack(fill="both", expand=True)
        
        # Left Panel (Program / Config Tabview)
        self.left_panel = ctk.CTkFrame(self.main_pane, corner_radius=0, fg_color="transparent")
        self.left_panel.grid_columnconfigure(0, weight=1)
        self.main_pane.add(self.left_panel, width=380, minsize=300, stretch="never")
        
        # Middle Panel (3D Viewer Container)
        self.middle_panel = ctk.CTkFrame(self.main_pane, corner_radius=0, fg_color="transparent")
        self.middle_panel.grid_columnconfigure(0, weight=1)
        self.middle_panel.grid_rowconfigure(0, weight=1)
        self.main_pane.add(self.middle_panel, minsize=400, stretch="always")
        
        # Right Panel (Jog / Connection Panel)
        self.right_panel = ctk.CTkFrame(self.main_pane, corner_radius=0, fg_color="transparent")
        self.right_panel.grid_columnconfigure(0, weight=1)
        self.main_pane.add(self.right_panel, width=380, minsize=300, stretch="never")
        
        # Create Tabview inside left_panel
        self.tabview = ctk.CTkTabview(self.left_panel, corner_radius=10)
        self.tabview.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.tab_program = self.tabview.add("Program")
        self.tab_links = self.tabview.add("Robot Links")
        self.tab_objects = self.tabview.add("Objects (STL)")
        self.tab_config = self.tabview.add("Config")
        
        self.tab_program.grid_columnconfigure(0, weight=1)
        self.tab_links.grid_columnconfigure(0, weight=1)
        self.tab_objects.grid_columnconfigure(0, weight=1)
        self.tab_config.grid_columnconfigure(0, weight=1)
        
        # Create control elements directly inside the Right Panel
        self._create_connection_frame()
        self._create_coordinate_frame()
        self._create_jog_frame()
        
        # Create program editor elements inside the Program tab
        self._create_program_tab_widgets()

        # Create robot links alignment & calibration controls
        self._create_robot_links_tab_widgets()

        # Create custom objects controls inside the Objects tab
        self._create_objects_tab_widgets()

        # Create configuration elements inside the Config tab
        self._create_config_tab_widgets()

    def _create_connection_frame(self):
        conn_frame = ctk.CTkLabelFrame(self.right_panel, text="Serial Connection")
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
        jog_frame = ctk.CTkLabelFrame(self.right_panel, text="Joint Jog Controls")
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
        self.coord_frame = ctk.CTkLabelFrame(self.right_panel, text="Cartesian Position (TCP)")
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
        self.viewer.launch(self.middle_panel)
        # Apply initial joint rotation values to the VTK assembly
        self.viewer.update_joints(self.joint_angles)

    def _on_serial_feedback(self, data):
        """Callback run when receiving feedback strings from the serial port."""
        print(f"[UI Serial Feedback]: {data}")
        # If we are executing a program on the real robot and receive "done", trigger the next step
        if self.executing_real and "done" in data.lower():
            self.after(10, self._next_real_step)

    def _create_config_tab_widgets(self):
        # 0. Preset Selector Frame
        preset_frame = ctk.CTkLabelFrame(self.tab_config, text="Robot Presets (Mẫu Cấu Hình Robot)")
        preset_frame.grid(row=0, column=0, padx=10, pady=(10, 5), sticky="ew")
        preset_frame.grid_columnconfigure(0, weight=1)
        preset_frame.grid_columnconfigure(1, weight=0)

        presets = list(self.config.get_presets().keys())
        self.config_preset_menu = ctk.CTkOptionMenu(
            preset_frame,
            values=presets,
            command=self._on_preset_selected
        )
        self.config_preset_menu.grid(row=0, column=0, padx=8, pady=8, sticky="ew")

        load_preset_btn = ctk.CTkButton(
            preset_frame,
            text="Nạp Preset",
            width=90,
            command=lambda: self._on_preset_selected(self.config_preset_menu.get())
        )
        load_preset_btn.grid(row=0, column=1, padx=(0, 8), pady=8)

        # 1. DH Parameters grid frame
        dh_frame = ctk.CTkLabelFrame(self.tab_config, text="DH Parameters Editor (J1-J6)")
        dh_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        
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
        path_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")
        path_frame.grid_columnconfigure(0, weight=1)
        
        self.stl_path_entry = ctk.CTkEntry(path_frame, placeholder_text="Select folder containing STL files")
        self.stl_path_entry.insert(0, self.viewer.stl_dir)
        self.stl_path_entry.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        
        browse_btn = ctk.CTkButton(path_frame, text="Browse", width=60, command=self._browse_stl_dir)
        browse_btn.grid(row=0, column=1, padx=5, pady=10)
        
        # 3. Apply Button
        apply_btn = ctk.CTkButton(self.tab_config, text="Apply & Save Config", height=40, command=self._apply_and_save_config)
        apply_btn.grid(row=3, column=0, padx=10, pady=10, sticky="ew")

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

    def _create_program_tab_widgets(self):
        import tkinter as tk
        
        # 1. Motion Step Frame
        motion_frame = ctk.CTkLabelFrame(self.tab_program, text="Motion Step (MJ/ML)")
        motion_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        motion_frame.grid_columnconfigure((0, 1), weight=1)
        
        self.move_type_var = ctk.StringVar(value="PTP (MJ)")
        move_type_menu = ctk.CTkOptionMenu(motion_frame, values=["PTP (MJ)", "Linear (ML)"], variable=self.move_type_var)
        move_type_menu.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        capture_btn = ctk.CTkButton(motion_frame, text="Capture Pose", command=self._capture_current_pose)
        capture_btn.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        self.captured_pose_lbl = ctk.CTkLabel(
            motion_frame, 
            text="Captured: J1=0.0°, J2=0.0°...\nXYZ: [267.0, -43.0, 412.0]", 
            font=("Arial", 10),
            justify="left"
        )
        self.captured_pose_lbl.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="w")
        
        # S, Ac, Dc inputs
        inputs_frame = ctk.CTkFrame(motion_frame, fg_color="transparent")
        inputs_frame.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        inputs_frame.grid_columnconfigure((0, 1, 2), weight=1)
        
        ctk.CTkLabel(inputs_frame, text="Speed").grid(row=0, column=0)
        self.sim_speed_entry = ctk.CTkEntry(inputs_frame, width=50, justify="center")
        self.sim_speed_entry.insert(0, "50")
        self.sim_speed_entry.grid(row=1, column=0, padx=2)
        
        ctk.CTkLabel(inputs_frame, text="Acc").grid(row=0, column=1)
        self.sim_acc_entry = ctk.CTkEntry(inputs_frame, width=50, justify="center")
        self.sim_acc_entry.insert(0, "50")
        self.sim_acc_entry.grid(row=1, column=1, padx=2)
        
        ctk.CTkLabel(inputs_frame, text="Dec").grid(row=0, column=2)
        self.sim_dec_entry = ctk.CTkEntry(inputs_frame, width=50, justify="center")
        self.sim_dec_entry.insert(0, "50")
        self.sim_dec_entry.grid(row=1, column=2, padx=2)
        
        add_motion_btn = ctk.CTkButton(motion_frame, text="Add Motion Step", command=self._add_motion_step)
        add_motion_btn.grid(row=3, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        # 2. Action Step Frame
        action_frame = ctk.CTkLabelFrame(self.tab_program, text="Action/Wait Step")
        action_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        action_frame.grid_columnconfigure((0, 1), weight=1)
        
        self.action_type_var = ctk.StringVar(value="Gripper ON")
        action_menu = ctk.CTkOptionMenu(
            action_frame, 
            values=["Gripper ON", "Gripper OFF", "Wait (Dừng chờ)"], 
            variable=self.action_type_var
        )
        action_menu.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        self.wait_time_entry = ctk.CTkEntry(action_frame, width=60, justify="center")
        self.wait_time_entry.insert(0, "1.0")
        self.wait_time_entry.grid(row=0, column=1, padx=5, pady=5)
        
        add_action_btn = ctk.CTkButton(action_frame, text="Add Action Step", command=self._add_action_step)
        add_action_btn.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        # 3. Program List Frame
        sequence_frame = ctk.CTkLabelFrame(self.tab_program, text="Program Sequence")
        sequence_frame.grid(row=2, column=0, padx=10, pady=5, sticky="nsew")
        sequence_frame.grid_columnconfigure((0, 1), weight=1)
        
        list_frame = ctk.CTkFrame(sequence_frame)
        list_frame.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        self.step_listbox = tk.Listbox(
            list_frame, 
            height=5, 
            bg="#2b2b2b", 
            fg="white", 
            selectbackground="#1F538D", 
            selectforeground="white",
            bd=0, 
            highlightthickness=0, 
            font=("Arial", 9)
        )
        self.step_listbox.pack(side="left", fill="both", expand=True)
        
        scrollbar = ctk.CTkScrollbar(list_frame, command=self.step_listbox.yview)
        scrollbar.pack(side="right", fill="y")
        self.step_listbox.config(yscrollcommand=scrollbar.set)
        
        remove_btn = ctk.CTkButton(sequence_frame, text="Remove Selected", command=self._remove_selected_step)
        remove_btn.grid(row=1, column=0, padx=3, pady=5, sticky="ew")
        
        clear_btn = ctk.CTkButton(sequence_frame, text="Clear All", fg_color="red", hover_color="#900", command=self._clear_program)
        clear_btn.grid(row=1, column=1, padx=3, pady=5, sticky="ew")
        
        # 4. Sim & Real Controls Frame
        ctrl_frame = ctk.CTkLabelFrame(self.tab_program, text="Simulate & Execute")
        ctrl_frame.grid(row=3, column=0, padx=10, pady=5, sticky="ew")
        ctrl_frame.grid_columnconfigure((0, 1), weight=1)
        
        sim_btn = ctk.CTkButton(ctrl_frame, text="Run Simulation", command=self._start_simulation)
        sim_btn.grid(row=0, column=0, padx=3, pady=5, sticky="ew")
        
        real_btn = ctk.CTkButton(ctrl_frame, text="Run Real Robot", fg_color="#1eaa59", hover_color="#13773e", command=self._start_real_execution)
        real_btn.grid(row=0, column=1, padx=3, pady=5, sticky="ew")
        
        stop_btn = ctk.CTkButton(ctrl_frame, text="Stop Execution", fg_color="red", hover_color="#900", command=self._stop_execution)
        stop_btn.grid(row=1, column=0, columnspan=2, padx=3, pady=5, sticky="ew")
        
        self.sim_status_lbl = ctk.CTkLabel(ctrl_frame, text="Mô phỏng: Sẵn sàng", text_color="gray", font=("Arial", 11, "bold"))
        self.sim_status_lbl.grid(row=2, column=0, columnspan=2, padx=5, pady=2)

    def _capture_current_pose(self):
        self.captured_joints = list(self.joint_angles)
        coords = self.kinematics.forward(self.captured_joints)
        self.captured_coords = coords
        self.captured_pose_lbl.configure(
            text=f"Captured: J1={self.captured_joints[0]:.1f}°, J2={self.captured_joints[1]:.1f}°...\nXYZ: [{coords[0]:.1f}, {coords[1]:.1f}, {coords[2]:.1f}]"
        )

    def _add_motion_step(self):
        m_type = self.move_type_var.get()
        try:
            speed = int(self.sim_speed_entry.get().strip())
            acc = int(self.sim_acc_entry.get().strip())
            dec = int(self.sim_dec_entry.get().strip())
        except ValueError:
            from tkinter import messagebox
            messagebox.showerror("Error", "Tốc độ/Gia tốc/Giảm tốc phải là số nguyên!")
            return
            
        step = {
            "type": "motion",
            "motion_type": "MJ" if "MJ" in m_type else "ML",
            "joints": list(self.captured_joints),
            "coords": list(self.captured_coords),
            "speed": speed,
            "acc": acc,
            "dec": dec
        }
        self.program_steps.append(step)
        display_text = f"[{len(self.program_steps)}] {step['motion_type']} -> X:{step['coords'][0]:.1f} Y:{step['coords'][1]:.1f} Z:{step['coords'][2]:.1f} (S:{speed})"
        self.step_listbox.insert("end", display_text)

    def _add_action_step(self):
        act_type = self.action_type_var.get()
        if "Wait" in act_type:
            try:
                val = float(self.wait_time_entry.get().strip())
            except ValueError:
                from tkinter import messagebox
                messagebox.showerror("Error", "Thời gian chờ phải là số thực!")
                return
            step = {
                "type": "wait",
                "val": val
            }
            display_text = f"[{len(self.program_steps)+1}] WAIT -> {val:.1f} giây"
        else:
            action_code = "DO1on" if "ON" in act_type else "DO1off"
            step = {
                "type": "action",
                "action_type": action_code
            }
            display_text = f"[{len(self.program_steps)+1}] GRIPPER -> {'ON (Đóng)' if 'ON' in act_type else 'OFF (Mở)'}"
            
        self.program_steps.append(step)
        self.step_listbox.insert("end", display_text)

    def _remove_selected_step(self):
        try:
            sel_idx = self.step_listbox.curselection()[0]
            self.step_listbox.delete(sel_idx)
            self.program_steps.pop(sel_idx)
            # Rebuild display indexes
            self.step_listbox.delete(0, "end")
            for i, step in enumerate(self.program_steps):
                if step["type"] == "motion":
                    display_text = f"[{i+1}] {step['motion_type']} -> X:{step['coords'][0]:.1f} Y:{step['coords'][1]:.1f} Z:{step['coords'][2]:.1f} (S:{step['speed']})"
                elif step["type"] == "wait":
                    display_text = f"[{i+1}] WAIT -> {step['val']:.1f} giây"
                else:
                    display_text = f"[{i+1}] GRIPPER -> {'ON (Đóng)' if step['action_type'] == 'DO1on' else 'OFF (Mở)'}"
                self.step_listbox.insert("end", display_text)
        except IndexError:
            pass

    def _clear_program(self):
        self.program_steps.clear()
        self.step_listbox.delete(0, "end")

    def _start_simulation(self):
        if not self.program_steps:
            from tkinter import messagebox
            messagebox.showerror("Error", "Chương trình chưa có bước nào để mô phỏng!")
            return
            
        self._stop_execution()
        self.simulating = True
        self.sim_step_idx = 0
        self.sim_status_lbl.configure(text="Mô phỏng: Đang chạy...", text_color="orange")
        self.start_sim_angles = list(self.joint_angles)
        self._run_sim_step()

    def _stop_execution(self):
        self.simulating = False
        self.executing_real = False
        self.sim_status_lbl.configure(text="Mô phỏng: Đã dừng", text_color="gray")

    def _run_sim_step(self):
        if not self.simulating:
            return
            
        if self.sim_step_idx >= len(self.program_steps):
            self.simulating = False
            self.sim_status_lbl.configure(text="Mô phỏng: Hoàn thành", text_color="green")
            self._update_robot_pose(self.start_sim_angles)
            from tkinter import messagebox
            messagebox.showinfo("Simulation", "Mô phỏng hoàn thành thành công!")
            return
            
        self.step_listbox.selection_clear(0, "end")
        self.step_listbox.selection_set(self.sim_step_idx)
        self.step_listbox.activate(self.sim_step_idx)
        self.step_listbox.see(self.sim_step_idx)
        
        step = self.program_steps[self.sim_step_idx]
        
        if step["type"] == "motion":
            target_joints = step["joints"]
            self._interpolate_move(target_joints)
        elif step["type"] == "wait":
            duration = step["val"]
            self.sim_status_lbl.configure(text=f"Mô phỏng: Chờ {duration:.1f}s...", text_color="cyan")
            self.after(int(duration * 1000), self._next_sim_step)
        elif step["type"] == "action":
            action = step["action_type"]
            self.sim_status_lbl.configure(text=f"Mô phỏng: Kẹp {'ON' if action == 'DO1on' else 'OFF'}...", text_color="magenta")
            self.after(500, self._next_sim_step)

    def _interpolate_move(self, target_joints):
        start_joints = list(self.joint_angles)
        frames = 20
        step_sizes = [(target_joints[i] - start_joints[i]) / frames for i in range(6)]
        
        def step_fn(frame):
            if not self.simulating:
                return
            if frame >= frames:
                self._update_robot_pose(target_joints)
                self._next_sim_step()
            else:
                current = [start_joints[i] + step_sizes[i] * frame for i in range(6)]
                self._update_robot_pose(current)
                self.after(20, step_fn, frame + 1)
        step_fn(0)

    def _update_robot_pose(self, angles):
        self.joint_angles = list(angles)
        for i in range(6):
            self.sliders[i].set(angles[i])
            self.angle_labels[i].configure(text=f"{angles[i]:.2f}°")
        if self.viewer.vtk_running:
            self.viewer.update_joints(self.joint_angles)
        self._update_coordinates()

    def _next_sim_step(self):
        if self.simulating:
            self.sim_step_idx += 1
            self._run_sim_step()

    def _start_real_execution(self):
        if not self.serial.running:
            from tkinter import messagebox
            messagebox.showerror("Error", "Cánh tay robot chưa được kết nối! Vui lòng kết nối Serial trước.")
            return
        if not self.program_steps:
            from tkinter import messagebox
            messagebox.showerror("Error", "Chương trình chưa có bước nào để chạy!")
            return
            
        self._stop_execution()
        self.executing_real = True
        self.real_step_idx = 0
        self.sim_status_lbl.configure(text="Robot Thật: Đang chạy...", text_color="orange")
        self._execute_real_step()

    def _execute_real_step(self):
        if not self.executing_real:
            return
            
        if self.real_step_idx >= len(self.program_steps):
            self.executing_real = False
            self.sim_status_lbl.configure(text="Robot Thật: Hoàn thành", text_color="green")
            from tkinter import messagebox
            messagebox.showinfo("Execution", "Chạy chương trình trên robot thật hoàn thành!")
            return
            
        self.step_listbox.selection_clear(0, "end")
        self.step_listbox.selection_set(self.real_step_idx)
        self.step_listbox.activate(self.real_step_idx)
        self.step_listbox.see(self.real_step_idx)
        
        step = self.program_steps[self.real_step_idx]
        
        if step["type"] == "motion":
            joints = step["joints"]
            speed = step["speed"]
            acc = step["acc"]
            dec = step["dec"]
            self._update_robot_pose(joints)
            self.serial.send_move_command(joints, speed=speed, acc=acc, dec=dec)
        elif step["type"] == "wait":
            duration = step["val"]
            self.sim_status_lbl.configure(text=f"Robot Thật: Chờ {duration:.1f}s...", text_color="cyan")
            self.after(int(duration * 1000), self._next_real_step)
        elif step["type"] == "action":
            action = step["action_type"]
            self.sim_status_lbl.configure(text=f"Robot Thật: Kích hoạt {action}...", text_color="magenta")
            self.serial.send_command(action)
            self.after(500, self._next_real_step)

    def _next_real_step(self):
        if self.executing_real:
            self.real_step_idx += 1
            self._execute_real_step()

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
        
    # -------------------------------------------------------------------------
    # Robot Links Alignment & STL Calibration (Căn chỉnh Khớp 3D Robot)
    # -------------------------------------------------------------------------
    def _create_robot_links_tab_widgets(self):
        """Constructs the UI for inspecting, importing, and translating/rotating robot links."""
        self.links_scroll_frame = ctk.CTkScrollableFrame(self.tab_links, fg_color="transparent")
        self.links_scroll_frame.pack(fill="both", expand=True, padx=5, pady=5)
        self.links_scroll_frame.grid_columnconfigure(0, weight=1)

        # 1. Robot Model Profile Manager
        preset_frame = ctk.CTkLabelFrame(self.links_scroll_frame, text="🤖 Hồ Sơ Mô Hình Robot (Robot Model Profiles)")
        preset_frame.grid(row=0, column=0, padx=5, pady=(2, 6), sticky="ew")
        preset_frame.grid_columnconfigure(0, weight=1)

        # Current active model label
        active_name = self.config.get_active_model_name()
        self.active_model_lbl = ctk.CTkLabel(
            preset_frame,
            text=f"Đang dùng: {active_name}",
            font=("Arial", 11, "bold"),
            text_color="#3B8ED0",
            anchor="w"
        )
        self.active_model_lbl.grid(row=0, column=0, padx=8, pady=(8, 2), sticky="ew")

        # Model selector dropdown
        model_names = list(self.config.get_saved_models().keys())
        self.preset_menu = ctk.CTkOptionMenu(
            preset_frame,
            values=model_names if model_names else ["(Chưa có model)"],
            command=lambda v: None
        )
        self.preset_menu.set(active_name)
        self.preset_menu.grid(row=1, column=0, padx=6, pady=(2, 4), sticky="ew")

        # Row of action buttons
        model_btn_row = ctk.CTkFrame(preset_frame, fg_color="transparent")
        model_btn_row.grid(row=2, column=0, padx=6, pady=(0, 4), sticky="ew")
        model_btn_row.grid_columnconfigure((0, 1), weight=1)

        load_btn = ctk.CTkButton(
            model_btn_row,
            text="📂 Mở Model",
            height=30,
            fg_color="#1f538d",
            hover_color="#14375e",
            command=self._on_load_model
        )
        load_btn.grid(row=0, column=0, padx=2, pady=2, sticky="ew")

        new_btn = ctk.CTkButton(
            model_btn_row,
            text="✚ Model Mới",
            height=30,
            fg_color="#4a7d45",
            hover_color="#2e5c2a",
            command=self._on_new_model
        )
        new_btn.grid(row=0, column=1, padx=2, pady=2, sticky="ew")

        delete_btn = ctk.CTkButton(
            model_btn_row,
            text="🗑 Xóa Model",
            height=30,
            fg_color="#8d1f1f",
            hover_color="#5e1414",
            command=self._on_delete_model
        )
        delete_btn.grid(row=0, column=2, padx=2, pady=2, sticky="ew")
        model_btn_row.grid_columnconfigure((0, 1, 2), weight=1)

        # Save As new model
        save_as_row = ctk.CTkFrame(preset_frame, fg_color="transparent")
        save_as_row.grid(row=3, column=0, padx=6, pady=(0, 4), sticky="ew")
        save_as_row.grid_columnconfigure(0, weight=1)
        save_as_row.grid_columnconfigure(1, weight=0)

        self.save_model_name_entry = ctk.CTkEntry(
            save_as_row,
            placeholder_text="Tên model mới...",
            height=30
        )
        self.save_model_name_entry.grid(row=0, column=0, padx=(0, 4), sticky="ew")

        save_as_btn = ctk.CTkButton(
            save_as_row,
            text="💾 Lưu Mới",
            height=30,
            width=80,
            fg_color="#1eaa59",
            hover_color="#13773e",
            command=self._on_save_model_as
        )
        save_as_btn.grid(row=0, column=1, sticky="ew")

        # Export / Import row
        io_row = ctk.CTkFrame(preset_frame, fg_color="transparent")
        io_row.grid(row=4, column=0, padx=6, pady=(0, 8), sticky="ew")
        io_row.grid_columnconfigure((0, 1), weight=1)

        export_btn = ctk.CTkButton(
            io_row,
            text="⬆ Xuất JSON",
            height=28,
            fg_color="#555",
            hover_color="#333",
            font=("Arial", 11),
            command=self._on_export_model
        )
        export_btn.grid(row=0, column=0, padx=2, sticky="ew")

        import_btn = ctk.CTkButton(
            io_row,
            text="⬇ Nhập JSON",
            height=28,
            fg_color="#555",
            hover_color="#333",
            font=("Arial", 11),
            command=self._on_import_model
        )
        import_btn.grid(row=0, column=1, padx=2, sticky="ew")

        # 2. Selected Link Selection Frame
        link_sel_frame = ctk.CTkLabelFrame(self.links_scroll_frame, text="Khâu / Khớp Đang Chọn (Selected Link)")
        link_sel_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        link_sel_frame.grid_columnconfigure(0, weight=1)

        display_options = list(self.link_display_map.keys())
        self.link_selector = ctk.CTkOptionMenu(
            link_sel_frame,
            values=display_options,
            command=self._on_select_link
        )
        self.link_selector.grid(row=0, column=0, padx=8, pady=(8, 6), sticky="ew")

        # Action buttons row: Reset & Save
        btn_box = ctk.CTkFrame(link_sel_frame, fg_color="transparent")
        btn_box.grid(row=1, column=0, padx=6, pady=(0, 8), sticky="ew")
        btn_box.grid_columnconfigure((0, 1), weight=1)

        self.reset_link_btn = ctk.CTkButton(
            btn_box,
            text="🔄 Đặt lại khâu này",
            fg_color="#555",
            hover_color="#333",
            command=self._reset_selected_link_transform
        )
        self.reset_link_btn.grid(row=0, column=0, padx=3, pady=2, sticky="ew")

        self.save_links_btn = ctk.CTkButton(
            btn_box,
            text="💾 Lưu cấu hình khớp",
            fg_color="#1eaa59",
            hover_color="#13773e",
            command=self._save_robot_links_config
        )
        self.save_links_btn.grid(row=0, column=1, padx=3, pady=2, sticky="ew")

        # 3. STL File Management Card
        stl_card = ctk.CTkLabelFrame(self.links_scroll_frame, text="File STL 3D của Khớp (STL Geometry)")
        stl_card.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        stl_card.grid_columnconfigure(0, weight=1)

        self.link_stl_lbl = ctk.CTkLabel(
            stl_card,
            text="File STL: Link Base-1.STL",
            font=("Arial", 11),
            anchor="w",
            text_color="#3B8ED0"
        )
        self.link_stl_lbl.grid(row=0, column=0, columnspan=2, padx=8, pady=(6, 4), sticky="w")

        browse_stl_btn = ctk.CTkButton(
            stl_card,
            text="📁 Đổi File STL cho Khớp này (Browse)",
            height=32,
            fg_color="#1f538d",
            hover_color="#14375e",
            command=self._on_link_stl_browse
        )
        browse_stl_btn.grid(row=1, column=0, columnspan=2, padx=8, pady=(2, 8), sticky="ew")

        # 4. Position Translation Sliders (X, Y, Z mm)
        pos_frame = ctk.CTkLabelFrame(self.links_scroll_frame, text="Dịch chuyển Gốc STL (Position Offset - mm)")
        pos_frame.grid(row=3, column=0, padx=5, pady=5, sticky="ew")
        pos_frame.grid_columnconfigure(0, weight=1)

        self.link_pos_sliders = {}
        self.link_pos_entries = {}

        pos_axes = [
            ("X", -1000.0, 1000.0),
            ("Y", -1000.0, 1000.0),
            ("Z", -1000.0, 1500.0)
        ]

        for idx, (axis, min_val, max_val) in enumerate(pos_axes):
            axis_card = ctk.CTkFrame(pos_frame, fg_color="#2b2b2b", corner_radius=6)
            axis_card.grid(row=idx, column=0, padx=6, pady=4, sticky="ew")
            axis_card.grid_columnconfigure(1, weight=1)

            lbl = ctk.CTkLabel(axis_card, text=f"{axis}:", font=("Arial", 12, "bold"), width=22)
            lbl.grid(row=0, column=0, padx=(6, 2), pady=4, sticky="w")

            entry = ctk.CTkEntry(axis_card, width=70, height=26, justify="center", font=("Arial", 11, "bold"))
            entry.insert(0, "0.0")
            entry.grid(row=0, column=1, padx=2, pady=4, sticky="w")
            entry.bind("<Return>", lambda e: self._on_link_entry_update())
            entry.bind("<FocusOut>", lambda e: self._on_link_entry_update())
            self.link_pos_entries[axis] = entry

            unit_lbl = ctk.CTkLabel(axis_card, text="mm", font=("Arial", 10), text_color="gray")
            unit_lbl.grid(row=0, column=2, padx=(2, 4), pady=4, sticky="w")

            step_box = ctk.CTkFrame(axis_card, fg_color="transparent")
            step_box.grid(row=0, column=3, padx=2, pady=2, sticky="e")

            for step in [-10, -1, 1, 10]:
                text = f"{step:+d}" if step > 0 else str(step)
                btn = ctk.CTkButton(
                    step_box,
                    text=text,
                    width=30,
                    height=22,
                    font=("Arial", 9),
                    command=lambda a=axis, s=step: self._step_link_pos(a, s)
                )
                btn.pack(side="left", padx=1)

            slider = ctk.CTkSlider(
                axis_card,
                from_=min_val,
                to=max_val,
                height=16,
                command=lambda val, a=axis: self._on_link_pos_slider_move(a, val)
            )
            slider.set(0.0)
            slider.grid(row=1, column=0, columnspan=4, padx=6, pady=(2, 6), sticky="ew")
            self.link_pos_sliders[axis] = slider

        # 5. Rotation Sliders (Rx, Ry, Rz deg)
        rot_frame = ctk.CTkLabelFrame(self.links_scroll_frame, text="Góc xoay Ban đầu STL (Rotation - độ °)")
        rot_frame.grid(row=4, column=0, padx=5, pady=5, sticky="ew")
        rot_frame.grid_columnconfigure(0, weight=1)

        self.link_rot_sliders = {}
        self.link_rot_entries = {}

        rot_axes = [
            ("Rx", "Roll (X)"),
            ("Ry", "Pitch (Y)"),
            ("Rz", "Yaw (Z)")
        ]

        for idx, (axis, name) in enumerate(rot_axes):
            axis_card = ctk.CTkFrame(rot_frame, fg_color="#2b2b2b", corner_radius=6)
            axis_card.grid(row=idx, column=0, padx=6, pady=4, sticky="ew")
            axis_card.grid_columnconfigure(1, weight=1)

            lbl = ctk.CTkLabel(axis_card, text=f"{axis}:", font=("Arial", 12, "bold"), width=22)
            lbl.grid(row=0, column=0, padx=(6, 2), pady=4, sticky="w")

            entry = ctk.CTkEntry(axis_card, width=70, height=26, justify="center", font=("Arial", 11, "bold"))
            entry.insert(0, "0.0")
            entry.grid(row=0, column=1, padx=2, pady=4, sticky="w")
            entry.bind("<Return>", lambda e: self._on_link_entry_update())
            entry.bind("<FocusOut>", lambda e: self._on_link_entry_update())
            self.link_rot_entries[axis] = entry

            unit_lbl = ctk.CTkLabel(axis_card, text="°", font=("Arial", 11, "bold"), text_color="gray")
            unit_lbl.grid(row=0, column=2, padx=(2, 4), pady=4, sticky="w")

            step_box = ctk.CTkFrame(axis_card, fg_color="transparent")
            step_box.grid(row=0, column=3, padx=2, pady=2, sticky="e")

            for step in [-15, -1, 1, 15]:
                text = f"{step:+d}°" if step > 0 else f"{step}°"
                btn = ctk.CTkButton(
                    step_box,
                    text=text,
                    width=32,
                    height=22,
                    font=("Arial", 9),
                    command=lambda a=axis, s=step: self._step_link_rot(a, s)
                )
                btn.pack(side="left", padx=1)

            slider = ctk.CTkSlider(
                axis_card,
                from_=-180.0,
                to=180.0,
                height=16,
                command=lambda val, a=axis: self._on_link_rot_slider_move(a, val)
            )
            slider.set(0.0)
            slider.grid(row=1, column=0, columnspan=4, padx=6, pady=(2, 6), sticky="ew")
            self.link_rot_sliders[axis] = slider

        # 6. Joint Axis, Color, Scale Card
        prop_frame = ctk.CTkLabelFrame(self.links_scroll_frame, text="Trục quay & Hiển thị (Joint Axis & Style)")
        prop_frame.grid(row=5, column=0, padx=5, pady=5, sticky="ew")
        prop_frame.grid_columnconfigure(1, weight=1)

        # Joint Axis Menu
        ctk.CTkLabel(prop_frame, text="Trục quay khớp:", font=("Arial", 11, "bold")).grid(row=0, column=0, padx=8, pady=5, sticky="w")
        self.link_axis_menu = ctk.CTkOptionMenu(
            prop_frame,
            values=[
                "+Z (Quay quanh Z thuận)",
                "-Z (Quay quanh Z nghịch)",
                "+Y (Quay quanh Y thuận)",
                "-Y (Quay quanh Y nghịch)",
                "+X (Quay quanh X thuận)",
                "-X (Quay quanh X nghịch)",
                "None (Cố định)"
            ],
            command=self._on_link_axis_change
        )
        self.link_axis_menu.grid(row=0, column=1, padx=8, pady=5, sticky="ew")

        # Color Menu
        ctk.CTkLabel(prop_frame, text="Màu sắc:", font=("Arial", 11, "bold")).grid(row=1, column=0, padx=8, pady=5, sticky="w")
        self.link_color_menu = ctk.CTkOptionMenu(
            prop_frame,
            values=["Silver", "Orange", "DimGray", "SlateBlue", "LimeGreen", "Gold", "Crimson", "Cyan", "White"],
            command=self._on_link_color_change
        )
        self.link_color_menu.grid(row=1, column=1, padx=8, pady=5, sticky="ew")

        # Scale Slider
        ctk.CTkLabel(prop_frame, text="Tỉ lệ (Scale):", font=("Arial", 11, "bold")).grid(row=2, column=0, padx=8, pady=5, sticky="w")
        s_box = ctk.CTkFrame(prop_frame, fg_color="transparent")
        s_box.grid(row=2, column=1, padx=8, pady=5, sticky="ew")
        s_box.grid_columnconfigure(0, weight=1)

        self.link_scale_slider = ctk.CTkSlider(
            s_box,
            from_=0.1,
            to=5.0,
            height=16,
            command=self._on_link_scale_slider_move
        )
        self.link_scale_slider.set(1.0)
        self.link_scale_slider.grid(row=0, column=0, padx=(0, 5), sticky="ew")

        self.link_scale_lbl = ctk.CTkLabel(s_box, text="1.00x", width=45, font=("Arial", 11, "bold"))
        self.link_scale_lbl.grid(row=0, column=1, sticky="e")

        # Load initial values for "Base"
        self._load_selected_link_to_ui()

    def _on_select_link(self, display_name):
        """Callback when user selects a link in the dropdown."""
        link_key = self.link_display_map.get(display_name, "Base")
        self.selected_link_key = link_key
        self._load_selected_link_to_ui()

    def _load_selected_link_to_ui(self):
        """Populates sliders, entries, and options with the selected link's configuration."""
        cfg = self.config.get_link_config(self.selected_link_key)
        self._updating_link_ui = True

        # Update STL label
        stl_list = cfg.get("stl_files", [])
        stl_names = ", ".join([os.path.basename(f) for f in stl_list]) if stl_list else "(Chưa có STL)"
        self.link_stl_lbl.configure(text=f"File STL: {stl_names}")

        # Update Position
        pos = cfg.get("offset_pos", [0.0, 0.0, 0.0])
        for idx, axis in enumerate(["X", "Y", "Z"]):
            val = pos[idx]
            self.link_pos_entries[axis].delete(0, "end")
            self.link_pos_entries[axis].insert(0, f"{val:.2f}")
            if axis in ["X", "Y"]:
                self.link_pos_sliders[axis].set(max(-1000.0, min(1000.0, val)))
            else:
                self.link_pos_sliders[axis].set(max(-1000.0, min(1500.0, val)))

        # Update Rotation
        rot = cfg.get("offset_rot", [0.0, 0.0, 0.0])
        for idx, axis in enumerate(["Rx", "Ry", "Rz"]):
            val = rot[idx]
            self.link_rot_entries[axis].delete(0, "end")
            self.link_rot_entries[axis].insert(0, f"{val:.2f}")
            self.link_rot_sliders[axis].set(max(-180.0, min(180.0, val)))

        # Update Axis
        axis_raw = cfg.get("joint_axis", "None" if self.selected_link_key == "Base" else "+Z")
        for val in self.link_axis_menu._values:
            if val.startswith(axis_raw):
                self.link_axis_menu.set(val)
                break
        else:
            self.link_axis_menu.set("+Z (Quay quanh Z thuận)")

        # Update Color
        color = cfg.get("color", "Silver")
        self.link_color_menu.set(color)

        # Update Scale
        scale = cfg.get("scale", 1.0)
        self.link_scale_slider.set(scale)
        self.link_scale_lbl.configure(text=f"{scale:.2f}x")

        self._updating_link_ui = False

    def _on_link_stl_browse(self):
        """Opens file dialog to choose an STL file for the current link."""
        from tkinter import filedialog, messagebox
        file_path = filedialog.askopenfilename(
            title=f"Chọn file STL cho {self.selected_link_key}",
            filetypes=[("STL 3D Model", "*.stl *.STL"), ("All Files", "*.*")]
        )
        if not file_path:
            return
        if not os.path.exists(file_path):
            messagebox.showerror("Lỗi", f"Không tìm thấy file: {file_path}")
            return
            
        self.viewer.update_link_stl(self.selected_link_key, [file_path])
        self._load_selected_link_to_ui()
        messagebox.showinfo("Thành công", f"Đã nạp file STL '{os.path.basename(file_path)}' cho {self.selected_link_key}!")

    def _on_link_pos_slider_move(self, axis, val):
        """Callback when link position slider moves."""
        if self._updating_link_ui:
            return
        cfg = self.config.get_link_config(self.selected_link_key)
        axis_idx = {"X": 0, "Y": 1, "Z": 2}[axis]
        pos = list(cfg.get("offset_pos", [0.0, 0.0, 0.0]))
        pos[axis_idx] = float(val)

        self._updating_link_ui = True
        self.link_pos_entries[axis].delete(0, "end")
        self.link_pos_entries[axis].insert(0, f"{val:.2f}")
        self._updating_link_ui = False

        self.viewer.update_link_offset(self.selected_link_key, pos=pos)

    def _on_link_rot_slider_move(self, axis, val):
        """Callback when link rotation slider moves."""
        if self._updating_link_ui:
            return
        cfg = self.config.get_link_config(self.selected_link_key)
        axis_idx = {"Rx": 0, "Ry": 1, "Rz": 2}[axis]
        rot = list(cfg.get("offset_rot", [0.0, 0.0, 0.0]))
        rot[axis_idx] = float(val)

        self._updating_link_ui = True
        self.link_rot_entries[axis].delete(0, "end")
        self.link_rot_entries[axis].insert(0, f"{val:.2f}")
        self._updating_link_ui = False

        self.viewer.update_link_offset(self.selected_link_key, rot=rot)

    def _on_link_entry_update(self):
        """Callback when user edits numeric entries for the link."""
        if self._updating_link_ui:
            return
        try:
            x = float(self.link_pos_entries["X"].get().strip())
            y = float(self.link_pos_entries["Y"].get().strip())
            z = float(self.link_pos_entries["Z"].get().strip())
            rx = float(self.link_rot_entries["Rx"].get().strip())
            ry = float(self.link_rot_entries["Ry"].get().strip())
            rz = float(self.link_rot_entries["Rz"].get().strip())
        except ValueError:
            return

        self._updating_link_ui = True
        self.link_pos_sliders["X"].set(max(-1000.0, min(1000.0, x)))
        self.link_pos_sliders["Y"].set(max(-1000.0, min(1000.0, y)))
        self.link_pos_sliders["Z"].set(max(-1000.0, min(1500.0, z)))
        self.link_rot_sliders["Rx"].set(max(-180.0, min(180.0, rx)))
        self.link_rot_sliders["Ry"].set(max(-180.0, min(180.0, ry)))
        self.link_rot_sliders["Rz"].set(max(-180.0, min(180.0, rz)))
        self._updating_link_ui = False

        self.viewer.update_link_offset(
            self.selected_link_key,
            pos=[x, y, z],
            rot=[rx, ry, rz]
        )

    def _step_link_pos(self, axis, delta):
        """Steps position by fixed offset."""
        cfg = self.config.get_link_config(self.selected_link_key)
        axis_idx = {"X": 0, "Y": 1, "Z": 2}[axis]
        pos = list(cfg.get("offset_pos", [0.0, 0.0, 0.0]))
        pos[axis_idx] += float(delta)

        self._updating_link_ui = True
        self.link_pos_entries[axis].delete(0, "end")
        self.link_pos_entries[axis].insert(0, f"{pos[axis_idx]:.2f}")
        if axis in ["X", "Y"]:
            self.link_pos_sliders[axis].set(max(-1000.0, min(1000.0, pos[axis_idx])))
        else:
            self.link_pos_sliders[axis].set(max(-1000.0, min(1500.0, pos[axis_idx])))
        self._updating_link_ui = False

        self.viewer.update_link_offset(self.selected_link_key, pos=pos)

    def _step_link_rot(self, axis, delta):
        """Steps rotation angle by fixed step."""
        cfg = self.config.get_link_config(self.selected_link_key)
        axis_idx = {"Rx": 0, "Ry": 1, "Rz": 2}[axis]
        rot = list(cfg.get("offset_rot", [0.0, 0.0, 0.0]))
        new_val = rot[axis_idx] + float(delta)
        while new_val > 180.0:
            new_val -= 360.0
        while new_val < -180.0:
            new_val += 360.0
        rot[axis_idx] = new_val

        self._updating_link_ui = True
        self.link_rot_entries[axis].delete(0, "end")
        self.link_rot_entries[axis].insert(0, f"{new_val:.2f}")
        self.link_rot_sliders[axis].set(new_val)
        self._updating_link_ui = False

        self.viewer.update_link_offset(self.selected_link_key, rot=rot)

    def _on_link_axis_change(self, axis_val):
        """Callback when user changes joint axis."""
        axis_code = axis_val.split()[0] if axis_val else "+Z"
        self.viewer.update_link_joint_axis(self.selected_link_key, axis_code)

    def _on_link_color_change(self, color_val):
        """Callback when user changes link color."""
        self.viewer.update_link_color(self.selected_link_key, color_val)

    def _on_link_scale_slider_move(self, val):
        """Callback when user moves link scale slider."""
        if self._updating_link_ui:
            return
        s_val = float(val)
        self.link_scale_lbl.configure(text=f"{s_val:.2f}x")
        self.viewer.update_link_offset(self.selected_link_key, scale=s_val)

    def _save_robot_links_config(self):
        """Persists current robot links configuration to defaults.json."""
        from tkinter import messagebox
        self.config.save_config()
        messagebox.showinfo("Thành công", "Đã lưu cấu hình cơ cấu và khớp robot thành công!")

    def _reset_selected_link_transform(self):
        """Resets the transform for the selected link to zero."""
        self.viewer.reset_link_transform(self.selected_link_key)
        self._load_selected_link_to_ui()

    # --- Robot Model Profile Management ---

    def _refresh_model_list_ui(self):
        """Refreshes model dropdown list and active model label."""
        model_names = list(self.config.get_saved_models().keys())
        active = self.config.get_active_model_name()
        if hasattr(self, "preset_menu"):
            self.preset_menu.configure(values=model_names)
            self.preset_menu.set(active)
        if hasattr(self, "config_preset_menu"):
            self.config_preset_menu.configure(values=model_names)
            self.config_preset_menu.set(active)
        if hasattr(self, "active_model_lbl"):
            self.active_model_lbl.configure(text=f"Đang dùng: {active}")

    def _on_new_model(self):
        """Creates a blank new robot model: clears 3D scene and resets all 7 link STL slots."""
        from tkinter import messagebox, simpledialog
        
        name = simpledialog.askstring(
            "Tạo Model Mới",
            "Nhập tên model mới:\n(Sẽ xóa sạch toàn bộ STL hiện tại trong 3D viewer)",
            initialvalue="Robot Model Mới"
        )
        if not name or not name.strip():
            return
        name = name.strip()

        # Build blank 7-link template — no STL files, all offsets zero
        blank_links = {
            "Base":   {"name": "Base (Khớp đế)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "None", "color": "Silver", "scale": 1.0, "opacity": 1.0},
            "Link 1": {"name": "Khớp 1 (J1)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "+Z", "color": "Silver", "scale": 1.0, "opacity": 1.0},
            "Link 2": {"name": "Khớp 2 (J2)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "+Z", "color": "Silver", "scale": 1.0, "opacity": 1.0},
            "Link 3": {"name": "Khớp 3 (J3)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "+Y", "color": "Silver", "scale": 1.0, "opacity": 1.0},
            "Link 4": {"name": "Khớp 4 (J4)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "+Y", "color": "Silver", "scale": 1.0, "opacity": 1.0},
            "Link 5": {"name": "Khớp 5 (J5)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "+X", "color": "Silver", "scale": 1.0, "opacity": 1.0},
            "Link 6": {"name": "Khớp 6 (J6)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "+Y", "color": "Silver", "scale": 1.0, "opacity": 1.0},
        }

        # Write blank model into config
        self.config.config_data["current_robot_model"] = name
        self.config.config_data["current_robot_model_type"] = "custom"
        self.config.config_data["robot_links"] = blank_links
        
        # Save it immediately as a user model
        self.config.save_current_as_model(name, description=f"Model trống tạo mới: {name}")

        # Clear 3D scene entirely
        self.viewer.clear_robot()
        if self.viewer.render_window:
            self.viewer.render_window.Render()

        # Reset UI
        self._refresh_model_list_ui()
        self._load_selected_link_to_ui()
        self._reload_dh_entries_from_config()
        self._update_coordinates()

        messagebox.showinfo(
            "Tạo Model Mới",
            f"Đã tạo model trống '{name}'.\n\n"
            "Bây giờ hãy:\n"
            "1. Chọn từng Khớp 1→6 bên dưới\n"
            "2. Bấm '📁 Đổi File STL' để gắn file STL\n"
            "3. Kéo slider để căn chỉnh vị trí/góc xoay\n"
            "4. Nhập tên và bấm '💾 Lưu Mới' để lưu lại"
        )

    def _on_load_model(self):
        """Loads selected robot model into active configuration."""
        from tkinter import messagebox
        name = self.preset_menu.get()
        if not name or name == "(Chưa có model)":
            messagebox.showwarning("Cảnh báo", "Vui lòng chọn model cần mở!")
            return
        if self.config.load_robot_model(name):
            self.kinematics.initialize_kinematics()
            self.viewer.reload_robot()
            self._load_selected_link_to_ui()
            self._reload_dh_entries_from_config()
            self._update_coordinates()
            self._refresh_model_list_ui()
            messagebox.showinfo("Thành công", f"Đã mở model '{name}'!")
        else:
            messagebox.showerror("Lỗi", f"Không thể mở model '{name}'")

    def _on_save_model_as(self):
        """Saves current state as a new named robot model."""
        from tkinter import messagebox
        name = self.save_model_name_entry.get().strip() if hasattr(self, "save_model_name_entry") else ""
        if not name:
            messagebox.showwarning("Cảnh báo", "Vui lòng nhập tên cho model mới!")
            return
        # Confirm overwrite if exists
        existing = self.config.get_saved_models()
        if name in existing:
            ok = messagebox.askyesno("Xác nhận", f"Model '{name}' đã tồn tại. Ghi đè?")
            if not ok:
                return
        if self.config.save_current_as_model(name, description=f"Model tùy chỉnh: {name}"):
            self._refresh_model_list_ui()
            if hasattr(self, "save_model_name_entry"):
                self.save_model_name_entry.delete(0, "end")
            messagebox.showinfo("Thành công", f"Đã lưu model '{name}' thành công!")
        else:
            messagebox.showerror("Lỗi", "Không thể lưu model!")

    def _on_delete_model(self):
        """Deletes selected custom robot model."""
        from tkinter import messagebox
        name = self.preset_menu.get()
        if not name or name == "(Chưa có model)":
            messagebox.showwarning("Cảnh báo", "Vui lòng chọn model cần xóa!")
            return
        ok = messagebox.askyesno("Xác nhận xóa", f"Bạn có chắc muốn xóa model '{name}'?\n(Không thể xóa model hệ thống)")
        if not ok:
            return
        success, msg = self.config.delete_saved_model(name)
        if success:
            self._refresh_model_list_ui()
            self.viewer.reload_robot()
            self._load_selected_link_to_ui()
            self._reload_dh_entries_from_config()
        messagebox.showinfo("Kết quả", msg)

    def _on_export_model(self):
        """Exports selected robot model to a JSON file."""
        from tkinter import filedialog, messagebox
        name = self.preset_menu.get()
        if not name or name == "(Chưa có model)":
            messagebox.showwarning("Cảnh báo", "Vui lòng chọn model cần xuất!")
            return
        file_path = filedialog.asksaveasfilename(
            title=f"Xuất model '{name}'...",
            defaultextension=".json",
            initialfile=name.replace(" ", "_") + ".json",
            filetypes=[("Robot Model JSON", "*.json"), ("All Files", "*.*")]
        )
        if not file_path:
            return
        if self.config.export_model_to_json(name, file_path):
            messagebox.showinfo("Thành công", f"Đã xuất model ra:\n{file_path}")
        else:
            messagebox.showerror("Lỗi", "Không thể xuất file JSON!")

    def _on_import_model(self):
        """Imports a robot model from a JSON file."""
        from tkinter import filedialog, messagebox
        file_path = filedialog.askopenfilename(
            title="Nhập file model robot JSON...",
            filetypes=[("Robot Model JSON", "*.json"), ("All Files", "*.*")]
        )
        if not file_path:
            return
        success, result = self.config.import_model_from_json(file_path)
        if success:
            self._refresh_model_list_ui()
            messagebox.showinfo("Thành công", f"Đã nhập model '{result}' thành công!")
        else:
            messagebox.showerror("Lỗi", result)

    def _on_preset_selected(self, preset_name):
        """Backward compat: loads a model by name (used by Config tab preset menus)."""
        from tkinter import messagebox
        if self.config.load_robot_model(preset_name):
            self.kinematics.initialize_kinematics()
            self.viewer.reload_robot()
            self._load_selected_link_to_ui()
            self._reload_dh_entries_from_config()
            self._update_coordinates()
            self._refresh_model_list_ui()
            messagebox.showinfo("Model", f"Đã nạp model '{preset_name}'!")
        else:
            messagebox.showerror("Lỗi", f"Không thể nạp model '{preset_name}'")

    def _reload_dh_entries_from_config(self):
        """Refreshes DH parameter entry boxes in the Config tab."""
        param_keys = ["Θ", "α", "d", "a"]
        for row_idx in range(6):
            for param in param_keys:
                if hasattr(self, "dh_entries") and (row_idx+1, param) in self.dh_entries:
                    cfg_key = f"J{row_idx+1}{param}DHpar"
                    val = self.config.get(cfg_key, "0.0")
                    entry = self.dh_entries[(row_idx+1, param)]
                    entry.delete(0, "end")
                    entry.insert(0, str(val))

    # -------------------------------------------------------------------------
    # Custom STL 3D Objects Management & UI (Quản lý vật thể 3D STL tùy chỉnh)
    # -------------------------------------------------------------------------
    def _load_saved_custom_objects(self):
        """Loads saved custom STL objects configuration from ConfigManager."""
        saved = self.config.get("custom_stl_objects", [])
        if isinstance(saved, list):
            for obj in saved:
                if isinstance(obj, dict) and "id" in obj and "file_path" in obj:
                    if os.path.exists(obj["file_path"]):
                        self.viewer.add_custom_object(
                            obj_id=obj["id"],
                            file_path=obj["file_path"],
                            name=obj.get("name"),
                            position=obj.get("position", [0.0, 0.0, 0.0]),
                            rotation=obj.get("rotation", [0.0, 0.0, 0.0]),
                            scale=obj.get("scale", 1.0),
                            color=obj.get("color", "LimeGreen"),
                            opacity=obj.get("opacity", 1.0),
                            parent=obj.get("parent", "World (Tọa độ thế giới)"),
                            visible=obj.get("visible", True)
                        )

    def _save_custom_objects(self):
        """Saves current custom STL objects configuration to ConfigManager."""
        saved_list = []
        for obj_id, obj in self.viewer.custom_objects.items():
            saved_list.append({
                "id": obj["id"],
                "name": obj["name"],
                "file_path": obj["file_path"],
                "position": obj["position"],
                "rotation": obj["rotation"],
                "scale": obj["scale"],
                "color": obj["color"],
                "opacity": obj["opacity"],
                "parent": obj["parent"],
                "visible": obj["visible"]
            })
        self.config.set("custom_stl_objects", saved_list)

    def _create_objects_tab_widgets(self):
        """Constructs the UI for adding, inspecting, and manipulating custom STL objects."""
        # Scrollable container for object controls
        self.obj_scroll_frame = ctk.CTkScrollableFrame(self.tab_objects, fg_color="transparent")
        self.obj_scroll_frame.pack(fill="both", expand=True, padx=5, pady=5)
        self.obj_scroll_frame.grid_columnconfigure(0, weight=1)

        # 1. Add STL file button
        add_btn = ctk.CTkButton(
            self.obj_scroll_frame,
            text="📁 Thêm file STL (Add STL File)",
            font=("Arial", 13, "bold"),
            height=38,
            fg_color="#1f538d",
            hover_color="#14375e",
            command=self._on_add_stl_file
        )
        add_btn.grid(row=0, column=0, padx=5, pady=(5, 8), sticky="ew")

        # 2. Selected Object Management Frame
        sel_frame = ctk.CTkLabelFrame(self.obj_scroll_frame, text="Vật thể đang chọn (Selected Object)")
        sel_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        sel_frame.grid_columnconfigure(0, weight=1)

        # Object dropdown
        self.obj_selector = ctk.CTkOptionMenu(
            sel_frame,
            values=["(Chưa có vật thể nào)"],
            command=self._on_select_object
        )
        self.obj_selector.grid(row=0, column=0, columnspan=3, padx=8, pady=(8, 6), sticky="ew")

        # Action buttons row
        btn_frame = ctk.CTkFrame(sel_frame, fg_color="transparent")
        btn_frame.grid(row=1, column=0, columnspan=3, padx=5, pady=(0, 8), sticky="ew")
        btn_frame.grid_columnconfigure((0, 1, 2), weight=1)

        self.obj_vis_btn = ctk.CTkButton(
            btn_frame,
            text="👁️ Ẩn/Hiện",
            width=70,
            command=self._toggle_object_visibility
        )
        self.obj_vis_btn.grid(row=0, column=0, padx=2, pady=2, sticky="ew")

        self.obj_reset_btn = ctk.CTkButton(
            btn_frame,
            text="🔄 Đặt lại",
            width=70,
            command=self._reset_object_transform
        )
        self.obj_reset_btn.grid(row=0, column=1, padx=2, pady=2, sticky="ew")

        self.obj_del_btn = ctk.CTkButton(
            btn_frame,
            text="🗑️ Xóa",
            width=70,
            fg_color="#c0392b",
            hover_color="#962d22",
            command=self._delete_object
        )
        self.obj_del_btn.grid(row=0, column=2, padx=2, pady=2, sticky="ew")

        # 3. Position Controls Frame (X, Y, Z mm)
        pos_frame = ctk.CTkLabelFrame(self.obj_scroll_frame, text="Điều chỉnh Tọa độ (Position - mm)")
        pos_frame.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        pos_frame.grid_columnconfigure(0, weight=1)

        self.pos_sliders = {}
        self.pos_entries = {}
        
        pos_axes = [
            ("X", -1000.0, 1000.0),
            ("Y", -1000.0, 1000.0),
            ("Z", -500.0, 1500.0)
        ]
        
        for idx, (axis, min_val, max_val) in enumerate(pos_axes):
            axis_card = ctk.CTkFrame(pos_frame, fg_color="#2b2b2b", corner_radius=6)
            axis_card.grid(row=idx, column=0, padx=6, pady=4, sticky="ew")
            axis_card.grid_columnconfigure(1, weight=1)

            # Header row: Axis label + Entry + Unit
            lbl = ctk.CTkLabel(axis_card, text=f"{axis}:", font=("Arial", 12, "bold"), width=22)
            lbl.grid(row=0, column=0, padx=(6, 2), pady=4, sticky="w")

            entry = ctk.CTkEntry(axis_card, width=70, height=26, justify="center", font=("Arial", 11, "bold"))
            entry.insert(0, "0.0")
            entry.grid(row=0, column=1, padx=2, pady=4, sticky="w")
            entry.bind("<Return>", lambda e: self._on_entry_update())
            entry.bind("<FocusOut>", lambda e: self._on_entry_update())
            self.pos_entries[axis] = entry

            unit_lbl = ctk.CTkLabel(axis_card, text="mm", font=("Arial", 10), text_color="gray")
            unit_lbl.grid(row=0, column=2, padx=(2, 4), pady=4, sticky="w")

            # Quick step buttons row inside header
            step_box = ctk.CTkFrame(axis_card, fg_color="transparent")
            step_box.grid(row=0, column=3, padx=2, pady=2, sticky="e")
            
            for step in [-10, -1, 1, 10]:
                text = f"{step:+d}" if step > 0 else str(step)
                btn = ctk.CTkButton(
                    step_box,
                    text=text,
                    width=30,
                    height=22,
                    font=("Arial", 9),
                    command=lambda a=axis, s=step: self._step_pos(a, s)
                )
                btn.pack(side="left", padx=1)

            # Slider
            slider = ctk.CTkSlider(
                axis_card,
                from_=min_val,
                to=max_val,
                height=16,
                command=lambda val, a=axis: self._on_pos_slider_move(a, val)
            )
            slider.set(0.0)
            slider.grid(row=1, column=0, columnspan=4, padx=6, pady=(2, 6), sticky="ew")
            self.pos_sliders[axis] = slider

        # 4. Rotation Controls Frame (Rx, Ry, Rz deg)
        rot_frame = ctk.CTkLabelFrame(self.obj_scroll_frame, text="Điều chỉnh Góc quay (Rotation - độ °)")
        rot_frame.grid(row=3, column=0, padx=5, pady=5, sticky="ew")
        rot_frame.grid_columnconfigure(0, weight=1)

        self.rot_sliders = {}
        self.rot_entries = {}
        
        rot_axes = [
            ("Rx", "Roll (X)"),
            ("Ry", "Pitch (Y)"),
            ("Rz", "Yaw (Z)")
        ]
        
        for idx, (axis, name) in enumerate(rot_axes):
            axis_card = ctk.CTkFrame(rot_frame, fg_color="#2b2b2b", corner_radius=6)
            axis_card.grid(row=idx, column=0, padx=6, pady=4, sticky="ew")
            axis_card.grid_columnconfigure(1, weight=1)

            lbl = ctk.CTkLabel(axis_card, text=f"{axis}:", font=("Arial", 12, "bold"), width=22)
            lbl.grid(row=0, column=0, padx=(6, 2), pady=4, sticky="w")

            entry = ctk.CTkEntry(axis_card, width=70, height=26, justify="center", font=("Arial", 11, "bold"))
            entry.insert(0, "0.0")
            entry.grid(row=0, column=1, padx=2, pady=4, sticky="w")
            entry.bind("<Return>", lambda e: self._on_entry_update())
            entry.bind("<FocusOut>", lambda e: self._on_entry_update())
            self.rot_entries[axis] = entry

            unit_lbl = ctk.CTkLabel(axis_card, text="°", font=("Arial", 11, "bold"), text_color="gray")
            unit_lbl.grid(row=0, column=2, padx=(2, 4), pady=4, sticky="w")

            # Quick step buttons row for rotation
            step_box = ctk.CTkFrame(axis_card, fg_color="transparent")
            step_box.grid(row=0, column=3, padx=2, pady=2, sticky="e")
            
            for step in [-15, -1, 1, 15]:
                text = f"{step:+d}°" if step > 0 else f"{step}°"
                btn = ctk.CTkButton(
                    step_box,
                    text=text,
                    width=32,
                    height=22,
                    font=("Arial", 9),
                    command=lambda a=axis, s=step: self._step_rot(a, s)
                )
                btn.pack(side="left", padx=1)

            slider = ctk.CTkSlider(
                axis_card,
                from_=-180.0,
                to=180.0,
                height=16,
                command=lambda val, a=axis: self._on_rot_slider_move(a, val)
            )
            slider.set(0.0)
            slider.grid(row=1, column=0, columnspan=4, padx=6, pady=(2, 6), sticky="ew")
            self.rot_sliders[axis] = slider

        # 5. Attributes & Attachment Frame
        attr_frame = ctk.CTkLabelFrame(self.obj_scroll_frame, text="Gắn kết & Hiển thị (Parent & Visuals)")
        attr_frame.grid(row=4, column=0, padx=5, pady=5, sticky="ew")
        attr_frame.grid_columnconfigure(1, weight=1)

        # Parent attachment option
        ctk.CTkLabel(attr_frame, text="Gắn vào (Parent):", font=("Arial", 11, "bold")).grid(row=0, column=0, padx=8, pady=5, sticky="w")
        self.parent_menu = ctk.CTkOptionMenu(
            attr_frame,
            values=[
                "World (Tọa độ thế giới)",
                "End-Effector (Link 6 - Đầu kẹp)",
                "Link 5",
                "Link 4",
                "Link 3",
                "Link 2",
                "Link 1",
                "Base (Đế robot)"
            ],
            command=self._on_parent_change
        )
        self.parent_menu.grid(row=0, column=1, padx=8, pady=5, sticky="ew")

        # Color dropdown
        ctk.CTkLabel(attr_frame, text="Màu sắc (Color):", font=("Arial", 11, "bold")).grid(row=1, column=0, padx=8, pady=5, sticky="w")
        self.color_menu = ctk.CTkOptionMenu(
            attr_frame,
            values=[
                "LimeGreen",
                "Crimson",
                "DeepSkyBlue",
                "Gold",
                "OrangeRed",
                "MediumOrchid",
                "Silver",
                "DarkOrange",
                "Cyan",
                "White",
                "DimGray"
            ],
            command=self._on_color_change
        )
        self.color_menu.grid(row=1, column=1, padx=8, pady=5, sticky="ew")

        # Scale slider
        ctk.CTkLabel(attr_frame, text="Tỉ lệ (Scale):", font=("Arial", 11, "bold")).grid(row=2, column=0, padx=8, pady=5, sticky="w")
        scale_box = ctk.CTkFrame(attr_frame, fg_color="transparent")
        scale_box.grid(row=2, column=1, padx=8, pady=5, sticky="ew")
        scale_box.grid_columnconfigure(0, weight=1)
        
        self.scale_slider = ctk.CTkSlider(
            scale_box,
            from_=0.1,
            to=5.0,
            height=16,
            command=self._on_scale_slider_move
        )
        self.scale_slider.set(1.0)
        self.scale_slider.grid(row=0, column=0, padx=(0, 5), sticky="ew")
        
        self.scale_lbl = ctk.CTkLabel(scale_box, text="1.00x", width=45, font=("Arial", 11, "bold"))
        self.scale_lbl.grid(row=0, column=1, sticky="e")

        # Opacity slider
        ctk.CTkLabel(attr_frame, text="Độ trong suốt:", font=("Arial", 11, "bold")).grid(row=3, column=0, padx=8, pady=(5, 8), sticky="w")
        opacity_box = ctk.CTkFrame(attr_frame, fg_color="transparent")
        opacity_box.grid(row=3, column=1, padx=8, pady=(5, 8), sticky="ew")
        opacity_box.grid_columnconfigure(0, weight=1)
        
        self.opacity_slider = ctk.CTkSlider(
            opacity_box,
            from_=0.1,
            to=1.0,
            height=16,
            command=self._on_opacity_slider_move
        )
        self.opacity_slider.set(1.0)
        self.opacity_slider.grid(row=0, column=0, padx=(0, 5), sticky="ew")
        
        self.opacity_lbl = ctk.CTkLabel(opacity_box, text="100%", width=45, font=("Arial", 11, "bold"))
        self.opacity_lbl.grid(row=0, column=1, sticky="e")

        # Refresh UI state with any loaded objects
        self._refresh_objects_list_ui()

    def _refresh_objects_list_ui(self):
        """Updates the dropdown and loads currently selected object details."""
        if not self.viewer.custom_objects:
            self.obj_selector.configure(values=["(Chưa có vật thể nào)"])
            self.obj_selector.set("(Chưa có vật thể nào)")
            self.selected_obj_id = None
            self._updating_obj_ui = True
            for axis in ["X", "Y", "Z"]:
                self.pos_entries[axis].delete(0, "end")
                self.pos_entries[axis].insert(0, "0.0")
                self.pos_sliders[axis].set(0.0)
            for axis in ["Rx", "Ry", "Rz"]:
                self.rot_entries[axis].delete(0, "end")
                self.rot_entries[axis].insert(0, "0.0")
                self.rot_sliders[axis].set(0.0)
            self.scale_slider.set(1.0)
            self.scale_lbl.configure(text="1.00x")
            self.opacity_slider.set(1.0)
            self.opacity_lbl.configure(text="100%")
            self._updating_obj_ui = False
            return

        names = [obj["name"] for obj in self.viewer.custom_objects.values()]
        self.obj_selector.configure(values=names)

        if not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            self.selected_obj_id = list(self.viewer.custom_objects.keys())[0]

        selected_name = self.viewer.custom_objects[self.selected_obj_id]["name"]
        self.obj_selector.set(selected_name)
        self._load_selected_obj_to_ui()

    def _on_select_object(self, selected_name):
        """Callback when an object is chosen in the dropdown."""
        for obj_id, obj in self.viewer.custom_objects.items():
            if obj["name"] == selected_name:
                self.selected_obj_id = obj_id
                self._load_selected_obj_to_ui()
                break

    def _load_selected_obj_to_ui(self):
        """Populates sliders, entries, and visual dropdowns with the selected object's data."""
        if not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            return

        obj = self.viewer.custom_objects[self.selected_obj_id]
        self._updating_obj_ui = True

        pos = obj.get("position", [0.0, 0.0, 0.0])
        for idx, axis in enumerate(["X", "Y", "Z"]):
            val = pos[idx]
            self.pos_entries[axis].delete(0, "end")
            self.pos_entries[axis].insert(0, f"{val:.2f}")
            if axis in ["X", "Y"]:
                self.pos_sliders[axis].set(max(-1000.0, min(1000.0, val)))
            else:
                self.pos_sliders[axis].set(max(-500.0, min(1500.0, val)))

        rot = obj.get("rotation", [0.0, 0.0, 0.0])
        for idx, axis in enumerate(["Rx", "Ry", "Rz"]):
            val = rot[idx]
            self.rot_entries[axis].delete(0, "end")
            self.rot_entries[axis].insert(0, f"{val:.2f}")
            self.rot_sliders[axis].set(max(-180.0, min(180.0, val)))

        scale = obj.get("scale", 1.0)
        self.scale_slider.set(scale)
        self.scale_lbl.configure(text=f"{scale:.2f}x")

        opacity = obj.get("opacity", 1.0)
        self.opacity_slider.set(opacity)
        self.opacity_lbl.configure(text=f"{int(opacity*100)}%")

        color = obj.get("color", "LimeGreen")
        self.color_menu.set(color)

        parent = obj.get("parent", "World (Tọa độ thế giới)")
        self.parent_menu.set(parent)

        vis = obj.get("visible", True)
        self.obj_vis_btn.configure(text="👁️ Ẩn" if vis else "👁️ Hiện")

        self._updating_obj_ui = False

    def _on_add_stl_file(self):
        """Opens file dialog to browse for an STL file and adds it to the scene."""
        from tkinter import filedialog, messagebox
        file_path = filedialog.askopenfilename(
            title="Chọn file STL 3D",
            filetypes=[("STL 3D Model", "*.stl *.STL"), ("All Files", "*.*")]
        )
        if not file_path:
            return

        if not os.path.exists(file_path):
            messagebox.showerror("Lỗi", f"Không tìm thấy file: {file_path}")
            return

        base_name = os.path.splitext(os.path.basename(file_path))[0]
        existing_names = [obj.get("name") for obj in self.viewer.custom_objects.values()]
        unique_name = base_name
        counter = 1
        while unique_name in existing_names:
            unique_name = f"{base_name}_{counter}"
            counter += 1

        obj_id = f"stl_{int(time.time()*1000)}"
        self.viewer.add_custom_object(
            obj_id=obj_id,
            file_path=file_path,
            name=unique_name,
            position=[0.0, 0.0, 0.0],
            rotation=[0.0, 0.0, 0.0],
            scale=1.0,
            color="LimeGreen",
            opacity=1.0,
            parent="World (Tọa độ thế giới)",
            visible=True
        )
        self.selected_obj_id = obj_id
        self._save_custom_objects()
        self._refresh_objects_list_ui()

    def _on_pos_slider_move(self, axis, val):
        """Callback when position slider moves."""
        if self._updating_obj_ui or not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            return
        obj = self.viewer.custom_objects[self.selected_obj_id]
        axis_idx = {"X": 0, "Y": 1, "Z": 2}[axis]
        pos = list(obj["position"])
        pos[axis_idx] = float(val)

        self._updating_obj_ui = True
        self.pos_entries[axis].delete(0, "end")
        self.pos_entries[axis].insert(0, f"{val:.2f}")
        self._updating_obj_ui = False

        self.viewer.update_custom_object(self.selected_obj_id, position=pos)
        self._save_custom_objects()

    def _on_rot_slider_move(self, axis, val):
        """Callback when rotation slider moves."""
        if self._updating_obj_ui or not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            return
        obj = self.viewer.custom_objects[self.selected_obj_id]
        axis_idx = {"Rx": 0, "Ry": 1, "Rz": 2}[axis]
        rot = list(obj["rotation"])
        rot[axis_idx] = float(val)

        self._updating_obj_ui = True
        self.rot_entries[axis].delete(0, "end")
        self.rot_entries[axis].insert(0, f"{val:.2f}")
        self._updating_obj_ui = False

        self.viewer.update_custom_object(self.selected_obj_id, rotation=rot)
        self._save_custom_objects()

    def _on_entry_update(self):
        """Callback when user edits numeric entries and presses Enter or leaves focus."""
        if self._updating_obj_ui or not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            return
        try:
            x = float(self.pos_entries["X"].get().strip())
            y = float(self.pos_entries["Y"].get().strip())
            z = float(self.pos_entries["Z"].get().strip())
            rx = float(self.rot_entries["Rx"].get().strip())
            ry = float(self.rot_entries["Ry"].get().strip())
            rz = float(self.rot_entries["Rz"].get().strip())
        except ValueError:
            return

        self._updating_obj_ui = True
        self.pos_sliders["X"].set(max(-1000.0, min(1000.0, x)))
        self.pos_sliders["Y"].set(max(-1000.0, min(1000.0, y)))
        self.pos_sliders["Z"].set(max(-500.0, min(1500.0, z)))
        self.rot_sliders["Rx"].set(max(-180.0, min(180.0, rx)))
        self.rot_sliders["Ry"].set(max(-180.0, min(180.0, ry)))
        self.rot_sliders["Rz"].set(max(-180.0, min(180.0, rz)))
        self._updating_obj_ui = False

        self.viewer.update_custom_object(
            self.selected_obj_id,
            position=[x, y, z],
            rotation=[rx, ry, rz]
        )
        self._save_custom_objects()

    def _step_pos(self, axis, delta):
        """Steps position by a fixed offset (+10, +1, -1, -10)."""
        if not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            return
        obj = self.viewer.custom_objects[self.selected_obj_id]
        axis_idx = {"X": 0, "Y": 1, "Z": 2}[axis]
        pos = list(obj["position"])
        pos[axis_idx] += float(delta)

        self._updating_obj_ui = True
        self.pos_entries[axis].delete(0, "end")
        self.pos_entries[axis].insert(0, f"{pos[axis_idx]:.2f}")
        if axis in ["X", "Y"]:
            self.pos_sliders[axis].set(max(-1000.0, min(1000.0, pos[axis_idx])))
        else:
            self.pos_sliders[axis].set(max(-500.0, min(1500.0, pos[axis_idx])))
        self._updating_obj_ui = False

        self.viewer.update_custom_object(self.selected_obj_id, position=pos)
        self._save_custom_objects()

    def _step_rot(self, axis, delta):
        """Steps rotation angle by a fixed step."""
        if not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            return
        obj = self.viewer.custom_objects[self.selected_obj_id]
        axis_idx = {"Rx": 0, "Ry": 1, "Rz": 2}[axis]
        rot = list(obj["rotation"])
        new_val = rot[axis_idx] + float(delta)
        while new_val > 180.0:
            new_val -= 360.0
        while new_val < -180.0:
            new_val += 360.0
        rot[axis_idx] = new_val

        self._updating_obj_ui = True
        self.rot_entries[axis].delete(0, "end")
        self.rot_entries[axis].insert(0, f"{new_val:.2f}")
        self.rot_sliders[axis].set(new_val)
        self._updating_obj_ui = False

        self.viewer.update_custom_object(self.selected_obj_id, rotation=rot)
        self._save_custom_objects()

    def _on_parent_change(self, parent_val):
        """Callback when parent attachment dropdown changes."""
        if not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            return
        self.viewer.update_custom_object(self.selected_obj_id, parent=parent_val)
        self._save_custom_objects()

    def _on_color_change(self, color_val):
        """Callback when color dropdown changes."""
        if not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            return
        self.viewer.update_custom_object(self.selected_obj_id, color=color_val)
        self._save_custom_objects()

    def _on_scale_slider_move(self, val):
        """Callback when scale slider moves."""
        if self._updating_obj_ui or not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            return
        scale_val = float(val)
        self.scale_lbl.configure(text=f"{scale_val:.2f}x")
        self.viewer.update_custom_object(self.selected_obj_id, scale=scale_val)
        self._save_custom_objects()

    def _on_opacity_slider_move(self, val):
        """Callback when opacity slider moves."""
        if self._updating_obj_ui or not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            return
        op_val = float(val)
        self.opacity_lbl.configure(text=f"{int(op_val*100)}%")
        self.viewer.update_custom_object(self.selected_obj_id, opacity=op_val)
        self._save_custom_objects()

    def _toggle_object_visibility(self):
        """Toggles the visibility of the selected object."""
        if not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            return
        obj = self.viewer.custom_objects[self.selected_obj_id]
        cur_vis = obj.get("visible", True)
        new_vis = not cur_vis
        self.viewer.update_custom_object(self.selected_obj_id, visible=new_vis)
        self.obj_vis_btn.configure(text="👁️ Ẩn" if new_vis else "👁️ Hiện")
        self._save_custom_objects()

    def _reset_object_transform(self):
        """Resets the position and rotation of the selected object to (0,0,0)."""
        if not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            return
        self.viewer.update_custom_object(
            self.selected_obj_id,
            position=[0.0, 0.0, 0.0],
            rotation=[0.0, 0.0, 0.0],
            scale=1.0
        )
        self._load_selected_obj_to_ui()
        self._save_custom_objects()

    def _delete_object(self):
        """Deletes the selected object after user confirmation."""
        if not self.selected_obj_id or self.selected_obj_id not in self.viewer.custom_objects:
            return
        from tkinter import messagebox
        obj_name = self.viewer.custom_objects[self.selected_obj_id].get("name", "vật thể")
        if messagebox.askyesno("Xác nhận xóa", f"Bạn có chắc muốn xóa vật thể '{obj_name}' khỏi không gian 3D?"):
            self.viewer.remove_custom_object(self.selected_obj_id)
            self.selected_obj_id = None
            self._save_custom_objects()
            self._refresh_objects_list_ui()

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
