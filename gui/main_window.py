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
        
        # Create Tabview inside left_panel (removed Control tab)
        self.tabview = ctk.CTkTabview(self.left_panel, corner_radius=10)
        self.tabview.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.tab_program = self.tabview.add("Program")
        self.tab_config = self.tabview.add("Config")
        
        self.tab_program.grid_columnconfigure(0, weight=1)
        self.tab_config.grid_columnconfigure(0, weight=1)
        
        # Create control elements directly inside the Right Panel
        self._create_connection_frame()
        self._create_coordinate_frame()
        self._create_jog_frame()
        
        # Create configuration elements inside the Config tab
        self._create_config_tab_widgets()
        
        # Create program editor elements inside the Program tab
        self._create_program_tab_widgets()

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
