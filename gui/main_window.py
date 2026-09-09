import sys
import os
import time
import copy
import customtkinter as ctk
from serial.tools import list_ports

# Add root folder to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.config_manager import ConfigManager
from core.kinematics import Kinematics
from core.serial_driver import SerialDriver
from core.undo_redo_manager import UndoRedoManager
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
        
        # 1b. Initialize Undo/Redo Manager
        self.undo_manager = UndoRedoManager(max_history=60, on_change_callback=self._update_undo_redo_ui)
        self._slider_debounce_timers = {}
        self._pre_slider_states = {}
        self._entry_debounce_timer = None
        
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
        self.viewer.on_part_picked_cb = self._on_3d_part_picked
        
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
        
        # State variables for Program Editor & Simulator
        self.program_steps = []
        self.simulating = False
        self.executing_real = False
        self.captured_joints = list(self.joint_angles)
        self.captured_coords = self.kinematics.forward(self.joint_angles)
        
        # 4. Construct Layout
        self._create_widgets()
        self._update_coordinates()
        
        # Global Keyboard Shortcuts for Undo / Redo
        self.bind_all("<Control-z>", lambda e: self._on_undo())
        self.bind_all("<Control-Z>", lambda e: self._on_undo())
        self.bind_all("<Control-y>", lambda e: self._on_redo())
        self.bind_all("<Control-Y>", lambda e: self._on_redo())
        self.bind_all("<Control-Shift-Z>", lambda e: self._on_redo())
        self.bind_all("<Control-Shift-z>", lambda e: self._on_redo())
        
        # Force Tkinter to update and map widgets to obtain valid HWND and layout sizes
        self.update()
        
        # 5. Launch 3D Viewer directly in the right panel
        self._launch_viewer()

    def _on_undo(self):
        """Executes undo action."""
        desc = self.undo_manager.undo()
        if desc:
            print(f"[UNDO] Đã hoàn tác: {desc}")

    def _on_redo(self):
        """Executes redo action."""
        desc = self.undo_manager.redo()
        if desc:
            print(f"[REDO] Đã làm lại: {desc}")

    def _update_undo_redo_ui(self):
        """Updates Undo and Redo button states in toolbar."""
        if hasattr(self, "undo_btn"):
            can_u = self.undo_manager.can_undo()
            self.undo_btn.configure(
                state="normal" if can_u else "disabled",
                fg_color="#1f538d" if can_u else "#333333",
                hover_color="#14375e" if can_u else "#333333"
            )
    def _activate_tkinter_focus(self, target_widget=None):
        """Forces Windows OS keyboard and mouse focus back to Tkinter."""
        try:
            import ctypes
            ctypes.windll.user32.ReleaseCapture()
            hwnd = (target_widget.winfo_id() if target_widget else None) or self.winfo_id()
            if hwnd:
                ctypes.windll.user32.SetFocus(int(hwnd))
        except Exception:
            pass

    def _bind_entry_focus_handlers(self, entry_widget, *extra_clickable_widgets):
        """Ensures 100% reliable focus and text auto-selection when clicking entry or associated labels."""
        def grab_focus(event=None):
            self._activate_tkinter_focus(entry_widget)
            entry_widget.focus_set()
            if hasattr(entry_widget, "_entry"):
                entry_widget._entry.focus_set()
            entry_widget.after(10, lambda: entry_widget.select_range(0, "end"))

        entry_widget.bind("<FocusIn>", grab_focus)
        entry_widget.bind("<Button-1>", grab_focus)
        if hasattr(entry_widget, "_entry"):
            entry_widget._entry.bind("<FocusIn>", grab_focus)
            entry_widget._entry.bind("<Button-1>", grab_focus)
        if hasattr(entry_widget, "_canvas"):
            entry_widget._canvas.bind("<Button-1>", grab_focus)
            
        for w in extra_clickable_widgets:
            if w is not None:
                w.bind("<Button-1>", grab_focus)

    def _create_widgets(self):
        import tkinter as tk
        
        # Create PanedWindow for 3-horizontal splits
        self.main_pane = tk.PanedWindow(self, orient=tk.HORIZONTAL, bg="#1a1a1a", bd=0, sashwidth=6, sashpad=2, relief="flat")
        self.main_pane.pack(fill="both", expand=True)
        
        # Left Panel (Program / Config Tabview)
        self.left_panel = ctk.CTkFrame(self.main_pane, corner_radius=0, fg_color="transparent")
        self.left_panel.grid_columnconfigure(0, weight=1)
        self.main_pane.add(self.left_panel, width=380, minsize=300, stretch="never")

        # Automatically release mouse capture and activate Tkinter whenever mouse crosses into side panels
        self.left_panel.bind("<Enter>", lambda e: self._activate_tkinter_focus())
        self.main_pane.bind("<Enter>", lambda e: self._activate_tkinter_focus())
        
        # Top Undo / Redo Toolbar
        self.undo_toolbar = ctk.CTkFrame(self.left_panel, fg_color="#1e1e1e", corner_radius=6)
        self.undo_toolbar.pack(fill="x", padx=6, pady=(6, 2))
        self.undo_toolbar.grid_columnconfigure((0, 1), weight=1)
        
        self.undo_btn = ctk.CTkButton(
            self.undo_toolbar,
            text="↶ Hoàn tác (Ctrl+Z)",
            height=28,
            font=("Arial", 11, "bold"),
            fg_color="#333333",
            state="disabled",
            command=self._on_undo
        )
        self.undo_btn.grid(row=0, column=0, padx=3, pady=4, sticky="ew")
        
        self.redo_btn = ctk.CTkButton(
            self.undo_toolbar,
            text="↷ Làm lại (Ctrl+Y)",
            height=28,
            font=("Arial", 11, "bold"),
            fg_color="#333333",
            state="disabled",
            command=self._on_redo
        )
        self.redo_btn.grid(row=0, column=1, padx=3, pady=4, sticky="ew")
        
        # Middle Panel (3D Viewer Container)
        self.middle_panel = ctk.CTkFrame(self.main_pane, corner_radius=0, fg_color="transparent")
        self.middle_panel.grid_columnconfigure(0, weight=1)
        self.middle_panel.grid_rowconfigure(0, weight=1)
        self.main_pane.add(self.middle_panel, minsize=400, stretch="always")
        
        # Right Panel (Jog / Connection Panel)
        self.right_panel = ctk.CTkFrame(self.main_pane, corner_radius=0, fg_color="transparent")
        self.right_panel.grid_columnconfigure(0, weight=1)
        self.right_panel.bind("<Enter>", lambda e: self._activate_tkinter_focus())
        self.main_pane.add(self.right_panel, width=380, minsize=300, stretch="never")
        
        # Create Tabview inside left_panel (Chỉ 2 Tab chính: Program và Robot Model)
        self.tabview = ctk.CTkTabview(self.left_panel, corner_radius=10)
        self.tabview.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.tab_program = self.tabview.add("Program")
        self.tab_links = self.tabview.add("Robot Model")
        
        self.tab_program.grid_columnconfigure(0, weight=1)
        self.tab_links.grid_columnconfigure(0, weight=1)
        
        # Create control elements directly inside the Right Panel
        self._create_connection_frame()
        self._create_coordinate_frame()
        self._create_jog_frame()
        
        # Create program editor elements inside the Program tab
        self._create_program_tab_widgets()

        # Create unified robot model & kinematics controls inside Robot Model tab
        self._create_robot_links_tab_widgets()

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
        jog_frame = ctk.CTkLabelFrame(self.right_panel, text="Joint Jog Controls (Góc Khớp °)")
        jog_frame.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
        jog_frame.grid_columnconfigure(0, weight=1)
        
        self.sliders = []
        self.angle_entries = []
        self._updating_jog = False
        
        joint_names = ["J1 Base", "J2 Shoulder", "J3 Elbow", "J4 Wrist", "J5 Pitch", "J6 Roll"]
        mins = [-170.0, -42.0, -89.0, -180.0, -105.0, -180.0]
        maxs = [170.0, 90.0, 52.0, 180.0, 105.0, 180.0]
        
        for i in range(6):
            j_row = ctk.CTkFrame(jog_frame)
            j_row.grid(row=i, column=0, padx=5, pady=4, sticky="ew")
            j_row.grid_columnconfigure(1, weight=1)
            
            lbl = ctk.CTkLabel(j_row, text=joint_names[i], width=85, anchor="w", font=("Arial", 11, "bold"))
            lbl.grid(row=0, column=0, padx=(5, 2))
            
            slider = ctk.CTkSlider(
                j_row, 
                from_=mins[i], 
                to=maxs[i], 
                command=lambda val, idx=i: self._on_slider_move(idx, val)
            )
            slider.set(self.joint_angles[i])
            slider.grid(row=0, column=1, padx=4, sticky="ew")
            self.sliders.append(slider)
            
            entry = ctk.CTkEntry(j_row, width=64, height=28, justify="center", font=("Arial", 11, "bold"))
            entry.insert(0, f"{self.joint_angles[i]:.1f}")
            entry.grid(row=0, column=2, padx=2)
            entry.bind("<Return>", lambda e, idx=i: self._on_jog_entry_update(idx))
            entry.bind("<FocusOut>", lambda e, idx=i: self._on_jog_entry_update(idx))
            self.angle_entries.append(entry)
            
            deg_lbl = ctk.CTkLabel(j_row, text="°", width=12, font=("Arial", 11, "bold"), cursor="hand2")
            deg_lbl.grid(row=0, column=3, padx=(0, 4))
            self._bind_entry_focus_handlers(entry, lbl, deg_lbl)
            
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

    def _apply_joint_angles_from_undo(self, target_angles):
        """Restores joint angles from undo/redo without pushing a new undo action."""
        self.joint_angles = [float(a) for a in target_angles]
        self._updating_jog = True
        for i in range(6):
            if i < len(self.sliders):
                self.sliders[i].set(self.joint_angles[i])
            if hasattr(self, "angle_entries") and i < len(self.angle_entries):
                self.angle_entries[i].delete(0, "end")
                self.angle_entries[i].insert(0, f"{self.joint_angles[i]:.1f}")
            self.config.set(f"J{i+1}AngCur", f"{self.joint_angles[i]:.4f}")
        self._updating_jog = False
        if self.viewer.vtk_running:
            self.viewer.update_joints(self.joint_angles)
        self._update_coordinates()
        self.serial.send_move_command(self.joint_angles)

    def _finalize_jog_undo(self):
        """Pushes debounced jog change into UndoRedoManager."""
        if "jog" in self._pre_slider_states:
            old_j = list(self._pre_slider_states.pop("jog"))
            new_j = list(self.joint_angles)
            if old_j != new_j:
                self.undo_manager.push(
                    "Jog góc khớp robot",
                    lambda o=old_j: self._apply_joint_angles_from_undo(o),
                    lambda n=new_j: self._apply_joint_angles_from_undo(n)
                )

    def _on_slider_move(self, idx, value):
        if self._updating_jog:
            return
        
        # Save snapshot before move starts
        if "jog" not in self._pre_slider_states:
            self._pre_slider_states["jog"] = list(self.joint_angles)

        val_f = float(value)
        self.joint_angles[idx] = val_f
        if hasattr(self, "angle_entries") and idx < len(self.angle_entries):
            self._updating_jog = True
            self.angle_entries[idx].delete(0, "end")
            self.angle_entries[idx].insert(0, f"{val_f:.1f}")
            self._updating_jog = False
        
        # Save angle to config (save current position)
        self.config.set(f"J{idx+1}AngCur", f"{val_f:.4f}")
        
        # Trigger 3D model update
        if self.viewer.vtk_running:
            self.viewer.update_joints(self.joint_angles)
            
        # Update XYZ math calculations
        self._update_coordinates()
        
        # Send serial updates
        self.serial.send_move_command(self.joint_angles)

        # Schedule debounced undo push
        if "jog" in self._slider_debounce_timers and self._slider_debounce_timers["jog"]:
            self.after_cancel(self._slider_debounce_timers["jog"])
        self._slider_debounce_timers["jog"] = self.after(350, self._finalize_jog_undo)

    def _on_jog_entry_update(self, idx):
        if not hasattr(self, "angle_entries") or idx >= len(self.angle_entries):
            return
        try:
            val = float(self.angle_entries[idx].get().strip())
        except ValueError:
            return
        mins = [-170.0, -42.0, -89.0, -180.0, -105.0, -180.0]
        maxs = [170.0, 90.0, 52.0, 180.0, 105.0, 180.0]
        clamped = max(mins[idx], min(maxs[idx], val))
        
        old_j = list(self.joint_angles)
        self.joint_angles[idx] = clamped
        if idx < len(self.sliders):
            self.sliders[idx].set(clamped)
        
        self._updating_jog = True
        self.angle_entries[idx].delete(0, "end")
        self.angle_entries[idx].insert(0, f"{clamped:.1f}")
        self._updating_jog = False
        
        self.config.set(f"J{idx+1}AngCur", f"{clamped:.4f}")
        if self.viewer.vtk_running:
            self.viewer.update_joints(self.joint_angles)
        self._update_coordinates()
        self.serial.send_move_command(self.joint_angles)

        new_j = list(self.joint_angles)
        if old_j != new_j:
            self.undo_manager.push(
                f"Chỉnh góc khớp J{idx+1} -> {clamped:.1f}°",
                lambda o=old_j: self._apply_joint_angles_from_undo(o),
                lambda n=new_j: self._apply_joint_angles_from_undo(n)
            )

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
        self._load_selected_link_to_ui()

    def _on_serial_feedback(self, data):
        """Callback run when receiving feedback strings from the serial port."""
        print(f"[UI Serial Feedback]: {data}")
        # If we are executing a program on the real robot and receive "done", trigger the next step
        if self.executing_real and "done" in data.lower():
            self.after(10, self._next_real_step)

    def _apply_and_save_config(self):
        """Validates, saves DH parameters, reinitializes kinematics, and updates coordinates."""
        from tkinter import messagebox
        
        # 1. Validate and read DH entries
        param_keys = ["Θ", "α", "d", "a"]
        temp_data = {}
        for row_idx in range(6):
            for param in param_keys:
                if (row_idx+1, param) not in self.dh_entries:
                    continue
                entry = self.dh_entries[(row_idx+1, param)]
                val = entry.get().strip()
                try:
                    float(val)
                except ValueError:
                    messagebox.showerror("Lỗi", f"Giá trị không hợp lệ '{val}' cho Khớp J{row_idx+1} {param}")
                    return
                temp_data[f"J{row_idx+1}{param}DHpar"] = val
                
        # 2. Save values in ConfigManager and sync with active model
        for k, v in temp_data.items():
            self.config.config_data[k] = v
        self.config.sync_active_model_data()
        self.config.save_config()
        
        # 3. Re-initialize Kinematics and update coordinates
        self.kinematics.initialize_kinematics()
        self._update_coordinates()
        
        messagebox.showinfo("Thành công", "Đã áp dụng và lưu thông số động học DH thành công!")

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
        self._updating_jog = True
        for i in range(6):
            if i < len(self.sliders):
                self.sliders[i].set(angles[i])
            if hasattr(self, "angle_entries") and i < len(self.angle_entries):
                self.angle_entries[i].delete(0, "end")
                self.angle_entries[i].insert(0, f"{angles[i]:.1f}")
        self._updating_jog = False
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
        old_j = list(self.joint_angles)
        home_j = [0.0] * 6
        if old_j != home_j:
            self.undo_manager.push(
                "Go to Home (Về Home)",
                lambda o=old_j: self._apply_joint_angles_from_undo(o),
                lambda h=home_j: self._apply_joint_angles_from_undo(h)
            )
        self._apply_joint_angles_from_undo(home_j)
        
    # -------------------------------------------------------------------------
    # Robot Links Alignment & Individual STL Part Management (Căn chỉnh Khớp & Chi Tiết STL)
    # -------------------------------------------------------------------------
    def _create_robot_links_tab_widgets(self):
        """Constructs the UI for model profiles, individual STL part manipulation, and link kinematics."""
        self.links_scroll_frame = ctk.CTkScrollableFrame(self.tab_links, fg_color="transparent")
        self.links_scroll_frame.pack(fill="both", expand=True, padx=5, pady=5)
        self.links_scroll_frame.grid_columnconfigure(0, weight=1)

        # =========================================================================
        # 1. Robot Model Profile Manager
        # =========================================================================
        preset_frame = ctk.CTkLabelFrame(self.links_scroll_frame, text="🤖 Hồ Sơ Mô Hình Robot (Robot Model Profiles)")
        preset_frame.grid(row=0, column=0, padx=5, pady=(2, 6), sticky="ew")
        preset_frame.grid_columnconfigure(0, weight=1)

        active_name = self.config.get_active_model_name()
        self.active_model_lbl = ctk.CTkLabel(
            preset_frame,
            text=f"Đang dùng: {active_name}",
            font=("Arial", 11, "bold"),
            text_color="#3B8ED0",
            anchor="w"
        )
        self.active_model_lbl.grid(row=0, column=0, padx=8, pady=(8, 2), sticky="ew")

        model_names = list(self.config.get_saved_models().keys())
        self.preset_menu = ctk.CTkOptionMenu(
            preset_frame,
            values=model_names if model_names else ["(Chưa có model)"],
            command=lambda v: None
        )
        self.preset_menu.set(active_name)
        self.preset_menu.grid(row=1, column=0, padx=6, pady=(2, 4), sticky="ew")

        model_btn_row = ctk.CTkFrame(preset_frame, fg_color="transparent")
        model_btn_row.grid(row=2, column=0, padx=6, pady=(0, 4), sticky="ew")
        model_btn_row.grid_columnconfigure((0, 1, 2), weight=1)

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

        # =========================================================================
        # 2. Link-Level Alignment & 3D Offsets (Cấu hình Khâu & Dời Trục)
        # =========================================================================
        link_sel_frame = ctk.CTkLabelFrame(self.links_scroll_frame, text="⚙️ 2. Xây Dựng Khâu & Dời Trục 3D (Robot Links)")
        link_sel_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        link_sel_frame.grid_columnconfigure(0, weight=1)

        # Dropdown to select Link
        display_options = list(self.link_display_map.keys())
        self.link_selector = ctk.CTkOptionMenu(
            link_sel_frame,
            values=display_options,
            command=self._on_select_link
        )
        self.link_selector.grid(row=0, column=0, padx=8, pady=(8, 4), sticky="ew")

        # STL Part Selector & Operations row
        stl_card = ctk.CTkFrame(link_sel_frame, fg_color="#242424", corner_radius=6)
        stl_card.grid(row=1, column=0, padx=6, pady=4, sticky="ew")
        stl_card.grid_columnconfigure(0, weight=1)

        stl_menu_row = ctk.CTkFrame(stl_card, fg_color="transparent")
        stl_menu_row.grid(row=0, column=0, padx=6, pady=(6, 2), sticky="ew")
        stl_menu_row.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(stl_menu_row, text="File STL:", font=("Arial", 11, "bold")).grid(row=0, column=0, padx=(2, 6), sticky="w")
        
        self.link_stl_menu = ctk.CTkOptionMenu(
            stl_menu_row,
            values=["(Chưa có file STL nào)"],
            command=self._on_select_stl_part
        )
        self.link_stl_menu.grid(row=0, column=1, sticky="ew")

        self.link_stl_info_lbl = ctk.CTkLabel(
            stl_card,
            text="Kích thước bao (DxRxC): --",
            font=("Arial", 10, "bold"),
            text_color="#38bdf8",
            anchor="w"
        )
        self.link_stl_info_lbl.grid(row=1, column=0, padx=8, pady=(2, 4), sticky="ew")

        stl_btn_row = ctk.CTkFrame(stl_card, fg_color="transparent")
        stl_btn_row.grid(row=2, column=0, padx=6, pady=(0, 6), sticky="ew")
        stl_btn_row.grid_columnconfigure((0, 1, 2), weight=1)

        self.add_link_stl_btn = ctk.CTkButton(
            stl_btn_row,
            text="✚ Thêm STL",
            height=28,
            font=("Arial", 11, "bold"),
            fg_color="#1f538d",
            hover_color="#14375e",
            command=self._on_add_stl_to_link
        )
        self.add_link_stl_btn.grid(row=0, column=0, padx=2, pady=2, sticky="ew")

        self.del_link_stl_btn = ctk.CTkButton(
            stl_btn_row,
            text="🗑️ Xóa STL",
            height=28,
            font=("Arial", 11),
            fg_color="#8d1f1f",
            hover_color="#5e1414",
            command=self._on_delete_link_stl
        )
        self.del_link_stl_btn.grid(row=0, column=1, padx=2, pady=2, sticky="ew")

        self.vis_link_stl_btn = ctk.CTkButton(
            stl_btn_row,
            text="👁️ Ẩn/Hiện",
            height=28,
            font=("Arial", 11),
            fg_color="#555",
            hover_color="#333",
            command=self._toggle_link_stl_visibility
        )
        self.vis_link_stl_btn.grid(row=0, column=2, padx=2, pady=2, sticky="ew")

        # Joint Axis Menu
        axis_row = ctk.CTkFrame(link_sel_frame, fg_color="transparent")
        axis_row.grid(row=2, column=0, padx=6, pady=(2, 4), sticky="ew")
        axis_row.grid_columnconfigure(1, weight=1)

        ctk.CTkLabel(axis_row, text="Trục quay khớp:", font=("Arial", 11, "bold")).grid(row=0, column=0, padx=4, pady=2, sticky="w")
        self.link_axis_menu = ctk.CTkOptionMenu(
            axis_row,
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
        self.link_axis_menu.grid(row=0, column=1, padx=4, pady=2, sticky="ew")

        # Position Sliders (X, Y, Z mm)
        link_pos_card = ctk.CTkLabelFrame(link_sel_frame, text="Dời Trục Khớp (Joint Position Offset - mm)")
        link_pos_card.grid(row=3, column=0, padx=6, pady=4, sticky="ew")
        link_pos_card.grid_columnconfigure(0, weight=1)

        self.link_pos_sliders = {}
        self.link_pos_entries = {}

        link_pos_axes = [
            ("X", -1000.0, 1000.0),
            ("Y", -1000.0, 1000.0),
            ("Z", -1000.0, 1500.0)
        ]

        for idx, (axis, min_val, max_val) in enumerate(link_pos_axes):
            axis_card = ctk.CTkFrame(link_pos_card, fg_color="#2b2b2b", corner_radius=6)
            axis_card.grid(row=idx, column=0, padx=4, pady=3, sticky="ew")
            axis_card.grid_columnconfigure(1, weight=1)

            entry = ctk.CTkEntry(axis_card, width=75, height=28, justify="center", font=("Arial", 11, "bold"))
            entry.insert(0, "0.00")
            entry.grid(row=0, column=1, padx=2, pady=3, sticky="w")
            
            lbl = ctk.CTkLabel(axis_card, text=f"{axis}:", font=("Arial", 12, "bold"), width=22, cursor="hand2")
            lbl.grid(row=0, column=0, padx=(6, 2), pady=3, sticky="w")

            unit_lbl = ctk.CTkLabel(axis_card, text="mm", font=("Arial", 10), text_color="gray", cursor="hand2")
            unit_lbl.grid(row=0, column=2, padx=(2, 4), pady=3, sticky="w")

            entry.bind("<Return>", lambda e, a=axis: self._commit_link_axis_entry("pos", a))
            entry.bind("<FocusOut>", lambda e, a=axis: self._commit_link_axis_entry("pos", a))
            entry.bind("<KeyRelease>", lambda e, a=axis: self._debounce_link_axis_entry("pos", a))
            self._bind_entry_focus_handlers(entry, lbl, unit_lbl)
            self.link_pos_entries[axis] = entry

            step_box = ctk.CTkFrame(axis_card, fg_color="transparent")
            step_box.grid(row=0, column=3, padx=2, pady=2, sticky="e")

            for step in [-10, -1, 1, 10]:
                text = f"{step:+d}" if step > 0 else str(step)
                btn = ctk.CTkButton(
                    step_box,
                    text=text,
                    width=28,
                    height=20,
                    font=("Arial", 9),
                    command=lambda a=axis, s=step: self._step_link_pos(a, s)
                )
                btn.pack(side="left", padx=1)

            slider = ctk.CTkSlider(
                axis_card,
                from_=min_val,
                to=max_val,
                height=14,
                command=lambda val, a=axis: self._on_link_pos_slider_move(a, val)
            )
            slider.set(0.0)
            slider.grid(row=1, column=0, columnspan=4, padx=6, pady=(1, 4), sticky="ew")
            self.link_pos_sliders[axis] = slider

        # Rotation Sliders (Rx, Ry, Rz °)
        link_rot_card = ctk.CTkLabelFrame(link_sel_frame, text="Định Hướng Khớp (Joint Rotation Offset - độ °)")
        link_rot_card.grid(row=4, column=0, padx=6, pady=4, sticky="ew")
        link_rot_card.grid_columnconfigure(0, weight=1)

        self.link_rot_sliders = {}
        self.link_rot_entries = {}

        link_rot_axes = [
            ("Rx", "Roll (X)"),
            ("Ry", "Pitch (Y)"),
            ("Rz", "Yaw (Z)")
        ]

        for idx, (axis, name) in enumerate(link_rot_axes):
            axis_card = ctk.CTkFrame(link_rot_card, fg_color="#2b2b2b", corner_radius=6)
            axis_card.grid(row=idx, column=0, padx=4, pady=3, sticky="ew")
            axis_card.grid_columnconfigure(1, weight=1)

            entry = ctk.CTkEntry(axis_card, width=75, height=28, justify="center", font=("Arial", 11, "bold"))
            entry.insert(0, "0.00")
            entry.grid(row=0, column=1, padx=2, pady=3, sticky="w")

            lbl = ctk.CTkLabel(axis_card, text=f"{axis}:", font=("Arial", 12, "bold"), width=24, cursor="hand2")
            lbl.grid(row=0, column=0, padx=(6, 2), pady=3, sticky="w")

            unit_lbl = ctk.CTkLabel(axis_card, text="°", font=("Arial", 10), text_color="gray", cursor="hand2")
            unit_lbl.grid(row=0, column=2, padx=(2, 4), pady=3, sticky="w")

            entry.bind("<Return>", lambda e, a=axis: self._commit_link_axis_entry("rot", a))
            entry.bind("<FocusOut>", lambda e, a=axis: self._commit_link_axis_entry("rot", a))
            entry.bind("<KeyRelease>", lambda e, a=axis: self._debounce_link_axis_entry("rot", a))
            self._bind_entry_focus_handlers(entry, lbl, unit_lbl)
            self.link_rot_entries[axis] = entry

            unit_lbl = ctk.CTkLabel(axis_card, text="°", font=("Arial", 10), text_color="gray", cursor="hand2")
            unit_lbl.grid(row=0, column=2, padx=(2, 4), pady=3, sticky="w")
            unit_lbl.bind("<Button-1>", lambda e, ent=entry: (ent.focus_set(), ent.after(10, lambda: ent.select_range(0, "end"))))

            step_box = ctk.CTkFrame(axis_card, fg_color="transparent")
            step_box.grid(row=0, column=3, padx=2, pady=2, sticky="e")

            for step in [-15, -1, 1, 15]:
                text = f"{step:+d}°" if step > 0 else f"{step}°"
                btn = ctk.CTkButton(
                    step_box,
                    text=text,
                    width=30,
                    height=20,
                    font=("Arial", 9),
                    command=lambda a=axis, s=step: self._step_link_rot(a, s)
                )
                btn.pack(side="left", padx=1)

            slider = ctk.CTkSlider(
                axis_card,
                from_=-180.0,
                to=180.0,
                height=14,
                command=lambda val, a=axis: self._on_link_rot_slider_move(a, val)
            )
            slider.set(0.0)
            slider.grid(row=1, column=0, columnspan=4, padx=6, pady=(1, 4), sticky="ew")
            self.link_rot_sliders[axis] = slider

        btn_box = ctk.CTkFrame(link_sel_frame, fg_color="transparent")
        btn_box.grid(row=5, column=0, padx=6, pady=(2, 8), sticky="ew")
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

        # =========================================================================
        # 3. DH Parameters Editor (Bảng Động Học DH)
        # =========================================================================
        dh_frame = ctk.CTkLabelFrame(self.links_scroll_frame, text="📐 3. Bảng Động Học DH (DH Parameters J1-J6)")
        dh_frame.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        dh_frame.grid_columnconfigure(0, weight=1)
        dh_frame.grid_columnconfigure((1, 2, 3, 4), weight=2)

        # Header Labels
        headers = ["Khớp", "Theta (θ°)", "Alpha (α°)", "d (mm)", "a (mm)"]
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

                entry = ctk.CTkEntry(dh_frame, width=64, height=28, justify="center", font=("Arial", 11, "bold"))
                entry.insert(0, str(val))
                entry.grid(row=row_idx+1, column=col_idx+1, padx=2, pady=3)
                self._bind_entry_focus_handlers(entry, j_lbl)
                self.dh_entries[(row_idx+1, param)] = entry

        dh_apply_btn = ctk.CTkButton(
            dh_frame,
            text="💾 Áp Dụng & Lưu Bảng DH",
            height=34,
            font=("Arial", 11, "bold"),
            fg_color="#1eaa59",
            hover_color="#13773e",
            command=self._apply_and_save_config
        )
        dh_apply_btn.grid(row=7, column=0, columnspan=5, padx=8, pady=(8, 8), sticky="ew")

        # Initial UI Population
        self._load_selected_link_to_ui()

    # -------------------------------------------------------------------------
    # Link STL Management, Joint Offset & Alignment Handlers
    # -------------------------------------------------------------------------

    def _on_select_link(self, display_name):
        """Callback when user selects a link in the dropdown."""
        link_key = self.link_display_map.get(display_name, "Base")
        self.selected_link_key = link_key
        self.selected_stl_name = None
        self._load_selected_link_to_ui()

    def _on_select_stl_part(self, selected_name):
        """Callback when user selects a specific STL file in the STL dropdown."""
        if not selected_name or selected_name.startswith("("):
            return
        self.selected_stl_name = selected_name
        bounds = self.viewer.get_part_bounds(selected_name)
        if bounds and hasattr(self, "link_stl_info_lbl"):
            sx, sy, sz = bounds["size"]
            self.link_stl_info_lbl.configure(text=f"Kích thước bao (DxRxC): {sx:.1f} × {sy:.1f} × {sz:.1f} mm")
        self.viewer.highlight_part(selected_name)

    def _load_selected_link_to_ui(self):
        """Populates link controls with the selected link's configuration."""
        cfg = self.config.get_link_config(self.selected_link_key)
        self._updating_link_ui = True

        # 1. Update STL Dropdown Menu & Info
        stl_files = cfg.get("stl_files", [])
        if hasattr(self, "link_stl_menu"):
            if stl_files:
                names = [os.path.basename(f) for f in stl_files]
                self.link_stl_menu.configure(values=names)
                if not hasattr(self, "selected_stl_name") or not self.selected_stl_name or self.selected_stl_name not in names:
                    self.selected_stl_name = names[0]
                self.link_stl_menu.set(self.selected_stl_name)
                
                # Get bounds of selected STL part
                bounds = self.viewer.get_part_bounds(self.selected_stl_name)
                if bounds:
                    sx, sy, sz = bounds["size"]
                    self.link_stl_info_lbl.configure(text=f"Kích thước bao (DxRxC): {sx:.1f} × {sy:.1f} × {sz:.1f} mm")
                else:
                    self.link_stl_info_lbl.configure(text=f"File: {self.selected_stl_name}")
                self.viewer.highlight_part(self.selected_stl_name)
            else:
                self.selected_stl_name = None
                self.link_stl_menu.configure(values=["(Chưa có file STL nào)"])
                self.link_stl_menu.set("(Chưa có file STL nào)")
                self.link_stl_info_lbl.configure(text="Kích thước bao (DxRxC): --")
                self.viewer.clear_highlight()

        vis = cfg.get("visible", True)
        if hasattr(self, "vis_link_stl_btn"):
            self.vis_link_stl_btn.configure(text="👁️ Ẩn" if vis else "👁️ Hiện")

        # 2. Update Axis Menu
        axis_raw = cfg.get("joint_axis", "None" if self.selected_link_key == "Base" else "+Z")
        if hasattr(self, "link_axis_menu"):
            for val in self.link_axis_menu._values:
                if val.startswith(axis_raw):
                    self.link_axis_menu.set(val)
                    break
            else:
                self.link_axis_menu.set("+Z (Quay quanh Z thuận)")

        # 3. Update Position Entries & Sliders
        pos = cfg.get("offset_pos", [0.0, 0.0, 0.0])
        if hasattr(self, "link_pos_entries"):
            for idx, ax in enumerate(["X", "Y", "Z"]):
                val = pos[idx]
                if ax in self.link_pos_entries:
                    self.link_pos_entries[ax].delete(0, "end")
                    self.link_pos_entries[ax].insert(0, f"{val:.2f}")
                if hasattr(self, "link_pos_sliders") and ax in self.link_pos_sliders:
                    cur_from = self.link_pos_sliders[ax].cget("from_")
                    cur_to = self.link_pos_sliders[ax].cget("to")
                    if val < cur_from:
                        self.link_pos_sliders[ax].configure(from_=val - 100)
                    elif val > cur_to:
                        self.link_pos_sliders[ax].configure(to=val + 100)
                    self.link_pos_sliders[ax].set(val)

        # 4. Update Rotation Entries & Sliders
        rot = cfg.get("offset_rot", [0.0, 0.0, 0.0])
        if hasattr(self, "link_rot_entries"):
            for idx, ax in enumerate(["Rx", "Ry", "Rz"]):
                val = rot[idx]
                if ax in self.link_rot_entries:
                    self.link_rot_entries[ax].delete(0, "end")
                    self.link_rot_entries[ax].insert(0, f"{val:.2f}")
                if hasattr(self, "link_rot_sliders") and ax in self.link_rot_sliders:
                    self.link_rot_sliders[ax].set(val)

        self._updating_link_ui = False

    def _on_add_stl_to_link(self):
        """Opens file dialog to choose an STL file and adds it to the current link."""
        from tkinter import filedialog, messagebox
        file_path = filedialog.askopenfilename(
            title=f"Chọn file STL thêm vào {self.selected_link_key}",
            filetypes=[("STL 3D Model", "*.stl *.STL"), ("All Files", "*.*")]
        )
        if not file_path:
            return
        if not os.path.exists(file_path):
            messagebox.showerror("Lỗi", f"Không tìm thấy file: {file_path}")
            return
            
        link_key = self.selected_link_key
        success = self.viewer.add_stl_to_link(link_key, file_path)
        if success:
            part_name = os.path.basename(file_path)
            self.selected_stl_name = part_name
            self._load_selected_link_to_ui()
            messagebox.showinfo("Thành công", f"Đã thêm chi tiết '{part_name}' vào {link_key}!")
        else:
            messagebox.showerror("Lỗi", "Không thể nạp file STL!")

    def _on_delete_link_stl(self):
        """Deletes the currently selected STL file from this link."""
        from tkinter import messagebox
        cfg = self.config.get_link_config(self.selected_link_key)
        stl_files = cfg.get("stl_files", [])
        if not stl_files:
            messagebox.showwarning("Thông báo", f"{self.selected_link_key} chưa có file STL nào!")
            return
            
        target_name = getattr(self, "selected_stl_name", None)
        if not target_name:
            target_name = os.path.basename(stl_files[0])
            
        ok = messagebox.askyesno(
            "Xác nhận xóa",
            f"Bạn có chắc muốn xóa file STL '{target_name}' khỏi {self.selected_link_key}?"
        )
        if not ok:
            return
            
        self.viewer.remove_stl_from_link(self.selected_link_key, target_name)
        self.selected_stl_name = None
        self._load_selected_link_to_ui()
        messagebox.showinfo("Thành công", f"Đã xóa '{target_name}' khỏi {self.selected_link_key}!")

    def _toggle_link_stl_visibility(self):
        """Toggles visibility of the selected link's STL mesh."""
        vis = self.viewer.toggle_link_visibility(self.selected_link_key)
        if hasattr(self, "vis_link_stl_btn"):
            self.vis_link_stl_btn.configure(text="👁️ Ẩn" if vis else "👁️ Hiện")

    def _on_link_axis_change(self, axis_val):
        """Callback when user changes joint axis."""
        axis_code = axis_val.split()[0] if axis_val else "+Z"
        self.viewer.update_link_joint_axis(self.selected_link_key, axis_code)

    def _on_link_pos_slider_move(self, axis, val):
        """Callback when link position slider moves."""
        if self._updating_link_ui or not hasattr(self, "selected_link_key"):
            return
        axis_idx = {"X": 0, "Y": 1, "Z": 2}[axis]
        cfg = self.config.get_link_config(self.selected_link_key)
        pos = list(cfg.get("offset_pos", [0.0, 0.0, 0.0]))
        pos[axis_idx] = float(val)

        self._updating_link_ui = True
        if hasattr(self, "link_pos_entries") and axis in self.link_pos_entries:
            self.link_pos_entries[axis].delete(0, "end")
            self.link_pos_entries[axis].insert(0, f"{val:.2f}")
        self._updating_link_ui = False

        self.viewer.update_link_offset(self.selected_link_key, pos=pos)

    def _on_link_rot_slider_move(self, axis, val):
        """Callback when link rotation slider moves."""
        if self._updating_link_ui or not hasattr(self, "selected_link_key"):
            return
        axis_idx = {"Rx": 0, "Ry": 1, "Rz": 2}[axis]
        cfg = self.config.get_link_config(self.selected_link_key)
        rot = list(cfg.get("offset_rot", [0.0, 0.0, 0.0]))
        rot[axis_idx] = float(val)

        self._updating_link_ui = True
        if hasattr(self, "link_rot_entries") and axis in self.link_rot_entries:
            self.link_rot_entries[axis].delete(0, "end")
            self.link_rot_entries[axis].insert(0, f"{val:.2f}")
        self._updating_link_ui = False

        self.viewer.update_link_offset(self.selected_link_key, rot=rot)

    def _step_link_pos(self, axis, step):
        """Step adjusts link position."""
        if not hasattr(self, "link_pos_sliders") or axis not in self.link_pos_sliders:
            return
        cur_val = self.link_pos_sliders[axis].get()
        new_val = cur_val + step
        self.link_pos_sliders[axis].set(new_val)
        self._on_link_pos_slider_move(axis, new_val)

    def _step_link_rot(self, axis, step):
        """Step adjusts link rotation."""
        if not hasattr(self, "link_rot_sliders") or axis not in self.link_rot_sliders:
            return
        cur_val = self.link_rot_sliders[axis].get()
        new_val = cur_val + step
        self.link_rot_sliders[axis].set(new_val)
        self._on_link_rot_slider_move(axis, new_val)

    def _debounce_link_axis_entry(self, axis_type, axis_name):
        """Debounced live update when user is typing numbers in a specific axis entry."""
        if self._updating_link_ui or not hasattr(self, "selected_link_key"):
            return
        timer_key = f"link_{axis_type}_{axis_name}"
        if timer_key in self._slider_debounce_timers and self._slider_debounce_timers[timer_key]:
            self.after_cancel(self._slider_debounce_timers[timer_key])
        self._slider_debounce_timers[timer_key] = self.after(
            300, lambda at=axis_type, an=axis_name: self._apply_link_axis_entry(at, an, format_text=False)
        )

    def _commit_link_axis_entry(self, axis_type, axis_name):
        """Immediately commits and formats the value when Enter is pressed or focus leaves."""
        timer_key = f"link_{axis_type}_{axis_name}"
        if timer_key in self._slider_debounce_timers and self._slider_debounce_timers[timer_key]:
            self.after_cancel(self._slider_debounce_timers[timer_key])
        self._apply_link_axis_entry(axis_type, axis_name, format_text=True)

    def _apply_link_axis_entry(self, axis_type, axis_name, format_text=False):
        """Parses and applies numeric entry for a single axis independently."""
        if self._updating_link_ui or not hasattr(self, "selected_link_key"):
            return
            
        entries = self.link_pos_entries if axis_type == "pos" else self.link_rot_entries
        sliders = self.link_pos_sliders if axis_type == "pos" else self.link_rot_sliders
        
        if axis_name not in entries:
            return
            
        raw_text = entries[axis_name].get().strip()
        cfg = self.config.get_link_config(self.selected_link_key)
        cur_list = list(cfg.get("offset_pos" if axis_type == "pos" else "offset_rot", [0.0, 0.0, 0.0]))
        axis_idx = {"X": 0, "Y": 1, "Z": 2, "Rx": 0, "Ry": 1, "Rz": 2}[axis_name]
        cur_val = cur_list[axis_idx]

        if not raw_text or raw_text in ["-", "+", ".", "-.", "+."]:
            if format_text:
                self._updating_link_ui = True
                entries[axis_name].delete(0, "end")
                entries[axis_name].insert(0, f"{cur_val:.2f}")
                self._updating_link_ui = False
            return  # Incomplete intermediate typing - do not reject or crash
            
        try:
            val = float(raw_text)
        except ValueError:
            if format_text:
                self._updating_link_ui = True
                entries[axis_name].delete(0, "end")
                entries[axis_name].insert(0, f"{cur_val:.2f}")
                self._updating_link_ui = False
            return
            
        if axis_type == "pos":
            pos = cur_list
            pos[axis_idx] = val
            self.viewer.update_link_offset(self.selected_link_key, pos=pos)
            
            if axis_name in sliders:
                self._updating_link_ui = True
                cur_from = sliders[axis_name].cget("from_")
                cur_to = sliders[axis_name].cget("to")
                if val < cur_from:
                    sliders[axis_name].configure(from_=val - 100)
                elif val > cur_to:
                    sliders[axis_name].configure(to=val + 100)
                sliders[axis_name].set(val)
                if format_text:
                    entries[axis_name].delete(0, "end")
                    entries[axis_name].insert(0, f"{val:.2f}")
                self._updating_link_ui = False
        else:
            rot = cur_list
            rot[axis_idx] = val
            self.viewer.update_link_offset(self.selected_link_key, rot=rot)
            
            if axis_name in sliders:
                self._updating_link_ui = True
                sliders[axis_name].set(val)
                if format_text:
                    entries[axis_name].delete(0, "end")
                    entries[axis_name].insert(0, f"{val:.2f}")
                self._updating_link_ui = False

    def _reset_selected_link_transform(self):
        """Resets the transform for the selected link to zero."""
        self.viewer.reset_link_transform(self.selected_link_key)
        self._load_selected_link_to_ui()

    def _save_robot_links_config(self):
        """Persists current robot links configuration to defaults.json."""
        from tkinter import messagebox
        self.config.save_config()
        messagebox.showinfo("Thành công", "Đã lưu cấu hình cơ cấu và khớp robot thành công!")

    # -------------------------------------------------------------------------
    # 3D Interactive Picking Callback
    # -------------------------------------------------------------------------

    def _on_3d_part_picked(self, stl_name):
        """Callback when user clicks directly on any 3D STL part in the VTK viewport."""
        def update_ui():
            try:
                base_name = os.path.basename(stl_name)
                cfg = self.viewer.part_configs.get(base_name, {})
                link_key = cfg.get("link_key", "Base")
                
                # 1. Switch to Robot Model tab
                if hasattr(self, "tabview"):
                    self.tabview.set("Robot Model")
                    
                # 2. Select link in link dropdown and part in STL dropdown
                self.selected_link_key = link_key
                self.selected_stl_name = base_name
                if hasattr(self, "link_selector") and link_key in self.link_key_to_display:
                    self.link_selector.set(self.link_key_to_display[link_key])
                    
                # 3. Load UI controls with this link's parameters
                self._load_selected_link_to_ui()
                self._activate_tkinter_focus()
            except Exception as e:
                print(f"[ERROR] _on_3d_part_picked error: {e}")
                
        self.after(0, update_ui)

    # -------------------------------------------------------------------------
    # Robot Model Profile Management
    # -------------------------------------------------------------------------

    def _refresh_model_list_ui(self):
        """Refreshes model dropdown list and active model label."""
        model_names = list(self.config.get_saved_models().keys())
        active = self.config.get_active_model_name()
        if hasattr(self, "preset_menu"):
            self.preset_menu.configure(values=model_names if model_names else ["(Chưa có model)"])
            self.preset_menu.set(active)
        if hasattr(self, "active_model_lbl"):
            self.active_model_lbl.configure(text=f"Đang dùng: {active}")

    def _on_new_model(self):
        """Creates a blank new robot model: clears 3D scene and resets all 7 link slots."""
        from tkinter import messagebox, simpledialog
        
        name = simpledialog.askstring(
            "Tạo Model Mới",
            "Nhập tên model mới:\n(Sẽ khởi tạo model trống để nạp các file STL mới)",
            initialvalue="Robot Model Mới"
        )
        if not name or not name.strip():
            return
        name = name.strip()

        # Build blank 7-link template — no STL files, all offsets zero
        blank_links = {
            "Base":   {"name": "Base (Khớp đế)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "None", "color": "Silver", "scale": 1.0, "opacity": 1.0, "visible": True},
            "Link 1": {"name": "Khớp 1 (J1)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "+Z", "color": "Silver", "scale": 1.0, "opacity": 1.0, "visible": True},
            "Link 2": {"name": "Khớp 2 (J2)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "+Z", "color": "Silver", "scale": 1.0, "opacity": 1.0, "visible": True},
            "Link 3": {"name": "Khớp 3 (J3)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "+Y", "color": "Silver", "scale": 1.0, "opacity": 1.0, "visible": True},
            "Link 4": {"name": "Khớp 4 (J4)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "+Y", "color": "Silver", "scale": 1.0, "opacity": 1.0, "visible": True},
            "Link 5": {"name": "Khớp 5 (J5)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "+X", "color": "Silver", "scale": 1.0, "opacity": 1.0, "visible": True},
            "Link 6": {"name": "Khớp 6 (J6)", "stl_files": [],
                       "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                       "joint_axis": "+Y", "color": "Silver", "scale": 1.0, "opacity": 1.0, "visible": True},
        }

        # Write blank model into config
        self.config.config_data["current_robot_model"] = name
        self.config.config_data["current_robot_model_type"] = "custom"
        self.config.config_data["robot_links"] = blank_links
        self.config.config_data["custom_stl_objects"] = []
        
        # Save it immediately as a user model
        self.config.save_current_as_model(name, description=f"Model trống tạo mới: {name}")

        # Clear 3D scene
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
            "Các bước tiếp theo:\n"
            "1. Chọn từng Khớp (Base, Link 1 -> Link 6)\n"
            "2. Bấm '✚ Nạp / Đổi STL' để nạp file STL tương ứng\n"
            "3. Chọn Trục quay khớp và kéo slider để Dời Trục / Định Hướng\n"
            "4. Bấm '💾 Lưu cấu hình khớp' hoặc '💾 Lưu Mới'"
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
            self.viewer.reload_robot()
            self._load_selected_link_to_ui()
            self._reload_dh_entries_from_config()
            self._update_coordinates()
            messagebox.showinfo("Thành công", f"Đã nhập model '{result}' thành công!")
        else:
            messagebox.showerror("Lỗi", result)

    def _reload_dh_entries_from_config(self):
        """Refreshes DH parameter entry boxes from ConfigManager."""
        param_keys = ["Θ", "α", "d", "a"]
        for row_idx in range(6):
            for param in param_keys:
                if hasattr(self, "dh_entries") and (row_idx+1, param) in self.dh_entries:
                    cfg_key = f"J{row_idx+1}{param}DHpar"
                    val = self.config.get(cfg_key, "0.0")
                    entry = self.dh_entries[(row_idx+1, param)]
                    entry.delete(0, "end")
                    entry.insert(0, str(val))

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
