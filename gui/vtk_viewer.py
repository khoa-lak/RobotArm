import os
import sys
import vtk

class RobotInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    """Custom trackball interactor style that enables safe 3D mesh picking on left click without recursive event loop."""
    def __init__(self, viewer):
        super().__init__()
        self.viewer = viewer
        self.picker = vtk.vtkCellPicker()
        self.picker.SetTolerance(0.002)

    def _release_to_tkinter(self):
        try:
            import ctypes
            ctypes.windll.user32.ReleaseCapture()
            if hasattr(self.viewer, "root") and self.viewer.root:
                try:
                    root_hwnd = self.viewer.root.winfo_id()
                    if root_hwnd:
                        ctypes.windll.user32.SetFocus(int(root_hwnd))
                except Exception:
                    pass
                self.viewer.root.focus_set()
        except Exception:
            pass

    def OnLeftButtonDown(self):
        try:
            ren = self.GetDefaultRenderer() or self.viewer.renderer
            if ren and self.GetInteractor():
                pos = self.GetInteractor().GetEventPosition()
                if self.picker.Pick(pos[0], pos[1], 0, ren):
                    path = self.picker.GetPath()
                    picked_part_name = None
                    picked_obj_id = None
                    
                    if path:
                        # Search backwards from leaf node to identify the clicked actor
                        for i in range(path.GetNumberOfItems() - 1, -1, -1):
                            prop = path.GetItemAsObject(i).GetViewProp()
                            if prop == self.viewer.highlight_actor:
                                continue
                            for stl_name, act in self.viewer.part_actors.items():
                                if act == prop:
                                    picked_part_name = stl_name
                                    break
                            if picked_part_name:
                                break
                                
                            for obj_id, obj_data in self.viewer.custom_objects.items():
                                if obj_data.get("actor") == prop:
                                    picked_obj_id = obj_id
                                    break
                            if picked_obj_id:
                                break
                                
                    if picked_part_name:
                        self.viewer.highlight_part(picked_part_name)
                        if self.viewer.on_part_picked_cb:
                            self.viewer.on_part_picked_cb(picked_part_name)
                    elif picked_obj_id:
                        if self.viewer.on_object_picked_cb:
                            self.viewer.on_object_picked_cb(picked_obj_id)
        except Exception as e:
            print(f"[ERROR] 3D Pick Exception: {e}")

        # Call C++ base class method directly without recursive Python event triggering
        vtk.vtkInteractorStyleTrackballCamera.OnLeftButtonDown(self)

    def OnLeftButtonUp(self):
        vtk.vtkInteractorStyleTrackballCamera.OnLeftButtonUp(self)
        self._release_to_tkinter()

    def OnRightButtonUp(self):
        vtk.vtkInteractorStyleTrackballCamera.OnRightButtonUp(self)
        self._release_to_tkinter()

    def OnMiddleButtonUp(self):
        vtk.vtkInteractorStyleTrackballCamera.OnMiddleButtonUp(self)
        self._release_to_tkinter()


class VTKViewer:
    def __init__(self, root_widget, config_manager):
        self.root = root_widget
        self.config = config_manager
        
        self.vtk_running = False
        self.renderer = None
        self.render_window = None
        self.interactor = None
        self.is_ar4_model = True
        
        # List of 7 standard link identifiers
        self.link_keys = ["Base", "Link 1", "Link 2", "Link 3", "Link 4", "Link 5", "Link 6"]
        
        # Storage dictionaries for VTK actors, assemblies, transforms
        self.actors = {}
        self.link_actors = {}
        self.assemblies = {}
        self.joint_assemblies = {}
        self.visual_assemblies = {}
        self.base_transforms = {}
        self.joint_transforms = {}
        self.composite_transforms = {}
        
        # Per-part actor, transform, and config tracking
        self.part_actors = {}
        self.part_transforms = {}
        self.part_configs = {}
        
        # Custom standalone STL objects
        self.custom_objects = {}
        
        # Standard AR4 Color Palette
        self.color_map = {
            "Link Base-1.STL": "Silver",
            "Link Base-2.STL": "Orange",
            "Link Base-3.STL": "DimGray",
            "Link 1-1.STL": "Silver",
            "Link 1-2.STL": "DimGray",
            "Link 2-1.STL": "Silver",
            "Link 2-2.STL": "Orange",
            "Link 2-3.STL": "DimGray",
            "Link 3-1.STL": "Silver",
            "Link 3-2.STL": "DimGray",
            "Link 4-1.STL": "Silver",
            "Link 4-2.STL": "Orange",
            "Link 4-3.STL": "DimGray",
            "Link 5-1.STL": "Silver",
            "Link 5-2.STL": "DimGray",
            "Link 6-1.STL": "Silver",
            "Link 6-2.STL": "DimGray"
        }
        
        # Default Path to STL files
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.stl_dir = os.path.join(base_dir, "assets", "robot_model")
        
        # 3D Picker & Selection Highlight Callbacks
        self.on_part_picked_cb = None
        self.on_object_picked_cb = None
        self.highlight_actor = None
        self.highlight_link_key = None
        self.selected_part_name = None

    def launch(self, parent_widget):
        """Launches the VTK render window embedded in the parent Tkinter widget using Win32 SetParent."""
        if self.vtk_running:
            return
            
        self.vtk_running = True
        
        import ctypes
        from ctypes import wintypes
        import sys
        
        # 1. Create standard VTK Renderer and RenderWindow
        self.renderer = vtk.vtkRenderer()
        self.render_window = vtk.vtkRenderWindow()
        self.render_window.AddRenderer(self.renderer)
        self.render_window.BordersOff()
        
        # 2. Setup standard Interactor with custom RobotInteractorStyle
        self.interactor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetRenderWindow(self.render_window)
        style = RobotInteractorStyle(self)
        style.SetDefaultRenderer(self.renderer)
        self.interactor.SetInteractorStyle(style)
        
        self.renderer.SetBackground(vtk.vtkNamedColors().GetColor3d("LightSlateGray"))
        
        self._build_robot_actors()
        self._add_floor_grid()
        self.rebuild_custom_objects()
        
        # Configure initial camera view
        camera = self.renderer.GetActiveCamera()
        self.renderer.ResetCamera()
        camera.Dolly(2.5)
        camera.Azimuth(65)
        camera.Elevation(35)
        camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCameraClippingRange()
        
        # 3. Initialize VTK Window (generates HWND)
        self.interactor.Initialize()
        self.render_window.Render()
        
        # 4. Reparent using Win32 API
        hwnd_child = self.render_window.GetGenericWindowId()
        if isinstance(hwnd_child, str):
            hwnd_child_int = int(hwnd_child.strip('_').split('_')[0], 16)
        else:
            hwnd_child_int = int(hwnd_child)
            
        hwnd_parent = parent_widget.winfo_id()
        
        is_64bit = sys.maxsize > 2**32
        user32 = ctypes.windll.user32
        
        # Configure ctypes argument types for 32/64-bit safety
        user32.SetParent.argtypes = [wintypes.HWND, wintypes.HWND]
        user32.SetParent.restype = wintypes.HWND
        
        if is_64bit and hasattr(user32, "SetWindowLongPtrW"):
            SetWindowLong = user32.SetWindowLongPtrW
            GetWindowLong = user32.GetWindowLongPtrW
            LONG_PTR = ctypes.c_int64
        else:
            SetWindowLong = user32.SetWindowLongW
            GetWindowLong = user32.GetWindowLongW
            LONG_PTR = ctypes.c_long
            
        GetWindowLong.argtypes = [wintypes.HWND, ctypes.c_int]
        GetWindowLong.restype = LONG_PTR
        SetWindowLong.argtypes = [wintypes.HWND, ctypes.c_int, LONG_PTR]
        SetWindowLong.restype = LONG_PTR
        
        user32.MoveWindow.argtypes = [wintypes.HWND, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, wintypes.BOOL]
        user32.MoveWindow.restype = wintypes.BOOL
        
        # Perform Reparenting
        user32.SetParent(hwnd_child_int, int(hwnd_parent))
        
        # Change Window Styles to WS_CHILD | WS_VISIBLE
        GWL_STYLE = -16
        WS_CHILD = 0x40000000
        WS_POPUP = 0x80000000
        WS_VISIBLE = 0x10000000
        
        style_val = GetWindowLong(hwnd_child_int, GWL_STYLE)
        style_val = (style_val & ~WS_POPUP) | WS_CHILD | WS_VISIBLE
        SetWindowLong(hwnd_child_int, GWL_STYLE, style_val)
        
        # 5. Define Resize Event Handler
        def on_resize(event):
            if event is not None:
                canvas = getattr(parent_widget, '_canvas', None)
                target = canvas if canvas is not None else parent_widget
                if event.widget != target:
                    return
            w = parent_widget.winfo_width()
            h = parent_widget.winfo_height()
            if self.vtk_running and self.render_window:
                user32.MoveWindow(hwnd_child_int, 0, 0, w, h, True)
                self.render_window.SetSize(w, h)
                self.render_window.Render()
                
        parent_widget.bind("<Configure>", on_resize)
        on_resize(None)

    def _is_ar4_active(self):
        """Checks if the currently active model is the built-in AR4 model based on model type and STL filenames."""
        active_model = self.config.get_active_model_data() if hasattr(self.config, "get_active_model_data") else {}
        if isinstance(active_model, dict) and active_model.get("type") == "ar4_builtin":
            links = self.config.get_robot_links()
            base_files = links.get("Base", {}).get("stl_files", []) if links else []
            for f in base_files:
                if "Link Base-" in os.path.basename(f):
                    return True
            if not base_files:
                return True
        return False

    def _create_ar4_base_tf(self, link_key):
        """Creates the original Annin Robotics AR4 relative base transformation for each link."""
        tf = vtk.vtkTransform()
        tf.Identity()
        if link_key == "Link 1":
            tf.RotateX(180)
            tf.Translate(0, 0, -87.5)
        elif link_key == "Link 2":
            tf.RotateZ(180)
            tf.RotateX(270)
            tf.Translate(-64.15, 77.78, 8.87)
        elif link_key == "Link 3":
            tf.RotateZ(180)
            tf.RotateX(180)
            tf.Translate(0, 305, -27.84)
        elif link_key == "Link 4":
            tf.RotateY(90)
            tf.RotateX(180)
            tf.Translate(-36.7, 0, -75.94)
        elif link_key == "Link 5":
            tf.RotateZ(180)
            tf.RotateY(90)
            tf.Translate(147, 0, 44.88)
        elif link_key == "Link 6":
            tf.RotateY(90)
            tf.Translate(43.3, 0, 25)
        return tf

    def _build_robot_actors(self):
        """Builds robot actors supporting both built-in AR4 kinematics and user-defined direct joint offsets."""
        colors = vtk.vtkNamedColors()
        links_cfg = self.config.get_robot_links()
        is_ar4 = self._is_ar4_active()

        self.link_actors = {}
        self.assemblies = {}
        self.joint_assemblies = {}
        self.visual_assemblies = {}
        self.base_transforms = {}
        self.joint_transforms = {}
        self.composite_transforms = {}
        
        # 1. Create Assembly for each link
        for link_key in self.link_keys:
            link_asm = vtk.vtkAssembly()
            
            self.assemblies[link_key] = link_asm
            self.joint_assemblies[link_key] = link_asm
            self.visual_assemblies[link_key] = link_asm
            
            cfg = links_cfg.get(link_key, {})
            stl_files = cfg.get("stl_files", [])
            default_color = cfg.get("color", "Silver")
            
            actors_list = []
            
            for stl_item in stl_files:
                if os.path.isabs(stl_item):
                    file_path = stl_item
                else:
                    file_path = os.path.join(self.stl_dir, stl_item)
                    
                if not os.path.exists(file_path):
                    continue
                    
                stl_name = os.path.basename(file_path)
                try:
                    reader = vtk.vtkSTLReader()
                    reader.SetFileName(file_path)
                    reader.Update()
                    
                    mapper = vtk.vtkPolyDataMapper()
                    mapper.SetInputConnection(reader.GetOutputPort())
                    
                    actor = vtk.vtkActor()
                    actor.SetMapper(mapper)
                    
                    # Read per-part customization
                    saved_part_cfg = self.config.get_part_config(link_key, stl_name)
                    part_color = saved_part_cfg.get("color", self.color_map.get(stl_name, default_color))
                    part_visible = saved_part_cfg.get("visible", True)
                    part_pos = saved_part_cfg.get("pos", [0.0, 0.0, 0.0])
                    part_rot = saved_part_cfg.get("rot", [0.0, 0.0, 0.0])
                    part_scale = saved_part_cfg.get("scale", 1.0)
                    
                    try:
                        actor.GetProperty().SetColor(colors.GetColor3d(part_color))
                    except Exception:
                        actor.GetProperty().SetColor(0.75, 0.75, 0.75)
                    actor.SetVisibility(1 if part_visible else 0)
                    
                    # Per-part transform
                    part_tf = vtk.vtkTransform()
                    part_tf.Identity()
                    if part_pos != [0.0, 0.0, 0.0] or part_rot != [0.0, 0.0, 0.0] or part_scale != 1.0:
                        part_tf.Translate(part_pos[0], part_pos[1], part_pos[2])
                        part_tf.RotateZ(part_rot[2])
                        part_tf.RotateY(part_rot[1])
                        part_tf.RotateX(part_rot[0])
                        if part_scale != 1.0:
                            part_tf.Scale(part_scale, part_scale, part_scale)
                    actor.SetUserTransform(part_tf)
                    
                    link_asm.AddPart(actor)
                    actors_list.append((actor, file_path))
                    self.actors[stl_name] = actor
                    self.part_actors[stl_name] = actor
                    self.part_transforms[stl_name] = part_tf
                    self.part_configs[stl_name] = {
                        "pos": part_pos,
                        "rot": part_rot,
                        "scale": part_scale,
                        "color": part_color,
                        "visible": part_visible,
                        "file_path": file_path,
                        "link_key": link_key
                    }
                except Exception as e:
                    print(f"[ERROR] Failed to load STL {file_path}: {e}")
                    
            self.link_actors[link_key] = actors_list
            
            # Setup link base transforms
            if is_ar4:
                base_tf = self._create_ar4_base_tf(link_key)
            else:
                pos = cfg.get("offset_pos", [0.0, 0.0, 0.0])
                rot = cfg.get("offset_rot", [0.0, 0.0, 0.0])
                scale = cfg.get("scale", 1.0)
                base_tf = vtk.vtkTransform()
                self._apply_link_base_transform(base_tf, pos, rot, scale)
                
            joint_tf = vtk.vtkTransform()
            joint_tf.Identity()
            
            comp_tf = vtk.vtkTransform()
            comp_tf.Identity()
            comp_tf.Concatenate(base_tf)
            comp_tf.Concatenate(joint_tf)
            
            link_asm.SetUserTransform(comp_tf)
            self.base_transforms[link_key] = base_tf
            self.joint_transforms[link_key] = joint_tf
            self.composite_transforms[link_key] = comp_tf

        # 2. Build Kinematic Joint Chain: Base -> Link 1 -> Link 2 -> Link 3 -> Link 4 -> Link 5 -> Link 6 -> End-Effector
        ee_asm = vtk.vtkAssembly()
        self.joint_assemblies["End-Effector"] = ee_asm
        self.assemblies["End-Effector"] = ee_asm
        if "Link 6" in self.assemblies:
            self.assemblies["Link 6"].AddPart(ee_asm)
        
        for i in range(len(self.link_keys) - 1):
            parent_key = self.link_keys[i]
            child_key = self.link_keys[i + 1]
            if parent_key in self.assemblies and child_key in self.assemblies:
                self.assemblies[parent_key].AddPart(self.assemblies[child_key])
                
        # 3. Add Root Base Joint Frame to 3D Renderer
        if "Base" in self.assemblies and self.renderer:
            self.renderer.AddActor(self.assemblies["Base"])

        # 4. Apply Initial Joint Offsets and Joint Angles
        initial_angles = getattr(self.root, "joint_angles", [0.0] * 6)
        self.update_joints(initial_angles)

    def _apply_link_base_transform(self, base_tf, pos, rot, scale=1.0):
        """Applies translation, rotation and scale to the visual mesh transform of a robot link."""
        base_tf.Identity()
        base_tf.Translate(float(pos[0]), float(pos[1]), float(pos[2]))
        base_tf.RotateZ(float(rot[2]))
        base_tf.RotateY(float(rot[1]))
        base_tf.RotateX(float(rot[0]))
        if float(scale) != 1.0:
            s = float(scale)
            base_tf.Scale(s, s, s)

    def _get_joint_index(self, link_key):
        """Returns 0-indexed joint index (0 for Link 1 .. 5 for Link 6, None for Base)."""
        if str(link_key).startswith("Link "):
            try:
                return int(str(link_key).split()[1]) - 1
            except Exception:
                return None
        return None

    def update_joints(self, joint_angles):
        """Updates the 3D robot joints using AR4 signs (for built-in AR4) or configured joint axis (for custom models)."""
        if not self.vtk_running:
            return
            
        if self._is_ar4_active():
            # AR4 joint signs from Chris Annin's AR4 project:
            # J1: -Z, J2: +Z, J3: -Z, J4: -Z, J5: -Z, J6: +Z
            signs = [-1, 1, -1, -1, -1, 1]
            for i in range(1, 7):
                link_key = f"Link {i}"
                if link_key in self.joint_transforms and link_key in self.base_transforms and link_key in self.composite_transforms:
                    idx = i - 1
                    ang = (float(joint_angles[idx]) if len(joint_angles) > idx else 0.0) * signs[idx]
                    jt = self.joint_transforms[link_key]
                    jt.Identity()
                    jt.RotateZ(ang)
                    
                    ct = self.composite_transforms[link_key]
                    ct.Identity()
                    ct.Concatenate(self.base_transforms[link_key])
                    ct.Concatenate(jt)
        else:
            links_cfg = self.config.get_robot_links()
            for i in range(1, 7):
                link_key = f"Link {i}"
                if link_key in self.joint_transforms and link_key in self.base_transforms and link_key in self.composite_transforms:
                    cfg = links_cfg.get(link_key, {})
                    axis_str = cfg.get("joint_axis", "+Z").strip().upper()
                    idx = i - 1
                    ang = float(joint_angles[idx]) if len(joint_angles) > idx else 0.0
                    
                    jt = self.joint_transforms[link_key]
                    jt.Identity()
                    if "+Z" in axis_str or axis_str == "Z":
                        jt.RotateZ(ang)
                    elif "-Z" in axis_str:
                        jt.RotateZ(-ang)
                    elif "+Y" in axis_str or axis_str == "Y":
                        jt.RotateY(ang)
                    elif "-Y" in axis_str:
                        jt.RotateY(-ang)
                    elif "+X" in axis_str or axis_str == "X":
                        jt.RotateX(ang)
                    elif "-X" in axis_str:
                        jt.RotateX(-ang)
                    else:
                        jt.RotateZ(ang)
                        
                    ct = self.composite_transforms[link_key]
                    ct.Identity()
                    ct.Concatenate(self.base_transforms[link_key])
                    ct.Concatenate(jt)
                
        if self.render_window:
            self.render_window.Render()

    def update_link_offset(self, link_key, pos=None, rot=None, scale=None, render=True):
        """Updates position/rotation offset for a link in real time and re-applies joint transform."""
        if link_key not in self.base_transforms or link_key not in self.composite_transforms:
            return False
            
        cfg = self.config.update_link_config(link_key, pos=pos, rot=rot, scale=scale)
        p = cfg.get("offset_pos", [0.0, 0.0, 0.0])
        r = cfg.get("offset_rot", [0.0, 0.0, 0.0])
        s = cfg.get("scale", 1.0)
        
        base_tf = self.base_transforms[link_key]
        self._apply_link_base_transform(base_tf, p, r, s)
        
        ct = self.composite_transforms[link_key]
        ct.Identity()
        ct.Concatenate(base_tf)
        if link_key in self.joint_transforms:
            ct.Concatenate(self.joint_transforms[link_key])
            
        if render and self.render_window:
            self.render_window.Render()
        return True

    def update_link_joint_axis(self, link_key, joint_axis, render=True):
        """Updates joint rotation axis for a link and re-applies joint transform."""
        cfg = self.config.update_link_config(link_key, joint_axis=joint_axis)
        if hasattr(self.root, "joint_angles"):
            self.update_joints(self.root.joint_angles)
        elif render and self.render_window:
            self.render_window.Render()
        return True

    def reset_link_transform(self, link_key, render=True):
        """Resets link offset position and rotation to zero."""
        return self.update_link_offset(link_key, pos=[0.0, 0.0, 0.0], rot=[0.0, 0.0, 0.0], scale=1.0, render=render)

    # -------------------------------------------------------------------------
    # Individual STL Parts Management (Dịch chuyển, Xoay, Xóa, Thêm chi tiết STL)
    # -------------------------------------------------------------------------

    def get_all_model_stl_parts(self):
        """Returns a list of dictionaries with information on all STL parts in the active robot model."""
        parts = []
        for stl_name, cfg in self.part_configs.items():
            parts.append({
                "stl_name": stl_name,
                "link_key": cfg.get("link_key", "Base"),
                "file_path": cfg.get("file_path", ""),
                "pos": list(cfg.get("pos", [0.0, 0.0, 0.0])),
                "rot": list(cfg.get("rot", [0.0, 0.0, 0.0])),
                "scale": float(cfg.get("scale", 1.0)),
                "color": cfg.get("color", "Silver"),
                "visible": bool(cfg.get("visible", True))
            })
        return parts

    def update_part_transform(self, stl_name, pos=None, rot=None, scale=None, render=True):
        """Updates translation, rotation, and scale for an individual STL part."""
        base_name = os.path.basename(stl_name)
        if base_name not in self.part_transforms:
            return False
            
        part_tf = self.part_transforms[base_name]
        cfg = self.part_configs.get(base_name, {})
        
        if pos is not None:
            cfg["pos"] = [float(p) for p in pos]
        if rot is not None:
            cfg["rot"] = [float(r) for r in rot]
        if scale is not None:
            cfg["scale"] = float(scale)
            
        p = cfg.get("pos", [0.0, 0.0, 0.0])
        r = cfg.get("rot", [0.0, 0.0, 0.0])
        s = cfg.get("scale", 1.0)
        
        part_tf.Identity()
        part_tf.Translate(p[0], p[1], p[2])
        part_tf.RotateZ(r[2])
        part_tf.RotateY(r[1])
        part_tf.RotateX(r[0])
        if s != 1.0:
            part_tf.Scale(s, s, s)
            
        link_key = cfg.get("link_key")
        if link_key:
            self.config.update_part_config(link_key, base_name, pos=p, rot=r, scale=s)
            
        if self.highlight_actor and self.selected_part_name == base_name:
            self.highlight_actor.SetUserTransform(part_tf)
            
        if render and self.render_window:
            self.render_window.Render()
        return True

    def get_part_bounds(self, stl_name):
        """Returns local bounds (xmin, xmax, ymin, ymax, zmin, zmax) and dimensions for a part."""
        base_name = os.path.basename(stl_name)
        actor = self.part_actors.get(base_name)
        if not actor or not actor.GetMapper() or not actor.GetMapper().GetInput():
            return None
        poly_data = actor.GetMapper().GetInput()
        bounds = poly_data.GetBounds()
        sx = bounds[1] - bounds[0]
        sy = bounds[3] - bounds[2]
        sz = bounds[5] - bounds[4]
        cx = (bounds[0] + bounds[1]) / 2.0
        cy = (bounds[2] + bounds[3]) / 2.0
        cz = (bounds[4] + bounds[5]) / 2.0
        return {
            "bounds": bounds,
            "size": (sx, sy, sz),
            "center": (cx, cy, cz),
            "bottom_center": (cx, cy, bounds[4])
        }

    def center_part(self, stl_name, align_bottom=False):
        """Calculates geometry offset to place geometric center (or bottom center) of part at origin."""
        info = self.get_part_bounds(stl_name)
        if not info:
            return None
        if align_bottom:
            target_pos = [-info["center"][0], -info["center"][1], -info["bounds"][4]]
        else:
            target_pos = [-info["center"][0], -info["center"][1], -info["center"][2]]
        self.update_part_transform(stl_name, pos=target_pos)
        return target_pos

    def highlight_part(self, stl_name):
        """Highlights the selected STL part with a glowing outline box in the 3D scene."""
        if not self.vtk_running or not self.renderer:
            return
        self.clear_highlight()
        base_name = os.path.basename(stl_name)
        actor = self.part_actors.get(base_name)
        if not actor or not actor.GetMapper():
            return
            
        poly_data = actor.GetMapper().GetInput()
        if not poly_data:
            return
            
        outline = vtk.vtkOutlineFilter()
        outline.SetInputData(poly_data)
        
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(outline.GetOutputPort())
        
        box_actor = vtk.vtkActor()
        box_actor.SetMapper(mapper)
        box_actor.GetProperty().SetColor(1.0, 0.84, 0.0) # Gold / Yellow glow
        box_actor.GetProperty().SetLineWidth(2.5)
        box_actor.SetUserTransform(actor.GetUserTransform())
        
        cfg = self.part_configs.get(base_name, {})
        link_key = cfg.get("link_key")
        if link_key and link_key in self.visual_assemblies:
            self.visual_assemblies[link_key].AddPart(box_actor)
        elif link_key and link_key in self.joint_assemblies:
            self.joint_assemblies[link_key].AddPart(box_actor)
        else:
            self.renderer.AddActor(box_actor)
            
        self.highlight_actor = box_actor
        self.highlight_link_key = link_key
        self.selected_part_name = base_name
        if self.render_window:
            self.render_window.Render()

    def clear_highlight(self):
        """Removes the highlight outline actor."""
        if self.highlight_actor:
            if hasattr(self, "highlight_link_key") and self.highlight_link_key:
                if self.highlight_link_key in self.visual_assemblies:
                    try:
                        self.visual_assemblies[self.highlight_link_key].RemovePart(self.highlight_actor)
                    except Exception:
                        pass
                if self.highlight_link_key in self.joint_assemblies:
                    try:
                        self.joint_assemblies[self.highlight_link_key].RemovePart(self.highlight_actor)
                    except Exception:
                        pass
            if self.renderer:
                try:
                    self.renderer.RemoveActor(self.highlight_actor)
                except Exception:
                    pass
            self.highlight_actor = None
            self.highlight_link_key = None
            self.selected_part_name = None
            if self.render_window:
                self.render_window.Render()

    def set_part_visibility(self, stl_name, visible, render=True):
        """Toggles visibility of an individual STL component."""
        base_name = os.path.basename(stl_name)
        if base_name in self.part_actors:
            actor = self.part_actors[base_name]
            actor.SetVisibility(1 if visible else 0)
            cfg = self.part_configs.get(base_name, {})
            cfg["visible"] = bool(visible)
            link_key = cfg.get("link_key")
            if link_key:
                self.config.update_part_config(link_key, base_name, visible=visible)
            if render and self.render_window:
                self.render_window.Render()
            return True
        return False

    def set_part_color(self, stl_name, color_name, render=True):
        """Sets the color of an individual STL component."""
        base_name = os.path.basename(stl_name)
        if base_name in self.part_actors:
            actor = self.part_actors[base_name]
            colors = vtk.vtkNamedColors()
            try:
                actor.GetProperty().SetColor(colors.GetColor3d(color_name))
            except Exception:
                actor.GetProperty().SetColor(0.75, 0.75, 0.75)
            cfg = self.part_configs.get(base_name, {})
            cfg["color"] = color_name
            link_key = cfg.get("link_key")
            if link_key:
                self.config.update_part_config(link_key, base_name, color=color_name)
            if render and self.render_window:
                self.render_window.Render()
            return True
        return False

    def add_stl_to_link(self, link_key, file_path, pos=None, rot=None, scale=1.0, color="Silver", render=True):
        """Adds a new STL file to a robot link and updates the 3D pipeline."""
        if not os.path.exists(file_path):
            return False
            
        stl_name = os.path.basename(file_path)
        
        # Persist to config (starts at pos=[0,0,0], rot=[0,0,0], scale=1.0 preserving CAD origin)
        self.config.add_stl_to_link(link_key, file_path)
        self.config.update_part_config(
            link_key, 
            stl_name, 
            pos=pos or [0.0, 0.0, 0.0], 
            rot=rot or [0.0, 0.0, 0.0], 
            scale=scale, 
            color=color, 
            visible=True
        )
        
        # Always track in part_configs
        self.part_configs[stl_name] = {
            "pos": pos or [0.0, 0.0, 0.0],
            "rot": rot or [0.0, 0.0, 0.0],
            "scale": scale,
            "color": color,
            "visible": True,
            "file_path": file_path,
            "link_key": link_key
        }

        # Build actor in VTK scene if running
        if self.vtk_running and self.renderer:
            try:
                reader = vtk.vtkSTLReader()
                reader.SetFileName(file_path)
                reader.Update()
                
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(reader.GetOutputPort())
                
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                
                colors = vtk.vtkNamedColors()
                try:
                    actor.GetProperty().SetColor(colors.GetColor3d(color))
                except Exception:
                    actor.GetProperty().SetColor(0.75, 0.75, 0.75)
                    
                part_tf = vtk.vtkTransform()
                part_tf.Identity()
                p = pos or [0.0, 0.0, 0.0]
                r = rot or [0.0, 0.0, 0.0]
                if p != [0.0, 0.0, 0.0] or r != [0.0, 0.0, 0.0] or scale != 1.0:
                    part_tf.Translate(p[0], p[1], p[2])
                    part_tf.RotateZ(r[2])
                    part_tf.RotateY(r[1])
                    part_tf.RotateX(r[0])
                    if scale != 1.0:
                        part_tf.Scale(scale, scale, scale)
                actor.SetUserTransform(part_tf)
                
                # Attach to target visual assembly of this link
                target_vis_asm = self.visual_assemblies.get(link_key)
                if target_vis_asm:
                    target_vis_asm.AddPart(actor)
                elif link_key in self.assemblies:
                    self.assemblies[link_key].AddPart(actor)
                else:
                    self.renderer.AddActor(actor)
                    
                self.actors[stl_name] = actor
                self.part_actors[stl_name] = actor
                self.part_transforms[stl_name] = part_tf
                if link_key not in self.link_actors:
                    self.link_actors[link_key] = []
                self.link_actors[link_key].append((actor, file_path))
                
                if render and self.render_window:
                    self.render_window.Render()
            except Exception as e:
                print(f"[ERROR] Failed to add STL {file_path} to {link_key}: {e}")
                return False
                
        return True

    def remove_stl_from_link(self, link_key, stl_name, render=True):
        """Removes an STL file from a robot link and the 3D scene."""
        base_name = os.path.basename(stl_name)
        
        # Persist removal in config
        self.config.remove_stl_from_link(link_key, stl_name)
        
        # Remove from runtime maps
        actor = self.part_actors.pop(base_name, None) or self.actors.pop(base_name, None)
        self.part_transforms.pop(base_name, None)
        self.part_configs.pop(base_name, None)
        
        if actor:
            if link_key in self.visual_assemblies:
                try:
                    self.visual_assemblies[link_key].RemovePart(actor)
                except Exception:
                    pass
            if link_key in self.assemblies:
                try:
                    self.assemblies[link_key].RemovePart(actor)
                except Exception:
                    pass
            if self.renderer:
                try:
                    self.renderer.RemoveActor(actor)
                except Exception:
                    pass
                    
        if link_key in self.link_actors:
            self.link_actors[link_key] = [(a, f) for a, f in self.link_actors[link_key] if os.path.basename(f) != base_name]
            
        if render and self.render_window:
            self.render_window.Render()
            
        return True

    # -------------------------------------------------------------------------
    # Link Level Bounds & Info
    # -------------------------------------------------------------------------

    def get_link_bounds(self, link_key):
        """Returns combined bounding box and dimensions for all STL actors in a link."""
        actors = self.link_actors.get(link_key, [])
        if not actors:
            return None
        min_x, max_x = float('inf'), float('-inf')
        min_y, max_y = float('inf'), float('-inf')
        min_z, max_z = float('inf'), float('-inf')
        found = False
        for act, _ in actors:
            if act and act.GetMapper() and act.GetMapper().GetInput():
                b = act.GetMapper().GetInput().GetBounds()
                min_x = min(min_x, b[0])
                max_x = max(max_x, b[1])
                min_y = min(min_y, b[2])
                max_y = max(max_y, b[3])
                min_z = min(min_z, b[4])
                max_z = max(max_z, b[5])
                found = True
        if not found:
            return None
        return {
            "bounds": (min_x, max_x, min_y, max_y, min_z, max_z),
            "size": (max_x - min_x, max_y - min_y, max_z - min_z)
        }

    def set_link_visibility(self, link_key, visible, render=True):
        """Sets visibility for all STL actors in a link."""
        actors = self.link_actors.get(link_key, [])
        for act, _ in actors:
            if act:
                act.SetVisibility(1 if visible else 0)
        cfg = self.config.get_link_config(link_key)
        cfg["visible"] = bool(visible)
        self.config.set_link_config(link_key, cfg)
        if render and self.render_window:
            self.render_window.Render()

    def toggle_link_visibility(self, link_key, render=True):
        """Toggles visibility for all STL actors in a link."""
        cfg = self.config.get_link_config(link_key)
        vis = not cfg.get("visible", True)
        self.set_link_visibility(link_key, vis, render=render)
        return vis

    def update_link_joint_axis(self, link_key, axis_str, render=True):
        """Updates joint axis and recalculates joint rotation."""
        cfg = self.config.get_link_config(link_key)
        cfg["joint_axis"] = axis_str
        self.config.set_link_config(link_key, cfg)
        if hasattr(self.root, "joint_angles"):
            self.update_joints(self.root.joint_angles)
        elif render and self.render_window:
            self.render_window.Render()

    def update_link_color(self, link_key, color_name, render=True):
        """Updates the visual color of all actors belonging to a robot link."""
        if link_key not in self.link_actors:
            return
        colors = vtk.vtkNamedColors()
        for actor, _ in self.link_actors[link_key]:
            try:
                actor.GetProperty().SetColor(colors.GetColor3d(color_name))
            except Exception:
                actor.GetProperty().SetColor(0.75, 0.75, 0.75)
                
        cfg = self.config.get_link_config(link_key)
        cfg["color"] = color_name
        self.config.set_link_config(link_key, cfg)
        
        if render and self.render_window:
            self.render_window.Render()

    def update_link_stl(self, link_key, stl_files):
        """Assigns new STL files to a link and reloads the robot hierarchy."""
        cfg = self.config.get_link_config(link_key)
        if isinstance(stl_files, str):
            stl_files = [stl_files]
        cfg["stl_files"] = stl_files
        self.config.set_link_config(link_key, cfg)
        self.reload_robot()

    def reset_link_transform(self, link_key, render=True):
        """Resets offset position, rotation, and scale to defaults (0,0,0) and scale=1."""
        cfg = self.config.get_link_config(link_key)
        cfg["offset_pos"] = [0.0, 0.0, 0.0]
        cfg["offset_rot"] = [0.0, 0.0, 0.0]
        cfg["scale"] = 1.0
        self.config.set_link_config(link_key, cfg)
        self.update_link_offset(link_key, pos=[0, 0, 0], rot=[0, 0, 0], scale=1.0, render=render)

    def clear_robot(self):
        """Removes ALL robot actors from the renderer and clears internal maps."""
        if not self.vtk_running or not self.renderer:
            return
            
        self.clear_highlight()
        
        for asm in list(self.joint_assemblies.values()):
            try:
                self.renderer.RemoveActor(asm)
            except Exception:
                pass
                
        for asm in list(self.visual_assemblies.values()):
            try:
                self.renderer.RemoveActor(asm)
            except Exception:
                pass

        for key, asm in list(self.assemblies.items()):
            try:
                self.renderer.RemoveActor(asm)
            except Exception:
                pass
                
        for key, actor in list(self.actors.items()):
            try:
                self.renderer.RemoveActor(actor)
            except Exception:
                pass

        self.actors.clear()
        self.part_actors.clear()
        self.part_transforms.clear()
        self.part_configs.clear()
        self.link_actors.clear()
        self.assemblies.clear()
        self.joint_assemblies.clear()
        self.visual_assemblies.clear()
        self.base_transforms.clear()
        self.joint_transforms.clear()
        self.composite_transforms.clear()

    def reload_robot(self, new_stl_dir=None):
        """Reloads the robot actors, completely clearing old ones first."""
        if not self.vtk_running:
            return

        # 1. Remove EVERY existing robot actor from renderer
        self.clear_robot()

        # 2. Reset STL directory if provided
        if new_stl_dir:
            self.stl_dir = new_stl_dir

        # 3. Rebuild robot actors for active model type
        self._build_robot_actors()
        self.rebuild_custom_objects()

        # 4. Apply current joint angles
        if hasattr(self.root, "joint_angles"):
            self.update_joints(self.root.joint_angles)

        # 5. Reset camera and re-render
        if self.render_window:
            self.renderer.ResetCamera()
            self.render_window.Render()

    def _add_floor_grid(self):
        """Adds a standard grid layout on the ground plane (Z=0)."""
        grid = vtk.vtkPolyData()
        points = vtk.vtkPoints()
        lines = vtk.vtkCellArray()
        
        size = 800
        spacing = 50
        
        for i in range(-size, size + spacing, spacing):
            p1_x = points.InsertNextPoint(i, -size, 0)
            p1_y = points.InsertNextPoint(i, size, 0)
            lines.InsertNextCell(2)
            lines.InsertCellPoint(p1_x)
            lines.InsertCellPoint(p1_y)
            
            p2_x = points.InsertNextPoint(-size, i, 0)
            p2_y = points.InsertNextPoint(size, i, 0)
            lines.InsertNextCell(2)
            lines.InsertCellPoint(p2_x)
            lines.InsertCellPoint(p2_y)
            
        grid.SetPoints(points)
        grid.SetLines(lines)
        
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(grid)
        
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.7, 0.7, 0.7)
        actor.GetProperty().SetOpacity(0.5)
        
        self.renderer.AddActor(actor)

    # ---------------------------------------------------------
    # Custom STL Objects Management (Tùy chỉnh vật thể 3D STL)
    # ---------------------------------------------------------
    def add_custom_object(self, obj_id, file_path, name=None, position=None, rotation=None, scale=1.0, color="LimeGreen", opacity=1.0, parent="World", visible=True):
        """Adds a custom STL object to the scene."""
        if position is None:
            position = [0.0, 0.0, 0.0]
        if rotation is None:
            rotation = [0.0, 0.0, 0.0]
            
        obj_data = {
            "id": obj_id,
            "name": name if name else os.path.basename(file_path),
            "file_path": file_path,
            "position": [float(p) for p in position],
            "rotation": [float(r) for r in rotation],
            "scale": float(scale),
            "color": color,
            "opacity": float(opacity),
            "parent": parent,
            "visible": bool(visible),
            "actor": None,
            "transform": None
        }
        
        self.custom_objects[obj_id] = obj_data
        
        if self.vtk_running and self.renderer:
            self._create_custom_actor(obj_id)
            if self.render_window:
                self.render_window.Render()
        return True

    def _create_custom_actor(self, obj_id):
        """Creates or recreates the VTK pipeline for a custom STL object."""
        if obj_id not in self.custom_objects:
            return
        
        obj = self.custom_objects[obj_id]
        file_path = obj["file_path"]
        
        if not os.path.exists(file_path):
            print(f"[WARNING] Custom STL not found: {file_path}")
            return
            
        try:
            reader = vtk.vtkSTLReader()
            reader.SetFileName(file_path)
            reader.Update()
            
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(reader.GetOutputPort())
            
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            
            # Setup transform
            transform = vtk.vtkTransform()
            actor.SetUserTransform(transform)
            
            obj["actor"] = actor
            obj["transform"] = transform
            
            self._apply_custom_object_appearance(obj_id)
            self._apply_custom_object_transform(obj_id)
            self._attach_custom_actor(obj_id)
            
        except Exception as e:
            print(f"[ERROR] Failed to load custom STL {file_path}: {e}")

    def _attach_custom_actor(self, obj_id):
        """Attaches the custom actor to either the renderer (World) or a robot link joint frame assembly."""
        if obj_id not in self.custom_objects:
            return
        obj = self.custom_objects[obj_id]
        actor = obj.get("actor")
        if not actor or not self.renderer:
            return
            
        parent = obj.get("parent", "World")
        
        # Detach first from wherever it might be
        self.renderer.RemoveActor(actor)
        for asm in list(self.joint_assemblies.values()) + list(self.visual_assemblies.values()) + list(self.assemblies.values()):
            try:
                asm.RemovePart(actor)
            except Exception:
                pass
                
        # Attach to target
        if parent == "World" or "World" in str(parent) or not parent:
            self.renderer.AddActor(actor)
        else:
            # Map friendly parent name to joint assembly key
            target_key = "Link 6"
            if "End-Effector" in parent or "Đầu kẹp" in parent or "Flange" in parent:
                target_key = "End-Effector" if "End-Effector" in self.joint_assemblies else "Link 6"
            elif "Link 6" in parent:
                target_key = "Link 6"
            elif "Link 5" in parent:
                target_key = "Link 5"
            elif "Link 4" in parent:
                target_key = "Link 4"
            elif "Link 3" in parent:
                target_key = "Link 3"
            elif "Link 2" in parent:
                target_key = "Link 2"
            elif "Link 1" in parent:
                target_key = "Link 1"
            elif "Base" in parent or "Đế" in parent:
                target_key = "Base"
                
            if target_key in self.joint_assemblies:
                self.joint_assemblies[target_key].AddPart(actor)
            elif target_key in self.assemblies:
                self.assemblies[target_key].AddPart(actor)
            else:
                self.renderer.AddActor(actor)

    def _apply_custom_object_transform(self, obj_id):
        """Updates the vtkTransform for a custom object based on position, rotation, scale."""
        if obj_id not in self.custom_objects:
            return
        obj = self.custom_objects[obj_id]
        transform = obj.get("transform")
        if not transform:
            return
            
        pos = obj.get("position", [0.0, 0.0, 0.0])
        rot = obj.get("rotation", [0.0, 0.0, 0.0])
        scale = obj.get("scale", 1.0)
        
        transform.Identity()
        transform.Translate(pos[0], pos[1], pos[2])
        transform.RotateZ(rot[2])
        transform.RotateY(rot[1])
        transform.RotateX(rot[0])
        transform.Scale(scale, scale, scale)

    def _apply_custom_object_appearance(self, obj_id):
        """Sets color, opacity, visibility."""
        if obj_id not in self.custom_objects:
            return
        obj = self.custom_objects[obj_id]
        actor = obj.get("actor")
        if not actor:
            return
            
        color = obj.get("color", "LimeGreen")
        opacity = obj.get("opacity", 1.0)
        visible = obj.get("visible", True)
        
        actor.SetVisibility(1 if visible else 0)
        actor.GetProperty().SetOpacity(float(opacity))
        
        colors = vtk.vtkNamedColors()
        if isinstance(color, str):
            try:
                actor.GetProperty().SetColor(colors.GetColor3d(color))
            except Exception:
                actor.GetProperty().SetColor(0.2, 0.8, 0.2)
        elif isinstance(color, (list, tuple)) and len(color) == 3:
            actor.GetProperty().SetColor(*color)

    def update_custom_object(self, obj_id, position=None, rotation=None, scale=None, color=None, opacity=None, parent=None, visible=None, name=None):
        """Updates properties of a custom STL object and renders changes."""
        if obj_id not in self.custom_objects:
            return False
            
        obj = self.custom_objects[obj_id]
        
        if position is not None:
            obj["position"] = [float(p) for p in position]
        if rotation is not None:
            obj["rotation"] = [float(r) for r in rotation]
        if scale is not None:
            obj["scale"] = float(scale)
        if color is not None:
            obj["color"] = color
        if opacity is not None:
            obj["opacity"] = float(opacity)
        if visible is not None:
            obj["visible"] = bool(visible)
        if name is not None:
            obj["name"] = name
            
        parent_changed = False
        if parent is not None and parent != obj.get("parent"):
            obj["parent"] = parent
            parent_changed = True
            
        if self.vtk_running:
            if obj.get("actor") is None:
                self._create_custom_actor(obj_id)
            else:
                self._apply_custom_object_appearance(obj_id)
                self._apply_custom_object_transform(obj_id)
                if parent_changed:
                    self._attach_custom_actor(obj_id)
            if self.render_window:
                self.render_window.Render()
        return True

    def remove_custom_object(self, obj_id):
        """Removes a custom object from the scene and data store."""
        if obj_id not in self.custom_objects:
            return False
            
        obj = self.custom_objects.pop(obj_id)
        actor = obj.get("actor")
        if actor and self.renderer:
            self.renderer.RemoveActor(actor)
            for asm in self.assemblies.values():
                try:
                    asm.RemovePart(actor)
                except Exception:
                    pass
            if self.render_window:
                self.render_window.Render()
        return True

    def clear_custom_objects(self):
        """Removes all custom objects."""
        for obj_id in list(self.custom_objects.keys()):
            self.remove_custom_object(obj_id)

    def get_custom_object_bounds(self, obj_id):
        """Returns local bounds and dimensions for a custom object."""
        if obj_id not in self.custom_objects:
            return None
        obj = self.custom_objects[obj_id]
        actor = obj.get("actor")
        if not actor or not actor.GetMapper() or not actor.GetMapper().GetInput():
            return None
        poly_data = actor.GetMapper().GetInput()
        bounds = poly_data.GetBounds()
        sx = bounds[1] - bounds[0]
        sy = bounds[3] - bounds[2]
        sz = bounds[5] - bounds[4]
        cx = (bounds[0] + bounds[1]) / 2.0
        cy = (bounds[2] + bounds[3]) / 2.0
        cz = (bounds[4] + bounds[5]) / 2.0
        return {
            "bounds": bounds,
            "size": (sx, sy, sz),
            "center": (cx, cy, cz),
            "bottom_center": (cx, cy, bounds[4])
        }

    def center_custom_object(self, obj_id, align_bottom=False):
        """Calculates geometry offset to place center (or bottom center) of custom object at origin."""
        info = self.get_custom_object_bounds(obj_id)
        if not info:
            return None
        if align_bottom:
            target_pos = [-info["center"][0], -info["center"][1], -info["bounds"][4]]
        else:
            target_pos = [-info["center"][0], -info["center"][1], -info["center"][2]]
        self.update_custom_object(obj_id, position=target_pos)
        return target_pos

    def rebuild_custom_objects(self):
        """Re-initializes all custom STL objects in the VTK scene."""
        if not self.vtk_running or not self.renderer:
            return
        for obj_id in list(self.custom_objects.keys()):
            self._create_custom_actor(obj_id)
