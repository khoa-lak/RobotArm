import os
import sys
import vtk

class VTKViewer:
    def __init__(self, root_widget, config_manager):
        self.root = root_widget
        self.config = config_manager
        
        self.vtk_running = False
        self.renderer = None
        self.render_window = None
        self.interactor = None
        
        # List of 7 link identifiers
        self.link_keys = ["Base", "Link 1", "Link 2", "Link 3", "Link 4", "Link 5", "Link 6"]
        
        # Dictionary storage for VTK components
        self.actors = {}
        self.link_actors = {}
        self.assemblies = {}
        self.base_transforms = {}
        self.joint_transforms = {}
        self.composite_transforms = {}
        self.custom_objects = {}
        
        # Color Map matching standard AR4 look
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
        
        # Path to STL files
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.stl_dir = os.path.join(base_dir, "assets", "robot_model")

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
        
        # 2. Setup standard Interactor
        self.interactor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetRenderWindow(self.render_window)
        style = vtk.vtkInteractorStyleTrackballCamera()
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
        
        # 6. Start the periodic event pump loop to keep interactor responsive
        def event_loop():
            if self.vtk_running and self.interactor:
                self.interactor.ProcessEvents()
                parent_widget.after(10, event_loop)
                
        event_loop()


    def _apply_link_base_transform(self, base_tf, pos, rot, scale=1.0):
        """Applies translation, rotation and scale to the base transform of a robot link."""
        base_tf.Identity()
        base_tf.Translate(float(pos[0]), float(pos[1]), float(pos[2]))
        base_tf.RotateZ(float(rot[2]))
        base_tf.RotateY(float(rot[1]))
        base_tf.RotateX(float(rot[0]))
        if float(scale) != 1.0:
            s = float(scale)
            base_tf.Scale(s, s, s)

    def update_joints(self, joint_angles):
        """Updates the rotations of the robot parts.
        joint_angles: list of 6 angles in degrees [J1, J2, J3, J4, J5, J6]
        """
        if not self.vtk_running:
            return
            
        links_cfg = self.config.get_robot_links()
        
        for i in range(1, 7):
            link_key = f"Link {i}"
            if link_key in self.joint_transforms:
                cfg = links_cfg.get(link_key, {})
                axis_str = cfg.get("joint_axis", "-Z" if i in [1, 3, 4, 5] else "+Z")
                angle = joint_angles[i - 1]
                
                jt = self.joint_transforms[link_key]
                jt.Identity()
                
                # Apply rotation based on configured axis
                if axis_str == "+Z":
                    jt.RotateZ(angle)
                elif axis_str == "-Z":
                    jt.RotateZ(-angle)
                elif axis_str == "+Y":
                    jt.RotateY(angle)
                elif axis_str == "-Y":
                    jt.RotateY(-angle)
                elif axis_str == "+X":
                    jt.RotateX(angle)
                elif axis_str == "-X":
                    jt.RotateX(-angle)
                else:
                    jt.RotateZ(angle)
                    
                ct = self.composite_transforms[link_key]
                ct.Identity()
                ct.Concatenate(self.base_transforms[link_key])
                ct.Concatenate(jt)
                
        if self.render_window:
            self.render_window.Render()

    def update_link_offset(self, link_key, pos=None, rot=None, scale=None, render=True):
        """Live updates base transform (position, rotation, scale) for a robot link."""
        if link_key not in self.base_transforms:
            return
            
        cfg = self.config.get_link_config(link_key)
        if pos is not None:
            cfg["offset_pos"] = [float(p) for p in pos]
        if rot is not None:
            cfg["offset_rot"] = [float(r) for r in rot]
        if scale is not None:
            cfg["scale"] = float(scale)
            
        base_tf = self.base_transforms[link_key]
        self._apply_link_base_transform(
            base_tf, 
            cfg.get("offset_pos", [0.0, 0.0, 0.0]), 
            cfg.get("offset_rot", [0.0, 0.0, 0.0]), 
            cfg.get("scale", 1.0)
        )
        
        ct = self.composite_transforms[link_key]
        ct.Identity()
        ct.Concatenate(base_tf)
        ct.Concatenate(self.joint_transforms[link_key])
        
        if render and self.render_window:
            self.render_window.Render()

    def update_link_joint_axis(self, link_key, axis_str, render=True):
        """Updates joint axis and recalculates joint rotation."""
        if link_key not in self.joint_transforms:
            return
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

    def reload_robot(self, new_stl_dir=None):
        """Reloads the robot actors, potentially from a new STL folder."""
        if not self.vtk_running:
            return
            
        # 1. Remove old Base actor from renderer
        if "Base" in self.assemblies:
            root = self.assemblies["Base"]
            self.renderer.RemoveActor(root)
        elif "Link Base-1.STL" in self.assemblies:
            root = self.assemblies["Link Base-1.STL"]
            self.renderer.RemoveActor(root)
            
        # 2. Reset STL directory if provided
        if new_stl_dir:
            self.stl_dir = new_stl_dir
            
        # 3. Clear existing components
        self.actors.clear()
        self.link_actors.clear()
        self.assemblies.clear()
        self.base_transforms.clear()
        self.joint_transforms.clear()
        self.composite_transforms.clear()
        
        # 4. Rebuild robot actors and add them
        self._build_robot_actors()
        self.rebuild_custom_objects()
        
        # 5. Apply current joint rotation values
        if hasattr(self.root, "joint_angles"):
            self.update_joints(self.root.joint_angles)
            
        # 6. Re-render
        if self.render_window:
            self.render_window.Render()

    def _build_robot_actors(self):
        """Loads STL files, applies offsets, and builds parent-child joints chain using config."""
        colors = vtk.vtkNamedColors()
        links_cfg = self.config.get_robot_links()
        
        self.link_actors = {}
        
        for link_key in self.link_keys:
            cfg = links_cfg.get(link_key, {})
            stl_files = cfg.get("stl_files", [])
            pos = cfg.get("offset_pos", [0.0, 0.0, 0.0])
            rot = cfg.get("offset_rot", [0.0, 0.0, 0.0])
            scale = cfg.get("scale", 1.0)
            default_color = cfg.get("color", "Silver")
            
            # Create assembly for this link
            asm = vtk.vtkAssembly()
            actors_list = []
            
            for stl_item in stl_files:
                if os.path.isabs(stl_item):
                    file_path = stl_item
                else:
                    file_path = os.path.join(self.stl_dir, stl_item)
                    
                if not os.path.exists(file_path):
                    continue
                    
                try:
                    reader = vtk.vtkSTLReader()
                    reader.SetFileName(file_path)
                    reader.Update()
                    
                    mapper = vtk.vtkPolyDataMapper()
                    mapper.SetInputConnection(reader.GetOutputPort())
                    
                    actor = vtk.vtkActor()
                    actor.SetMapper(mapper)
                    
                    # Apply color mapping
                    color_name = self.color_map.get(os.path.basename(file_path), default_color)
                    try:
                        actor.GetProperty().SetColor(colors.GetColor3d(color_name))
                    except Exception:
                        actor.GetProperty().SetColor(0.75, 0.75, 0.75)
                        
                    asm.AddPart(actor)
                    actors_list.append((actor, file_path))
                    self.actors[os.path.basename(file_path)] = actor
                except Exception as e:
                    print(f"[ERROR] Failed to load STL {file_path}: {e}")
                    
            self.link_actors[link_key] = actors_list
            
            base_tf = vtk.vtkTransform()
            self._apply_link_base_transform(base_tf, pos, rot, scale)
            
            joint_tf = vtk.vtkTransform()
            joint_tf.Identity()
            
            comp_tf = vtk.vtkTransform()
            comp_tf.Identity()
            comp_tf.Concatenate(base_tf)
            comp_tf.Concatenate(joint_tf)
            
            asm.SetUserTransform(comp_tf)
            
            self.assemblies[link_key] = asm
            self.base_transforms[link_key] = base_tf
            self.joint_transforms[link_key] = joint_tf
            self.composite_transforms[link_key] = comp_tf
            
        # Build hierarchy: Base -> Link 1 -> Link 2 -> Link 3 -> Link 4 -> Link 5 -> Link 6
        for i in range(len(self.link_keys) - 1):
            parent_key = self.link_keys[i]
            child_key = self.link_keys[i + 1]
            if parent_key in self.assemblies and child_key in self.assemblies:
                self.assemblies[parent_key].AddPart(self.assemblies[child_key])
                
        if "Base" in self.assemblies:
            self.renderer.AddActor(self.assemblies["Base"])

    def _add_floor_grid(self):
        """Adds a standard grid layout on the ground plane (Z=0)."""
        grid = vtk.vtkPolyData()
        points = vtk.vtkPoints()
        lines = vtk.vtkCellArray()
        
        size = 800
        spacing = 50
        count = 0
        
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
        actor.GetProperty().SetColor(0.7, 0.7, 0.7) # Grid color
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
        """Attaches the custom actor to either the renderer (World) or a robot link assembly."""
        if obj_id not in self.custom_objects:
            return
        obj = self.custom_objects[obj_id]
        actor = obj.get("actor")
        if not actor or not self.renderer:
            return
            
        parent = obj.get("parent", "World")
        
        # Detach first from wherever it might be
        self.renderer.RemoveActor(actor)
        for asm in self.assemblies.values():
            try:
                asm.RemovePart(actor)
            except Exception:
                pass
                
        # Attach to target
        if parent == "World" or not parent:
            self.renderer.AddActor(actor)
        else:
            # Map friendly parent name to assembly key
            target_key = "Link 6"
            if "Link 6" in parent or "End-Effector" in parent or "Đầu kẹp" in parent:
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
            elif "Base" in parent:
                target_key = "Base"
                
            if target_key in self.assemblies:
                self.assemblies[target_key].AddPart(actor)
            else:
                # Fallback to world
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

    def rebuild_custom_objects(self):
        """Re-initializes all custom STL objects in the VTK scene."""
        if not self.vtk_running or not self.renderer:
            return
        for obj_id in list(self.custom_objects.keys()):
            self._create_custom_actor(obj_id)
