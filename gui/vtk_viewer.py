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
        
        # Dictionary storage for VTK components
        self.actors = {}
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


    def update_joints(self, joint_angles):
        """Updates the rotations of the robot parts.
        joint_angles: list of 6 angles in degrees
        """
        if not self.vtk_running:
            return
            
        # Match direction and scale map from original AR4 HMI
        angles = {
            "Link 1-1.STL": -joint_angles[0],
            "Link 2-1.STL": joint_angles[1],
            "Link 3-1.STL": -joint_angles[2],
            "Link 4-1.STL": -joint_angles[3],
            "Link 5-1.STL": -joint_angles[4],
            "Link 6-1.STL": joint_angles[5]
        }
        
        for stl, angle in angles.items():
            if stl in self.joint_transforms:
                jt = self.joint_transforms[stl]
                jt.Identity()
                jt.RotateZ(angle)
                
                ct = self.composite_transforms[stl]
                ct.Identity()
                ct.Concatenate(self.base_transforms[stl])
                ct.Concatenate(jt)
                
        self.render_window.Render()

    def reload_robot(self, new_stl_dir=None):
        """Reloads the robot actors, potentially from a new STL folder."""
        if not self.vtk_running:
            return
            
        # 1. Remove the old root assembly actor from the renderer
        if "Link Base-1.STL" in self.assemblies:
            root = self.assemblies["Link Base-1.STL"]
            self.renderer.RemoveActor(root)
            
        # 2. Reset STL directory if provided
        if new_stl_dir:
            self.stl_dir = new_stl_dir
            
        # 3. Clear existing components
        self.actors.clear()
        self.assemblies.clear()
        self.base_transforms.clear()
        self.joint_transforms.clear()
        self.composite_transforms.clear()
        
        # 4. Rebuild robot actors and add them
        self._build_robot_actors()
        self.rebuild_custom_objects()
        
        # 5. Apply current joint rotation values
        self.update_joints(self.root.joint_angles)
        
        # 6. Reset camera view and re-render
        self.renderer.ResetCamera()
        self.render_window.Render()

    def _build_robot_actors(self):
        """Loads STL files, applies offsets, and builds parent-child joints chain."""
        colors = vtk.vtkNamedColors()
        stl_files = list(self.color_map.keys())
        
        for stl in stl_files:
            file_path = os.path.join(self.stl_dir, stl)
            if not os.path.exists(file_path):
                print(f"[WARNING] STL File not found: {file_path}")
                continue
                
            reader = vtk.vtkSTLReader()
            reader.SetFileName(file_path)
            reader.Update()
            
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(reader.GetOutputPort())
            
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            
            # Apply color mapping
            color_name = self.color_map.get(stl, "Silver")
            actor.GetProperty().SetColor(colors.GetColor3d(color_name))
            
            base_tf = vtk.vtkTransform()
            joint_tf = vtk.vtkTransform()
            comp_tf = vtk.vtkTransform()
            
            # Hardware offsets and alignment parameters from original project
            if stl == "Link 1-1.STL":
                base_tf.RotateX(180)
                base_tf.Translate(0, 0, -87.5)
            elif stl == "Link 2-1.STL":
                base_tf.RotateZ(180)
                base_tf.RotateX(270)
                base_tf.Translate(-64.15, 77.78, 8.87)
            elif stl == "Link 3-1.STL":
                base_tf.RotateZ(180)
                base_tf.RotateX(180)
                base_tf.Translate(0, 305, -27.84)
            elif stl == "Link 4-1.STL":
                base_tf.RotateY(90)
                base_tf.RotateX(180)
                base_tf.Translate(-36.7, 0, -75.94)
            elif stl == "Link 5-1.STL":
                base_tf.RotateZ(180)
                base_tf.RotateY(90)
                base_tf.Translate(147, 0, 44.88)
            elif stl == "Link 6-1.STL":
                base_tf.RotateY(90)
                base_tf.Translate(43.3, 0, 25)
                
            comp_tf.Concatenate(base_tf)
            comp_tf.Concatenate(joint_tf)
            
            asm = vtk.vtkAssembly()
            asm.AddPart(actor)
            asm.SetUserTransform(comp_tf)
            
            self.actors[stl] = actor
            self.assemblies[stl] = asm
            self.base_transforms[stl] = base_tf
            self.joint_transforms[stl] = joint_tf
            self.composite_transforms[stl] = comp_tf

        # Build parenting hierarchy (Kinematic Tree)
        # Verify assemblies exist before chaining
        try:
            root = self.assemblies["Link Base-1.STL"]
            root.AddPart(self.assemblies["Link Base-2.STL"])
            self.assemblies["Link Base-2.STL"].AddPart(self.assemblies["Link Base-3.STL"])
            self.assemblies["Link Base-3.STL"].AddPart(self.assemblies["Link 1-1.STL"])
            self.assemblies["Link 1-1.STL"].AddPart(self.assemblies["Link 1-2.STL"])
            self.assemblies["Link 1-2.STL"].AddPart(self.assemblies["Link 2-1.STL"])
            self.assemblies["Link 2-1.STL"].AddPart(self.assemblies["Link 2-2.STL"])
            self.assemblies["Link 2-2.STL"].AddPart(self.assemblies["Link 2-3.STL"])
            self.assemblies["Link 2-3.STL"].AddPart(self.assemblies["Link 3-1.STL"])
            self.assemblies["Link 3-1.STL"].AddPart(self.assemblies["Link 3-2.STL"])
            self.assemblies["Link 3-2.STL"].AddPart(self.assemblies["Link 4-1.STL"])
            self.assemblies["Link 4-1.STL"].AddPart(self.assemblies["Link 4-2.STL"])
            self.assemblies["Link 4-2.STL"].AddPart(self.assemblies["Link 4-3.STL"])
            self.assemblies["Link 4-3.STL"].AddPart(self.assemblies["Link 5-1.STL"])
            self.assemblies["Link 5-1.STL"].AddPart(self.assemblies["Link 5-2.STL"])
            self.assemblies["Link 5-2.STL"].AddPart(self.assemblies["Link 6-1.STL"])
            self.assemblies["Link 6-1.STL"].AddPart(self.assemblies["Link 6-2.STL"])
            
            self.renderer.AddActor(root)
        except KeyError as e:
            print(f"[ERROR] Fail to build joint hierarchy. Missing STL component: {e}")

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
            target_key = parent
            if "Link 6-2" in parent or "End-Effector" in parent or "Đầu kẹp" in parent or "Link 6 (End-Effector)" in parent:
                target_key = "Link 6-2.STL"
            elif "Link 6-1" in parent:
                target_key = "Link 6-1.STL"
            elif "Link 5" in parent:
                target_key = "Link 5-2.STL"
            elif "Link 4" in parent:
                target_key = "Link 4-3.STL"
            elif "Link 3" in parent:
                target_key = "Link 3-2.STL"
            elif "Link 2" in parent:
                target_key = "Link 2-3.STL"
            elif "Link 1" in parent:
                target_key = "Link 1-2.STL"
            elif "Base" in parent:
                target_key = "Link Base-3.STL"
                
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
