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

    def launch(self):
        """Launches the non-blocking VTK render window."""
        if self.vtk_running:
            return
            
        self.vtk_running = True
        
        self.renderer = vtk.vtkRenderer()
        self.render_window = vtk.vtkRenderWindow()
        self.render_window.SetWindowName("Robot 3D Viewer")
        
        self.interactor = vtk.vtkRenderWindowInteractor()
        self.render_window.AddRenderer(self.renderer)
        self.interactor.SetRenderWindow(self.render_window)
        
        # Interactor style (allow camera rotation)
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.interactor.SetInteractorStyle(style)
        
        self.render_window.SetSize(800, 600)
        self.renderer.SetBackground(vtk.vtkNamedColors().GetColor3d("LightSlateGray"))
        
        self._build_robot_actors()
        self._add_floor_grid()
        
        # Configure initial camera view
        camera = self.renderer.GetActiveCamera()
        self.renderer.ResetCamera()
        camera.Dolly(2.5)
        camera.Azimuth(65)
        camera.Elevation(35)
        camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCameraClippingRange()
        
        # Handle close window event
        self.interactor.AddObserver("ExitEvent", self._on_close)
        
        self.interactor.Initialize()
        self.render_window.Render()
        
        # Start the periodic update loop linked to Tkinter
        self._periodic_update()

    def _on_close(self, obj, event):
        self.vtk_running = False
        print("[INFO] VTK Viewer closed.")

    def _periodic_update(self):
        """Tkinter-driven rendering loop (Runs ~60 FPS)."""
        if self.vtk_running and self.render_window:
            self.render_window.Render()
            self.root.after(16, self._periodic_update)

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
