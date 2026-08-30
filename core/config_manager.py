import os
import json

class ConfigManager:
    def __init__(self, config_path=None):
        if config_path is None:
            # Default to adjacent config directory
            base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.config_path = os.path.join(base_dir, "config", "defaults.json")
        else:
            self.config_path = config_path
            
        self.config_data = {}
        self.load_config()

    def load_config(self):
        """Loads configuration from defaults.json."""
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, "r", encoding="utf-8") as f:
                    self.config_data = json.load(f)
            except Exception as e:
                print(f"Error loading configuration: {e}")
                self.config_data = {}
        else:
            print(f"Configuration file not found at {self.config_path}")
            self.config_data = {}

    def save_config(self):
        """Saves current configuration to defaults.json."""
        try:
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
            with open(self.config_path, "w", encoding="utf-8") as f:
                json.dump(self.config_data, f, indent=4)
            return True
        except Exception as e:
            print(f"Error saving configuration: {e}")
            return False

    def get(self, key, default=None):
        """Retrieves a configuration value."""
        return self.config_data.get(key, default)

    def set(self, key, value):
        """Sets a configuration value and saves it."""
        self.config_data[key] = value
        self.save_config()

    def get_dh_parameters(self):
        """Helper to return formatted DH parameters for J1-J6."""
        dh = {
            "theta": [
                float(self.get("J1ΘDHpar", 0)),
                float(self.get("J2ΘDHpar", 0)),
                float(self.get("J3ΘDHpar", 0)),
                float(self.get("J4ΘDHpar", 0)),
                float(self.get("J5ΘDHpar", 0)),
                float(self.get("J6ΘDHpar", 0))
            ],
            "alpha": [
                float(self.get("J1αDHpar", 0)),
                float(self.get("J2αDHpar", 0)),
                float(self.get("J3αDHpar", 0)),
                float(self.get("J4αDHpar", 0)),
                float(self.get("J5αDHpar", 0)),
                float(self.get("J6αDHpar", 0))
            ],
            "d": [
                float(self.get("J1dDHpar", 0)),
                float(self.get("J2dDHpar", 0)),
                float(self.get("J3dDHpar", 0)),
                float(self.get("J4dDHpar", 0)),
                float(self.get("J5dDHpar", 0)),
                float(self.get("J6dDHpar", 0))
            ],
            "a": [
                float(self.get("J1aDHpar", 0)),
                float(self.get("J2aDHpar", 0)),
                float(self.get("J3aDHpar", 0)),
                float(self.get("J4aDHpar", 0)),
                float(self.get("J5aDHpar", 0)),
                float(self.get("J6aDHpar", 0))
            ]
        }
        return dh

    def get_robot_links_defaults(self):
        """Returns the default configuration for the 7 robot links (Base, Link 1 -> Link 6)."""
        return {
            "Base": {
                "name": "Base (Khớp đế)",
                "stl_files": ["Link Base-1.STL", "Link Base-2.STL", "Link Base-3.STL"],
                "offset_pos": [0.0, 0.0, 0.0],
                "offset_rot": [0.0, 0.0, 0.0],
                "joint_axis": "None",
                "color": "Silver",
                "scale": 1.0,
                "opacity": 1.0
            },
            "Link 1": {
                "name": "Khớp 1 (J1)",
                "stl_files": ["Link 1-1.STL", "Link 1-2.STL"],
                "offset_pos": [0.0, 0.0, -87.5],
                "offset_rot": [180.0, 0.0, 0.0],
                "joint_axis": "-Z",
                "color": "Silver",
                "scale": 1.0,
                "opacity": 1.0
            },
            "Link 2": {
                "name": "Khớp 2 (J2)",
                "stl_files": ["Link 2-1.STL", "Link 2-2.STL", "Link 2-3.STL"],
                "offset_pos": [-64.15, 77.78, 8.87],
                "offset_rot": [270.0, 0.0, 180.0],
                "joint_axis": "+Z",
                "color": "Silver",
                "scale": 1.0,
                "opacity": 1.0
            },
            "Link 3": {
                "name": "Khớp 3 (J3)",
                "stl_files": ["Link 3-1.STL", "Link 3-2.STL"],
                "offset_pos": [0.0, 305.0, -27.84],
                "offset_rot": [180.0, 0.0, 180.0],
                "joint_axis": "-Z",
                "color": "Silver",
                "scale": 1.0,
                "opacity": 1.0
            },
            "Link 4": {
                "name": "Khớp 4 (J4)",
                "stl_files": ["Link 4-1.STL", "Link 4-2.STL", "Link 4-3.STL"],
                "offset_pos": [-36.7, 0.0, -75.94],
                "offset_rot": [180.0, 90.0, 0.0],
                "joint_axis": "-Z",
                "color": "Silver",
                "scale": 1.0,
                "opacity": 1.0
            },
            "Link 5": {
                "name": "Khớp 5 (J5)",
                "stl_files": ["Link 5-1.STL", "Link 5-2.STL"],
                "offset_pos": [147.0, 0.0, 44.88],
                "offset_rot": [0.0, 90.0, 180.0],
                "joint_axis": "-Z",
                "color": "Silver",
                "scale": 1.0,
                "opacity": 1.0
            },
            "Link 6": {
                "name": "Khớp 6 (J6)",
                "stl_files": ["Link 6-1.STL", "Link 6-2.STL"],
                "offset_pos": [43.3, 0.0, 25.0],
                "offset_rot": [0.0, 90.0, 0.0],
                "joint_axis": "+Z",
                "color": "Silver",
                "scale": 1.0,
                "opacity": 1.0
            }
        }

    def get_robot_links(self):
        """Retrieves robot links configuration, falling back to defaults if not yet populated."""
        links = self.get("robot_links")
        if not links or not isinstance(links, dict):
            defaults = self.get_robot_links_defaults()
            self.set("robot_links", defaults)
            return defaults
        # Ensure all 7 keys exist
        defaults = self.get_robot_links_defaults()
        for k, v in defaults.items():
            if k not in links:
                links[k] = v
        return links

    def save_robot_links(self, links_data):
        """Saves robot links configuration to file."""
        self.set("robot_links", links_data)

    def get_link_config(self, link_key):
        """Retrieves configuration for a specific link."""
        links = self.get_robot_links()
        return links.get(link_key, self.get_robot_links_defaults().get(link_key, {}))

    def set_link_config(self, link_key, link_data):
        """Updates configuration for a specific link and saves."""
        links = self.get_robot_links()
        links[link_key] = link_data
        self.save_robot_links(links)

    def get_presets(self):
        """Returns built-in presets for robot kinematics and 3D links."""
        return {
            "AR4 Standard (Mặc định)": {
                "dh": {
                    "J1ΘDHpar": "0", "J2ΘDHpar": "-90", "J3ΘDHpar": "0", "J4ΘDHpar": "0", "J5ΘDHpar": "0", "J6ΘDHpar": "180",
                    "J1αDHpar": "0", "J2αDHpar": "-90", "J3αDHpar": "0", "J4αDHpar": "-90", "J5αDHpar": "90", "J6αDHpar": "-90",
                    "J1dDHpar": "169.77", "J2dDHpar": "0", "J3dDHpar": "0", "J4dDHpar": "222.63", "J5dDHpar": "0", "J6dDHpar": "41",
                    "J1aDHpar": "0", "J2aDHpar": "64.2", "J3aDHpar": "305", "J4aDHpar": "0", "J5aDHpar": "0", "J6aDHpar": "0"
                },
                "links": self.get_robot_links_defaults()
            },
            "New Mechanism (Cơ cấu Mới 141.5-40-165-132-58)": {
                "dh": {
                    "J1ΘDHpar": "0", "J2ΘDHpar": "-90", "J3ΘDHpar": "0", "J4ΘDHpar": "0", "J5ΘDHpar": "0", "J6ΘDHpar": "0",
                    "J1αDHpar": "0", "J2αDHpar": "-90", "J3αDHpar": "0", "J4αDHpar": "-90", "J5αDHpar": "90", "J6αDHpar": "-90",
                    "J1dDHpar": "141.5", "J2dDHpar": "0", "J3dDHpar": "56.4", "J4dDHpar": "132", "J5dDHpar": "0", "J6dDHpar": "58",
                    "J1aDHpar": "0", "J2aDHpar": "40", "J3aDHpar": "165", "J4aDHpar": "0", "J5aDHpar": "0", "J6aDHpar": "0"
                },
                "links": {
                    "Base": {
                        "name": "Base (Khớp đế)",
                        "stl_files": ["Link Base-1.STL"],
                        "offset_pos": [0.0, 0.0, 0.0],
                        "offset_rot": [0.0, 0.0, 0.0],
                        "joint_axis": "None",
                        "color": "Silver",
                        "scale": 1.0,
                        "opacity": 1.0
                    },
                    "Link 1": {
                        "name": "Khớp 1 (J1)",
                        "stl_files": ["Link 1-1.STL"],
                        "offset_pos": [0.0, 0.0, 0.0],
                        "offset_rot": [0.0, 0.0, 0.0],
                        "joint_axis": "-Z",
                        "color": "Silver",
                        "scale": 1.0,
                        "opacity": 1.0
                    },
                    "Link 2": {
                        "name": "Khớp 2 (J2)",
                        "stl_files": ["Link 2-1.STL"],
                        "offset_pos": [0.0, 40.0, 141.5],
                        "offset_rot": [0.0, 0.0, 0.0],
                        "joint_axis": "+Z",
                        "color": "Orange",
                        "scale": 1.0,
                        "opacity": 1.0
                    },
                    "Link 3": {
                        "name": "Khớp 3 (J3)",
                        "stl_files": ["Link 3-1.STL"],
                        "offset_pos": [56.4, 0.0, 165.0],
                        "offset_rot": [0.0, 0.0, 0.0],
                        "joint_axis": "+Y",
                        "color": "Silver",
                        "scale": 1.0,
                        "opacity": 1.0
                    },
                    "Link 4": {
                        "name": "Khớp 4 (J4)",
                        "stl_files": ["Link 4-1.STL"],
                        "offset_pos": [0.0, 132.0, 0.0],
                        "offset_rot": [0.0, 0.0, 0.0],
                        "joint_axis": "+Y",
                        "color": "Orange",
                        "scale": 1.0,
                        "opacity": 1.0
                    },
                    "Link 5": {
                        "name": "Khớp 5 (J5)",
                        "stl_files": ["Link 5-1.STL"],
                        "offset_pos": [0.0, 0.0, 0.0],
                        "offset_rot": [0.0, 0.0, 0.0],
                        "joint_axis": "+X",
                        "color": "Silver",
                        "scale": 1.0,
                        "opacity": 1.0
                    },
                    "Link 6": {
                        "name": "Khớp 6 (J6)",
                        "stl_files": ["Link 6-1.STL"],
                        "offset_pos": [0.0, 58.0, 0.0],
                        "offset_rot": [0.0, 0.0, 0.0],
                        "joint_axis": "+Y",
                        "color": "Orange",
                        "scale": 1.0,
                        "opacity": 1.0
                    }
                }
            }
        }

    def apply_preset(self, preset_name):
        """Applies a preset to config (legacy compat - delegates to load_robot_model)."""
        return self.load_robot_model(preset_name)

    # -------------------------------------------------------------------------
    # Robot Model Profile Management (Quản lý hồ sơ mô hình robot)
    # -------------------------------------------------------------------------

    def get_builtin_models(self):
        """Returns the built-in system robot model presets (cannot be deleted)."""
        return {
            "AR4 Standard (Gốc)": {
                "type": "ar4_builtin",
                "description": "Robot AR4 6-DOF nguyên bản (18 chi tiết STL theo cụm gốc)",
                "dh_parameters": {
                    "J1ΘDHpar": "0", "J2ΘDHpar": "-90", "J3ΘDHpar": "0",
                    "J4ΘDHpar": "0", "J5ΘDHpar": "0", "J6ΘDHpar": "180",
                    "J1αDHpar": "0", "J2αDHpar": "-90", "J3αDHpar": "0",
                    "J4αDHpar": "-90", "J5αDHpar": "90", "J6αDHpar": "-90",
                    "J1dDHpar": "169.77", "J2dDHpar": "0", "J3dDHpar": "0",
                    "J4dDHpar": "222.63", "J5dDHpar": "0", "J6dDHpar": "41",
                    "J1aDHpar": "0", "J2aDHpar": "64.2", "J3aDHpar": "305",
                    "J4aDHpar": "0", "J5aDHpar": "0", "J6aDHpar": "0"
                },
                "joint_limits": {
                    "J1PosLim": "170", "J1NegLim": "170",
                    "J2PosLim": "90",  "J2NegLim": "42",
                    "J3PosLim": "52",  "J3NegLim": "89",
                    "J4PosLim": "180", "J4NegLim": "180",
                    "J5PosLim": "105", "J5NegLim": "105",
                    "J6PosLim": "180", "J6NegLim": "180"
                },
                "links": self.get_robot_links_defaults()
            },
            "Cơ cấu Mới (141.5-40-165-132-58)": {
                "type": "custom",
                "description": "Cơ cấu robot 6 bậc tùy biến theo kích thước 141.5-40-165-132-58",
                "dh_parameters": {
                    "J1ΘDHpar": "0", "J2ΘDHpar": "-90", "J3ΘDHpar": "0",
                    "J4ΘDHpar": "0", "J5ΘDHpar": "0", "J6ΘDHpar": "0",
                    "J1αDHpar": "0", "J2αDHpar": "-90", "J3αDHpar": "0",
                    "J4αDHpar": "-90", "J5αDHpar": "90", "J6αDHpar": "-90",
                    "J1dDHpar": "141.5", "J2dDHpar": "0", "J3dDHpar": "56.4",
                    "J4dDHpar": "132", "J5dDHpar": "0", "J6dDHpar": "58",
                    "J1aDHpar": "0", "J2aDHpar": "40", "J3aDHpar": "165",
                    "J4aDHpar": "0", "J5aDHpar": "0", "J6aDHpar": "0"
                },
                "joint_limits": {
                    "J1PosLim": "180", "J1NegLim": "180",
                    "J2PosLim": "180", "J2NegLim": "180",
                    "J3PosLim": "180", "J3NegLim": "180",
                    "J4PosLim": "180", "J4NegLim": "180",
                    "J5PosLim": "180", "J5NegLim": "180",
                    "J6PosLim": "180", "J6NegLim": "180"
                },
                "links": {
                    "Base":   {"name": "Base (Khớp đế)", "stl_files": ["Link Base-1.STL"],
                               "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                               "joint_axis": "None", "color": "Silver", "scale": 1.0, "opacity": 1.0},
                    "Link 1": {"name": "Khớp 1 (J1)", "stl_files": ["Link 1-1.STL"],
                               "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                               "joint_axis": "-Z", "color": "Silver", "scale": 1.0, "opacity": 1.0},
                    "Link 2": {"name": "Khớp 2 (J2)", "stl_files": ["Link 2-1.STL"],
                               "offset_pos": [0.0, 40.0, 141.5], "offset_rot": [0.0, 0.0, 0.0],
                               "joint_axis": "+Z", "color": "Orange", "scale": 1.0, "opacity": 1.0},
                    "Link 3": {"name": "Khớp 3 (J3)", "stl_files": ["Link 3-1.STL"],
                               "offset_pos": [56.4, 0.0, 165.0], "offset_rot": [0.0, 0.0, 0.0],
                               "joint_axis": "+Y", "color": "Silver", "scale": 1.0, "opacity": 1.0},
                    "Link 4": {"name": "Khớp 4 (J4)", "stl_files": ["Link 4-1.STL"],
                               "offset_pos": [0.0, 132.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                               "joint_axis": "+Y", "color": "Orange", "scale": 1.0, "opacity": 1.0},
                    "Link 5": {"name": "Khớp 5 (J5)", "stl_files": ["Link 5-1.STL"],
                               "offset_pos": [0.0, 0.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                               "joint_axis": "+X", "color": "Silver", "scale": 1.0, "opacity": 1.0},
                    "Link 6": {"name": "Khớp 6 (J6)", "stl_files": ["Link 6-1.STL"],
                               "offset_pos": [0.0, 58.0, 0.0], "offset_rot": [0.0, 0.0, 0.0],
                               "joint_axis": "+Y", "color": "Orange", "scale": 1.0, "opacity": 1.0}
                }
            }
        }

    def get_saved_models(self):
        """Returns all models: built-in + user-saved."""
        models = dict(self.get_builtin_models())
        custom_saved = self.get("saved_robot_models", {})
        if isinstance(custom_saved, dict):
            models.update(custom_saved)
        return models

    def get_active_model_name(self):
        """Gets the name of the currently active robot model."""
        return self.get("current_robot_model", "AR4 Standard (Gốc)")

    def get_active_model_data(self):
        """Gets the full model dictionary of the currently active model."""
        models = self.get_saved_models()
        name = self.get_active_model_name()
        return models.get(name, models.get("AR4 Standard (Gốc)"))

    def load_robot_model(self, model_name):
        """Loads a robot model profile into active configuration."""
        # Also support old preset names for backward compat
        name_map = {
            "AR4 Standard (Mặc định)": "AR4 Standard (Gốc)",
            "New Mechanism (Cơ cấu Mới 141.5-40-165-132-58)": "Cơ cấu Mới (141.5-40-165-132-58)",
        }
        model_name = name_map.get(model_name, model_name)

        models = self.get_saved_models()
        if model_name not in models:
            return False
        model = models[model_name]

        # 1. Apply DH Parameters
        for k, v in model.get("dh_parameters", {}).items():
            self.config_data[k] = v

        # 2. Apply Joint Limits
        for k, v in model.get("joint_limits", {}).items():
            self.config_data[k] = v

        # 3. Apply links config and model type
        self.config_data["current_robot_model"] = model_name
        self.config_data["current_robot_model_type"] = model.get("type", "custom")
        if "links" in model:
            self.config_data["robot_links"] = model["links"]

        self.save_config()
        return True

    def save_current_as_model(self, model_name, description=""):
        """Saves current state (DH params, limits, links) as a named robot model."""
        if not model_name:
            return False

        dh_params = {}
        for j in range(1, 7):
            for p in ["Θ", "α", "d", "a"]:
                key = f"J{j}{p}DHpar"
                dh_params[key] = str(self.get(key, "0"))

        joint_limits = {}
        for j in range(1, 7):
            joint_limits[f"J{j}PosLim"] = str(self.get(f"J{j}PosLim", "180"))
            joint_limits[f"J{j}NegLim"] = str(self.get(f"J{j}NegLim", "180"))

        model_data = {
            "type": "custom",
            "name": model_name,
            "description": description,
            "dh_parameters": dh_params,
            "joint_limits": joint_limits,
            "links": self.get_robot_links()
        }

        saved_models = self.get("saved_robot_models", {})
        if not isinstance(saved_models, dict):
            saved_models = {}
        saved_models[model_name] = model_data
        self.config_data["saved_robot_models"] = saved_models
        self.config_data["current_robot_model"] = model_name
        self.config_data["current_robot_model_type"] = "custom"
        self.save_config()
        return True

    def delete_saved_model(self, model_name):
        """Deletes a custom saved model (cannot delete built-in models)."""
        if model_name in self.get_builtin_models():
            return False, "Không thể xóa model mặc định của hệ thống!"

        saved_models = self.get("saved_robot_models", {})
        if isinstance(saved_models, dict) and model_name in saved_models:
            del saved_models[model_name]
            self.config_data["saved_robot_models"] = saved_models
            if self.get("current_robot_model") == model_name:
                self.load_robot_model("AR4 Standard (Gốc)")
            else:
                self.save_config()
            return True, "Đã xóa model thành công!"
        return False, "Không tìm thấy model để xóa!"

    def export_model_to_json(self, model_name, file_path):
        """Exports a robot model profile to a JSON file."""
        models = self.get_saved_models()
        if model_name not in models:
            return False
        try:
            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(models[model_name], f, indent=4, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"[ERROR] Exporting model: {e}")
            return False

    def import_model_from_json(self, file_path):
        """Imports a robot model profile from a JSON file."""
        if not os.path.exists(file_path):
            return False, "File không tồn tại!"
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                return False, "Cấu trúc file không hợp lệ!"
            model_name = data.get("name") or os.path.splitext(os.path.basename(file_path))[0]
            saved_models = self.get("saved_robot_models", {})
            if not isinstance(saved_models, dict):
                saved_models = {}
            saved_models[model_name] = data
            self.config_data["saved_robot_models"] = saved_models
            self.save_config()
            return True, model_name
        except Exception as e:
            return False, f"Lỗi đọc file JSON: {e}"
