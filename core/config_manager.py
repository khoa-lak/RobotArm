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
