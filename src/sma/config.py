import json
from pathlib import Path

CONFIG_DIR = Path.home() / ".sma"
CONFIG_FILE = CONFIG_DIR / "config.json"

class ConfigManager:
    def __init__(self):
        self._ensure_config()

    def _ensure_config(self):
        if not CONFIG_DIR.exists():
            CONFIG_DIR.mkdir(parents=True)
        if not CONFIG_FILE.exists():
            with open(CONFIG_FILE, 'w') as f:
                json.dump({}, f)

    def load(self):
        with open(CONFIG_FILE, 'r') as f:
            return json.load(f)

    def save(self, data):
        with open(CONFIG_FILE, 'w') as f:
            json.dump(data, f, indent=4)

    def register_robot(self, name, ip, user, password=None):
        data = self.load()
        # Preserve existing workspaces if re-registering
        existing_ws = []
        if name in data and "workspaces" in data[name]:
            existing_ws = data[name]["workspaces"]
            
        data[name] = {
            "ip": ip, 
            "user": user, 
            "password": password,
            "workspaces": existing_ws # Default to empty or existing
        }
        self.save(data)

    def get_robot(self, name):
        data = self.load()
        return data.get(name)

    # --- NEW METHODS ---
    def add_workspace(self, robot_name, ws_path):
        data = self.load()
        if robot_name not in data:
            return False
        
        # Ensure list exists
        if "workspaces" not in data[robot_name]:
            data[robot_name]["workspaces"] = []
            
        # Avoid duplicates
        if ws_path not in data[robot_name]["workspaces"]:
            data[robot_name]["workspaces"].append(ws_path)
            self.save(data)
        return True

    def get_workspaces(self, robot_name):
        data = self.load()
        if robot_name in data:
            return data[robot_name].get("workspaces", [])
        return []

config_manager = ConfigManager()