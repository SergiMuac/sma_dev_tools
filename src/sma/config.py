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
        data[name] = {"ip": ip, "user": user, "password": password}
        self.save(data)

    def get_robot(self, name):
        data = self.load()
        return data.get(name)

# Create the singleton instance
config_manager = ConfigManager()