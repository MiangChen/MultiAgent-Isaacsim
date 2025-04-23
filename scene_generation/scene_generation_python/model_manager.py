# model_manager.py

import os
import yaml
from pathlib import Path


class ModelManager:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ModelManager, cls).__new__(cls)
            _current_dir = Path(__file__).resolve().parent
            _config_path = _current_dir.parent / "config" / "llm_cfg.yaml"
            try:
                with open(_config_path, "r") as config_file:
                    cls._config = yaml.safe_load(config_file)
            except FileNotFoundError:
                print(f"Error: Configuration file '{_config_path}' not found.")
        return cls._instance

    def allocate(self, model_family: str = "QWEN"):
        api_base = self._config["api_base"].get(model_family)
        api_key = self._config["api_key"].get(model_family)
        model = self._config["model"].get(model_family)

        if not api_base or not api_key or not model:
            raise ValueError(f"Configuration for {model_family} is incomplete or missing.")

        print(f"Loaded API base: {api_base}")  # Debug output
        print(f"Loaded API key: {api_key}")  # Debug output
        print(f"Loaded model: {model}")  # Debug output

        return api_base, api_key, model


# 单例对象
model_manager = ModelManager()
