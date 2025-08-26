import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # we can also run as headless.
import carb

settings = carb.settings.get_settings()
value = settings.get("/persistent/isaac/asset_root/default")

# print(value)