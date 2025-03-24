# from https://docs.isaacsim.omniverse.nvidia.com/4.5.0/python_scripting/core_api_overview.html

from isaacsim import SimulationApp

# Simple example showing how to start and stop the helper
simulation_app = SimulationApp({"headless": False})

# Get the utility to enable extensions
from isaacsim.core.utils.extensions import enable_extension

# Enable the layers and stage windows in the UI
enable_extension("omni.kit.widget.stage")
enable_extension("omni.kit.widget.layers")


simulation_app.update()  # Render a single frame
# while True:
#     print("loop")
#     time.sleep(1)
#     print(time.time())

simulation_app.close()  # Cleanup application

