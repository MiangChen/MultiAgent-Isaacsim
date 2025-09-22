#!/usr/bin/env python3

import os
from isaacsim import SimulationApp


def initialize_simulation_app(args):
    """Initialize SimulationApp with common settings."""

    CONFIG = {
        "renderer": "RaytracedLighting",
        "headless": not args.gui,
        "anti_aliasing": 0,
    }
    print(f"Initializing Isaac Sim with headless={not args.gui}")
    exp = "isaacsim.exp.base.zero_delay.kit"
    simulation_app = SimulationApp(CONFIG, experience=f'{os.environ["EXP_PATH"]}/{exp}')

    # Set some settings to reduce the GPU/CPU usage
    simulation_app.set_setting("/rtx/ecoMode/enabled", False)
    simulation_app.set_setting("/rtx/post/dlss/execMode", 0)
    simulation_app.set_setting("/rtx-transient/dlssg/enabled", False)
    simulation_app.set_setting("/rtx-transient/dldenoiser/enabled", False)

    simulation_app.set_setting("/rtx/directLighting/enabled", False)
    simulation_app.set_setting("/rtx/indirectDiffuse/enabled", False)
    simulation_app.set_setting("/rtx/sceneDb/ambientLightIntensity", 3.0)

    simulation_app.set_setting("/rtx/reflections/enabled", False)
    simulation_app.set_setting("/rtx/translucency/enabled", False)
    simulation_app.set_setting("/rtx/raytracing/subsurface/enabled", False)
    simulation_app.set_setting("/rtx/caustics/enabled", False)

    simulation_app.set_setting("/persistent/physics/numThreads", 0)

    if not args.gui:
        # disable the viewport if not using any gui
        import omni

        omni.kit.viewport.utility.get_active_viewport().updates_enabled = False

    return simulation_app
