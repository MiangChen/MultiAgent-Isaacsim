from typing import Any

import gymnasium as gym

from scene import Simulator
class Env(gym.Env):
    """
    Gym Env for a single environment with a single learning agent.
    ----------------------------------------------------------------------
    """

    RESET_INFO_TASK_RUNTIME = 'task_runtime'

    def __init__(self, simulator: Simulator) -> None:
        self._render = None
        self._runtime = simulator
        self._robot_name = None
        self._current_task_name = None
        self._validate()

        # from grutopia.core.runner import SimulatorRunner  # noqa E402.
        #
        # self._runner = SimulatorRunner(simulator_runtime=simulator_runtime)

        # ========================= import space ============================
        # import grutopia.core.util.space as space  # noqa E402.
        #
        # self._space = space
        # self.action_space = self._get_action_space()
        # self.observation_space = self._get_observation_space()
        # ===================================================================

        # log.info(f'==================== {self._robot_name} ======================')
        return
