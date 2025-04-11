
class Simulator:
    """SimulatorRuntime"""

    def __init__(
        self,
        config_path: str = None,
        headless: bool = True,
        # webrtc: bool = False,
        # native: bool = False,
        # config_class: Config = None,
    ):

        self.env_num = 1
        self.config_file_path = config_path
        self.config = None

        # Init Isaac Sim
        from isaacsim import SimulationApp  # noqa

        self.headless = headless
        self._simulation_app = SimulationApp(
            {'headless': self.headless, 'anti_aliasing': 0, 'hide_ui': False, 'multi_gpu': False}
        )
        print("IsaacSim Init ")


if __name__ == "__main__":
    simulator = Simulator()

