class Server:
    
    def __init__(self, config_path: str = None):
        self._config_path = config_path
        self._simulation_app = None
        self._world = None
        self._initialize()
    
    def _initialize(self):
        from physics_engine.isaacsim_simulation_app import start_isaacsim_simulation_app
        self._simulation_app = start_isaacsim_simulation_app()
    
    def get_world(self, **kwargs):
        if self._world is None:
            from simulation.world import World
            self._world = World(self._simulation_app, **kwargs)
        return self._world
    
    def get_simulation_app(self):
        return self._simulation_app
    
    def close(self):
        if self._simulation_app:
            self._simulation_app.close()
            self._simulation_app = None
            self._world = None
