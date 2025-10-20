import carb
import omni.ext

from .mcp_server import MCPNetworkServer  # Use relative imports
from scene.scene_manager import SceneManager  # Import the new classes


class MCPExtension(omni.ext.IExt):
    _instance = None  # This will hold the SceneManager instance

    @classmethod
    def get_instance(cls):
        """Gets the singleton instance of the SceneManager."""
        return cls._instance

    def on_startup(self, ext_id: str):
        print(f"MCPExtension starting up: {ext_id}")

        # 1. Create the SceneManager - this is the core service
        self.scene_manager = SceneManager()
        MCPExtension._instance = self.scene_manager
        print("SceneManager instance created and registered.")

        # 2. Get server settings
        settings = carb.settings.get_settings()
        host = settings.get("/exts/isaac.sim.mcp/server.host") or "localhost"
        port = settings.get("/exts/isaac.sim.mcp/server.port") or 8766

        # 3. Create and start the network server, giving it the SceneManager
        self.server = MCPNetworkServer(self.scene_manager, host, port)
        self.server.start()

    def on_shutdown(self):
        print("MCPExtension shutting down.")

        # Stop the server first
        if hasattr(self, "server"):
            self.server.stop()

        # Clean up
        self.server = None
        self.scene_manager = None
        MCPExtension._instance = None
