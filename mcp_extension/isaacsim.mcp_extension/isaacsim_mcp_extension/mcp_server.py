# =============================================================================
# MCP Server Module - Model Context Protocol Network Server
# =============================================================================
#
# This module provides TCP socket communication server for the MCP extension,
# handling client connections, command delegation, and response management.
#
# =============================================================================

# Standard library imports
import json
import socket
import threading
import traceback

# Local project imports
from physics_engine.omni_utils import run_coroutine


class MCPNetworkServer:
    """
    Handles all TCP socket communication. Listens for clients, receives commands,
    delegates them to a SceneManager instance, and sends back responses.
    """

    def __init__(self, scene_manager, host="localhost", port=8766):
        self.scene_manager = scene_manager
        self.host = host
        self.port = port
        self.socket = None
        self.server_thread = None
        self.running = False

    def start(self):
        if self.running:
            print("Server is already running")
            return

        self.running = True
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.listen(1)

            self.server_thread = threading.Thread(target=self._server_loop)
            self.server_thread.daemon = True
            self.server_thread.start()
            print(f"Isaac Sim MCP server started on {self.host}:{self.port}")
        except Exception as e:
            print(f"Failed to start server: {str(e)}")
            self.stop()

    def stop(self):
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=1.0)
        print("Isaac Sim MCP server stopped")

    def _server_loop(self):
        self.socket.settimeout(1.0)
        while self.running:
            try:
                client, address = self.socket.accept()
                print(f"Connected to client: {address}")
                client_thread = threading.Thread(
                    target=self._handle_client, args=(client,)
                )
                client_thread.daemon = True
                client_thread.start()
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Error accepting connection: {str(e)}")

    def _handle_client(self, client):
        buffer = b""
        try:
            while self.running:
                data = client.recv(16384)
                if not data:
                    break
                buffer += data
                try:
                    command = json.loads(buffer.decode("utf-8"))
                    buffer = b""

                    # Schedule command execution on the main thread
                    run_coroutine(self._execute_and_respond(command, client))

                except json.JSONDecodeError:
                    continue  # Wait for more data
        except Exception as e:
            print(f"Error in client handler: {str(e)}")
        finally:
            client.close()
            print("Client disconnected")

    async def _execute_and_respond(self, command, client) -> None:
        """Wrapper to execute command via SceneManager and send response."""
        try:
            cmd_type = command.get("type")
            params = command.get("params", {})
            response = self.scene_manager.execute_command(cmd_type, params)

            client.sendall(json.dumps(response).encode("utf-8"))
        except Exception as e:
            print(f"Error executing and responding to command: {str(e)}")
            traceback.print_exc()
            error_response = {"status": "error", "message": str(e)}
            client.sendall(json.dumps(error_response).encode("utf-8"))
