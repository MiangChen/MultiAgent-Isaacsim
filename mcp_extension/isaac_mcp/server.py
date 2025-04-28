"""
MIT License

Copyright (c) 2023-2025 omni-mcp

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

# isaac_sim_mcp_server.py
import time
from mcp.server.fastmcp import FastMCP, Context, Image
import socket
import json
import asyncio
import logging
from dataclasses import dataclass
from contextlib import asynccontextmanager
from typing import AsyncIterator, Dict, Any, List
import os
from pathlib import Path
import base64
from urllib.parse import urlparse

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("IsaacMCPServer")


@dataclass
class IsaacConnection:
    host: str
    port: int
    sock: socket.socket = (
        None  # Changed from 'socket' to 'sock' to avoid naming conflict
    )

    def connect(self) -> bool:
        """Connect to the Isaac addon socket server"""
        if self.sock:
            return True

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            logger.info(f"Connected to Isaac at {self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Isaac: {str(e)}")
            self.sock = None
            return False

    def disconnect(self):
        """Disconnect from the Isaac addon"""
        if self.sock:
            try:
                self.sock.close()
            except Exception as e:
                logger.error(f"Error disconnecting from Isaac: {str(e)}")
            finally:
                self.sock = None

    def receive_full_response(self, sock, buffer_size=16384):
        """Receive the complete response, potentially in multiple chunks"""
        chunks = []
        # Use a consistent timeout value that matches the addon's timeout
        sock.settimeout(300.0)  # Match the extension's timeout

        try:
            while True:
                try:
                    logger.info("Waiting for data from Isaac")
                    # time.sleep(0.5)
                    chunk = sock.recv(buffer_size)
                    if not chunk:
                        # If we get an empty chunk, the connection might be closed
                        if (
                                not chunks
                        ):  # If we haven't received anything yet, this is an error
                            raise Exception(
                                "Connection closed before receiving any data"
                            )
                        break

                    chunks.append(chunk)

                    # Check if we've received a complete JSON object
                    try:
                        data = b"".join(chunks)
                        json.loads(data.decode("utf-8"))
                        # If we get here, it parsed successfully
                        logger.info(f"Received complete response ({len(data)} bytes)")
                        return data
                    except json.JSONDecodeError:
                        # Incomplete JSON, continue receiving
                        continue
                except socket.timeout:
                    # If we hit a timeout during receiving, break the loop and try to use what we have
                    logger.warning("Socket timeout during chunked receive")
                    break
                except (ConnectionError, BrokenPipeError, ConnectionResetError) as e:
                    logger.error(f"Socket connection error during receive: {str(e)}")
                    raise  # Re-raise to be handled by the caller
        except socket.timeout:
            logger.warning("Socket timeout during chunked receive")
        except Exception as e:
            logger.error(f"Error during receive: {str(e)}")
            raise

        # If we get here, we either timed out or broke out of the loop
        # Try to use what we have
        if chunks:
            data = b"".join(chunks)
            logger.info(f"Returning data after receive completion ({len(data)} bytes)")
            try:
                # Try to parse what we have
                json.loads(data.decode("utf-8"))
                return data
            except json.JSONDecodeError:
                # If we can't parse it, it's incomplete
                raise Exception("Incomplete JSON response received")
        else:
            raise Exception("No data received")

    def send_command(
            self, command_type: str, params: Dict[str, Any] = None
    ) -> Dict[str, Any]:
        """Send a command to Isaac and return the response"""
        if not self.sock and not self.connect():
            raise ConnectionError("Not connected to Isaac")

        command = {"type": command_type, "params": params or {}}

        try:
            # Log the command being sent
            logger.info(f"Sending command: {command_type} with params: {params}")

            # Send the command
            self.sock.sendall(json.dumps(command).encode("utf-8"))
            logger.info(f"Command sent, waiting for response...")

            # Set a timeout for receiving - use the same timeout as in receive_full_response
            self.sock.settimeout(300.0)  # Match the extension's timeout

            # Receive the response using the improved receive_full_response method
            response_data = self.receive_full_response(self.sock)
            logger.info(f"Received {len(response_data)} bytes of data")

            response = json.loads(response_data.decode("utf-8"))
            logger.info(f"Response parsed, status: {response.get('status', 'unknown')}")

            if response.get("status") == "error":
                logger.error(f"Isaac error: {response.get('message')}")
                raise Exception(response.get("message", "Unknown error from Isaac"))

            return response.get("result", {})
        except socket.timeout:
            logger.error("Socket timeout while waiting for response from Isaac")
            # Don't try to reconnect here - let the get_isaac_connection handle reconnection
            # Just invalidate the current socket so it will be recreated next time
            self.sock = None
            raise Exception(
                "Timeout waiting for Isaac response - try simplifying your request"
            )
        except (ConnectionError, BrokenPipeError, ConnectionResetError) as e:
            logger.error(f"Socket connection error: {str(e)}")
            self.sock = None
            raise Exception(f"Connection to Isaac lost: {str(e)}")
        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON response from Isaac: {str(e)}")
            # Try to log what was received
            if "response_data" in locals() and response_data:
                logger.error(f"Raw response (first 200 bytes): {response_data[:200]}")
            raise Exception(f"Invalid response from Isaac: {str(e)}")
        except Exception as e:
            logger.error(f"Error communicating with Isaac: {str(e)}")
            # Don't try to reconnect here - let the get_isaac_connection handle reconnection
            self.sock = None
            raise Exception(f"Communication error with Isaac: {str(e)}")


@asynccontextmanager
async def server_lifespan(server: FastMCP) -> AsyncIterator[Dict[str, Any]]:
    """Manage server startup and shutdown lifecycle"""
    # We don't need to create a connection here since we're using the global connection
    # for resources and tools

    try:
        # Just log that we're starting up
        logger.info("IsaacMCP server starting up")

        # Try to connect to Isaac on startup to verify it's available
        try:
            # This will initialize the global connection if needed
            isaac = get_isaac_connection()
            logger.info("Successfully connected to Isaac on startup")
        except Exception as e:
            logger.warning(f"Could not connect to Isaac on startup: {str(e)}")
            logger.warning(
                "Make sure the Isaac addon is running before using Isaac resources or tools"
            )

        # Return an empty context - we're using the global connection
        yield {}
    finally:
        # Clean up the global connection on shutdown
        global _isaac_connection
        if _isaac_connection:
            logger.info("Disconnecting from Isaac Sim on shutdown")
            _isaac_connection.disconnect()
            _isaac_connection = None
        logger.info("Isaac SimMCP server shut down")


# Create the MCP server with lifespan support
mcp = FastMCP(
    "IsaacSimMCP",
    description="Isaac Sim integration through the Model Context Protocol",
    lifespan=server_lifespan,
)
_isaac_connection = None


def get_isaac_connection():
    """Get or create a persistent Isaac connection"""
    global _isaac_connection, _polyhaven_enabled  # Add _polyhaven_enabled to globals

    # If we have an existing connection, check if it's still valid
    if _isaac_connection is not None:
        try:

            return _isaac_connection
        except Exception as e:
            # Connection is dead, close it and create a new one
            logger.warning(f"Existing connection is no longer valid: {str(e)}")
            try:
                _isaac_connection.disconnect()
            except:
                pass
            _isaac_connection = None

    # Create a new connection if needed
    if _isaac_connection is None:
        _isaac_connection = IsaacConnection(host="localhost", port=8766)
        if not _isaac_connection.connect():
            logger.error("Failed to connect to Isaac")
            _isaac_connection = None
            raise Exception(
                "Could not connect to Isaac. Make sure the Isaac addon is running."
            )
        logger.info("Created new persistent connection to Isaac")

    return _isaac_connection


@mcp.tool()
def get_scene_info(ctx: Context) -> str:
    """Ping status of Isaac Sim Extension Server"""
    try:
        isaac = get_isaac_connection()
        result = isaac.send_command("get_scene_info")
        print("result: ", result)

        # Just return the JSON representation of what Isaac sent us
        return json.dumps(result, indent=2)
    except Exception as e:
        logger.error(f"Error getting scene info from Isaac: {str(e)}")
        return {
            "status": "error",
            "error": str(e),
            "message": "Error getting scene info",
        }


@mcp.tool("create_physics_scene")
def create_physics_scene(
        objects: List[Dict[str, Any]] = [],
        floor: bool = True,
        gravity: List[float] = [0, -0.981, 0],
        scene_name: str = "physics_scene",
) -> Dict[str, Any]:
    """Create a physics scene with multiple objects. Before create physics scene, you need to call get_scene_info() first to verify availability of connection.

    Args:
        objects: List of objects to create. Each object should have at least 'type' and 'position' and 'name'.()
        objects  = [
        {"path": "/World/Cube","name":"Cube1", "type": "Cube", "size": 20, "position": [0, 10, 0]},
        {"path": "/World/Sphere","name":"Sphere1", "type": "Sphere", "radius": 5, "position": [5, 20, 0]},
        {"path": "/World/Cone", "name":"Cone1", "type": "Cone", "height": 8, "radius": 3, "position": [-5, 15, 0]}
        ]
        floor: Whether to create a floor. deafult is True
        gravity: The gravity vector. Default is [0, 0, -9.81] (m/s^2).
        scene_name: The name of the scene. deafult is "physics_scene"

    Returns:
        Dictionary with result information.
    """
    params = {"objects": objects, "floor": floor}

    if gravity is not None:
        params["gravity"] = gravity
    if scene_name is not None:
        params["scene_name"] = scene_name
    try:
        # Get the global connection
        isaac = get_isaac_connection()

        result = isaac.send_command("create_physics_scene", params)
        return f"create_physics_scene successfully: {result.get('result', '')}, {result.get('message', '')}"
    except Exception as e:
        logger.error(f"Error create_physics_scene: {str(e)}")
        # return f"Error create_physics_scene: {str(e)}"
        return {
            "status": "error",
            "error": str(e),
            "message": "Error create_physics_scene",
        }


@mcp.tool("create_robot")
def create_robot(robot_type: str = "g1", position: List[float] = [0, 0, 0]) -> str:
    """Create a robot in the Isaac scene. Directly create robot prim in stage at the right position. For any creation of robot, you need to call create_physics_scene() first. call create_robot() as first attmpt beofre call execute_script().

    Args:
        robot_type: The type of robot to create. Available options:
            - "franka": Franka Emika Panda robot
            - "jetbot": NVIDIA JetBot robot
            - "carter": Carter delivery robot
            - "g1": Unitree G1 quadruped robot (default)
            - "go1": Unitree Go1 quadruped robot

    Returns:
        String with result information.
    """
    isaac = get_isaac_connection()
    result = isaac.send_command(
        "create_robot", {"robot_type": robot_type, "position": position}
    )
    return f"create_robot successfully: {result.get('result', '')}, {result.get('message', '')}"


@mcp.prompt()
def asset_creation_strategy() -> str:
    """Defines the preferred strategy for creating assets in Isaac Sim"""
    return """
    0. Before anything, always check the scene from get_scene_info(), retrive rool path of assset through return value of assets_root_path.
    1. If the scene is empty, create a physics scene with create_physics_scene()
    """


def main():
    """Run the MCP server"""
    mcp.run()


if __name__ == "__main__":
    main()
