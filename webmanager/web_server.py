"""
FastAPI WebSocket server for the WebManager system.
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
import asyncio
import json
import threading
from typing import Dict, List, Any
import time
import logging
from pathlib import Path

from .models import WebSocketMessage, MessageType, BinaryDataType, BinaryDataMetadata

logger = logging.getLogger(__name__)


class WebSocketManager:
    """Manages WebSocket connections and message broadcasting."""
    
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.data_store: Dict[str, Any] = {
            'robot_positions': {},
            'performance_metrics': {},
            'ros_graph': {'nodes': [], 'topics': [], 'connections': []},
            'chart_data': {}
        }
    
    async def connect(self, websocket: WebSocket):
        """Accept a new WebSocket connection and send initial data."""
        try:
            await websocket.accept()
            self.active_connections.append(websocket)
            await self.send_initial_data(websocket)
            
            # Log client information if available
            client_info = f"{websocket.client.host}:{websocket.client.port}" if websocket.client else "unknown"
            logger.info(f"WebSocket client connected from {client_info}. Total connections: {len(self.active_connections)}")
            
        except Exception as e:
            logger.error(f"Error accepting WebSocket connection: {e}")
            raise
    
    def disconnect(self, websocket: WebSocket):
        """Remove a WebSocket connection."""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            
            # Log client information if available
            client_info = f"{websocket.client.host}:{websocket.client.port}" if websocket.client else "unknown"
            logger.info(f"WebSocket client {client_info} disconnected. Total connections: {len(self.active_connections)}")
        else:
            logger.debug("Attempted to disconnect WebSocket that was not in active connections")
    
    async def send_initial_data(self, websocket: WebSocket):
        """Send initial data to a newly connected client."""
        message = WebSocketMessage(
            type=MessageType.INITIAL_DATA,
            data=self.data_store
        )
        try:
            await websocket.send_text(json.dumps(message.to_dict()))
        except Exception as e:
            logger.error(f"Error sending initial data: {e}")
    
    async def broadcast_update(self, data_type: str, data: Any):
        """Broadcast data update to all connected clients."""
        # Update data store
        self.data_store[data_type] = data
        
        message = WebSocketMessage(
            type=MessageType.DATA_UPDATE,
            data_type=data_type,
            data=data
        )
        
        await self._broadcast_message(message)
    
    async def broadcast_chart_data(self, chart_type: str, chart_data: Any):
        """Broadcast chart data to all connected clients."""
        message = WebSocketMessage(
            type=MessageType.CHART_DATA,
            data_type=chart_type,
            data=chart_data
        )
        
        await self._broadcast_message(message)
    
    async def send_binary_data(self, data_type: str, binary_data: bytes, metadata: Dict[str, Any] = None):
        """Send binary data to all connected clients with optional metadata."""
        if not self.active_connections:
            return
        
        header_data = {
            "type": MessageType.BINARY_DATA.value,
            "data_type": data_type,
            "size": len(binary_data),
            "timestamp": time.time()
        }
        
        # Add optional metadata
        if metadata:
            header_data["metadata"] = metadata
        
        message_header = json.dumps(header_data).encode() + b'\n'
        
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_bytes(message_header + binary_data)
            except Exception as e:
                logger.error(f"Error sending binary data to client: {e}")
                disconnected.append(connection)
        
        # Remove disconnected clients
        for conn in disconnected:
            self.disconnect(conn)
    
    async def send_compressed_image(self, image_data: bytes, format: str = "jpeg", quality: int = 80, metadata: Dict[str, Any] = None):
        """Send compressed image data with optimized encoding."""
        try:
            # If image_data is raw, we might need to compress it
            # For now, assume it's already in the desired format
            compressed_data = image_data
            
            image_metadata = {
                "format": format,
                "quality": quality,
                "compressed_size": len(compressed_data)
            }
            
            if metadata:
                image_metadata.update(metadata)
            
            await self.send_binary_data("camera_frame", compressed_data, image_metadata)
            
        except Exception as e:
            logger.error(f"Error sending compressed image: {e}")
    
    def get_connection_count(self) -> int:
        """Get the number of active connections."""
        return len(self.active_connections)
    
    def is_connected(self) -> bool:
        """Check if there are any active connections."""
        return len(self.active_connections) > 0
    
    async def _broadcast_message(self, message: WebSocketMessage):
        """Internal method to broadcast a message to all clients."""
        if not self.active_connections:
            return
        
        message_text = json.dumps(message.to_dict())
        disconnected = []
        
        for connection in self.active_connections:
            try:
                await connection.send_text(message_text)
            except Exception as e:
                logger.error(f"Error broadcasting message: {e}")
                disconnected.append(connection)
        
        # Remove disconnected clients
        for conn in disconnected:
            self.disconnect(conn)


class WebUIServer:
    """FastAPI web server for the WebManager interface."""
    
    def __init__(self, host: str = "0.0.0.0", port: int = 8080, data_collector=None):
        self.app = FastAPI(title="Isaac Sim WebManager", version="1.0.0")
        self.host = host
        self.port = port
        self.websocket_manager = WebSocketManager()
        self.server_thread = None
        self.running = False
        self.data_collector = data_collector
        
        self.setup_routes()
    
    def setup_routes(self):
        """Set up FastAPI routes and static file serving."""
        # Mount static files
        ui_path = Path(__file__).parent.parent / "UI"
        static_path = ui_path / "static"
        templates_path = ui_path / "templates"
        
        if static_path.exists():
            self.app.mount("/static", StaticFiles(directory=str(static_path)), name="static")
        
        @self.app.get("/")
        async def get_dashboard():
            """Serve the main dashboard HTML page."""
            dashboard_file = templates_path / "dashboard.html"
            if dashboard_file.exists():
                with open(dashboard_file, 'r', encoding='utf-8') as f:
                    return HTMLResponse(content=f.read())
            else:
                return HTMLResponse(content="<h1>Dashboard not found</h1>", status_code=404)
        
        @self.app.get("/health")
        async def health_check():
            """Health check endpoint."""
            return {
                "status": "healthy",
                "connections": len(self.websocket_manager.active_connections),
                "timestamp": time.time()
            }
        
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            """WebSocket endpoint for real-time communication."""
            await self.websocket_manager.connect(websocket)
            try:
                while True:
                    data = await websocket.receive_text()
                    message = json.loads(data)
                    await self.handle_websocket_message(websocket, message)
            except WebSocketDisconnect:
                self.websocket_manager.disconnect(websocket)
            except Exception as e:
                logger.error(f"WebSocket error: {e}")
                self.websocket_manager.disconnect(websocket)
    
    async def handle_websocket_message(self, websocket: WebSocket, message: Dict):
        """Handle incoming WebSocket messages from clients with enhanced chart request processing."""
        message_type = message.get("type")
        
        if message_type == "request_chart_update":
            chart_type = message.get("chart_type")
            chart_options = message.get("options", {})
            if chart_type:
                await self.handle_chart_request(chart_type, chart_options)
            else:
                logger.warning("Chart update request missing chart_type")
                
        elif message_type == "request_all_charts":
            # Handle request for all available chart types
            await self.handle_all_charts_request()
            
        elif message_type == "trajectory_config":
            # Handle trajectory-specific configuration
            await self.handle_trajectory_config(message.get("config", {}))
            
        elif message_type == "chart_config":
            # Handle chart configuration updates
            await self.handle_chart_config(message.get("config", {}))
            
        elif message_type == "camera_control":
            await self.handle_camera_control(message.get("action"))
            
        elif message_type == "ping":
            # Handle ping for connection health check
            await self.handle_ping(websocket)
            
        else:
            logger.warning(f"Unknown message type: {message_type}")
    
    async def handle_all_charts_request(self):
        """Handle request for all available chart types."""
        if not self.data_collector:
            logger.warning("No data collector available for chart generation")
            return
        
        try:
            available_types = self.data_collector.get_available_chart_types()
            
            # Generate all chart types
            for chart_type in available_types:
                await self.handle_chart_request(chart_type)
            
            logger.debug(f"Generated all {len(available_types)} chart types")
            
        except Exception as e:
            logger.error(f"Error handling all charts request: {e}")
    
    async def handle_chart_config(self, config: Dict[str, Any]):
        """Handle chart configuration updates from client."""
        try:
            # Update chart generation settings if data collector is available
            if self.data_collector:
                if "update_rate" in config:
                    rate = float(config["update_rate"])
                    if 0.1 <= rate <= 10.0:  # Reasonable bounds
                        self.data_collector.set_chart_update_rate(rate)
                        logger.info(f"Chart update rate changed to {rate}Hz")
                    else:
                        logger.warning(f"Invalid chart update rate: {rate}")
                
                if "enabled" in config:
                    enabled = bool(config["enabled"])
                    if enabled:
                        self.data_collector.enable_chart_generation()
                    else:
                        self.data_collector.disable_chart_generation()
            
            # Send acknowledgment
            ack_message = {
                "type": "chart_config_ack",
                "config": config,
                "timestamp": time.time()
            }
            await self.websocket_manager._broadcast_message(
                WebSocketMessage(type=MessageType.DATA_UPDATE, data_type="config_ack", data=ack_message)
            )
            
        except Exception as e:
            logger.error(f"Error handling chart config: {e}")
    
    async def handle_trajectory_config(self, config: Dict[str, Any]):
        """
        Handle trajectory-specific configuration updates from client.
        Supports rolling window configuration and trajectory display options.
        """
        try:
            if not self.data_collector:
                logger.warning("No data collector available for trajectory configuration")
                return
            
            # Handle trajectory window configuration
            if "window_seconds" in config:
                window_seconds = float(config["window_seconds"])
                if 10.0 <= window_seconds <= 3600.0:  # Reasonable bounds: 10s to 1 hour
                    self.data_collector.set_trajectory_window(window_seconds)
                    logger.info(f"Trajectory window set to {window_seconds} seconds")
                else:
                    logger.warning(f"Invalid trajectory window: {window_seconds}")
            
            # Handle trajectory cache management
            if config.get("clear_cache", False):
                self.data_collector.clear_trajectory_cache()
                logger.info("Trajectory cache cleared")
            
            # Handle trajectory update requests
            if config.get("force_update", False):
                await self.handle_chart_request("robot_trajectories", {"enable_rolling_window": True})
            
            # Send acknowledgment with current trajectory info
            trajectory_info = self.data_collector.get_trajectory_cache_info()
            ack_message = {
                "type": "trajectory_config_ack",
                "config": config,
                "trajectory_info": trajectory_info,
                "timestamp": time.time()
            }
            await self.websocket_manager._broadcast_message(
                WebSocketMessage(type=MessageType.DATA_UPDATE, data_type="trajectory_config_ack", data=ack_message)
            )
            
        except Exception as e:
            logger.error(f"Error handling trajectory config: {e}")
    
    async def handle_ping(self, websocket: WebSocket):
        """Handle ping message for connection health check."""
        try:
            pong_message = {
                "type": "pong",
                "timestamp": time.time(),
                "server_time": time.time()
            }
            await websocket.send_text(json.dumps(pong_message))
        except Exception as e:
            logger.error(f"Error handling ping: {e}")
    
    async def handle_chart_request(self, chart_type: str, options: Dict[str, Any] = None):
        """
        Handle chart data request from client with efficient processing and options support.
        Implements optimized chart update request handling with trajectory-specific options.
        """
        logger.debug(f"Chart update requested: {chart_type} with options: {options}")
        
        if not self.data_collector:
            logger.warning("No data collector available for chart generation")
            await self._send_error_chart(chart_type, "No data collector available")
            return
        
        try:
            # Check if chart type is valid
            available_types = self.data_collector.get_available_chart_types()
            if chart_type not in available_types:
                logger.warning(f"Invalid chart type requested: {chart_type}")
                await self._send_error_chart(chart_type, f"Invalid chart type: {chart_type}")
                return
            
            # Generate chart data using the data collector with options
            chart_options = options or {}
            chart_data = self.data_collector.generate_chart_data(chart_type, **chart_options)
            
            # Use the data collector's efficient serialization
            serialized_data = self.data_collector._serialize_chart_data(chart_data)
            
            # Send chart data to clients
            await self.send_chart_data(chart_type, serialized_data)
            
            logger.debug(f"Chart data generated and sent for type: {chart_type}")
            
        except Exception as e:
            logger.error(f"Error handling chart request for {chart_type}: {e}")
            await self._send_error_chart(chart_type, f"Generation error: {str(e)}")
    
    async def _send_error_chart(self, chart_type: str, error_message: str):
        """Send an error chart to clients."""
        empty_chart = {
            "chart_type": chart_type,
            "series": [],
            "title": f"Error: {error_message}",
            "metadata": {
                "error": True,
                "error_message": error_message,
                "generated_at": time.time()
            }
        }
        await self.send_chart_data(chart_type, empty_chart)
    
    async def handle_camera_control(self, action: str):
        """Handle camera control request from client."""
        # This will be implemented in later tasks
        logger.info(f"Camera control requested: {action}")
    
    def start_server(self):
        """Start the web server in a separate thread."""
        if self.running:
            logger.warning("Server is already running")
            return
        
        self.running = True
        self.server_thread = threading.Thread(target=self._run_server, daemon=True)
        self.server_thread.start()
        logger.info(f"WebUI server starting on {self.host}:{self.port}")
        logger.info(f"Server thread started: {self.server_thread.name}")
    
    def stop_server(self):
        """Stop the web server gracefully."""
        if not self.running:
            logger.info("Server is not running")
            return
        
        logger.info("Stopping WebUI server...")
        self.running = False
        
        # Close all WebSocket connections
        if self.websocket_manager.active_connections:
            logger.info(f"Closing {len(self.websocket_manager.active_connections)} active WebSocket connections")
            # Create a copy of the list to avoid modification during iteration
            connections = list(self.websocket_manager.active_connections)
            for connection in connections:
                try:
                    # Send a close message to the client
                    asyncio.create_task(connection.close(code=1001, reason="Server shutting down"))
                except Exception as e:
                    logger.debug(f"Error closing WebSocket connection: {e}")
            
            # Clear the connections list
            self.websocket_manager.active_connections.clear()
        
        # Wait for server thread to finish
        if self.server_thread and self.server_thread.is_alive():
            logger.info("Waiting for server thread to finish...")
            self.server_thread.join(timeout=5)
            if self.server_thread.is_alive():
                logger.warning("Server thread did not stop gracefully within 5 seconds")
            else:
                logger.info("Server thread stopped successfully")
        
        logger.info("WebUI server stopped")
    
    def _run_server(self):
        """Internal method to run the server."""
        import uvicorn
        try:
            uvicorn.run(
                self.app,
                host=self.host,
                port=self.port,
                log_level="info"
            )
        except Exception as e:
            logger.error(f"Server error: {e}")
        finally:
            self.running = False
    
    # Public API methods for updating data
    async def update_robot_positions(self, positions: Dict[str, Dict]):
        """Update robot position data and broadcast to clients."""
        await self.websocket_manager.broadcast_update("robot_positions", positions)
    
    async def update_performance_metrics(self, metrics: Dict[str, float]):
        """Update performance metrics and broadcast to clients."""
        await self.websocket_manager.broadcast_update("performance_metrics", metrics)
    
    async def update_ros_graph(self, graph_data: Dict):
        """Update ROS graph data and broadcast to clients."""
        await self.websocket_manager.broadcast_update("ros_graph", graph_data)
    
    async def send_camera_frame(self, frame_data: bytes, camera_id: str = "default", metadata: Dict[str, Any] = None):
        """Send camera frame data to clients with camera identification."""
        frame_metadata = {"camera_id": camera_id}
        if metadata:
            frame_metadata.update(metadata)
        
        await self.websocket_manager.send_binary_data("camera_frame", frame_data, frame_metadata)
    
    async def send_compressed_camera_frame(self, frame_data: bytes, camera_id: str = "default", 
                                         format: str = "jpeg", quality: int = 80, 
                                         width: int = None, height: int = None):
        """Send compressed camera frame with optimization parameters."""
        metadata = {
            "camera_id": camera_id,
            "format": format,
            "quality": quality
        }
        
        if width and height:
            metadata.update({"width": width, "height": height})
        
        await self.websocket_manager.send_compressed_image(frame_data, format, quality, metadata)
    
    async def send_chart_data(self, chart_type: str, chart_data: Dict):
        """Send chart data to clients."""
        await self.websocket_manager.broadcast_chart_data(chart_type, chart_data)
    
    def set_data_collector(self, data_collector):
        """Set the data collector for chart generation."""
        self.data_collector = data_collector
        logger.info("Data collector set for chart generation")
    
    def get_available_chart_types(self) -> List[str]:
        """Get available chart types from data collector."""
        if self.data_collector:
            return self.data_collector.get_available_chart_types()
        return []
    
    def get_server_stats(self) -> Dict[str, Any]:
        """Get server statistics for monitoring and status reporting."""
        stats = {
            "running": self.running,
            "host": self.host,
            "port": self.port,
            "active_connections": len(self.websocket_manager.active_connections),
            "server_thread_alive": self.server_thread.is_alive() if self.server_thread else False,
            "data_collector_available": self.data_collector is not None,
            "available_chart_types": self.get_available_chart_types()
        }
        
        # Add chart generation statistics if data collector is available
        if self.data_collector:
            stats.update({
                "chart_generation_enabled": self.data_collector.is_chart_generation_enabled(),
                "chart_update_rate": self.data_collector.chart_update_rate,
                "chart_generation_stats": self.data_collector.get_chart_generation_stats(),
                "collection_stats": self.data_collector.get_collection_stats(),
                "trajectory_cache_info": self.data_collector.get_trajectory_cache_info()
            })
        
        return stats
    
    def is_healthy(self) -> bool:
        """Check if the server is in a healthy state."""
        return (
            self.running and 
            (self.server_thread is None or self.server_thread.is_alive())
        )