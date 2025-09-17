2025-09-03 20:28:10,713 - root - INFO - log_manager.py:60 - LogManager: Logging automatically configured. Log file at: /home/ubuntu/PycharmProjects/multiagent-isaacsim/logs/app.md
2025-09-03 20:28:10,733 - ros.ros_manager - INFO - ros_manager.py:38 - ROS nodes built successfully.
2025-09-03 20:28:12,111 - ros.ros_manager - INFO - ros_manager.py:49 - ROS manager thread started.
2025-09-03 20:28:12,111 - webmanager.process_manager - INFO - process_manager.py:171 - Health monitoring started (interval: 30.0s)
2025-09-03 20:28:19,141 - __main__ - INFO - main.py:64 - Received SIGINT (Ctrl+C), initiating graceful shutdown...
2025-09-03 20:28:19,142 - webmanager.process_manager - INFO - process_manager.py:245 - Graceful shutdown requested
2025-09-03 20:28:19,142 - webmanager.process_manager - INFO - process_manager.py:255 - Executing 2 shutdown callbacks...
2025-09-03 20:28:19,195 - ros.ros_manager - INFO - ros_manager.py:75 - ROS executor shutdown complete.
2025-09-03 20:28:19,195 - ros.ros_manager - INFO - ros_manager.py:84 - ROS manager stopped.
2025-09-03 20:28:19,196 - webmanager.process_manager - INFO - process_manager.py:264 - All shutdown callbacks completed
2025-09-03 20:28:19,196 - __main__ - INFO - main.py:84 - Immediate shutdown requested
2025-09-03 20:28:19,196 - webmanager.process_manager - INFO - process_manager.py:280 - Cleaning up process manager resources...
2025-09-03 20:28:19,196 - webmanager.process_manager - INFO - process_manager.py:296 - Process manager cleanup completed
2025-09-03 20:31:01,334 - root - INFO - log_manager.py:90 - LogManager: Logging automatically configured. Log file resolved to: /home/ubuntu/PycharmProjects/multiagent-isaacsim/logs/app.md
2025-09-03 20:31:01,355 - ros.ros_manager - INFO - ros_manager.py:38 - ROS nodes built successfully.
2025-09-03 20:31:02,856 - ros.ros_manager - INFO - ros_manager.py:49 - ROS manager thread started.
2025-09-03 20:31:02,856 - webmanager.process_manager - INFO - process_manager.py:171 - Health monitoring started (interval: 30.0s)
2025-09-03 20:31:10,542 - __main__ - INFO - main.py:533 - Starting WebManager system...
2025-09-03 20:31:10,542 - __main__ - INFO - main.py:109 - Initializing WebManager system...
2025-09-03 20:31:10,542 - __main__ - INFO - main.py:110 - WebManager configuration: host=0.0.0.0, port=8080
2025-09-03 20:31:10,542 - __main__ - INFO - main.py:113 - Data collection rate: 5.0Hz, max history: 1000
2025-09-03 20:31:10,542 - webmanager.integration - INFO - integration.py:61 - WebManagerSystem created (host=0.0.0.0, port=8080)
2025-09-03 20:31:10,542 - webmanager.integration - INFO - integration.py:80 - Initializing WebManager system...
2025-09-03 20:31:10,543 - webmanager.chart_generator - INFO - chart_generator.py:55 - ChartDataGenerator initialized with 500 trajectory points, 200 performance points, and 300.0s trajectory window
2025-09-03 20:31:10,543 - webmanager.data_collector - INFO - data_collector.py:191 - DataCollector initialized with 1000 max history, 5.0Hz collection rate, and 2.0Hz chart update rate
2025-09-03 20:31:10,546 - webmanager.robot_data_collector - INFO - robot_data_collector.py:75 - Loaded 0 robots from config
2025-09-03 20:31:10,546 - webmanager.robot_data_collector - INFO - robot_data_collector.py:48 - SwarmManagerRobotDataSource initialized with 0 robots
2025-09-03 20:31:10,546 - webmanager.integration - INFO - integration.py:127 - Robot data source initialized with swarm manager
2025-09-03 20:31:10,547 - webmanager.performance_data_collector - INFO - performance_data_collector.py:73 - IsaacSimPerformanceDataSource initialized
2025-09-03 20:31:10,547 - webmanager.integration - INFO - integration.py:141 - Performance data source initialized
2025-09-03 20:31:10,547 - webmanager.ros_data_collector - INFO - ros_data_collector.py:55 - ROS2GraphDataSource initialized (ROS2 available: True)
2025-09-03 20:31:10,547 - webmanager.integration - INFO - integration.py:145 - ROS data source initialized
2025-09-03 20:31:10,547 - webmanager.camera_data_collector - INFO - camera_data_collector.py:143 - Discovered cameras: ['default', 'viewport', 'viewport_Viewport_Robot_2', 'viewport_Viewport', 'scene_camera_0', 'scene_camera_1', 'scene_camera_2', 'scene_camera_3', 'scene_camera_4', 'scene_camera_5', 'jetbot_0_camera', 'jetbot_1_camera', 'h1_0_camera', 'cf2x_0_camera']
2025-09-03 20:31:10,547 - webmanager.camera_data_collector - INFO - camera_data_collector.py:64 - IsaacSimCameraDataSource initialized with 14 cameras
2025-09-03 20:31:10,547 - webmanager.integration - INFO - integration.py:153 - Camera data source initialized
2025-09-03 20:31:10,547 - webmanager.data_collector - INFO - data_collector.py:196 - Robot data source registered
2025-09-03 20:31:10,547 - webmanager.data_collector - INFO - data_collector.py:201 - Performance data source registered
2025-09-03 20:31:10,547 - webmanager.data_collector - INFO - data_collector.py:206 - ROS data source registered
2025-09-03 20:31:10,547 - webmanager.data_collector - INFO - data_collector.py:211 - Camera data source registered
2025-09-03 20:31:10,547 - webmanager.integration - INFO - integration.py:169 - All data sources registered with data collector
2025-09-03 20:31:10,547 - webmanager.web_server - INFO - web_server.py:530 - Data collector set for chart generation
2025-09-03 20:31:10,547 - webmanager.camera_data_collector - INFO - camera_data_collector.py:873 - CameraStreamManager initialized
2025-09-03 20:31:10,547 - webmanager.integration - INFO - integration.py:114 - WebManager system initialized successfully
2025-09-03 20:31:10,547 - __main__ - INFO - main.py:142 - WebManager system initialized successfully
2025-09-03 20:31:10,547 - __main__ - INFO - main.py:143 - WebManager web interface will be available at http://0.0.0.0:8080
2025-09-03 20:31:10,547 - webmanager.process_manager - INFO - process_manager.py:312 - Process status changed to: running
2025-09-03 20:31:10,547 - __main__ - INFO - main.py:554 - WebManager system started successfully
2025-09-03 20:31:10,547 - __main__ - INFO - main.py:555 - Access the web interface at: http://0.0.0.0:8080
2025-09-03 20:31:10,547 - __main__ - INFO - main.py:570 - Starting main simulation loop...
2025-09-03 20:32:06,065 - __main__ - INFO - main.py:64 - Received SIGINT (Ctrl+C), initiating graceful shutdown...
2025-09-03 20:32:06,065 - webmanager.process_manager - INFO - process_manager.py:245 - Graceful shutdown requested
2025-09-03 20:32:06,065 - webmanager.process_manager - INFO - process_manager.py:255 - Executing 2 shutdown callbacks...
2025-09-03 20:32:06,065 - __main__ - INFO - main.py:195 - Initiating WebManager system shutdown...
2025-09-03 20:32:06,065 - webmanager.integration - WARNING - integration.py:199 - WebManager system is not running
2025-09-03 20:32:06,065 - __main__ - INFO - main.py:210 - WebManager final status: {'initialized': True, 'running': False, 'web_server': {'host': '0.0.0.0', 'port': 8080, 'active_connections': 0}, 'data_collection': {'active': False, 'collection_rate': 5.0, 'max_history': 1000, 'stats': {'total_collections': 0, 'failed_collections': 0, 'last_collection_time': 0, 'average_collection_time': 0}}, 'data_sources': {'robot_data_source': True, 'performance_data_source': True, 'ros_data_source': True, 'camera_data_source': True}, 'camera_streaming': {}}
2025-09-03 20:32:06,065 - __main__ - INFO - main.py:233 - WebManager system stopped successfully
2025-09-03 20:32:06,093 - ros.ros_manager - INFO - ros_manager.py:75 - ROS executor shutdown complete.
2025-09-03 20:32:06,094 - ros.ros_manager - INFO - ros_manager.py:84 - ROS manager stopped.
2025-09-03 20:32:06,094 - webmanager.process_manager - INFO - process_manager.py:264 - All shutdown callbacks completed
2025-09-03 20:32:06,094 - __main__ - INFO - main.py:84 - Immediate shutdown requested
2025-09-03 20:32:06,094 - webmanager.process_manager - INFO - process_manager.py:280 - Cleaning up process manager resources...
2025-09-03 20:32:06,094 - webmanager.process_manager - INFO - process_manager.py:296 - Process manager cleanup completed
2025-09-03 20:32:06,112 - __main__ - INFO - main.py:625 - Shutdown requested, exiting main loop
2025-09-03 20:32:06,112 - __main__ - INFO - main.py:630 - Starting graceful shutdown...
2025-09-03 20:32:06,112 - __main__ - INFO - main.py:634 - Stopping WebManager system...
2025-09-03 20:32:06,113 - __main__ - INFO - main.py:195 - Initiating WebManager system shutdown...
2025-09-03 20:32:06,113 - __main__ - INFO - main.py:233 - WebManager system stopped successfully
2025-09-03 20:32:06,113 - ros.ros_manager - INFO - ros_manager.py:84 - ROS manager stopped.
2025-09-03 20:32:06,113 - __main__ - INFO - main.py:639 - Simulation loop ended, starting cleanup...
2025-09-03 20:32:06,113 - __main__ - INFO - main.py:643 - Cleaning up process manager...
2025-09-03 20:32:06,113 - webmanager.process_manager - INFO - process_manager.py:312 - Process status changed to: stopping
2025-09-03 20:32:06,113 - __main__ - INFO - main.py:648 - Waiting for graceful shutdown (timeout: 10.0s)...
2025-09-03 20:32:06,113 - __main__ - INFO - main.py:655 - Final process status: uptime=64.8s, memory=5872.2MB, errors=0
2025-09-03 20:32:06,113 - webmanager.process_manager - INFO - process_manager.py:280 - Cleaning up process manager resources...
2025-09-03 20:32:06,113 - webmanager.process_manager - INFO - process_manager.py:296 - Process manager cleanup completed
2025-09-03 20:32:06,113 - __main__ - INFO - main.py:663 - Process manager cleanup completed
2025-09-03 20:32:06,114 - __main__ - INFO - main.py:671 - --- Simulation finished. Manually closing application. ---
2025-09-03 21:02:48,763 - root - INFO - log_manager.py:90 - LogManager: Logging automatically configured. Log file resolved to: /home/ubuntu/PycharmProjects/multiagent-isaacsim/logs/app.md
2025-09-03 21:02:48,783 - ros.ros_manager - INFO - ros_manager.py:38 - ROS nodes built successfully.
2025-09-03 21:03:17,644 - root - INFO - log_manager.py:90 - LogManager: Logging automatically configured. Log file resolved to: /home/ubuntu/PycharmProjects/multiagent-isaacsim/logs/app.md
2025-09-03 21:03:17,664 - ros.ros_manager - INFO - ros_manager.py:38 - ROS nodes built successfully.
2025-09-03 21:03:19,061 - ros.ros_manager - INFO - ros_manager.py:49 - ROS manager thread started.
2025-09-03 21:03:19,062 - webmanager.process_manager - INFO - process_manager.py:171 - Health monitoring started (interval: 30.0s)
2025-09-03 21:03:27,085 - __main__ - INFO - main.py:533 - Starting WebManager system...
2025-09-03 21:03:27,085 - __main__ - INFO - main.py:109 - Initializing WebManager system...
2025-09-03 21:03:27,085 - __main__ - INFO - main.py:110 - WebManager configuration: host=0.0.0.0, port=8080
2025-09-03 21:03:27,085 - __main__ - INFO - main.py:113 - Data collection rate: 5.0Hz, max history: 1000
2025-09-03 21:03:27,085 - webmanager.integration - INFO - integration.py:61 - WebManagerSystem created (host=0.0.0.0, port=8080)
2025-09-03 21:03:27,085 - webmanager.integration - INFO - integration.py:80 - Initializing WebManager system...
2025-09-03 21:03:27,085 - webmanager.chart_generator - INFO - chart_generator.py:55 - ChartDataGenerator initialized with 500 trajectory points, 200 performance points, and 300.0s trajectory window
2025-09-03 21:03:27,086 - webmanager.data_collector - INFO - data_collector.py:191 - DataCollector initialized with 1000 max history, 5.0Hz collection rate, and 2.0Hz chart update rate
2025-09-03 21:03:27,089 - webmanager.robot_data_collector - INFO - robot_data_collector.py:75 - Loaded 0 robots from config
2025-09-03 21:03:27,089 - webmanager.robot_data_collector - INFO - robot_data_collector.py:48 - SwarmManagerRobotDataSource initialized with 0 robots
2025-09-03 21:03:27,089 - webmanager.integration - INFO - integration.py:127 - Robot data source initialized with swarm manager
2025-09-03 21:03:27,089 - webmanager.performance_data_collector - INFO - performance_data_collector.py:73 - IsaacSimPerformanceDataSource initialized
2025-09-03 21:03:27,089 - webmanager.integration - INFO - integration.py:141 - Performance data source initialized
2025-09-03 21:03:27,089 - webmanager.ros_data_collector - INFO - ros_data_collector.py:55 - ROS2GraphDataSource initialized (ROS2 available: True)
2025-09-03 21:03:27,089 - webmanager.integration - INFO - integration.py:145 - ROS data source initialized
2025-09-03 21:03:27,089 - webmanager.camera_data_collector - INFO - camera_data_collector.py:143 - Discovered cameras: ['default', 'viewport', 'viewport_Viewport_Robot_2', 'viewport_Viewport', 'scene_camera_0', 'scene_camera_1', 'scene_camera_2', 'scene_camera_3', 'scene_camera_4', 'scene_camera_5', 'jetbot_0_camera', 'jetbot_1_camera', 'h1_0_camera', 'cf2x_0_camera']
2025-09-03 21:03:27,089 - webmanager.camera_data_collector - INFO - camera_data_collector.py:64 - IsaacSimCameraDataSource initialized with 14 cameras
2025-09-03 21:03:27,089 - webmanager.integration - INFO - integration.py:153 - Camera data source initialized
2025-09-03 21:03:27,089 - webmanager.data_collector - INFO - data_collector.py:196 - Robot data source registered
2025-09-03 21:03:27,089 - webmanager.data_collector - INFO - data_collector.py:201 - Performance data source registered
2025-09-03 21:03:27,090 - webmanager.data_collector - INFO - data_collector.py:206 - ROS data source registered
2025-09-03 21:03:27,090 - webmanager.data_collector - INFO - data_collector.py:211 - Camera data source registered
2025-09-03 21:03:27,090 - webmanager.integration - INFO - integration.py:169 - All data sources registered with data collector
2025-09-03 21:03:27,090 - webmanager.web_server - INFO - web_server.py:530 - Data collector set for chart generation
2025-09-03 21:03:27,090 - webmanager.camera_data_collector - INFO - camera_data_collector.py:873 - CameraStreamManager initialized
2025-09-03 21:03:27,090 - webmanager.integration - INFO - integration.py:114 - WebManager system initialized successfully
2025-09-03 21:03:27,090 - __main__ - INFO - main.py:142 - WebManager system initialized successfully
2025-09-03 21:03:27,090 - __main__ - INFO - main.py:143 - WebManager web interface will be available at http://0.0.0.0:8080
2025-09-03 21:03:27,090 - webmanager.process_manager - INFO - process_manager.py:312 - Process status changed to: running
2025-09-03 21:03:27,090 - __main__ - INFO - main.py:554 - WebManager system started successfully
2025-09-03 21:03:27,090 - __main__ - INFO - main.py:555 - Access the web interface at: http://0.0.0.0:8080
2025-09-03 21:03:27,090 - __main__ - INFO - main.py:570 - Starting main simulation loop...
2025-09-03 21:03:29,608 - __main__ - INFO - main.py:64 - Received SIGINT (Ctrl+C), initiating graceful shutdown...
2025-09-03 21:03:29,608 - webmanager.process_manager - INFO - process_manager.py:245 - Graceful shutdown requested
2025-09-03 21:03:29,608 - webmanager.process_manager - INFO - process_manager.py:255 - Executing 2 shutdown callbacks...
2025-09-03 21:03:29,608 - __main__ - INFO - main.py:195 - Initiating WebManager system shutdown...
2025-09-03 21:03:29,608 - webmanager.integration - WARNING - integration.py:199 - WebManager system is not running
2025-09-03 21:03:29,609 - __main__ - INFO - main.py:210 - WebManager final status: {'initialized': True, 'running': False, 'web_server': {'host': '0.0.0.0', 'port': 8080, 'active_connections': 0}, 'data_collection': {'active': False, 'collection_rate': 5.0, 'max_history': 1000, 'stats': {'total_collections': 0, 'failed_collections': 0, 'last_collection_time': 0, 'average_collection_time': 0}}, 'data_sources': {'robot_data_source': True, 'performance_data_source': True, 'ros_data_source': True, 'camera_data_source': True}, 'camera_streaming': {}}
2025-09-03 21:03:29,609 - __main__ - INFO - main.py:233 - WebManager system stopped successfully
2025-09-03 21:03:29,618 - ros.ros_manager - INFO - ros_manager.py:75 - ROS executor shutdown complete.
2025-09-03 21:03:29,619 - ros.ros_manager - INFO - ros_manager.py:84 - ROS manager stopped.
2025-09-03 21:03:29,619 - webmanager.process_manager - INFO - process_manager.py:264 - All shutdown callbacks completed
2025-09-03 21:03:29,619 - __main__ - INFO - main.py:84 - Immediate shutdown requested
2025-09-03 21:03:29,619 - webmanager.process_manager - INFO - process_manager.py:280 - Cleaning up process manager resources...
2025-09-03 21:03:29,619 - webmanager.process_manager - INFO - process_manager.py:296 - Process manager cleanup completed
2025-09-03 21:03:29,637 - __main__ - INFO - main.py:625 - Shutdown requested, exiting main loop
2025-09-03 21:03:29,637 - __main__ - INFO - main.py:630 - Starting graceful shutdown...
2025-09-03 21:03:29,637 - __main__ - INFO - main.py:634 - Stopping WebManager system...
2025-09-03 21:03:29,637 - __main__ - INFO - main.py:195 - Initiating WebManager system shutdown...
2025-09-03 21:03:29,637 - __main__ - INFO - main.py:233 - WebManager system stopped successfully
2025-09-03 21:03:29,637 - ros.ros_manager - INFO - ros_manager.py:84 - ROS manager stopped.
2025-09-03 21:03:29,637 - __main__ - INFO - main.py:639 - Simulation loop ended, starting cleanup...
2025-09-03 21:03:29,637 - __main__ - INFO - main.py:643 - Cleaning up process manager...
2025-09-03 21:03:29,637 - webmanager.process_manager - INFO - process_manager.py:312 - Process status changed to: stopping
2025-09-03 21:03:29,637 - __main__ - INFO - main.py:648 - Waiting for graceful shutdown (timeout: 10.0s)...
2025-09-03 21:03:29,638 - __main__ - INFO - main.py:655 - Final process status: uptime=12.0s, memory=4683.7MB, errors=0
2025-09-03 21:03:29,638 - webmanager.process_manager - INFO - process_manager.py:280 - Cleaning up process manager resources...
2025-09-03 21:03:29,638 - webmanager.process_manager - INFO - process_manager.py:296 - Process manager cleanup completed
2025-09-03 21:03:29,638 - __main__ - INFO - main.py:663 - Process manager cleanup completed
2025-09-03 21:03:29,639 - __main__ - INFO - main.py:671 - --- Simulation finished. Manually closing application. ---
2025-09-03 21:19:58,478 - root - INFO - log_manager.py:90 - LogManager: Logging automatically configured. Log file resolved to: /home/ubuntu/PycharmProjects/multiagent-isaacsim/logs/app.md
2025-09-03 21:19:58,499 - ros.ros_manager - INFO - ros_manager.py:38 - ROS nodes built successfully.
2025-09-03 21:19:59,926 - ros.ros_manager - INFO - ros_manager.py:49 - ROS manager thread started.
2025-09-03 21:20:08,053 - __main__ - INFO - main.py:317 - Starting main simulation loop...
2025-09-03 21:20:52,297 - root - INFO - log_manager.py:90 - LogManager: Logging automatically configured. Log file resolved to: /home/ubuntu/PycharmProjects/multiagent-isaacsim/logs/app.md
2025-09-03 21:20:52,317 - ros.ros_manager - INFO - ros_manager.py:38 - ROS nodes built successfully.
2025-09-03 21:20:53,763 - ros.ros_manager - INFO - ros_manager.py:49 - ROS manager thread started.
2025-09-03 21:21:01,413 - __main__ - INFO - main.py:323 - Starting main simulation loop...
2025-09-03 21:21:10,571 - ros.ros_manager - INFO - ros_manager.py:75 - ROS executor shutdown complete.
