// WebSocket Message Handler
// Handles parsing, routing, and processing of different message types

class WebSocketMessageHandler {
    constructor(store) {
        this.store = store;
        this.messageProcessors = new Map();
        this.validator = new MessageValidator();
        this.setupMessageProcessors();
    }

    setupMessageProcessors() {
        // Initial data processor
        this.messageProcessors.set('initial_data', (message) => {
            this.processInitialData(message);
        });

        // Real-time data update processor
        this.messageProcessors.set('data_update', (message) => {
            this.processDataUpdate(message);
        });

        // Chart data processor
        this.messageProcessors.set('chart_data', (message) => {
            this.processChartData(message);
        });

        // Binary data processor
        this.messageProcessors.set('binary_data', (message) => {
            this.processBinaryData(message);
        });

        // Error message processor
        this.messageProcessors.set('error', (message) => {
            this.processError(message);
        });

        // Status update processor
        this.messageProcessors.set('status_update', (message) => {
            this.processStatusUpdate(message);
        });
    }

    processMessage(message) {
        try {
            // Validate and sanitize message
            const validation = this.validator.validateMessage(message);
            if (!validation.valid) {
                console.error('Message validation failed:', validation.error, message);
                return;
            }

            if (validation.warning) {
                console.warn('Message validation warning:', validation.warning);
            }

            // Sanitize the message
            const sanitizedMessage = this.validator.sanitizeMessage(message);
            if (!sanitizedMessage) {
                console.error('Message sanitization failed:', message);
                return;
            }

            const processor = this.messageProcessors.get(sanitizedMessage.type);
            if (processor) {
                processor(sanitizedMessage);
            } else {
                console.warn('Unknown message type:', sanitizedMessage.type);
            }

            // Update last message time
            this.store.lastUpdateTime = new Date();
            
        } catch (error) {
            console.error('Error processing WebSocket message:', error, message);
        }
    }

    processInitialData(message) {
        const { data } = message;
        
        if (!data) {
            console.warn('Initial data message missing data field');
            return;
        }

        // Load robot positions
        if (data.robot_positions) {
            this.store.robots = this.processRobotPositions(data.robot_positions);
            console.log(`Loaded ${Object.keys(this.store.robots).length} robots`);
        }

        // Load performance metrics
        if (data.performance_metrics) {
            this.store.performance.current = this.processPerformanceMetrics(data.performance_metrics);
            console.log('Loaded performance metrics');
        }

        // Load ROS graph
        if (data.ros_graph) {
            this.store.rosGraph = {
                ...this.processRosGraph(data.ros_graph),
                lastUpdate: new Date()
            };
            console.log(`Loaded ROS graph with ${this.store.rosGraph.nodes.length} nodes`);
        }

        // Load chart data if available
        if (data.chart_data) {
            Object.entries(data.chart_data).forEach(([chartType, chartData]) => {
                this.updateChartData(chartType, chartData);
            });
        }

        console.log('Initial data processing complete');
    }

    processDataUpdate(message) {
        const { data_type, data, timestamp } = message;
        
        if (!data_type || data === undefined) {
            console.warn('Data update message missing required fields');
            return;
        }

        const updateTime = timestamp ? new Date(timestamp) : new Date();

        switch (data_type) {
            case 'robot_positions':
                this.updateRobotPositions(data, updateTime);
                break;
                
            case 'performance_metrics':
                this.updatePerformanceMetrics(data, updateTime);
                break;
                
            case 'ros_graph':
                this.updateRosGraph(data, updateTime);
                break;
                
            case 'camera_frame':
                this.updateCameraFrame(data, updateTime);
                break;
                
            default:
                console.warn('Unknown data update type:', data_type);
        }
    }

    processChartData(message) {
        const { data_type, data } = message;
        
        if (!data_type || !data) {
            console.warn('Chart data message missing required fields');
            return;
        }

        this.updateChartData(data_type, data);
        
        // Remove from pending requests
        this.store.ui.chartUpdateRequests.delete(data_type);
        
        console.log(`Chart data updated: ${data_type}`);
    }

    processBinaryData(message) {
        const { data_type, size } = message;
        
        switch (data_type) {
            case 'camera_frame':
                this.store.camera.frameCount++;
                this.store.camera.lastFrameTime = new Date();
                this.store.camera.streamActive = true;
                console.log(`Received camera frame (${size} bytes)`);
                break;
                
            default:
                console.warn('Unknown binary data type:', data_type);
        }
    }

    processError(message) {
        const { error_code, error_message, details } = message;
        
        console.error('Server error:', error_code, error_message, details);
        
        // Add to performance alerts if it's a system error
        if (error_code && error_code.startsWith('SYSTEM_')) {
            this.store.performance.alerts.push({
                type: 'error',
                message: error_message || 'System error occurred',
                timestamp: new Date(),
                details
            });
        }
    }

    processStatusUpdate(message) {
        const { component, status, details } = message;
        
        console.log(`Status update - ${component}: ${status}`, details);
        
        // Update component-specific status
        switch (component) {
            case 'simulation':
                this.store.ui.simulationStatus = status;
                break;
            case 'ros':
                this.store.ui.rosStatus = status;
                break;
            case 'camera':
                this.store.camera.status = status;
                break;
        }
    }

    // Data processing helpers
    processRobotPositions(positions) {
        const processed = {};
        
        Object.entries(positions).forEach(([robotId, position]) => {
            // Validate individual robot position
            const validation = this.validator.validateRobotPosition(position);
            if (!validation.valid) {
                console.warn(`Invalid robot position for ${robotId}:`, validation.error);
                return;
            }

            processed[robotId] = {
                id: robotId,
                x: position.x || 0,
                y: position.y || 0,
                z: position.z || 0,
                orientation: position.orientation || { roll: 0, pitch: 0, yaw: 0 },
                status: position.status || 'unknown',
                battery_level: position.battery_level,
                last_update: new Date()
            };
        });
        
        return processed;
    }

    processPerformanceMetrics(metrics) {
        // Validate performance metrics
        const validation = this.validator.validatePerformanceMetrics(metrics);
        if (!validation.valid) {
            console.warn('Invalid performance metrics:', validation.error);
            return {
                fps: 0,
                cpu_usage: 0,
                memory_usage: 0,
                ros_message_rate: 0,
                timestamp: Date.now()
            };
        }

        return {
            fps: metrics.fps || 0,
            cpu_usage: metrics.cpu_usage || 0,
            memory_usage: metrics.memory_usage || 0,
            ros_message_rate: metrics.ros_message_rate || 0,
            timestamp: metrics.timestamp || Date.now()
        };
    }

    processRosGraph(graphData) {
        return {
            nodes: graphData.nodes || [],
            topics: graphData.topics || [],
            connections: graphData.connections || []
        };
    }

    // Update methods
    updateRobotPositions(data, timestamp) {
        const updatedRobots = this.processRobotPositions(data);
        
        // Merge with existing robots, preserving additional data
        Object.entries(updatedRobots).forEach(([robotId, robotData]) => {
            if (this.store.robots[robotId]) {
                this.store.robots[robotId] = {
                    ...this.store.robots[robotId],
                    ...robotData,
                    last_update: timestamp
                };
            } else {
                this.store.robots[robotId] = robotData;
            }
        });

        // Request trajectory chart update if robots moved
        this.requestChartUpdate('robot_trajectories');
    }

    updatePerformanceMetrics(data, timestamp) {
        const metrics = this.processPerformanceMetrics(data);
        this.store.performance.current = metrics;
        
        // Add to history
        this.store.performance.history.push({
            ...metrics,
            timestamp: timestamp.getTime()
        });
        
        // Maintain history size limit
        if (this.store.performance.history.length > 1000) {
            this.store.performance.history.shift();
        }

        // Request performance chart update
        this.requestChartUpdate('performance_trends');
    }

    updateRosGraph(data, timestamp) {
        this.store.rosGraph = {
            ...this.processRosGraph(data),
            lastUpdate: timestamp
        };
    }

    updateCameraFrame(data, timestamp) {
        this.store.camera.frameCount++;
        this.store.camera.lastFrameTime = timestamp;
        this.store.camera.streamActive = true;
    }

    updateChartData(chartType, data) {
        switch (chartType) {
            case 'robot_trajectories':
                this.store.charts.trajectories = data;
                break;
            case 'performance_trends':
                this.store.charts.performance = data;
                break;
            default:
                console.warn('Unknown chart type:', chartType);
        }
        
        this.store.charts.lastUpdate = new Date();
    }

    // Chart update request helper
    requestChartUpdate(chartType) {
        if (this.store.ui.chartUpdateRequests.has(chartType)) {
            return; // Already requested
        }
        
        // This will be called by the WebSocket manager
        if (this.onChartUpdateRequest) {
            this.onChartUpdateRequest(chartType);
        }
    }

    // Set chart update request callback
    setChartUpdateCallback(callback) {
        this.onChartUpdateRequest = callback;
    }
}

// Export for use in main application
window.WebSocketMessageHandler = WebSocketMessageHandler;