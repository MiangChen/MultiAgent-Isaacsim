const { createApp, ref, reactive, onMounted, onUnmounted, computed, watch } = Vue;

// WebSocket connection manager with enhanced reconnection logic
class WebSocketManager {
    constructor() {
        this.websocket = null;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 10;
        this.reconnectInterval = 1000; // Start with 1 second
        this.maxReconnectInterval = 30000; // Max 30 seconds
        this.reconnectTimer = null;
        this.isManualClose = false;
        this.messageHandlers = new Map();
        this.connectionCallbacks = [];
    }

    connect() {
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            return Promise.resolve();
        }

        return new Promise((resolve, reject) => {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.host}/ws`;
            
            this.websocket = new WebSocket(wsUrl);
            
            this.websocket.onopen = () => {
                console.log('WebSocket connected');
                this.reconnectAttempts = 0;
                this.reconnectInterval = 1000;
                this.isManualClose = false;
                
                // Notify all connection callbacks
                this.connectionCallbacks.forEach(callback => callback('connected'));
                resolve();
            };
            
            this.websocket.onclose = (event) => {
                console.log('WebSocket disconnected', event);
                this.connectionCallbacks.forEach(callback => callback('disconnected'));
                
                if (!this.isManualClose && this.reconnectAttempts < this.maxReconnectAttempts) {
                    this.scheduleReconnect();
                }
            };
            
            this.websocket.onmessage = (event) => {
                try {
                    const message = JSON.parse(event.data);
                    this.handleMessage(message);
                } catch (error) {
                    console.error('Error parsing WebSocket message:', error);
                }
            };
            
            this.websocket.onerror = (error) => {
                console.error('WebSocket error:', error);
                this.connectionCallbacks.forEach(callback => callback('error'));
                reject(error);
            };
        });
    }

    scheduleReconnect() {
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
        }

        this.reconnectAttempts++;
        const delay = Math.min(
            this.reconnectInterval * Math.pow(2, this.reconnectAttempts - 1),
            this.maxReconnectInterval
        );

        console.log(`Attempting to reconnect in ${delay}ms (attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts})`);
        
        this.reconnectTimer = setTimeout(() => {
            this.connect().catch(() => {
                // Connection failed, will try again if under max attempts
            });
        }, delay);
    }

    handleMessage(message) {
        const handler = this.messageHandlers.get(message.type);
        if (handler) {
            handler(message);
        }
    }

    onMessage(type, handler) {
        this.messageHandlers.set(type, handler);
    }

    onConnectionChange(callback) {
        this.connectionCallbacks.push(callback);
    }

    send(data) {
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            this.websocket.send(JSON.stringify(data));
            return true;
        }
        console.warn('WebSocket not connected, message not sent:', data);
        return false;
    }

    close() {
        this.isManualClose = true;
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
        }
        if (this.websocket) {
            this.websocket.close();
        }
    }

    isConnected() {
        return this.websocket && this.websocket.readyState === WebSocket.OPEN;
    }
}

// Reactive data store for dashboard data
const createDataStore = () => {
    return reactive({
        // Connection state
        connectionStatus: 'disconnected',
        lastUpdateTime: null,
        
        // Robot data with enhanced structure
        robots: {},
        robotCount: computed(() => Object.keys(this.robots).length),
        activeRobots: computed(() => 
            Object.entries(this.robots).filter(([_, robot]) => robot.status === 'active')
        ),
        
        // Performance metrics with history
        performance: {
            current: {
                fps: 0,
                cpu_usage: 0,
                memory_usage: 0,
                ros_message_rate: 0
            },
            history: [],
            alerts: []
        },
        
        // Chart data
        charts: {
            trajectories: null,
            performance: null,
            lastUpdate: null
        },
        
        // ROS graph data
        rosGraph: {
            nodes: [],
            topics: [],
            connections: [],
            lastUpdate: null
        },
        
        // Camera data
        camera: {
            streamActive: false,
            frameCount: 0,
            lastFrameTime: null
        },
        
        // UI state
        ui: {
            selectedRobot: null,
            activePanel: 'overview',
            chartUpdateRequests: new Set()
        }
    });
};

const DashboardApp = {
    components: {
        'robot-status-panel': RobotStatusPanel,
        'performance-panel': PerformancePanel,
        'chart-panel': ChartPanel,
        'ros-graph-panel': RosGraphPanel,
        'camera-panel': CameraPanel
    },
    setup() {
        // Initialize data store
        const store = createDataStore();
        
        // Initialize WebSocket manager
        const wsManager = new WebSocketManager();
        
        // Connection status management
        wsManager.onConnectionChange((status) => {
            store.connectionStatus = status;
            if (status === 'connected') {
                store.lastUpdateTime = new Date();
            }
        });
        
        // Initialize message handler
        const messageHandler = new WebSocketMessageHandler(store);
        
        // Set up chart update callback
        messageHandler.setChartUpdateCallback((chartType) => {
            requestChartUpdate(chartType);
        });

        // Message handlers for different data types
        const setupMessageHandlers = () => {
            // Route all messages through the message handler
            wsManager.onMessage('initial_data', (message) => {
                messageHandler.processMessage(message);
            });

            wsManager.onMessage('data_update', (message) => {
                messageHandler.processMessage(message);
            });

            wsManager.onMessage('chart_data', (message) => {
                messageHandler.processMessage(message);
            });

            wsManager.onMessage('binary_data', (message) => {
                messageHandler.processMessage(message);
            });

            wsManager.onMessage('error', (message) => {
                messageHandler.processMessage(message);
            });

            wsManager.onMessage('status_update', (message) => {
                messageHandler.processMessage(message);
            });

            // Handle any other message types
            wsManager.messageHandlers.set('*', (message) => {
                if (!wsManager.messageHandlers.has(message.type)) {
                    console.warn('Unhandled message type:', message.type);
                    messageHandler.processMessage(message);
                }
            });
        };

        // Chart update request function with throttling
        const requestChartUpdate = (chartType, options = {}) => {
            if (store.ui.chartUpdateRequests.has(chartType)) {
                return; // Request already pending
            }
            
            const requestData = {
                type: 'request_chart_update',
                chart_type: chartType,
                ...options
            };
            
            if (wsManager.send(requestData)) {
                store.ui.chartUpdateRequests.add(chartType);
                console.log(`Chart update requested: ${chartType}`);
            }
        };

        // Batch chart update requests
        const requestMultipleChartUpdates = (chartTypes) => {
            chartTypes.forEach(chartType => {
                requestChartUpdate(chartType);
            });
        };

        // Utility functions
        const selectRobot = (robotId) => {
            store.ui.selectedRobot = robotId;
            console.log(`Selected robot: ${robotId}`);
        };

        const setActivePanel = (panelName) => {
            store.ui.activePanel = panelName;
            console.log(`Active panel: ${panelName}`);
        };

        const refreshData = () => {
            if (wsManager.send({ type: 'refresh_all_data' })) {
                console.log('Data refresh requested');
            }
        };

        // Specific refresh functions
        const refreshRobotData = () => {
            if (wsManager.send({ type: 'refresh_data', data_type: 'robot_positions' })) {
                console.log('Robot data refresh requested');
            }
        };

        const refreshPerformanceData = () => {
            if (wsManager.send({ type: 'refresh_data', data_type: 'performance_metrics' })) {
                console.log('Performance data refresh requested');
            }
        };

        const refreshRosGraph = () => {
            if (wsManager.send({ type: 'refresh_data', data_type: 'ros_graph' })) {
                console.log('ROS graph refresh requested');
            }
        };

        // Camera control functions
        const toggleCameraFullscreen = () => {
            // Implementation will be handled by camera component
            console.log('Camera fullscreen toggle requested');
        };

        // Error handling
        const errorMessage = ref('');
        const isLoading = ref(false);

        const showError = (message) => {
            errorMessage.value = message;
            setTimeout(() => {
                errorMessage.value = '';
            }, 5000);
        };

        const clearError = () => {
            errorMessage.value = '';
        };

        // UI state management
        const trajectoryTimeRange = ref('1h');
        const performanceTimeRange = ref('5m');
        const selectedCamera = ref('camera1');

        // Watch for time range changes to update charts
        watch(trajectoryTimeRange, (newRange) => {
            requestChartUpdate('robot_trajectories', { time_range: newRange });
        });

        watch(performanceTimeRange, (newRange) => {
            requestChartUpdate('performance_trends', { time_range: newRange });
        });

        // Reset graph layout function
        const resetGraphLayout = () => {
            // This will be handled by the ROS graph component
            console.log('Graph layout reset requested');
        };

        // Performance monitoring
        const checkPerformanceAlerts = () => {
            const { current } = store.performance;
            const alerts = [];
            
            if (current.fps < 30) {
                alerts.push({ type: 'warning', message: 'Low FPS detected', value: current.fps });
            }
            if (current.cpu_usage > 80) {
                alerts.push({ type: 'error', message: 'High CPU usage', value: current.cpu_usage });
            }
            if (current.memory_usage > 90) {
                alerts.push({ type: 'error', message: 'High memory usage', value: current.memory_usage });
            }
            
            store.performance.alerts = alerts;
        };

        // Watch for performance changes to trigger alerts
        watch(() => store.performance.current, checkPerformanceAlerts, { deep: true });

        // Lifecycle management
        onMounted(async () => {
            setupMessageHandlers();
            try {
                await wsManager.connect();
                console.log('Dashboard initialized successfully');
            } catch (error) {
                console.error('Failed to initialize dashboard:', error);
            }
        });
        
        onUnmounted(() => {
            wsManager.close();
        });
        
        // Expose reactive data and methods to template
        return {
            // Data store
            store,
            
            // Connection status (for backward compatibility)
            connectionStatus: computed(() => store.connectionStatus),
            robotData: computed(() => store.robots),
            performanceMetrics: computed(() => store.performance.current),
            chartData: computed(() => store.charts.trajectories),
            performanceChartData: computed(() => store.charts.performance),
            rosGraphData: computed(() => store.rosGraph),
            cameraStreamUrl: computed(() => store.camera.streamActive ? '/camera/stream' : ''),
            
            // UI state
            trajectoryTimeRange,
            performanceTimeRange,
            selectedCamera,
            errorMessage,
            isLoading,
            
            // Methods
            selectRobot,
            setActivePanel,
            refreshData,
            refreshRobotData,
            refreshPerformanceData,
            refreshRosGraph,
            requestChartUpdate,
            requestMultipleChartUpdates,
            toggleCameraFullscreen,
            resetGraphLayout,
            showError,
            clearError,
            
            // Component event handlers
            handleChartRefresh: (chartId) => {
                console.log('Refreshing chart:', chartId);
                requestChartUpdate(chartId);
            },
            
            handleTimeRangeChange: (event) => {
                console.log('Time range changed:', event);
                requestChartUpdate(event.chartId, { time_range: event.timeRange });
            },
            
            handleChartClick: (event) => {
                console.log('Chart clicked:', event);
                // Handle chart click logic - could be used for drill-down or selection
            },
            
            handleRosGraphRefresh: () => {
                console.log('Refreshing ROS graph');
                refreshRosGraph();
            },
            
            handleCameraClick: (event) => {
                console.log('Camera clicked:', event);
                // Handle camera click logic - could be used for camera control
            },
            
            // WebSocket manager (for advanced usage)
            wsManager,
            
            // Message handler (for advanced usage)
            messageHandler
        };
    }
};

// Create and mount the Vue application
const app = createApp(DashboardApp);

// Global error handler
app.config.errorHandler = (err, instance, info) => {
    console.error('Vue error:', err, info);
};

// Mount the application
app.mount('#app');