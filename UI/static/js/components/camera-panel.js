// Camera Stream Panel Component
const CameraPanel = {
    name: 'CameraPanel',
    props: {
        streamUrl: {
            type: String,
            default: ''
        }
    },
    template: `
        <div class="camera-panel">
            <div class="panel-header">
                <h3>Camera Stream</h3>
                <div class="camera-controls">
                    <select v-model="selectedCamera" @change="switchCamera" class="camera-selector">
                        <option value="main">Main Camera</option>
                        <option value="robot1">Robot 1 Camera</option>
                        <option value="robot2">Robot 2 Camera</option>
                        <option value="overhead">Overhead View</option>
                    </select>
                    <select v-model="streamQuality" @change="changeQuality" class="quality-selector">
                        <option value="high">High (1080p)</option>
                        <option value="medium">Medium (720p)</option>
                        <option value="low">Low (480p)</option>
                    </select>
                    <button @click="toggleStream" class="stream-btn" :class="{ active: isStreaming }">
                        {{ isStreaming ? '⏸️' : '▶️' }}
                    </button>
                    <button @click="takeSnapshot" class="snapshot-btn">📷</button>
                    <button @click="toggleFullscreen" class="fullscreen-btn">
                        {{ isFullscreen ? '⤓' : '⤢' }}
                    </button>
                </div>
            </div>
            
            <div class="camera-container" :class="{ 'fullscreen': isFullscreen }">
                <div class="stream-wrapper" v-show="isStreaming && !isLoading">
                    <canvas 
                        ref="cameraCanvas" 
                        class="camera-canvas"
                        @click="handleCanvasClick"
                        @mousemove="handleMouseMove"
                    ></canvas>
                    <div class="stream-overlay">
                        <div class="stream-info">
                            <div class="fps-counter">{{ currentFps }} FPS</div>
                            <div class="resolution-info">{{ currentResolution }}</div>
                        </div>
                        <div class="crosshair" v-if="showCrosshair" :style="crosshairStyle"></div>
                    </div>
                </div>
                
                <div v-if="isLoading" class="stream-loading">
                    <div class="loading-spinner"></div>
                    <div class="loading-text">Connecting to camera...</div>
                </div>
                
                <div v-if="!isStreaming && !isLoading" class="stream-placeholder">
                    <div class="placeholder-icon">📹</div>
                    <div class="placeholder-text">Camera stream stopped</div>
                    <button @click="startStream" class="start-stream-btn">Start Stream</button>
                </div>
                
                <div v-if="hasError" class="stream-error">
                    <div class="error-icon">⚠️</div>
                    <div class="error-text">{{ errorMessage }}</div>
                    <button @click="retryConnection" class="retry-btn">Retry Connection</button>
                </div>
            </div>
            
            <div class="camera-info" v-if="isStreaming && !isLoading">
                <div class="info-section">
                    <div class="info-item">
                        <span class="info-label">Camera:</span>
                        <span class="info-value">{{ selectedCamera }}</span>
                    </div>
                    <div class="info-item">
                        <span class="info-label">Quality:</span>
                        <span class="info-value">{{ streamQuality }}</span>
                    </div>
                    <div class="info-item">
                        <span class="info-label">Latency:</span>
                        <span class="info-value">{{ latency }}ms</span>
                    </div>
                </div>
                <div class="stream-stats">
                    <div class="stat-item">
                        <span class="stat-label">Frames:</span>
                        <span class="stat-value">{{ frameCount }}</span>
                    </div>
                    <div class="stat-item">
                        <span class="stat-label">Dropped:</span>
                        <span class="stat-value">{{ droppedFrames }}</span>
                    </div>
                </div>
            </div>
        </div>
    `,
    data() {
        return {
            isStreaming: false,
            isLoading: false,
            isFullscreen: false,
            hasError: false,
            errorMessage: '',
            selectedCamera: 'main',
            streamQuality: 'medium',
            showCrosshair: false,
            crosshairStyle: { left: '50%', top: '50%' },
            
            // Stream stats
            currentFps: 0,
            currentResolution: '1280x720',
            latency: 0,
            frameCount: 0,
            droppedFrames: 0,
            
            // Internal state
            websocket: null,
            canvas: null,
            ctx: null,
            lastFrameTime: 0,
            fpsCounter: 0,
            fpsInterval: null
        };
    },
    mounted() {
        this.initCanvas();
        this.startFpsCounter();
        
        // Handle fullscreen events
        document.addEventListener('fullscreenchange', this.handleFullscreenChange);
        document.addEventListener('keydown', this.handleKeydown);
        
        // Auto-start stream if URL is provided
        if (this.streamUrl) {
            this.startStream();
        }
    },
    
    beforeUnmount() {
        this.stopStream();
        this.stopFpsCounter();
        document.removeEventListener('fullscreenchange', this.handleFullscreenChange);
        document.removeEventListener('keydown', this.handleKeydown);
    },
    
    methods: {
        initCanvas() {
            this.canvas = this.$refs.cameraCanvas;
            if (this.canvas) {
                this.ctx = this.canvas.getContext('2d');
                this.resizeCanvas();
                
                // Add resize listener
                window.addEventListener('resize', this.resizeCanvas);
            }
        },
        
        resizeCanvas() {
            if (!this.canvas) return;
            
            const container = this.canvas.parentElement;
            const rect = container.getBoundingClientRect();
            
            // Set canvas size to match container
            this.canvas.width = rect.width;
            this.canvas.height = rect.height;
            
            // Update resolution display
            this.currentResolution = `${this.canvas.width}x${this.canvas.height}`;
        },
        
        startStream() {
            this.isLoading = true;
            this.hasError = false;
            this.errorMessage = '';
            
            // Connect to WebSocket for binary frame data
            this.connectWebSocket();
        },
        
        stopStream() {
            this.isStreaming = false;
            this.isLoading = false;
            
            if (this.websocket) {
                this.websocket.close();
                this.websocket = null;
            }
            
            // Clear canvas
            if (this.ctx && this.canvas) {
                this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
            }
        },
        
        connectWebSocket() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.host}/ws`;
            
            this.websocket = new WebSocket(wsUrl);
            this.websocket.binaryType = 'arraybuffer';
            
            this.websocket.onopen = () => {
                console.log('Camera WebSocket connected');
                this.isLoading = false;
                this.isStreaming = true;
                
                // Request camera stream
                this.requestCameraStream();
            };
            
            this.websocket.onclose = () => {
                console.log('Camera WebSocket disconnected');
                this.isStreaming = false;
                this.isLoading = false;
            };
            
            this.websocket.onmessage = (event) => {
                if (event.data instanceof ArrayBuffer) {
                    this.handleBinaryFrame(event.data);
                } else {
                    try {
                        const message = JSON.parse(event.data);
                        this.handleTextMessage(message);
                    } catch (e) {
                        console.error('Error parsing WebSocket message:', e);
                    }
                }
            };
            
            this.websocket.onerror = (error) => {
                console.error('Camera WebSocket error:', error);
                this.hasError = true;
                this.errorMessage = 'Failed to connect to camera stream';
                this.isLoading = false;
                this.isStreaming = false;
            };
        },
        
        handleBinaryFrame(arrayBuffer) {
            if (!this.ctx || !this.canvas) return;
            
            // Convert ArrayBuffer to Blob and create image
            const blob = new Blob([arrayBuffer], { type: 'image/jpeg' });
            const url = URL.createObjectURL(blob);
            
            const img = new Image();
            img.onload = () => {
                // Clear canvas and draw new frame
                this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
                
                // Calculate aspect ratio and draw image
                const aspectRatio = img.width / img.height;
                const canvasAspectRatio = this.canvas.width / this.canvas.height;
                
                let drawWidth, drawHeight, offsetX, offsetY;
                
                if (aspectRatio > canvasAspectRatio) {
                    // Image is wider than canvas
                    drawWidth = this.canvas.width;
                    drawHeight = this.canvas.width / aspectRatio;
                    offsetX = 0;
                    offsetY = (this.canvas.height - drawHeight) / 2;
                } else {
                    // Image is taller than canvas
                    drawWidth = this.canvas.height * aspectRatio;
                    drawHeight = this.canvas.height;
                    offsetX = (this.canvas.width - drawWidth) / 2;
                    offsetY = 0;
                }
                
                this.ctx.drawImage(img, offsetX, offsetY, drawWidth, drawHeight);
                
                // Update frame counter
                this.frameCount++;
                this.updateFps();
                
                // Cleanup
                URL.revokeObjectURL(url);
            };
            
            img.onerror = () => {
                console.error('Error loading camera frame');
                this.droppedFrames++;
                URL.revokeObjectURL(url);
            };
            
            img.src = url;
        },
        
        handleTextMessage(message) {
            if (message.type === 'camera_info') {
                // Update camera information
                this.latency = message.latency || 0;
                this.currentResolution = message.resolution || this.currentResolution;
            }
        },
        
        requestCameraStream() {
            if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
                this.websocket.send(JSON.stringify({
                    type: 'camera_control',
                    action: 'start_stream',
                    camera: this.selectedCamera,
                    quality: this.streamQuality
                }));
            }
        },
        
        toggleStream() {
            if (this.isStreaming) {
                this.stopStream();
            } else {
                this.startStream();
            }
        },
        
        switchCamera() {
            if (this.isStreaming) {
                this.requestCameraStream();
            }
        },
        
        changeQuality() {
            if (this.isStreaming) {
                this.requestCameraStream();
            }
        },
        
        takeSnapshot() {
            if (!this.canvas || !this.isStreaming) return;
            
            // Create download link for canvas image
            const link = document.createElement('a');
            link.download = `camera_snapshot_${Date.now()}.png`;
            link.href = this.canvas.toDataURL();
            link.click();
        },
        
        toggleFullscreen() {
            const container = this.$el.querySelector('.camera-container');
            
            if (!this.isFullscreen) {
                if (container.requestFullscreen) {
                    container.requestFullscreen();
                }
            } else {
                if (document.exitFullscreen) {
                    document.exitFullscreen();
                }
            }
        },
        
        handleFullscreenChange() {
            this.isFullscreen = !!document.fullscreenElement;
            
            // Resize canvas after fullscreen change
            setTimeout(() => {
                this.resizeCanvas();
            }, 100);
        },
        
        handleKeydown(event) {
            if (this.isFullscreen && event.key === 'Escape') {
                this.isFullscreen = false;
            }
        },
        
        handleCanvasClick(event) {
            if (!this.canvas) return;
            
            const rect = this.canvas.getBoundingClientRect();
            const x = event.clientX - rect.left;
            const y = event.clientY - rect.top;
            
            // Emit click event with normalized coordinates
            this.$emit('camera-click', {
                x: x / this.canvas.width,
                y: y / this.canvas.height,
                camera: this.selectedCamera
            });
        },
        
        handleMouseMove(event) {
            if (!this.canvas) return;
            
            const rect = this.canvas.getBoundingClientRect();
            const x = event.clientX - rect.left;
            const y = event.clientY - rect.top;
            
            // Update crosshair position
            this.crosshairStyle = {
                left: `${x}px`,
                top: `${y}px`
            };
        },
        
        retryConnection() {
            this.hasError = false;
            this.errorMessage = '';
            this.startStream();
        },
        
        startFpsCounter() {
            this.fpsInterval = setInterval(() => {
                this.currentFps = this.fpsCounter;
                this.fpsCounter = 0;
            }, 1000);
        },
        
        stopFpsCounter() {
            if (this.fpsInterval) {
                clearInterval(this.fpsInterval);
                this.fpsInterval = null;
            }
        },
        
        updateFps() {
            this.fpsCounter++;
        }
    }
};

// Register the component globally
if (typeof Vue !== 'undefined' && Vue.createApp) {
    // Component will be registered in the main app
}