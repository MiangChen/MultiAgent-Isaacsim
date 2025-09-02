// Performance Metrics Panel Component
const PerformancePanel = {
    name: 'PerformancePanel',
    props: {
        metrics: {
            type: Object,
            default: () => ({})
        }
    },
    template: `
        <div class="performance-panel">
            <div class="panel-header">
                <h3>Performance Metrics</h3>
                <div class="last-updated">{{ lastUpdatedText }}</div>
            </div>
            <div class="metrics-grid">
                <div class="metric-card fps-metric" :class="getMetricClass('fps')">
                    <div class="metric-header">
                        <div class="metric-icon">📊</div>
                        <div class="metric-label">FPS</div>
                    </div>
                    <div class="metric-value">{{ formatValue(metrics.fps, 1) }}</div>
                    <div class="metric-status" :class="getStatusClass('fps')">
                        {{ getStatusText('fps') }}
                    </div>
                    <div class="metric-bar">
                        <div 
                            class="metric-fill" 
                            :class="getBarClass('fps')"
                            :style="{ width: getFpsPercentage() + '%' }"
                        ></div>
                    </div>
                </div>

                <div class="metric-card cpu-metric" :class="getMetricClass('cpu')">
                    <div class="metric-header">
                        <div class="metric-icon">🖥️</div>
                        <div class="metric-label">CPU Usage</div>
                    </div>
                    <div class="metric-value">{{ formatValue(metrics.cpu_usage, 1) }}%</div>
                    <div class="metric-status" :class="getStatusClass('cpu')">
                        {{ getStatusText('cpu') }}
                    </div>
                    <div class="metric-bar">
                        <div 
                            class="metric-fill" 
                            :class="getBarClass('cpu')"
                            :style="{ width: (metrics.cpu_usage || 0) + '%' }"
                        ></div>
                    </div>
                </div>

                <div class="metric-card memory-metric" :class="getMetricClass('memory')">
                    <div class="metric-header">
                        <div class="metric-icon">💾</div>
                        <div class="metric-label">Memory Usage</div>
                    </div>
                    <div class="metric-value">{{ formatValue(metrics.memory_usage, 1) }}%</div>
                    <div class="metric-status" :class="getStatusClass('memory')">
                        {{ getStatusText('memory') }}
                    </div>
                    <div class="metric-bar">
                        <div 
                            class="metric-fill" 
                            :class="getBarClass('memory')"
                            :style="{ width: (metrics.memory_usage || 0) + '%' }"
                        ></div>
                    </div>
                </div>

                <div class="metric-card ros-metric" :class="getMetricClass('ros')">
                    <div class="metric-header">
                        <div class="metric-icon">🔄</div>
                        <div class="metric-label">ROS Msg Rate</div>
                    </div>
                    <div class="metric-value">{{ formatValue(metrics.ros_message_rate, 1) }} Hz</div>
                    <div class="metric-status" :class="getStatusClass('ros')">
                        {{ getStatusText('ros') }}
                    </div>
                    <div class="metric-bar">
                        <div 
                            class="metric-fill" 
                            :class="getBarClass('ros')"
                            :style="{ width: getRosPercentage() + '%' }"
                        ></div>
                    </div>
                </div>
            </div>
            
            <div class="performance-summary">
                <div class="summary-item">
                    <span class="summary-label">Overall Status:</span>
                    <span class="summary-value" :class="getOverallStatusClass()">
                        {{ getOverallStatus() }}
                    </span>
                </div>
                <div class="summary-item">
                    <span class="summary-label">Active Warnings:</span>
                    <span class="summary-value warning">{{ getWarningCount() }}</span>
                </div>
            </div>
        </div>
    `,
    data() {
        return {
            lastUpdated: null,
            thresholds: {
                fps: { good: 50, warning: 30 },
                cpu: { good: 70, warning: 85 },
                memory: { good: 70, warning: 85 },
                ros: { good: 20, warning: 10 }
            }
        };
    },
    computed: {
        lastUpdatedText() {
            if (!this.lastUpdated) return 'No data';
            const now = Date.now();
            const diff = now - this.lastUpdated;
            if (diff < 1000) return 'Just now';
            if (diff < 60000) return `${Math.floor(diff / 1000)}s ago`;
            return `${Math.floor(diff / 60000)}m ago`;
        }
    },
    watch: {
        metrics: {
            handler() {
                this.lastUpdated = Date.now();
            },
            deep: true
        }
    },
    methods: {
        formatValue(value, decimals = 0) {
            if (value === undefined || value === null) return 'N/A';
            return Number(value).toFixed(decimals);
        },
        
        getMetricClass(type) {
            const status = this.getMetricStatus(type);
            return `metric-${status}`;
        },
        
        getStatusClass(type) {
            return `status-${this.getMetricStatus(type)}`;
        },
        
        getBarClass(type) {
            return `bar-${this.getMetricStatus(type)}`;
        },
        
        getMetricStatus(type) {
            let value;
            switch (type) {
                case 'fps':
                    value = this.metrics.fps || 0;
                    if (value >= this.thresholds.fps.good) return 'good';
                    if (value >= this.thresholds.fps.warning) return 'warning';
                    return 'critical';
                case 'cpu':
                    value = this.metrics.cpu_usage || 0;
                    if (value <= this.thresholds.cpu.good) return 'good';
                    if (value <= this.thresholds.cpu.warning) return 'warning';
                    return 'critical';
                case 'memory':
                    value = this.metrics.memory_usage || 0;
                    if (value <= this.thresholds.memory.good) return 'good';
                    if (value <= this.thresholds.memory.warning) return 'warning';
                    return 'critical';
                case 'ros':
                    value = this.metrics.ros_message_rate || 0;
                    if (value >= this.thresholds.ros.good) return 'good';
                    if (value >= this.thresholds.ros.warning) return 'warning';
                    return 'critical';
                default:
                    return 'unknown';
            }
        },
        
        getStatusText(type) {
            const status = this.getMetricStatus(type);
            const statusMap = {
                good: 'Optimal',
                warning: 'Warning',
                critical: 'Critical',
                unknown: 'Unknown'
            };
            return statusMap[status];
        },
        
        getFpsPercentage() {
            const fps = this.metrics.fps || 0;
            return Math.min((fps / 60) * 100, 100);
        },
        
        getRosPercentage() {
            const rate = this.metrics.ros_message_rate || 0;
            return Math.min((rate / 50) * 100, 100);
        },
        
        getOverallStatus() {
            const statuses = ['fps', 'cpu', 'memory', 'ros'].map(type => this.getMetricStatus(type));
            
            if (statuses.includes('critical')) return 'Critical Issues';
            if (statuses.includes('warning')) return 'Performance Warning';
            if (statuses.every(s => s === 'good')) return 'All Systems Optimal';
            return 'Mixed Status';
        },
        
        getOverallStatusClass() {
            const status = this.getOverallStatus();
            if (status.includes('Critical')) return 'status-critical';
            if (status.includes('Warning')) return 'status-warning';
            if (status.includes('Optimal')) return 'status-good';
            return 'status-mixed';
        },
        
        getWarningCount() {
            return ['fps', 'cpu', 'memory', 'ros'].filter(type => {
                const status = this.getMetricStatus(type);
                return status === 'warning' || status === 'critical';
            }).length;
        }
    }
};

// Register the component globally
if (typeof Vue !== 'undefined' && Vue.createApp) {
    // Component will be registered in the main app
}