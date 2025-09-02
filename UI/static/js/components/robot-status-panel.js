// Robot Status Panel Component
const RobotStatusPanel = {
    name: 'RobotStatusPanel',
    props: {
        robots: {
            type: Object,
            default: () => ({})
        }
    },
    template: `
        <div class="robot-status-panel">
            <div class="panel-header">
                <h3>Robot Status</h3>
                <div class="robot-count">{{ robotCount }} robots</div>
            </div>
            <div class="robot-grid" v-if="robotCount > 0">
                <div 
                    v-for="(robot, robotId) in robots" 
                    :key="robotId"
                    class="robot-card"
                    :class="getStatusClass(robot.status)"
                >
                    <div class="robot-header">
                        <div class="robot-id">{{ robotId }}</div>
                        <div class="status-indicator" :class="robot.status">
                            <span class="status-dot"></span>
                            {{ formatStatus(robot.status) }}
                        </div>
                    </div>
                    <div class="robot-info">
                        <div class="position-info">
                            <div class="coord-group">
                                <span class="coord-label">X:</span>
                                <span class="coord-value">{{ formatCoordinate(robot.x) }}</span>
                            </div>
                            <div class="coord-group">
                                <span class="coord-label">Y:</span>
                                <span class="coord-value">{{ formatCoordinate(robot.y) }}</span>
                            </div>
                            <div class="coord-group" v-if="robot.z !== undefined">
                                <span class="coord-label">Z:</span>
                                <span class="coord-value">{{ formatCoordinate(robot.z) }}</span>
                            </div>
                        </div>
                        <div class="orientation-info" v-if="robot.orientation">
                            <div class="orientation-group">
                                <span class="orientation-label">Yaw:</span>
                                <span class="orientation-value">{{ formatAngle(robot.orientation.yaw) }}°</span>
                            </div>
                        </div>
                        <div class="battery-info" v-if="robot.battery_level !== undefined">
                            <div class="battery-group">
                                <span class="battery-label">Battery:</span>
                                <div class="battery-bar">
                                    <div 
                                        class="battery-fill" 
                                        :class="getBatteryClass(robot.battery_level)"
                                        :style="{ width: robot.battery_level + '%' }"
                                    ></div>
                                </div>
                                <span class="battery-value">{{ robot.battery_level }}%</span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            <div v-else class="no-robots">
                <div class="no-robots-icon">🤖</div>
                <div class="no-robots-text">No robots detected</div>
            </div>
        </div>
    `,
    computed: {
        robotCount() {
            return Object.keys(this.robots).length;
        }
    },
    methods: {
        formatStatus(status) {
            const statusMap = {
                'active': 'Active',
                'inactive': 'Inactive',
                'error': 'Error'
            };
            return statusMap[status] || 'Unknown';
        },
        
        getStatusClass(status) {
            return `status-${status}`;
        },
        
        formatCoordinate(value) {
            if (value === undefined || value === null) return 'N/A';
            return Number(value).toFixed(2);
        },
        
        formatAngle(value) {
            if (value === undefined || value === null) return 'N/A';
            return Number(value * 180 / Math.PI).toFixed(1);
        },
        
        getBatteryClass(level) {
            if (level > 60) return 'battery-high';
            if (level > 30) return 'battery-medium';
            return 'battery-low';
        }
    }
};

// Register the component globally
if (typeof Vue !== 'undefined' && Vue.createApp) {
    // Component will be registered in the main app
}