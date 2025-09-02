// Message Validation Utilities
// Provides validation and sanitization for WebSocket messages

class MessageValidator {
    constructor() {
        this.messageSchemas = new Map();
        this.setupSchemas();
    }

    setupSchemas() {
        // Initial data message schema
        this.messageSchemas.set('initial_data', {
            required: ['type', 'data'],
            optional: ['timestamp'],
            dataFields: {
                robot_positions: 'object',
                performance_metrics: 'object',
                ros_graph: 'object',
                chart_data: 'object'
            }
        });

        // Data update message schema
        this.messageSchemas.set('data_update', {
            required: ['type', 'data_type', 'data'],
            optional: ['timestamp'],
            dataTypes: [
                'robot_positions',
                'performance_metrics', 
                'ros_graph',
                'camera_frame'
            ]
        });

        // Chart data message schema
        this.messageSchemas.set('chart_data', {
            required: ['type', 'data_type', 'data'],
            optional: ['timestamp'],
            dataTypes: [
                'robot_trajectories',
                'performance_trends'
            ]
        });

        // Binary data message schema
        this.messageSchemas.set('binary_data', {
            required: ['type', 'data_type', 'size'],
            optional: ['timestamp'],
            dataTypes: ['camera_frame']
        });

        // Error message schema
        this.messageSchemas.set('error', {
            required: ['type', 'error_code', 'error_message'],
            optional: ['details', 'timestamp']
        });

        // Status update message schema
        this.messageSchemas.set('status_update', {
            required: ['type', 'component', 'status'],
            optional: ['details', 'timestamp']
        });
    }

    validateMessage(message) {
        try {
            // Basic structure validation
            if (!message || typeof message !== 'object') {
                return { valid: false, error: 'Message must be an object' };
            }

            if (!message.type || typeof message.type !== 'string') {
                return { valid: false, error: 'Message must have a string type field' };
            }

            // Schema-specific validation
            const schema = this.messageSchemas.get(message.type);
            if (!schema) {
                return { valid: true, warning: `Unknown message type: ${message.type}` };
            }

            // Check required fields
            for (const field of schema.required) {
                if (!(field in message)) {
                    return { valid: false, error: `Missing required field: ${field}` };
                }
            }

            // Validate data types if specified
            if (schema.dataTypes && message.data_type) {
                if (!schema.dataTypes.includes(message.data_type)) {
                    return { 
                        valid: false, 
                        error: `Invalid data_type: ${message.data_type}. Expected one of: ${schema.dataTypes.join(', ')}` 
                    };
                }
            }

            // Validate data fields for initial_data
            if (message.type === 'initial_data' && message.data) {
                const validation = this.validateInitialData(message.data, schema.dataFields);
                if (!validation.valid) {
                    return validation;
                }
            }

            return { valid: true };

        } catch (error) {
            return { valid: false, error: `Validation error: ${error.message}` };
        }
    }

    validateInitialData(data, expectedFields) {
        if (typeof data !== 'object') {
            return { valid: false, error: 'Initial data must be an object' };
        }

        // Check each field type
        for (const [field, expectedType] of Object.entries(expectedFields)) {
            if (field in data) {
                const actualType = typeof data[field];
                if (actualType !== expectedType) {
                    return { 
                        valid: false, 
                        error: `Field ${field} should be ${expectedType}, got ${actualType}` 
                    };
                }
            }
        }

        return { valid: true };
    }

    sanitizeMessage(message) {
        if (!message || typeof message !== 'object') {
            return null;
        }

        // Create a clean copy
        const sanitized = {};

        // Copy known safe fields
        const safeFields = ['type', 'data_type', 'data', 'timestamp', 'error_code', 'error_message', 'details', 'component', 'status', 'size'];
        
        for (const field of safeFields) {
            if (field in message) {
                sanitized[field] = message[field];
            }
        }

        // Sanitize timestamp
        if (sanitized.timestamp) {
            const timestamp = new Date(sanitized.timestamp);
            if (isNaN(timestamp.getTime())) {
                delete sanitized.timestamp;
            } else {
                sanitized.timestamp = timestamp.getTime();
            }
        }

        // Sanitize numeric fields
        if (sanitized.size && typeof sanitized.size !== 'number') {
            const size = parseInt(sanitized.size);
            sanitized.size = isNaN(size) ? 0 : size;
        }

        return sanitized;
    }

    validateRobotPosition(position) {
        if (!position || typeof position !== 'object') {
            return { valid: false, error: 'Robot position must be an object' };
        }

        const requiredFields = ['x', 'y'];
        for (const field of requiredFields) {
            if (!(field in position) || typeof position[field] !== 'number') {
                return { valid: false, error: `Robot position missing or invalid ${field}` };
            }
        }

        // Validate optional fields
        if ('z' in position && typeof position.z !== 'number') {
            return { valid: false, error: 'Robot position z must be a number' };
        }

        if ('status' in position) {
            const validStatuses = ['active', 'inactive', 'error', 'unknown'];
            if (!validStatuses.includes(position.status)) {
                return { valid: false, error: `Invalid robot status: ${position.status}` };
            }
        }

        return { valid: true };
    }

    validatePerformanceMetrics(metrics) {
        if (!metrics || typeof metrics !== 'object') {
            return { valid: false, error: 'Performance metrics must be an object' };
        }

        const numericFields = ['fps', 'cpu_usage', 'memory_usage', 'ros_message_rate'];
        for (const field of numericFields) {
            if (field in metrics && typeof metrics[field] !== 'number') {
                return { valid: false, error: `Performance metric ${field} must be a number` };
            }
        }

        // Validate ranges
        if (metrics.cpu_usage && (metrics.cpu_usage < 0 || metrics.cpu_usage > 100)) {
            return { valid: false, error: 'CPU usage must be between 0 and 100' };
        }

        if (metrics.memory_usage && (metrics.memory_usage < 0 || metrics.memory_usage > 100)) {
            return { valid: false, error: 'Memory usage must be between 0 and 100' };
        }

        if (metrics.fps && metrics.fps < 0) {
            return { valid: false, error: 'FPS cannot be negative' };
        }

        return { valid: true };
    }

    validateChartData(chartData) {
        if (!chartData || typeof chartData !== 'object') {
            return { valid: false, error: 'Chart data must be an object' };
        }

        if (!chartData.series || !Array.isArray(chartData.series)) {
            return { valid: false, error: 'Chart data must have a series array' };
        }

        // Validate each series
        for (let i = 0; i < chartData.series.length; i++) {
            const series = chartData.series[i];
            if (!series.name || typeof series.name !== 'string') {
                return { valid: false, error: `Series ${i} must have a name` };
            }
            if (!series.data || !Array.isArray(series.data)) {
                return { valid: false, error: `Series ${i} must have a data array` };
            }
        }

        return { valid: true };
    }
}

// Export for use in other modules
window.MessageValidator = MessageValidator;