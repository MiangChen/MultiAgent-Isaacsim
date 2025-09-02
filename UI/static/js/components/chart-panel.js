// Interactive Chart Panel Component
const ChartPanel = {
    name: 'ChartPanel',
    props: {
        chartData: {
            type: Object,
            default: () => ({})
        },
        chartId: {
            type: String,
            required: true
        },
        title: {
            type: String,
            default: 'Chart'
        },
        height: {
            type: String,
            default: '100%'
        }
    },
    template: `
        <div class="chart-panel">
            <div class="panel-header">
                <h3>{{ title }}</h3>
                <div class="chart-controls">
                    <select v-model="selectedTimeRange" @change="updateTimeRange" class="time-range-selector">
                        <option value="1m">Last 1 min</option>
                        <option value="5m">Last 5 min</option>
                        <option value="15m">Last 15 min</option>
                        <option value="1h">Last 1 hour</option>
                        <option value="all">All data</option>
                    </select>
                    <button @click="refreshChart" class="refresh-btn" :disabled="isLoading">
                        {{ isLoading ? '⟳' : '↻' }}
                    </button>
                    <button @click="toggleFullscreen" class="fullscreen-btn">
                        {{ isFullscreen ? '⤓' : '⤢' }}
                    </button>
                </div>
            </div>
            <div class="chart-container" :class="{ 'fullscreen': isFullscreen }">
                <div 
                    :id="chartId" 
                    class="chart-canvas"
                    :style="{ height: height }"
                    v-show="!isLoading && hasData"
                ></div>
                <div v-if="isLoading" class="chart-loading">
                    <div class="loading-spinner"></div>
                    <div class="loading-text">Loading chart data...</div>
                </div>
                <div v-if="!isLoading && !hasData" class="chart-empty">
                    <div class="empty-icon">📊</div>
                    <div class="empty-text">No data available</div>
                    <button @click="refreshChart" class="retry-btn">Retry</button>
                </div>
            </div>
            <div class="chart-info" v-if="hasData && !isLoading">
                <div class="data-points">{{ dataPointCount }} data points</div>
                <div class="last-update">Updated {{ lastUpdateText }}</div>
            </div>
        </div>
    `,
    data() {
        return {
            chart: null,
            isLoading: false,
            isFullscreen: false,
            selectedTimeRange: '5m',
            lastUpdate: null,
            resizeObserver: null
        };
    },
    computed: {
        hasData() {
            return this.chartData && 
                   this.chartData.series && 
                   this.chartData.series.length > 0 &&
                   this.chartData.series.some(s => s.data && s.data.length > 0);
        },
        
        dataPointCount() {
            if (!this.hasData) return 0;
            return this.chartData.series.reduce((total, series) => {
                return total + (series.data ? series.data.length : 0);
            }, 0);
        },
        
        lastUpdateText() {
            if (!this.lastUpdate) return 'Never';
            const now = Date.now();
            const diff = now - this.lastUpdate;
            if (diff < 1000) return 'just now';
            if (diff < 60000) return `${Math.floor(diff / 1000)}s ago`;
            return `${Math.floor(diff / 60000)}m ago`;
        }
    },
    watch: {
        chartData: {
            handler(newData) {
                if (newData && Object.keys(newData).length > 0) {
                    this.updateChart();
                    this.lastUpdate = Date.now();
                }
            },
            deep: true
        }
    },
    mounted() {
        this.initChart();
        this.setupResizeObserver();
        
        // Handle fullscreen events
        document.addEventListener('fullscreenchange', this.handleFullscreenChange);
        document.addEventListener('keydown', this.handleKeydown);
    },
    
    beforeUnmount() {
        if (this.chart) {
            this.chart.dispose();
        }
        if (this.resizeObserver) {
            this.resizeObserver.disconnect();
        }
        document.removeEventListener('fullscreenchange', this.handleFullscreenChange);
        document.removeEventListener('keydown', this.handleKeydown);
    },
    
    methods: {
        initChart() {
            if (typeof echarts === 'undefined') {
                console.error('ECharts library not loaded');
                return;
            }
            
            const chartElement = document.getElementById(this.chartId);
            if (!chartElement) {
                console.error(`Chart element with id ${this.chartId} not found`);
                return;
            }
            
            this.chart = echarts.init(chartElement, 'dark');
            
            // Set default options
            const defaultOption = {
                backgroundColor: 'transparent',
                grid: {
                    left: '3%',
                    right: '4%',
                    bottom: '3%',
                    containLabel: true
                },
                tooltip: {
                    trigger: 'axis',
                    backgroundColor: '#2d3748',
                    borderColor: '#4a5568',
                    textStyle: {
                        color: '#e2e8f0'
                    }
                },
                legend: {
                    textStyle: {
                        color: '#a0aec0'
                    }
                },
                toolbox: {
                    feature: {
                        dataZoom: {
                            yAxisIndex: 'none'
                        },
                        restore: {},
                        saveAsImage: {
                            backgroundColor: '#1a202c'
                        }
                    },
                    iconStyle: {
                        borderColor: '#a0aec0'
                    }
                },
                dataZoom: [
                    {
                        type: 'inside',
                        start: 0,
                        end: 100
                    },
                    {
                        start: 0,
                        end: 100,
                        handleIcon: 'M10.7,11.9v-1.3H9.3v1.3c-4.9,0.3-8.8,4.4-8.8,9.4c0,5,3.9,9.1,8.8,9.4v1.3h1.3v-1.3c4.9-0.3,8.8-4.4,8.8-9.4C19.5,16.3,15.6,12.2,10.7,11.9z M13.3,24.4H6.7V23.1h6.6V24.4z M13.3,19.6H6.7v-1.4h6.6V19.6z',
                        handleSize: '80%',
                        handleStyle: {
                            color: '#4a5568',
                            shadowBlur: 3,
                            shadowColor: 'rgba(0, 0, 0, 0.6)',
                            shadowOffsetX: 2,
                            shadowOffsetY: 2
                        }
                    }
                ]
            };
            
            this.chart.setOption(defaultOption);
            
            // Add click event for data points
            this.chart.on('click', this.handleChartClick);
            
            // Update chart if data is available
            if (this.hasData) {
                this.updateChart();
            }
        },
        
        updateChart() {
            if (!this.chart || !this.hasData) return;
            
            const option = this.buildChartOption();
            this.chart.setOption(option, true);
        },
        
        buildChartOption() {
            const data = this.chartData;
            
            const option = {
                title: {
                    text: data.title || this.title,
                    textStyle: {
                        color: '#e2e8f0',
                        fontSize: 16
                    }
                },
                xAxis: {
                    type: data.xAxisType || 'category',
                    data: data.xAxis || [],
                    axisLabel: {
                        color: '#a0aec0'
                    },
                    axisLine: {
                        lineStyle: {
                            color: '#4a5568'
                        }
                    }
                },
                yAxis: {
                    type: 'value',
                    name: data.yAxisLabel || '',
                    nameTextStyle: {
                        color: '#a0aec0'
                    },
                    axisLabel: {
                        color: '#a0aec0'
                    },
                    axisLine: {
                        lineStyle: {
                            color: '#4a5568'
                        }
                    },
                    splitLine: {
                        lineStyle: {
                            color: '#4a5568',
                            type: 'dashed'
                        }
                    }
                },
                series: data.series.map((series, index) => ({
                    name: series.name,
                    type: series.type || 'line',
                    data: series.data,
                    smooth: series.smooth !== false,
                    symbol: series.symbol || 'circle',
                    symbolSize: series.symbolSize || 4,
                    lineStyle: {
                        width: series.lineWidth || 2,
                        color: series.color || this.getDefaultColor(index)
                    },
                    itemStyle: {
                        color: series.color || this.getDefaultColor(index)
                    },
                    areaStyle: series.area ? {
                        opacity: 0.3,
                        color: series.color || this.getDefaultColor(index)
                    } : undefined
                }))
            };
            
            return option;
        },
        
        getDefaultColor(index) {
            const colors = [
                '#63b3ed', '#68d391', '#fbb040', '#fc8181', 
                '#b794f6', '#4fd1c7', '#f687b3', '#9ae6b4'
            ];
            return colors[index % colors.length];
        },
        
        refreshChart() {
            this.isLoading = true;
            
            // Emit refresh event to parent
            this.$emit('refresh-chart', this.chartId);
            
            // Simulate loading delay
            setTimeout(() => {
                this.isLoading = false;
            }, 500);
        },
        
        updateTimeRange() {
            // Emit time range change event
            this.$emit('time-range-changed', {
                chartId: this.chartId,
                timeRange: this.selectedTimeRange
            });
        },
        
        toggleFullscreen() {
            const container = this.$el.querySelector('.chart-container');
            
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
            
            // Resize chart after fullscreen change
            setTimeout(() => {
                if (this.chart) {
                    this.chart.resize();
                }
            }, 100);
        },
        
        handleKeydown(event) {
            if (this.isFullscreen && event.key === 'Escape') {
                this.isFullscreen = false;
            }
        },
        
        handleChartClick(params) {
            // Emit chart click event
            this.$emit('chart-click', {
                chartId: this.chartId,
                params: params
            });
        },
        
        setupResizeObserver() {
            if (typeof ResizeObserver !== 'undefined') {
                this.resizeObserver = new ResizeObserver(() => {
                    if (this.chart) {
                        this.chart.resize();
                    }
                });
                
                const chartElement = document.getElementById(this.chartId);
                if (chartElement) {
                    this.resizeObserver.observe(chartElement);
                }
            }
        }
    }
};

// Register the component globally
if (typeof Vue !== 'undefined' && Vue.createApp) {
    // Component will be registered in the main app
}