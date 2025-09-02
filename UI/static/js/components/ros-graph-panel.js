// ROS Graph Visualization Panel Component
const RosGraphPanel = {
    name: 'RosGraphPanel',
    props: {
        graphData: {
            type: Object,
            default: () => ({ nodes: [], topics: [], connections: [] })
        }
    },
    template: `
        <div class="ros-graph-panel">
            <div class="panel-header">
                <h3>ROS Communication Graph</h3>
                <div class="graph-controls">
                    <select v-model="layoutType" @change="updateLayout" class="layout-selector">
                        <option value="hierarchical">Hierarchical</option>
                        <option value="force">Force-directed</option>
                        <option value="circular">Circular</option>
                    </select>
                    <button @click="fitToView" class="fit-btn">Fit View</button>
                    <button @click="refreshGraph" class="refresh-btn">↻</button>
                </div>
            </div>
            
            <div class="graph-container">
                <div id="ros-network-graph" class="network-canvas"></div>
                <div v-if="isLoading" class="graph-loading">
                    <div class="loading-spinner"></div>
                    <div class="loading-text">Loading ROS graph...</div>
                </div>
                <div v-if="!isLoading && !hasData" class="graph-empty">
                    <div class="empty-icon">🔗</div>
                    <div class="empty-text">No ROS nodes detected</div>
                    <button @click="refreshGraph" class="retry-btn">Scan for nodes</button>
                </div>
            </div>
            
            <div class="graph-info" v-if="hasData && !isLoading">
                <div class="info-section">
                    <div class="info-item">
                        <span class="info-label">Nodes:</span>
                        <span class="info-value">{{ nodeCount }}</span>
                    </div>
                    <div class="info-item">
                        <span class="info-label">Topics:</span>
                        <span class="info-value">{{ topicCount }}</span>
                    </div>
                    <div class="info-item">
                        <span class="info-label">Connections:</span>
                        <span class="info-value">{{ connectionCount }}</span>
                    </div>
                </div>
                <div class="selected-info" v-if="selectedNode">
                    <div class="selected-title">Selected: {{ selectedNode.label }}</div>
                    <div class="selected-details">{{ getNodeDetails(selectedNode) }}</div>
                </div>
            </div>
            
            <div class="graph-legend">
                <div class="legend-item">
                    <div class="legend-color node-color"></div>
                    <span>ROS Nodes</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color topic-color"></div>
                    <span>Topics</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color service-color"></div>
                    <span>Services</span>
                </div>
            </div>
        </div>
    `,
    data() {
        return {
            network: null,
            isLoading: false,
            layoutType: 'hierarchical',
            selectedNode: null,
            networkData: {
                nodes: new vis.DataSet([]),
                edges: new vis.DataSet([])
            }
        };
    },
    computed: {
        hasData() {
            return this.graphData && 
                   ((this.graphData.nodes && this.graphData.nodes.length > 0) ||
                    (this.graphData.topics && this.graphData.topics.length > 0));
        },
        
        nodeCount() {
            return this.graphData.nodes ? this.graphData.nodes.length : 0;
        },
        
        topicCount() {
            return this.graphData.topics ? this.graphData.topics.length : 0;
        },
        
        connectionCount() {
            return this.graphData.connections ? this.graphData.connections.length : 0;
        }
    },
    watch: {
        graphData: {
            handler(newData) {
                if (newData && (newData.nodes || newData.topics)) {
                    this.updateGraph();
                }
            },
            deep: true
        }
    },
    mounted() {
        this.initNetwork();
        if (this.hasData) {
            this.updateGraph();
        }
    },
    
    beforeUnmount() {
        if (this.network) {
            this.network.destroy();
        }
    },
    
    methods: {
        initNetwork() {
            if (typeof vis === 'undefined') {
                console.error('Vis.js library not loaded');
                return;
            }
            
            const container = document.getElementById('ros-network-graph');
            if (!container) {
                console.error('Network graph container not found');
                return;
            }
            
            const options = {
                layout: {
                    hierarchical: {
                        enabled: this.layoutType === 'hierarchical',
                        direction: 'UD',
                        sortMethod: 'directed',
                        nodeSpacing: 150,
                        levelSeparation: 200
                    }
                },
                physics: {
                    enabled: this.layoutType === 'force',
                    stabilization: { iterations: 100 },
                    barnesHut: {
                        gravitationalConstant: -2000,
                        centralGravity: 0.3,
                        springLength: 95,
                        springConstant: 0.04,
                        damping: 0.09
                    }
                },
                nodes: {
                    shape: 'box',
                    margin: 10,
                    font: {
                        color: '#e2e8f0',
                        size: 12,
                        face: 'Arial'
                    },
                    borderWidth: 2,
                    shadow: {
                        enabled: true,
                        color: 'rgba(0,0,0,0.3)',
                        size: 5,
                        x: 2,
                        y: 2
                    }
                },
                edges: {
                    arrows: {
                        to: { enabled: true, scaleFactor: 0.8 }
                    },
                    color: {
                        color: '#4a5568',
                        highlight: '#63b3ed',
                        hover: '#63b3ed'
                    },
                    width: 2,
                    smooth: {
                        enabled: true,
                        type: 'dynamic',
                        roundness: 0.5
                    }
                },
                interaction: {
                    hover: true,
                    selectConnectedEdges: true,
                    tooltipDelay: 200
                },
                configure: {
                    enabled: false
                }
            };
            
            this.network = new vis.Network(container, this.networkData, options);
            
            // Add event listeners
            this.network.on('selectNode', this.handleNodeSelect);
            this.network.on('deselectNode', this.handleNodeDeselect);
            this.network.on('hoverNode', this.handleNodeHover);
            this.network.on('blurNode', this.handleNodeBlur);
        },
        
        updateGraph() {
            if (!this.network || !this.hasData) return;
            
            const nodes = [];
            const edges = [];
            
            // Add ROS nodes
            if (this.graphData.nodes) {
                this.graphData.nodes.forEach(node => {
                    nodes.push({
                        id: `node_${node.id || node.name}`,
                        label: node.name,
                        group: 'node',
                        color: {
                            background: '#2d3748',
                            border: '#68d391',
                            highlight: {
                                background: '#4a5568',
                                border: '#68d391'
                            }
                        },
                        title: `ROS Node: ${node.name}${node.namespace ? `\\nNamespace: ${node.namespace}` : ''}`,
                        nodeType: 'ros_node',
                        nodeData: node
                    });
                });
            }
            
            // Add topics as nodes
            if (this.graphData.topics) {
                this.graphData.topics.forEach(topic => {
                    nodes.push({
                        id: `topic_${topic.name}`,
                        label: topic.name,
                        group: 'topic',
                        shape: 'ellipse',
                        color: {
                            background: '#2d3748',
                            border: '#63b3ed',
                            highlight: {
                                background: '#4a5568',
                                border: '#63b3ed'
                            }
                        },
                        title: `Topic: ${topic.name}\\nType: ${topic.type}\\nPublishers: ${topic.publishers?.length || 0}\\nSubscribers: ${topic.subscribers?.length || 0}`,
                        nodeType: 'topic',
                        nodeData: topic
                    });
                    
                    // Add edges from publishers to topic
                    if (topic.publishers) {
                        topic.publishers.forEach(publisher => {
                            edges.push({
                                from: `node_${publisher}`,
                                to: `topic_${topic.name}`,
                                label: 'pub',
                                color: { color: '#68d391' },
                                arrows: { to: true }
                            });
                        });
                    }
                    
                    // Add edges from topic to subscribers
                    if (topic.subscribers) {
                        topic.subscribers.forEach(subscriber => {
                            edges.push({
                                from: `topic_${topic.name}`,
                                to: `node_${subscriber}`,
                                label: 'sub',
                                color: { color: '#fbb040' },
                                arrows: { to: true }
                            });
                        });
                    }
                });
            }
            
            // Add direct connections if provided
            if (this.graphData.connections) {
                this.graphData.connections.forEach(conn => {
                    edges.push({
                        from: `node_${conn.from}`,
                        to: `node_${conn.to}`,
                        label: conn.topic || '',
                        color: { color: '#fc8181' },
                        arrows: { to: true }
                    });
                });
            }
            
            // Update the network data
            this.networkData.nodes.clear();
            this.networkData.edges.clear();
            this.networkData.nodes.add(nodes);
            this.networkData.edges.add(edges);
            
            // Fit the network to view
            setTimeout(() => {
                this.fitToView();
            }, 500);
        },
        
        updateLayout() {
            if (!this.network) return;
            
            const options = {
                layout: {
                    hierarchical: {
                        enabled: this.layoutType === 'hierarchical',
                        direction: 'UD',
                        sortMethod: 'directed'
                    }
                },
                physics: {
                    enabled: this.layoutType === 'force'
                }
            };
            
            this.network.setOptions(options);
            
            if (this.layoutType === 'circular') {
                // Implement circular layout manually
                this.applyCircularLayout();
            }
        },
        
        applyCircularLayout() {
            const nodes = this.networkData.nodes.get();
            const radius = 200;
            const centerX = 0;
            const centerY = 0;
            
            nodes.forEach((node, index) => {
                const angle = (2 * Math.PI * index) / nodes.length;
                const x = centerX + radius * Math.cos(angle);
                const y = centerY + radius * Math.sin(angle);
                
                this.networkData.nodes.update({
                    id: node.id,
                    x: x,
                    y: y,
                    fixed: { x: true, y: true }
                });
            });
        },
        
        fitToView() {
            if (this.network) {
                this.network.fit({
                    animation: {
                        duration: 1000,
                        easingFunction: 'easeInOutQuad'
                    }
                });
            }
        },
        
        refreshGraph() {
            this.isLoading = true;
            
            // Emit refresh event to parent
            this.$emit('refresh-ros-graph');
            
            // Simulate loading delay
            setTimeout(() => {
                this.isLoading = false;
            }, 1000);
        },
        
        handleNodeSelect(event) {
            const nodeId = event.nodes[0];
            if (nodeId) {
                const node = this.networkData.nodes.get(nodeId);
                this.selectedNode = node;
            }
        },
        
        handleNodeDeselect() {
            this.selectedNode = null;
        },
        
        handleNodeHover(event) {
            // Add hover effects if needed
        },
        
        handleNodeBlur(event) {
            // Remove hover effects if needed
        },
        
        getNodeDetails(node) {
            if (!node || !node.nodeData) return '';
            
            if (node.nodeType === 'ros_node') {
                return `Type: ROS Node${node.nodeData.namespace ? `, Namespace: ${node.nodeData.namespace}` : ''}`;
            } else if (node.nodeType === 'topic') {
                const topic = node.nodeData;
                return `Type: ${topic.type}, Publishers: ${topic.publishers?.length || 0}, Subscribers: ${topic.subscribers?.length || 0}`;
            }
            
            return '';
        }
    }
};

// Register the component globally
if (typeof Vue !== 'undefined' && Vue.createApp) {
    // Component will be registered in the main app
}