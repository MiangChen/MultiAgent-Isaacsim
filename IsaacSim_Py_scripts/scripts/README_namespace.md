# IsaacSim ROS2 Namespace Support

This document describes how to use ROS2 namespace support in the IsaacSim scripts.

## Overview

The IsaacSim simulation scripts now support ROS2 namespaces to allow multiple simulation instances or proper topic organization.

## Supported Scripts

- `isaacsim_lidar.py` - LiDAR simulation with namespace support
- `isaacsim_perception.py` - Perception simulation with namespace support

## Usage

### Basic Usage without Namespace

```bash
# Default behavior (no namespace)
./ros_libraries/simulation/world_model/scripts/isaacsim_lidar.py --gui --world Isaac/Environments/Grid/default_environment.usd
```

Topics will be published as:
- `/front/pointcloud`
- `/back/pointcloud`
- `/front/depth`
- `/back/depth`
- `/collision`

### Usage with Namespace

```bash
# With namespace
./ros_libraries/simulation/world_model/scripts/isaacsim_lidar.py --gui --world Isaac/Environments/Grid/default_environment.usd --namespace drone1
```

Topics will be published as:
- `/drone1/front/pointcloud`
- `/drone1/back/pointcloud`
- `/drone1/front/depth`
- `/drone1/back/depth`
- `/drone1/collision`

### Multiple Drone Setup

```bash
# Terminal 1 - Drone 1
./ros_libraries/simulation/world_model/scripts/isaacsim_lidar.py --gui --world Isaac/Environments/Grid/default_environment.usd --namespace drone1

# Terminal 2 - Drone 2
./ros_libraries/simulation/world_model/scripts/isaacsim_lidar.py --world Isaac/Environments/Grid/default_environment.usd --namespace drone2
```

## Services

Services are also namespaced:
- `/drone1/set_entity_state`
- `/drone1/spawn_entity`
- `/drone1/pause_physics`
- `/drone1/unpause_physics`
- `/drone1/generate_omap`

## Node Names

The ROS2 node will be created with the specified namespace:
- Without namespace: `/isaacsim_interface`
- With namespace: `/drone1/isaacsim_interface`

## Troubleshooting

### Topics Not Appearing with Namespace

If topics are not appearing with the expected namespace, check:

1. The namespace argument is being passed correctly
2. Use `ros2 topic list` to see all available topics
3. Use `ros2 node list` to see the node names

### Example Debug Commands

```bash
# List all topics
ros2 topic list

# Show topic info with namespace
ros2 topic info /drone1/front/pointcloud -v

# Echo namespaced topic
ros2 topic echo /drone1/front/pointcloud
``` 