# uRAD ROS2 Package

A ROS2 package for interfacing with [uRAD radar](https://urad.es/en/). This package provides nodes for data acquisition, processing, and visualization of radar data.

## Overview

This package contains:
- Publisher and subscriber nodes for raw radar data
- Signal processing nodes for radar data
- Visualization tools
- Configuration utilities

## Installation

### Prerequisites
- ROS2 (tested on Humble)
- Python 3.8+
- uRAD SDK

### Building from Source
```bash
# Create a ROS2 workspace (if you don't already have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/MDkontroller/uRAD_ROS2.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select urad

# Source the setup file
source install/setup.bash
```

## Usage

### Basic Usage
```bash
# Terminal 1 - Start the radar publisher
ros2 run urad urad_pub_no_timer

# Terminal 2 - Start the zero frequency filtering node
ros2 run urad subscriber

```

### Using Launch Files
```bash
# Launch all nodes at once
ros2 launch urad radar_system.launch.py
```

### Configuration
Radar parameters can be configured in the launch files or by modifying the parameters in the node files.

## Nodes

| Node | Description |
|------|-------------|
| `publisher` | Basic publisher example |
| `subscriber` | Basic subscriber example |
| `test_pub` | Publishes real radar data with timer |
| `test_sub` | Processes radar data |
| `urad_pub_no_timer` | Publishes radar data without timer (better for real-time) |
| `zero_ff_node` | Removes zero-frequency components from radar data |

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cw_iq` | `interfaces/CwIq` | Raw I/Q data from radar |
| `/cw_iq_offsett` | `interfaces/CWIQZERO` | Processed I/Q data with DC offset removed |

## Parameters

### Radar Publisher Node
- `radar.mode`: Radar operation mode (default: 1)
- `radar.f0`: Start frequency in MHz (default: 125)
- `radar.bw`: Bandwidth in MHz (default: 0)
- `radar.ns`: Number of samples (default: 200)
- `radar.ntar`: Number of targets (default: 4)
- `radar.rmax`: Maximum range in m (default: 100)

### Zero Frequency Filter Node
- `max_voltage`: Maximum voltage for ADC conversion (default: 3.3)
- `adc_intervals`: Number of ADC intervals (default: 4096)

## Features

- Real-time radar data acquisition
- I/Q signal processing with DC offset removal
- Visualization of time-domain signals, I/Q constellation, and FFT
- Configurable parameters through ROS2 parameter system
- Launch files for easy system startup

## Architecture

```
┌─────────────────┐     ┌───────────────────────┐     ┌────────────────────┐
│  Radar Publisher│     │ Zero Frequency Filter  │     │  Radar Visualizer  │
│                 │     │                        │     │                     │
│ ┌─────────────┐ │     │ ┌──────────────────┐  │     │ ┌──────────────┐   │
│ │ Radar Config│ │     │ │ Signal Processing│  │     │ │ Time Domain  │   │
│ └─────────────┘ │     │ └──────────────────┘  │     │ └──────────────┘   │
│        │        │     │          │            │     │         │          │
│ ┌─────────────┐ │     │ ┌──────────────────┐  │     │ ┌──────────────┐   │
│ │ Raw Data    │─┼─────┼─►   DC Removal     │──┼─────┼─►Constellation │   │
│ └─────────────┘ │     │ └──────────────────┘  │     │ └──────────────┘   │
│                 │     │                        │     │         │          │
└─────────────────┘     └───────────────────────┘     │ ┌──────────────┐   │
                                                       │ │     FFT      │   │
                                                       │ └──────────────┘   │
                                                       │                    │
                                                       └────────────────────┘
```

## Development

### Code Structure
```
urad/
├── urad/
│   ├── __init__.py
│   ├── publisher.py
│   ├── subscriber.py
│   ├── test_pub.py
│   ├── test_sub.py
│   ├── urad_pub_no_timer.py
│   ├── zero_ff_node.py
│   └── uRAD_RP_SDK11.py
├── launch/
│   └── radar_system.launch.py
├── config/
│   └── radar_params.yaml
├── resource/
│   └── urad
├── setup.py
├── setup.cfg
├── package.xml
├── LICENSE
└── README.md
```

### Future Work
- Add support for multiple radar sensors
- Implement SLAM algorithms using radar data
- Integrate with ROS2 navigation stack
- Support for additional radar modes and configurations

## License
This project is licensed under the MIT License - see the LICENSE file for details.

## Author
- Nicolas Sarmiento - [MDKontroller](https://github.com/MDkontroller)