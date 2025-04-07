# uRAD ROS2 Package

A ROS2 package for interfacing with [uRAD radar](https://urad.es/en/). This package provides nodes for data acquisition, processing, and visualization of radar data.

## Overview

This package contains:
- Publisher nodes for radar data acquisition (both timer-based and continuous)
- Subscriber nodes for radar IQ Channels
- Signal processing nodes (zero frequency filtering)
- Basic Configuration for CW (Continues Wave) operation mode and parameter handling.
- Launch file for easy system startup

## Installation

### Prerequisites
- ROS2 (tested on Humble)
- Python 3.8+
- uRAD SDK (included in the package)
- NumPy

### Building from Source
```bash
# Create a ROS2 workspace (if you don't already have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/MDkontroller/uRAD_ROS2.git mdkontroller-urad_ros2

# Install Python dependencies
pip install numpy pyyaml

# Navigate back to workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select urad

# Source the setup file
source install/setup.bash
```

### Custom Message Interfaces
This package requires custom message interfaces. Make sure you also have the `interfaces` package cloned and built in your workspace. The interfaces package should define:

- `CwIq.msg`: For raw I/Q data from the radar
- `CWIQZERO.msg`: For processed I/Q data with DC offset removed

This package should be compiled using C++ and added to the urad package directory.

## Usage

### Basic Usage
```bash
# Terminal 1 - Start the radar publisher (continuous mode - better for real-time applications)
ros2 run urad urad_publisher_continuous

# Terminal 2 - Start the zero frequency filtering node
ros2 run urad urad_zero_ff

# Terminal 3 - Start the subscriber to process data
ros2 run urad urad_subscriber
```

### Using Launch Files
```bash
# Launch all nodes at once
ros2 launch urad radar_system.launch.py
```

### Using the Dummy Publisher for Testing
If you don't have physical radar hardware, you can use the dummy publisher for testing:

```bash
# Start the dummy radar publisher
ros2 run urad urad_dummy_publisher
```

## Configuration

Radar parameters can be configured in multiple ways:

1. **YAML Configuration** - Edit the files in the `config/` directory:
   - `urad_config.yaml`: General radar configuration
   - `radar_parameters.yaml`: Parameters structured for ROS2 parameter server

2. **Launch File Parameters** - Override parameters in launch files

3. **Command Line** - Set parameters via command line:
   ```bash
   ros2 run urad urad_publisher --ros-args -p radar.mode:=1 -p radar.f0:=125
   ```

## Nodes

| Node | Description |
|------|-------------|
| `urad_publisher` | Basic timer-based publisher for radar data |
| `urad_publisher_continuous` | Continuous publisher for radar data (better for real-time) |
| `urad_subscriber` | Processes and logs radar data |
| `urad_zero_ff` | Removes zero-frequency components from radar data |
| `urad_dummy_publisher` | Generates synthetic radar data for testing |

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cw_iq` | `interfaces/CwIq` | Raw I/Q data from radar |
| `/cw_iq_offsett` | `interfaces/CWIQZERO` | Processed I/Q data with DC offset removed |

## Parameters

### Radar Parameters
- `radar.mode`: Radar operation mode (default: 1)
- `radar.f0`: Start frequency in MHz (default: 125)
- `radar.bw`: Bandwidth in MHz (default: 0)
- `radar.ns`: Number of samples (default: 200)
- `radar.ntar`: Number of targets (default: 4)
- `radar.rmax`: Maximum range in m (default: 100)

### Zero Frequency Filter Parameters
- `max_voltage`: Maximum voltage for ADC conversion (default: 3.3)
- `adc_intervals`: Number of ADC intervals (default: 4096)

### Dummy Publisher Parameters
- `publish_rate`: Publication rate in Hz (default: 10.0)
- `samples`: Number of samples to generate (default: 200)
- `noise_level`: Noise level for simulation (default: 0.1)

## Architecture

```
┌─────────────────┐     ┌───────────────────────┐
│  Radar Publisher│     │ Zero Frequency Filter │
│                 │     │                       │   
│ ┌─────────────┐ │     │ ┌──────────────────┐  │  
│ │ Radar Config│ │     │ │ Signal Processing│  │ 
│ └─────────────┘ │     │ └──────────────────┘  │
│        │        │     │          │            │
│ ┌─────────────┐ │     │ ┌──────────────────┐  │
│ │ Raw Data    │─┼─────┼─►   DC Removal     │──|
│ └─────────────┘ │     │ └──────────────────┘  │     
│                 │     │                       │ 
└─────────────────┘     └───────────────────────┘
```

## Code Structure

The package is structured as follows:

```
mdkontroller-urad_ros2/
├── README.md
├── package.xml
├── setup.cfg
├── setup.py
├── config/
│   ├── radar_parameters.yaml    # ROS2 parameter structure
│   └── urad_config.yaml         # Direct configuration
├── launch/
│   └── radar_system.launch.py   # Launch file for the complete system
├── resource/
│   └── urad
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── urad/
    ├── __init__.py
    ├── dummy_radar_publisher.py  # Synthetic data publisher for testing
    ├── urad_base.py              # Base class for radar nodes
    ├── urad_publisher.py         # Timer-based publisher
    ├── urad_publisher_continuous.py  # Continuous publisher (no timer)
    ├── urad_subscriber.py        # Basic subscriber
    ├── uRAD_RP_SDK11.py          # uRAD SDK
    └── zero_ff_node.py           # Zero frequency filter
```

## Future Work
- Integrate with ROS2 navigation stack
- develop c++ drivers for Real-Time processing
- Support for additional radar modes and configurations

## License
This project is licensed under the MIT License - see the LICENSE file for details.

## Author
- Nicolas Sarmiento - [MDKontroller](https://github.com/MDkontroller)