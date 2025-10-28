```markdown
# ros_edge_compression_bridge

A robust ROS-based distributed compression framework designed for efficient edge-cloud robotics applications. This system enables real-time point cloud and image compression on resource-constrained embedded devices with seamless decompression and visualization on high-performance computing nodes.

---

## Table of Contents

- [Motivation](#motivation)  
- [Features](#features)  
- [Architecture](#architecture)  
- [Getting Started](#getting-started)  
  - [Prerequisites](#prerequisites)  
  - [Installation](#installation)  
  - [Build Instructions](#build-instructions)  
- [Usage](#usage)  
  - [Onboard (Edge) Setup](#onboard-edge-setup)  
  - [Ground Station (Cloud/High-Perf) Setup](#ground-station-cloudhigh-perf-setup)  
- [Supported Compression Modules](#supported-compression-modules)  
- [Customization & Extension](#customization-extension)  
- [Contributing](#contributing)  
- [License](#license)  

---

## Motivation

In many robotics and autonomous systems scenarios, data (such as point clouds and high-resolution images) must be transmitted from resource-limited edge devices (e.g., embedded robots, drones) to more capable ground or cloud nodes for processing and visualization. However, bandwidth, latency, and power constraints make raw data transmission impractical.  
This project addresses this challenge by providing:

- Real-time compression on the edge device  
- Lightweight transmission of compressed data  
- Decompression and visualization on cloud or ground station  
- Seamless bridging between ROS 1/ROS 2 environments  

---

## Features

- Support for both ROS 1 and ROS 2 message types and custom compression messages  
- Modular design: you can swap compression/decompression algorithms  
- Shell scripts to build all components, onboard modules, and ground station modules  
- ROS launch files for easy startup of compression/decompression pipelines  
- Example usage for point cloud and image compression pipelines  

---

## Architecture

```

[ Edge Device (ROS 1) ]  →  Compression Module →  Transmit →  Decompression Module → [ Ground Station (ROS 2) ]

````

- **Edge Device**: Runs the compression module (e.g., using modules like DRACO, JPEG XL) to encode data.  
- **Transmission**: The compressed data is sent over the network.  
- **Ground Station**: Receives compressed data, decompresses it, and feeds into ROS pipelines for further processing/visualisation.  
- Custom ROS messages (`custom_compression_msgs`) are used for encoded payloads and control.  

---

## Getting Started

### Prerequisites

- ROS 1 (e.g., melodic, noetic) and/or ROS 2 (e.g., foxy, humble) installed  
- CMake, Python (for scripts), build tools  
- Required compression libraries (e.g., DRACO, JPEG XL)  

### Installation

1. Clone the repository:  
   ```bash
   git clone https://github.com/eki-Shalll/ros_edge_compression_bridge.git  
   cd ros_edge_compression_bridge  
````

2. Update submodules (if applicable):

   ```bash
   git submodule update --init --recursive  
   ```

### Build Instructions

* To build **all** components:

  ```bash
  ./build_all.sh  
  ```

* To build **onboard (edge)** modules only:

  ```bash
  ./build_onboard.sh  
  ```

* To build **ground station (cloud/high-perf)** modules only:

  ```bash
  ./build_ground_station.sh  
  ```

---

## Usage

### Onboard (Edge) Setup

1. Launch the compression node (ROS 1):

   ```bash
   ./launch_compress-ros.sh  
   ```

2. Ensure your sensor publisher (e.g., point cloud, camera image) is active. The compression node will subscribe and publish compressed messages.

### Ground Station (Cloud) Setup

1. Launch the decompression/visualisation node (ROS 2):

   ```bash
   ./launch_onboard.sh  
   ./launch_ground_station.sh  
   ```

2. The decompressed data will be available in ROS 2 topics for further processing or display.

---

## Supported Compression Modules

* DRACO (for point clouds)
* JPEG XL (for images)
* Additional modules can be added under `jxl_compress_ros1_ws`, `jxl_decompress_ros2_ws`, `draco`, etc.

---

## Customization & Extension

* You can extend the compression framework by adding your own encoding/decoding module:

  * Add the algorithm under a new workspace (e.g., `my_compress_ws`)
  * Define ROS message types under `custom_msgs_ros1_ws` or `custom_msgs_ros2_ws`
  * Modify the bridge node to integrate new modules

* Adjust parameters via ROS parameters or launch file: topics, compression quality, network endpoints, etc.

---

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository.
2. Create your feature branch: `git checkout -b feature/my-new-module`
3. Commit your changes: `git commit -am 'Add new compression module'`
4. Push to the branch: `git push origin feature/my-new-module`
5. Open a pull request and describe your enhancement or fix.

Please ensure your code follows consistent style and includes tests/documentation when appropriate.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

**Happy compressing !**

```

