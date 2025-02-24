# micro-ROS Ethernet Publisher/Subscriber Example

This example demonstrates how to create a micro-ROS node on an ESP32 that communicates over Ethernet with a ROS 2 system. The node implements a simple request-response pattern where it listens for names and responds with greetings.

## Overview

The ESP32 node:
- Subscribes to the topic `micro_ros_name`
- When it receives a name (e.g., "John"), it publishes "Hello John!" to the topic `micro_ros_response`
- Uses Ethernet for communication with the ROS 2 system
- Operates on ROS 2 domain ID 8

## Hardware Requirements

- ESP32 development board
- LAN8710 or LAN8720 Ethernet PHY module
- Ethernet cable

### Ethernet PHY Wiring

Connect the LAN8720 module to the ESP32 using the following pins:

| ESP32 GPIO | LAN87XX Pin | Description |
|------------|-------------|-------------|
| GPIO 5     | POWER      | PHY Power   |
| GPIO 23    | MDC        | Clock       |
| GPIO 18    | MDIO       | Data        |
| GPIO 17    | Clock      | 50MHz Clock |

## Network Configuration

The example uses the following network configuration (configurable in `main.cpp`):

- ESP32 IP: 10.4.4.177
- Gateway: 10.4.4.1
- Netmask: 255.255.255.0
- Agent IP: 10.4.4.187
- Agent Port: 8888

## Software Setup

1. Install PlatformIO (if not already installed)
2. Install Docker and Docker Compose
3. Clone this repository
4. Navigate to the example directory:
   ```bash
   cd examples/ethernet_pubsub
   ```

## Building and Flashing

1. Build and flash the firmware:
   ```bash
   pio run -t upload
   ```

2. (Optional) Monitor the serial output:
   ```bash
   pio device monitor
   ```

## Running the micro-ROS Agent and RQT

1. Allow X11 connections from Docker (needed for RQT):
   ```bash
   xhost +local:docker
   ```

2. Start the micro-ROS agent and RQT:
   ```bash
   docker compose up
   ```

This will start:
- A micro-ROS agent listening on UDP port 8888
- RQT with proper X11 forwarding for visualization

## Testing the Example

1. In RQT:
   - Click on Plugins -> Topics -> Message Publisher
   - Add the topic `/micro_ros_name`
   - Set the message type to `std_msgs/String`
   - Enter a name in the `data` field and click the checkbox to publish

2. To view responses:
   - In RQT, click on Plugins -> Topics -> Topic Monitor
   - Subscribe to `/micro_ros_response`
   - You should see responses like "Hello <name>!" when you publish names

## Troubleshooting

- If RQT doesn't appear, ensure X11 forwarding is properly set up with `xhost +local:docker`
- Check the serial monitor for connection status and debugging information
- Verify that your network configuration matches the settings in `main.cpp`
- Ensure all Ethernet pins are properly connected

## Cleaning Up

1. Stop the Docker containers:
   ```bash
   docker compose down
   ```

2. Revoke X11 permissions (optional):
   ```bash
   xhost -local:docker
   ```
