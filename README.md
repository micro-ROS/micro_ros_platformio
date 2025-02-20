![banner](.images/banner-dark-theme.png#gh-dark-mode-only)
![banner](.images/banner-light-theme.png#gh-light-mode-only)

# micro-ROS for PlatformIO
This is a micro-ROS library for bare metal projects based on platformIO.

The build process for ROS 2 and micro-ROS is based on custom meta-build system tools and [CMake](https://cmake.org/).
PlatformIO will handle the full build process, including dependencies, compilation and linkage.

- [micro-ROS for PlatformIO](#micro-ros-for-platformio)
  - [Supported boards](#supported-boards)
  - [Requirements](#requirements)
  - [How to add to your project](#how-to-add-to-your-project)
  - [Library configuration](#library-configuration)
    - [ROS 2 distribution](#ros-2-distribution)
    - [Transport configuration](#transport-configuration)
    - [Extra packages](#extra-packages)
    - [Other configuration](#other-configuration)
  - [Extend library targets](#extend-library-targets)
    - [Transport implementation](#transport-implementation)
    - [Time source](#time-source)
  - [Using the micro-ROS Agent](#using-the-micro-ros-agent)
  - [Examples](#examples)
  - [Purpose of the Project](#purpose-of-the-project)
  - [License](#license)
  - [Known Issues/Limitations](#known-issueslimitations)

## Supported boards
Supported boards are:

| Board                                        | Platform      | Framework   | Transports                               | Default meta file        |
| -------------------------------------------- | ------------- | ----------- | ---------------------------------------- | ------------------------ |
| `portenta_h7_m7`                             | `ststm32`     | `arduino`   | `serial` <br/> `wifi`                    | `colcon.meta`            |
| `teensy41`                                   | `teensy`      | `arduino`   | `serial` <br/> `native_ethernet`         | `colcon.meta`            |
| `teensy40`                                   | `teensy`      | `arduino`   | `serial`                                 | `colcon.meta`            |
| `teensy36` <br/> `teensy35` <br/> `teensy31` | `teensy`      | `arduino`   | `serial`                                 | `colcon_lowmem.meta`     |
| `due`                                        | `atmelsam`    | `arduino`   | `serial`                                 | `colcon_verylowmem.meta` |
| `zero`                                       | `atmelsam`    | `arduino`   | `serial`                                 | `colcon_verylowmem.meta` |
| `olimex_e407`                                | `ststm32`     | `arduino`   | `serial`                                 | `colcon.meta`            |
| `esp32dev`                                   | `espressif32` | `arduino`   | `serial` <br/> `wifi` <br/> `ethernet`*   | `colcon.meta`            |
| `nanorp2040connect`                          | `raspberrypi` | `arduino`   | `serial` <br/> `wifi_nina`               | `colcon_verylowmem.meta` |
| `pico`                                       | `raspberrypi` | `arduino`   | `serial`                                 | `colcon.meta`            |

\* The ESP32 ethernet transport is community contributed.

The community is encouraged to open pull request with custom use cases.

## Requirements

- PlatformIO [local installation](https://docs.platformio.org/en/stable/core/installation.html) or [PlatformIO IDE for VSCode](https://platformio.org/install/ide?install=vscode)
- PlatformIO Core version 6.1.0 or greater
- PlatformIO needs  `git`, `cmake` and `pip3` to handle micro-ROS internal dependencies:

  ```bash
  apt install -y git cmake python3-pip
  ```
  
### Platform specific requirements

#### MacOS

XCode command line tools are distributed with toolchain that is not fully compatible with micro-ROS build process.
To fix this, install GNU [binutils](https://www.gnu.org/software/binutils/) using [Homebrew](https://brew.sh/):

```bash
brew install binutils
```

## How to add to your project

The library can be included as a regular git library dependence on your `platform.ini` file:

```ini
...
lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio
```

Now to proceed with the PlatformIO workflow:

```bash
pio lib install # Install dependencies
pio run # Build the firmware
pio run --target upload # Flash the firmware
```

After the library is compiled for first time the build process will be skipped, to trigger a library build and apply [library modifications](#library-configuration) on your next platformIO build:

```bash
pio run --target clean_microros  # Clean library
```

## Library configuration
This section details the different configuration parameters available on the project `platform.ini` file.
A explanation for adding custom targets is also present


### ROS 2 distribution
The target ROS 2 distribution can be configured with the `board_microros_distro = <distribution>`, supported values are:
  - `humble`
  - `iron`
  - `jazzy` *(default value)*
  - `rolling`

### Transport configuration
The transport can be configured with the `board_microros_transport = <transport>`, supported values and configurations are:
  - `serial` *(default value)*

    ```c
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    ```

  - `wifi`
  - `wifi_nina`

    ```c
    IPAddress agent_ip(192, 168, 1, 113);
    size_t agent_port = 8888;

    char ssid[] = "WIFI_SSID";
    char psk[]= "WIFI_PSK";

    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    ```

  - `native_ethernet`

    ```c
    byte local_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };
    IPAddress local_ip(192, 168, 1, 177);
    IPAddress agent_ip(192, 168, 1, 113);
    size_t agent_port = 8888;

    set_microros_native_ethernet_transports(local_mac, local_ip, agent_ip, agent_port);
    ```

  - `ethernet`

    ```c
    IPAddress client_ip(192, 168, 1, 177);
    IPAddress gateway(192, 168, 1, 1);
    IPAddress netmask(255, 255, 255, 0);
    IPAddress agent_ip(192, 168, 1, 113);
    size_t agent_port = 8888;

    // Optional hostname, defaults to nullptr (no hostname set)
    set_microros_ethernet_transports(client_ip, gateway, netmask, agent_ip, agent_port, "my-microros-device");
    ```

  - `