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

| Board                                        | Platform      | Framework   | Transports                       | Default meta file       |
| -------------------------------------------- | ------------- | ----------- | -------------------------------- | ------------------------ |
| `portenta_h7_m7`                             | `ststm32`     | `arduino`   | `serial` <br/> `wifi`            | `colcon.meta`            |
| `teensy41`                                   | `teensy`      | `arduino`   | `serial` <br/> `native_ethernet` | `colcon.meta`            |
| `teensy40`                                   | `teensy`      | `arduino`   | `serial`                         | `colcon.meta`            |
| `teensy36` <br/> `teensy35` <br/> `teensy31` | `teensy`      | `arduino`   | `serial`                         | `colcon_lowmem.meta`     |
| `due`                                        | `atmelsam`    | `arduino`   | `serial`                         | `colcon_verylowmem.meta` |
| `zero`                                       | `atmelsam`    | `arduino`   | `serial`                         | `colcon_verylowmem.meta` |
| `olimex_e407`                                | `ststm32`     | `arduino`   | `serial`                         | `colcon.meta`            |
| `esp32dev`                                   | `espressif32` | `arduino`   | `serial` <br/> `wifi`            | `colcon.meta`            |
| `nanorp2040connect`                          | `raspberrypi` | `arduino`   | `serial` <br/> `wifi_nina`       | `colcon_verylowmem.meta` |
| `pico`                                       | `raspberrypi` | `arduino`   | `serial`                         | `colcon.meta`|

The community is encouraged to open pull request with custom use cases.

## Requirements

- PlatformIO [local installation](https://docs.platformio.org/en/stable/core/installation.html) or [PlatformIO IDE for VSCode](https://platformio.org/install/ide?install=vscode)
- PlatformIO Core version 6.1.0 or greater
- PlatformIO needs  `git`, `cmake` and `pip3` to handle micro-ROS internal dependencies:

  ```bash
  apt install -y git cmake python3-pip
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
  - `humble` *(default value)*
  - `galactic`
  - `rolling`
  - `foxy`

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

  - `custom`

    The user will need to write transport functions in app code and provide it to the micro-ROS library using [`rmw_uros_set_custom_transport()` API](https://micro.ros.org/docs/tutorials/advanced/create_custom_transports/)

    ```c
    bool platformio_transport_open(struct uxrCustomTransport * transport) {...};
    bool platformio_transport_close(struct uxrCustomTransport * transport) {...};
    size_t platformio_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {...};
    size_t platformio_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {...};

    rmw_uros_set_custom_transport(
      MICROROS_TRANSPORTS_FRAMING_MODE, // Set the MICROROS_TRANSPORTS_FRAMING_MODE or MICROROS_TRANSPORTS_PACKET_MODE mode accordingly
      NULL,
      platformio_transport_open,
      platformio_transport_close,
      platformio_transport_write,
      platformio_transport_read
    );
    ```

### Extra packages
Colcon packages can be added to the build process using this two methods:
- Package directories copied on the `<Project_directory>/extra_packages` folder.
- Git repositories included on the `<Project_directory>/extra_packages/extra_packages.repos` yaml file.

This should be used for example when adding custom messages types or custom micro-ROS packages.

### Other configuration
Library packages can be configured with a customized meta file on the project main folder: `board_microros_user_meta = <file_name.meta>`.

This allows the user to customize the library memory resources or activate optional functionality such as multithreading, including configuration of user [Extra packages](#extra-packages).

- Documentation on available parameters can be found [here](https://micro.ros.org/docs/tutorials/advanced/microxrcedds_rmw_configuration) and [here]([microxrcedds_rmw_configuration](https://micro-xrce-dds.docs.eprosima.com/en/latest/client.html)).
- Default configurations can be found on the [metas](./metas) folder.

  *Note: the [common.meta](./metas/common.meta) file makes general adjustments to the library and shall not be modified by the user.*

## Extend library targets
This library can be easily adapted to different boards, transports or RTOS, to achieve this the user shall provide:

### Transport implementation

New transport implementations shall follow the signatures shown on [micro_ros_platformio.h](./platform_code/arduino/micro_ros_platformio.h), the [provided sources](./platform_code) can be used as reference along [this documentation](https://micro-xrce-dds.docs.eprosima.com/en/latest/transport.html#custom-transport). Contributed transport source code shall be added on the `./platform_code/<framework>/<board_microros_transport>` path. Example:

- `platform.ini`:
  ```ini
  framework = arduino
  board_microros_transport = wifi
  ```
- Transport source files: [platform_code/arduino/wifi](https://github.com/micro-ROS/micro_ros_platformio/tree/main/platform_code/arduino/wifi)
- Also, a `MICRO_ROS_TRANSPORT_<FRAMEWORK>_<TRANSPORT>` definition will be available:
  https://github.com/micro-ROS/micro_ros_platformio/blob/de7a61c7e86fdd0186ed8b7d8ec320994e8ebcbf/ci/src/main.cpp#L3

  *Note: `board_microros_transport = custom` should not be used, as it is used to add custom transports on user app code*

### Time source
micro-ROS needs a time source to handle executor spins and synchronize reliable communication. To achieve this, a `clock_gettime` [POSIX compliant](https://linux.die.net/man/3/clock_gettime) implementation is required, with a minimum resolution of 1 millisecond.

This method shall be included on a `clock_gettime.cpp` source file under the `./platform_code/<framework>/` path, an example implementation can be found on [clock_gettime.cpp](./platform_code/arduino/clock_gettime.cpp)

## Using the micro-ROS Agent
It is possible to use a **micro-ROS Agent** just by using this docker command:

```bash
# UDPv4 micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6

# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev [YOUR BOARD PORT] -v6

# TCPv4 micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO tcp4 --port 8888 -v6

# CAN-FD micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO canfd --dev [YOUR CAN INTERFACE] -v6
```

For the supported transports, only the `serial` and `udp4` versions shall be used, although users can develop
and use the agent to test their own `tcp4` and `canfd` custom transports.

It is also possible to use custom transports on a `micro-XRCE Agent` instance. More info available [here](https://micro-xrce-dds.docs.eprosima.com/en/latest/agent.html#custom-transport-agent).

## Examples
A simple publisher project using serial transport is available on the [examples](./examples) directory, this examples is meant to be modified with the user board.

- More micro-ROS usage examples are available on [micro-ROS-demos/rclc](https://github.com/micro-ROS/micro-ROS-demos/tree/galactic/rclc).
- For a complete micro-ROS tutorial, check [Programming with rcl and rclc](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/overview/) documentation.

## Purpose of the Project

This software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards, e.g., ISO 26262.

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open-source components included in this repository,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

- For `wifi_nina` transport, the following versioning shall be used:

    ```ini
    lib_deps =
      arduino-libraries/WiFiNINA@^1.8.13
    ```

- For `nanorp2040connect` board with `serial` transport, the library dependency finder shall be set to `chain+`:

    ```ini
    lib_ldf_mode = chain+
    ```
- For `pico` board with `serial` transport, the library dependency finder shall be set to `chain+`:

    ```ini
    lib_ldf_mode = chain+
    ```
