<br/>
<a>
   <p align="center">
      <img width="40%" src=".images/microros_logo.png">
      <img width="14%" src=".images/PlatformIO_logo.svg">
   </p>
</a>
<br/>

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
  - [Custom targets](#custom-targets)
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
  
The community is encouraged to open pull request with tested use cases on different boards, platforms, transports, ...

## Requirements

- PlatformIO [local installation](https://docs.platformio.org/en/stable/core/installation.html) or [PlatformIO IDE for VSCode](https://platformio.org/install/ide?install=vscode)
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

Note that after the library is compiled for first time, the build process will be skipped. To trigger a library build, <TODO(acuadros95): add clean step> This will regenerate the library on your next platformIO build, applying modifications listed on [Library configuration](#library-configuration).

## Library configuration
This section details the different configuration parameters available on the project `platform.ini` file.
A explanation for adding custom targets is also present


### ROS 2 distribution
The target ROS 2 distribution can be configured with the `board_microros_distro = <distribution>`, supported values are:
  - `galactic` *(default value)*

### Transport configuration
The transport can be configured with the `board_microros_transport = <transport>`, supported values are:
  - `serial` *(default value)*
  - `wifi`
  - `wifi_nina`
  - `native_ethernet`

### Extra packages
Folders added to `<Project_directory>/extra_packages` and entries added to `<Project_directory>/extra_packages/extra_packages.repos` will be taken into account by this build system.
This should be used for example when adding custom messages types or custom micro-ROS packages.

### Other configuration
Library packages can be configured with a customized meta file, configurable with the parameter `microros_user_meta = <file_name>.meta`. This file shall be on the project main folder.  
This allows the user to customize the library memory resources or activate optional functionality such as multithreading, including configuration of user [Extra packages](#extra-packages).

- Documentation on available parameters can be found [here](https://micro.ros.org/docs/tutorials/advanced/microxrcedds_rmw_configuration) and [here]([microxrcedds_rmw_configuration](https://micro-xrce-dds.docs.eprosima.com/en/latest/client.html)).
- Default configurations can be found on the [metas](./metas) folder.  
  
  *Note: the [common.meta](./metas/common.meta) file makes general adjustments to the library and shall not be modified by the user.*

## Custom targets
This library can be easily adapted to different boards, transports or RTOS, to achieve this:

- Transport: Users can include their custom transport following the signatures shown on [micro_ros_platformio.h](./platform_code/arduino/micro_ros_platformio.h) and the provided sources on [platform_code](./platform_code) as a reference. More info can be found [here](https://micro-xrce-dds.docs.eprosima.com/en/latest/transport.html#custom-transport).
- Time: micro-ROS needs a `clock_gettime` implementation, following POSIX implementation with an accuracy of at least 1 millisecond.  
  This method is used to retrieve the elapsed time on executor spins and reliable communication, an example implementation can be found on [clock_gettime.cpp](./platform_code/arduino/clock_gettime.cpp)

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
The following example projects are available:
- TODO

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

- TODO