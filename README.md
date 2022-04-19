<br/>

<a>
   <p align="center">
      <img width="14%" src=".images/PlatformIO_logo.svg">
      <img width="40%" src=".images/microros_logo.png">
   </p>
</a>
<br/>

# micro-ROS for PlatformIO
This is a micro-ROS library for baremetal projects based on platformIO.

The build process for ROS 2 and micro-ROS is based on custom meta-build system tools and [CMake](https://cmake.org/). PlatformIO will handle the full build process, including dependencies, compilation and linkage.
Users can modify the micro-ROS library configuration or [RMW parameters](https://micro.ros.org/docs/tutorials/advanced/microxrcedds_rmw_configuration/) by customizing the respective [.meta file](https://github.com/micro-ROS/micro_ros_arduino/tree/main/extras/library_generation).

- [micro-ROS for PlatformIO](#micro-ros-for-platformio)
  - [Supported boards](#supported-boards)
  - [How to add to your project](#how-to-add-to-your-project)
  - [Library configuration](#library-configuration)
    - [ROS 2 distribution](#ros-2-distribution)
    - [Transport configuration](#transport-configuration)
    - [Memory configuration](#memory-configuration)
    - [Extra packages](#extra-packages)
  - [Custom targets](#custom-targets)
  - [Examples](#examples)
  - [Purpose of the Project](#purpose-of-the-project)
  - [License](#license)
  - [Known Issues/Limitations](#known-issueslimitations)

## Supported boards

Tested boards are:


| Board               | Platform      | Framework | Transport         |
| ------------------- | ------------- | --------- | ----------------- |
| `portenta_h7_m7`    | `ststm32`     | `arduino` | `serial`          |
| `teensy41`          | `teensy`      | `arduino` | `serial`          |
| `teensy40`          | `teensy`      | `arduino` | `serial`          |
| `teensy36`          | `teensy`      | `arduino` | `serial`          |
| `teensy35`          | `teensy`      | `arduino` | `serial`          |
| `teensy31`          | `teensy`      | `arduino` | `serial`          |
| `due`               | `atmelsam`    | `arduino` | `serial`          |
| `zero`              | `atmelsam`    | `arduino` | `serial`          |
| `olimex_e407`       | `ststm32`     | `arduino` | `serial`          |
| `esp32dev`          | `espressif32` | `arduino` | `serial`          |
| `nanorp2040connect` | `raspberrypi` | `arduino` | `serial`          |
| `teensy41`          | `teensy`      | `arduino` | `native_ethernet` |
| `nanorp2040connect` | `raspberrypi` | `arduino` | `wifi_nina`       |
| `portenta_h7_m7`    | `ststm32`     | `arduino` | `wifi`            |
| `esp32dev`          | `espressif32` | `arduino` | `wifi`            |

The community is encouraged to test this library on different boards, platforms, transports, ...
Pull request with tested use cases are always welcome.

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

micro-ROS will be built once as a static library.

## Library configuration
This section details the multiple parameters that can be configured on the project `platform.ini` file.

Note that after the library is compiled for first time, the build process will be skipped. To trigger a library build, <TODO(acuadros95): add clean step>
This will regenerate the library on your next platformIO build, applying modifications on the following parameters.

### ROS 2 distribution
The target ROS 2 distribution can be configured with the `board_microros_distro = <distribution>`, supported values are:
  - `galactic` *(default value)*

### Transport configuration
The transport can be configured with the `board_microros_transport = <transport>`, supported values are:
  - `serial` *(default value)*
  - `wifi`
  - `wifi_nina`
  - `native_ethernet`

### Memory configuration
The middleware memory resources can be configured with a customized meta file, the `microros_user_meta = <file_name>.meta`, the file shall be on the project main folder.
Documentation on configurable values can be found [here](https://micro.ros.org/docs/tutorials/advanced/microxrcedds_rmw_configuration/).

Example configurations can be found on the [metas](./metas) folder.

### Extra packages
Folders added to `<Project_directory>/extra_packages` and entries added to `<Project_directory>/extra_packages/extra_packages.repos` will be taken into account by this build system.
This should be used for example when adding custom messages types or custom micro-ROS packages.

## Custom targets
This library can be easily adapted to different boards, transports or RTOS, to archieve this:

- Transport: Users can include their custom transport following the signatures shown on [./platform_code/arduino/micro_ros_platformio.h](./platform_code/arduino/micro_ros_platformio.h) and the provided sources on [./platform_code/arduino/<transport>](./platform_code/arduino) as a reference. More info can be found [here](https://micro-xrce-dds.docs.eprosima.com/en/latest/transport.html#custom-transport).
- Time: micro-ROS needs a `clock_gettime` implementation, following POSIX implementation with an accuracy of atleast 1 millisecond.  
  This method is used to retrieve the elapsed time on executor spins and reliable communication, an example implementation can be found on [clock_gettime.cpp](./platform_code/arduino/clock_gettime.cpp)

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

- When using provided precompiled libraries, users should take into account the already configured static memory pools in middleware layers. [More info here](https://micro.ros.org/docs/tutorials/advanced/microxrcedds_rmw_configuration/).
- micro-ROS transports should be refactored in order to provide a pluggable mechanisms. Only USB serial transports are provided.