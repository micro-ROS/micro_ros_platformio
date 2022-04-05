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
- [TODO(acuadros95): Add a platformIO example](#todoacuadros95-add-a-platformio-example)
  - [Known platformIO issues](#known-platformio-issues)
  - [Library configuration](#library-configuration)
  - [Purpose of the Project](#purpose-of-the-project)
  - [License](#license)
  - [Known Issues/Limitations](#known-issueslimitations)

## Supported boards

Supported boards are:

| Board                                                                               | Min version | State      | Details                                                                                             | .meta file               |
| ----------------------------------------------------------------------------------- | ----------- | ---------- | --------------------------------------------------------------------------------------------------- | ------------------------ |
| [Arduino Portenta H7 M7 Core](https://store.arduino.cc/portenta-h7)                 | v1.8.5      | Supported  | Official Arduino support                                                                            | `colcon.meta`            |
| [Arduino Nano RP2040 Connect](https://docs.arduino.cc/hardware/nano-rp2040-connect) | v1.8.5      | Supported  | Official Arduino support                                                                            | `colcon_verylowmem.meta` |
| [Teensy 4.0](https://www.pjrc.com/store/teensy40.html)                              | v1.8.5      | Not tested | [Based on Teensyduino](https://www.pjrc.com/teensy/td_download.html)                                | `colcon.meta`            |
| [Teensy 4.1](https://www.pjrc.com/store/teensy41.html)                              | v1.8.5      | Supported  | [Based on Teensyduino](https://www.pjrc.com/teensy/td_download.html)                                | `colcon.meta`            |
| [Teensy 3.2/3.1](https://www.pjrc.com/store/teensy32.html)                          | v1.8.5      | Supported  | [Based on Teensyduino](https://www.pjrc.com/teensy/td_download.html)                                | `colcon_lowmem.meta`     |
| [Teensy 3.5](https://www.pjrc.com/store/teensy35.html)                              | v1.8.5      | Not tested | [Based on Teensyduino](https://www.pjrc.com/teensy/td_download.html)                                | `colcon_lowmem.meta`     |
| [Teensy 3.6](https://www.pjrc.com/store/teensy36.html)                              | v1.8.5      | Supported  | [Based on Teensyduino](https://www.pjrc.com/teensy/td_download.html)                                | `colcon_lowmem.meta`     |
| [ESP32 Dev Module](https://docs.espressif.com/projects/arduino-esp32/en/latest/boards/ESP32-DevKitC-1.html) | v1.8.5  | Supported  | [Arduino core for the ESP32 (v2.0.2)](https://github.com/espressif/arduino-esp32/releases/tag/2.0.2) | `colcon.meta`   |

Community contributed boards are:

| Board                                                                                    | Min version | Contributor                                    | Details | .meta file               |
| ---------------------------------------------------------------------------------------- | ----------- | ---------------------------------------------- | ------- | ------------------------ |
| [Arduino Due](https://store.arduino.cc/arduino-due)                                      | -           | [@lukicdarkoo](https://github.com/lukicdarkoo) |         | `colcon_verylowmem.meta` |
| [Arduino Zero](https://store.arduino.cc/arduino-zero)                                    | -           | [@lukicdarkoo](https://github.com/lukicdarkoo) |         | `colcon_verylowmem.meta` |
| [STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/resources/STM32-E407.pdf) | -           | [@dominikn](https://github.com/dominikn)       |         | `colcon.meta`            |

You can find the available precompiled ROS 2 types for messages and services in [available_ros2_types](available_ros2_types).

## How to add to your project

For boards supported by micro-ROS, all you have to do to add the library to your project is including the following lines in the existing `platformio.ini` file:

```ini
[env:<YOUR_BOARD>]

...
lib_deps =
    https://github.com/micro-ROS/micro_ros_arduino

build_flags =
    -D <TARGET_DEFINITION>
```

| Board                       | <YOUR_BOARD>        | <TARGET_DEFINITION>                 |
| ----------------------------| ------------------- | ----------------------------------- |
| Arduino Portenta H7 M7 Core | portenta_h7_m7      | TARGET_PORTENTA_H7_M7               |
| Arduino Nano RP2040 Connect | nanorp2040connect   | ARDUINO_NANO_RP2040_CONNECT         |
| Teensy 4.1/4.0              | teensy41 / teensy40 | ARDUINO_TEENSY41                    |
| Teensy 3.6                  | teensy36            | ARDUINO_TEENSY36                    |
| Teensy 3.5                  | teensy35            | ARDUINO_TEENSY35                    |
| Teensy 3.2  / 3.1           | teensy31            | ARDUINO_TEENSY32 / ARDUINO_TEENSY31 |
| ESP32 Dev Module            | esp32dev            | ESP32                               |
| STM32-E407                  | olimex_e407         | TARGET_STM32F4                      |
| Arduino Due                 | due                 | -                                   |
| Arduino Zero                | zero                | -                                   |

Now to proceed with the PlatformIO workflow:

```bash
pio lib install # Install dependencies
pio run # Build the firmware
pio run --target upload # Flash the firmware
```

# TODO(acuadros95): Add a platformIO example
An example of a micro-ROS application using PlatformIO is available [here](https://github.com/husarion/micro_ros_stm32_template).

## Known platformIO issues

- Arduino Nano RP2040 Connect

  - The following versioning shall be used:
    ```
    lib_deps =
      arduino-libraries/WiFiNINA@^1.8.13
      ...

    platform_packages =
      framework-arduino-mbed @ ~2.4.1
    ```

  - Library dependency finder shall be set to `chain+`: `lib_ldf_mode = chain+`

    Related: https://github.com/micro-ROS/micro_ros_arduino/issues/780

- ESP32 Dev Module
  - Known issues with espressif32 arduino package, use `2.0.2` version:
    ```
    [env:esp32dev]
    platform = https://github.com/platformio/platform-espressif32.git#feature/arduino-upstream
    board = esp32dev
    framework = arduino
    lib_deps =
        https://github.com/micro-ROS/micro_ros_arduino.git
    build_flags =
        -D ESP32

    platform_packages =
      framework-arduinoespressif32@https://github.com/espressif/arduino-esp32.git#2.0.2
    ```

    Related: https://github.com/micro-ROS/micro_ros_arduino/issues/736, https://github.com/platformio/platform-espressif32/issues/616

## Library configuration

Folders added to `extras/library_generation/extra_packages` and entries added to `extras/library_generation/extra_packages/extra_packages.repos` will be taken into account by this build system.
This should be used for example when adding custom messages types or custom micro-ROS packages.

You can so [configure many parameters](https://micro.ros.org/docs/tutorials/advanced/microxrcedds_rmw_configuration/) of the library by editing the respective `.meta` file in the `extras/library_generation/` directory.

Note that after the library is compiled for first time, the build process will be skipped. To trigger a library build you can delete the generated `include` folder.  
This will regenerate the library on your next platformIO build.

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