import microros_builder.library_builder as library_builder

builder = library_builder.Build('/home/pgarrido/dev/micro-ROS/micro_ros_platformio/example_project/.pio/libdeps/teensy41/micro_ros_platformio/build')
# builder.download_dev_environment()
# builder.build_dev_environment()

# builder.download_mcu_environment()
# builder.build_mcu_environment('/home/pgarrido/dev/micro-ROS/micro_ros_platformio/meta_files/colcon.meta', "/home/pgarrido/dev/micro-ROS/micro_ros_platformio/example_project/.pio/libdeps/teensy41/micro_ros_platformio/platformIO_toolchain.cmake")

build.package_mcu_library()