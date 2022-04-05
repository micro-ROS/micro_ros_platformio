#!/bin/bash

# Exit on error
set -e

COMPONENT_DIR=$(pwd)
GENERATION_DIR=$COMPONENT_DIR/src
INSTALL_DIR=$COMPONENT_DIR/../../include

if [ $# -ge 1 ]; then
    TOOLCHAIN=$1
	echo "Using provided toolchain file: $TOOLCHAIN"
else
    echo "Missing toolchain argument"
    exit 1
fi

if [ $# -ge 2 ]; then
    COLCON_META=$2
	echo "Using provided meta: $COLCON_META"
else
	echo "Missing meta file argument"
fi

 mkdir -p $GENERATION_DIR/micro_ros_dev
 pushd $GENERATION_DIR/micro_ros_dev >/dev/null
 	# Remove previous build
 	rm -rf build install log src
 	git clone -b galactic https://github.com/ament/ament_cmake src/ament_cmake;
 	git clone -b galactic https://github.com/ament/ament_lint src/ament_lint;
 	git clone -b galactic https://github.com/ament/ament_package src/ament_package;
 	git clone -b galactic https://github.com/ament/googletest src/googletest;
 	git clone -b galactic https://github.com/ros2/ament_cmake_ros src/ament_cmake_ros;
 	git clone -b galactic https://github.com/ament/ament_index src/ament_index;
	colcon build --cmake-args -DBUILD_TESTING=OFF;
 popd >/dev/null


mkdir -p $GENERATION_DIR/micro_ros_src
pushd $GENERATION_DIR/micro_ros_src >/dev/null
	# Remove previous build
	rm -rf build install log src

	# Clone sources
	source ../micro_ros_dev/install/local_setup.bash
	git clone -b foxy https://github.com/eProsima/micro-CDR src/micro-CDR; \
	git clone -b foxy https://github.com/eProsima/Micro-XRCE-DDS-Client src/Micro-XRCE-DDS-Client; \
	git clone -b galactic https://github.com/micro-ROS/rcl src/rcl; \
	git clone -b galactic https://github.com/ros2/rclc src/rclc; \
	git clone -b galactic https://github.com/micro-ROS/micro_ros_utilities src/micro_ros_utilities; \
	git clone -b galactic https://github.com/micro-ROS/rcutils src/rcutils; \
	git clone -b galactic https://github.com/micro-ROS/micro_ros_msgs src/micro_ros_msgs; \
	git clone -b galactic https://github.com/micro-ROS/rmw-microxrcedds src/rmw-microxrcedds; \
	git clone -b galactic https://github.com/micro-ROS/rosidl_typesupport src/rosidl_typesupport; \
	git clone -b galactic https://github.com/micro-ROS/rosidl_typesupport_microxrcedds src/rosidl_typesupport_microxrcedds; \
	git clone -b galactic https://github.com/ros2/rosidl src/rosidl; \
	git clone -b galactic https://github.com/ros2/rmw src/rmw; \
	git clone -b galactic https://github.com/ros2/rcl_interfaces src/rcl_interfaces; \
	git clone -b galactic https://github.com/ros2/rosidl_defaults src/rosidl_defaults; \
	git clone -b galactic https://github.com/ros2/unique_identifier_msgs src/unique_identifier_msgs; \
	git clone -b galactic https://github.com/ros2/common_interfaces src/common_interfaces; \
	git clone -b galactic https://github.com/ros2/test_interface_files src/test_interface_files; \
	git clone -b galactic https://github.com/ros2/rmw_implementation src/rmw_implementation; \
	git clone -b galactic https://github.com/ros2/rcl_logging src/rcl_logging; \
	git clone -b galactic https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing src/ros2_tracing; \
	touch $GENERATION_DIR/micro_ros_src/src/rcl_logging/rcl_logging_log4cxx/COLCON_IGNORE; \
	touch $GENERATION_DIR/micro_ros_src/src/rcl_logging/rcl_logging_spdlog/COLCON_IGNORE; \
	touch $GENERATION_DIR/micro_ros_src/src/rosidl/rosidl_typesupport_introspection_cpp/COLCON_IGNORE; \
	touch $GENERATION_DIR/micro_ros_src/src/rcl/rcl_yaml_param_parser/COLCON_IGNORE; \
	touch $GENERATION_DIR/micro_ros_src/src/rclc/rclc_examples/COLCON_IGNORE; \
	cp -rf $COMPONENT_DIR/extra_packages src/extra_packages || :;
	vcs import --input src/extra_packages/extra_packages.repos src/

	# Build
	rm -rf build install log
   	colcon build \
		--merge-install \
		--packages-ignore-regex=.*_cpp \
		--metas $COLCON_META \
		--cmake-args \
		"--no-warn-unused-cli" \
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=OFF \
		-DTHIRDPARTY=ON \
		-DBUILD_SHARED_LIBS=OFF \
		-DBUILD_TESTING=OFF \
		-DCMAKE_BUILD_TYPE=Release \
		-DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN \
		-DCMAKE_VERBOSE_MAKEFILE=ON;

popd >/dev/null

# Create packaged library
mkdir -p $GENERATION_DIR/libmicroros; cd $GENERATION_DIR/libmicroros; \
for file in $(find $GENERATION_DIR/micro_ros_src/install/lib/ -name '*.a'); do \
	folder=$(echo $file | sed -E "s/(.+)\/(.+).a/\2/"); \
	mkdir -p $folder; cd $folder; ar x $file; \
	for f in *; do \
		mv $f ../$folder-$f; \
	done; \
	cd ..; rm -rf $folder; \
done ; \

rm -rf $INSTALL_DIR; mkdir -p $INSTALL_DIR;
ar rc libmicroros.a $(ls *.o *.obj 2> /dev/null); cp libmicroros.a $INSTALL_DIR; ranlib $INSTALL_DIR/libmicroros.a; \
cp -R $GENERATION_DIR/micro_ros_src/install/include/* $INSTALL_DIR;

# Generate extra files
find $GENERATION_DIR/micro_ros_dev \( -name "*.srv" -o -name "*.msg" -o -name "*.action" \) | awk -F"/" '{print $(NF-2)"/"$NF}' > $INSTALL_DIR/available_ros2_types

echo "" > $INSTALL_DIR/built_packages
for f in $(find $GENERATION_DIR/micro_ros_dev -name .git -type d); do pushd $f > /dev/null; echo $(git config --get remote.origin.url) $(git rev-parse HEAD) >> $INSTALL_DIR/built_packages; popd > /dev/null; done;

# Delete build sources
rm -rf $GENERATION_DIR;


