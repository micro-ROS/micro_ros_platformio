import os, sys
import json
import xml.etree.ElementTree as xml_parser

from .utils import run_cmd

class Package:
    def __init__(self, name, path):
        self.name = name
        self.path = path
        self.ignored = False

    def ignore(self):
        self.ignored = True
        ignore_path = self.path + '/COLCON_IGNORE'
        with open(ignore_path, 'a'):
            os.utime(ignore_path, None)

class Repository:
    def __init__(self, name, url, distribution, branch=None):
        self.name = name
        self.url = url
        self.distribution = distribution
        self.branch = distribution if branch is None else branch
        self.path = None

    def clone(self, folder):
        self.path = folder + "/" + self.name
        # TODO(pablogs) ensure that git is installed
        command = "git clone -b {} {} {}".format(self.branch, self.url, self.path)
        result = run_cmd(command)

        if 0 != result.returncode:
            print("{} clone failed: \n{}".format(self.name, result.stderr.decode("utf-8")))
            sys.exit(1)

    def get_packages(self):
        packages = []
        if os.path.exists(self.path + '/package.xml'):
            packages.append(Package(self.name, self.path))
        else:
            for root, dirs, files in os.walk(self.path):
                path = root.split(os.sep)
                if 'package.xml' in files:
                    package_name = Repository.get_package_name_from_package_xml(os.path.join(root, 'package.xml'))
                    package_path = os.path.join(os.getcwd(), root)
                    packages.append(Package(package_name, package_path))
                elif 'colcon.pkg' in files:
                    package_name = Repository.get_package_name_from_colcon_pkg(os.path.join(root, 'colcon.pkg'))
                    package_path = os.path.join(os.getcwd(), root)
                    packages.append(Package(package_name, package_path))
        return packages

    def get_package_name_from_package_xml(xml_file):
        root_node = xml_parser.parse(xml_file).getroot()
        name_node = root_node.find('name')
        if name_node is not None:
            return name_node.text
        return None

    def get_package_name_from_colcon_pkg(colcon_pkg):
        with open(colcon_pkg, 'r') as f:
            content = json.load(f)
            if content['name']:
                return content['name']
            return None

class Sources:
    dev_environments = {
        'galactic': [
            Repository("ament_cmake", "https://github.com/ament/ament_cmake", "galactic"),
            Repository("ament_lint", "https://github.com/ament/ament_lint", "galactic"),
            Repository("ament_package", "https://github.com/ament/ament_package", "galactic"),
            Repository("googletest", "https://github.com/ament/googletest", "galactic"),
            Repository("ament_cmake_ros", "https://github.com/ros2/ament_cmake_ros", "galactic"),
            Repository("ament_index", "https://github.com/ament/ament_index", "galactic")
        ],
        'rolling': [
            Repository("ament_cmake", "https://github.com/ament/ament_cmake", "rolling", "mastera"),
            Repository("ament_lint", "https://github.com/ament/ament_lint", "rolling", "master"),
            Repository("ament_package", "https://github.com/ament/ament_package", "rolling", "master"),
            Repository("googletest", "https://github.com/ament/googletest", "rolling", "ros2"),
            Repository("ament_cmake_ros", "https://github.com/ros2/ament_cmake_ros", "rolling", "master"),
            Repository("ament_index", "https://github.com/ament/ament_index", "rolling", "master")
        ]
    }

    mcu_environments = {
        'galactic': [
            Repository("micro-CDR", "https://github.com/eProsima/micro-CDR", "galactic", "ros2"),
            Repository("Micro-XRCE-DDS-Client", "https://github.com/eProsima/Micro-XRCE-DDS-Client", "galactic", "ros2"),
            Repository("rcl", "https://github.com/micro-ROS/rcl", "galactic"),
            Repository("rclc", "https://github.com/ros2/rclc", "galactic"),
            Repository("micro_ros_utilities", "https://github.com/micro-ROS/micro_ros_utilities", "galactic"),
            Repository("rcutils", "https://github.com/micro-ROS/rcutils", "galactic"),
            Repository("micro_ros_msgs", "https://github.com/micro-ROS/micro_ros_msgs", "galactic"),
            Repository("rmw-microxrcedds", "https://github.com/micro-ROS/rmw-microxrcedds", "galactic"),
            Repository("rosidl_typesupport", "https://github.com/micro-ROS/rosidl_typesupport", "galactic"),
            Repository("rosidl_typesupport_microxrcedds", "https://github.com/micro-ROS/rosidl_typesupport_microxrcedds", "galactic"),
            Repository("rosidl", "https://github.com/ros2/rosidl", "galactic"),
            Repository("rmw", "https://github.com/ros2/rmw", "galactic"),
            Repository("rcl_interfaces", "https://github.com/ros2/rcl_interfaces", "galactic"),
            Repository("rosidl_defaults", "https://github.com/ros2/rosidl_defaults", "galactic"),
            Repository("unique_identifier_msgs", "https://github.com/ros2/unique_identifier_msgs", "galactic"),
            Repository("common_interfaces", "https://github.com/ros2/common_interfaces", "galactic"),
            Repository("test_interface_files", "https://github.com/ros2/test_interface_files", "galactic"),
            Repository("rmw_implementation", "https://github.com/ros2/rmw_implementation", "galactic"),
            Repository("rcl_logging", "https://github.com/ros2/rcl_logging", "galactic"),
            Repository("ros2_tracing", "https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing", "galactic"),
        ],
        'rolling': [
            Repository("micro-CDR", "https://github.com/eProsima/micro-CDR", "rolling", "ros2"),
            Repository("Micro-XRCE-DDS-Client", "https://github.com/eProsima/Micro-XRCE-DDS-Client", "rolling", "ros2"),
            Repository("rcl", "https://github.com/micro-ROS/rcl", "rolling", "master"),
            Repository("rclc", "https://github.com/ros2/rclc", "rolling", "master"),
            Repository("micro_ros_utilities", "https://github.com/micro-ROS/micro_ros_utilities", "rolling", "main"),
            Repository("rcutils", "https://github.com/micro-ROS/rcutils", "rolling", "master"),
            Repository("micro_ros_msgs", "https://github.com/micro-ROS/micro_ros_msgs", "rolling", "main"),
            Repository("rmw-microxrcedds", "https://github.com/micro-ROS/rmw-microxrcedds", "rolling", "main"),
            Repository("rosidl_typesupport", "https://github.com/micro-ROS/rosidl_typesupport", "rolling", "master"),
            Repository("rosidl_typesupport_microxrcedds", "https://github.com/micro-ROS/rosidl_typesupport_microxrcedds", "rolling", "main"),
            Repository("rosidl", "https://github.com/ros2/rosidl", "rolling", "master"),
            Repository("rmw", "https://github.com/ros2/rmw", "rolling", "master"),
            Repository("rcl_interfaces", "https://github.com/ros2/rcl_interfaces", "rolling", "master"),
            Repository("rosidl_defaults", "https://github.com/ros2/rosidl_defaults", "rolling", "master"),
            Repository("unique_identifier_msgs", "https://github.com/ros2/unique_identifier_msgs", "rolling", "master"),
            Repository("common_interfaces", "https://github.com/ros2/common_interfaces", "rolling", "master"),
            Repository("test_interface_files", "https://github.com/ros2/test_interface_files", "rolling", "master"),
            Repository("rmw_implementation", "https://github.com/ros2/rmw_implementation", "rolling", "master"),
            Repository("rcl_logging", "https://github.com/ros2/rcl_logging", "rolling", "master"),
            Repository("ros2_tracing", "https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing", "rolling", "master"),
        ]
    }

    ignore_packages = {
        'galactic': ['rcl_logging_log4cxx', 'rcl_logging_spdlog', 'rcl_yaml_param_parser', 'rclc_examples'],
        'rolling': ['rcl_logging_log4cxx', 'rcl_logging_spdlog', 'rcl_yaml_param_parser', 'rclc_examples']
    }