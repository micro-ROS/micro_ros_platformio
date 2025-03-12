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
        if os.path.exists(self.path):
            command = f"cd {self.path} && git pull {self.url} {self.branch}"
            result = run_cmd(command)
            if 0 != result.returncode:
                print(f"{self.name} pull failed: \n{result.stderr.decode('utf-8')}")
                sys.exit(1)
            return

        command = "git clone -b {} {} {}".format(self.branch, self.url, self.path)
        if (self.name == "rmw-microxrcedds") :
                command = "git clone -b {} {} {} && cd {} && git reset --hard c31887f38c708f085d4a2117e47055e0176acc1e".format(self.branch, self.url, self.path,self.path)
                print(command)
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
         'humble': [
            Repository("ament_cmake", "https://github.com/ament/ament_cmake", "humble"),
            Repository("ament_lint", "https://github.com/ament/ament_lint", "humble"),
            Repository("ament_package", "https://github.com/ament/ament_package", "humble"),
            Repository("googletest", "https://github.com/ament/googletest", "humble"),
            Repository("ament_cmake_ros", "https://github.com/ros2/ament_cmake_ros", "humble"),
            Repository("ament_index", "https://github.com/ament/ament_index", "humble")
        ],
        'iron': [
            Repository("ament_cmake", "https://github.com/ament/ament_cmake", "iron"),
            Repository("ament_lint", "https://github.com/ament/ament_lint", "iron"),
            Repository("ament_package", "https://github.com/ament/ament_package", "iron"),
            Repository("googletest", "https://github.com/ament/googletest", "iron"),
            Repository("ament_cmake_ros", "https://github.com/ros2/ament_cmake_ros", "iron"),
            Repository("ament_index", "https://github.com/ament/ament_index", "iron")
        ],
        'jazzy': [
            Repository("ament_cmake", "https://github.com/ament/ament_cmake", "jazzy"),
            Repository("ament_lint", "https://github.com/ament/ament_lint", "jazzy"),
            Repository("ament_package", "https://github.com/ament/ament_package", "jazzy"),
            Repository("googletest", "https://github.com/ament/googletest", "jazzy"),
            Repository("ament_cmake_ros", "https://github.com/ros2/ament_cmake_ros", "jazzy"),
            Repository("ament_index", "https://github.com/ament/ament_index", "jazzy")
        ],
        'rolling': [
            Repository("ament_cmake", "https://github.com/ament/ament_cmake", "rolling"),
            Repository("ament_lint", "https://github.com/ament/ament_lint", "rolling"),
            Repository("ament_package", "https://github.com/ament/ament_package", "rolling"),
            Repository("googletest", "https://github.com/ament/googletest", "rolling"),
            Repository("ament_cmake_ros", "https://github.com/ros2/ament_cmake_ros", "rolling"),
            Repository("ament_index", "https://github.com/ament/ament_index", "rolling")
        ]
    }

    mcu_environments = {
        'humble': [
            Repository("micro-CDR", "https://github.com/eProsima/micro-CDR", "humble", "ros2"),
            Repository("Micro-XRCE-DDS-Client", "https://github.com/eProsima/Micro-XRCE-DDS-Client", "humble", "ros2"),
            Repository("rcl", "https://github.com/micro-ROS/rcl", "humble"),
            Repository("rclc", "https://github.com/ros2/rclc", "humble"),
            Repository("micro_ros_utilities", "https://github.com/micro-ROS/micro_ros_utilities", "humble"),
            Repository("rcutils", "https://github.com/micro-ROS/rcutils", "humble"),
            Repository("micro_ros_msgs", "https://github.com/micro-ROS/micro_ros_msgs", "humble"),
            Repository("rmw-microxrcedds", "https://github.com/micro-ROS/rmw-microxrcedds", "humble"),
            Repository("rosidl_typesupport", "https://github.com/micro-ROS/rosidl_typesupport", "humble"),
            Repository("rosidl_typesupport_microxrcedds", "https://github.com/micro-ROS/rosidl_typesupport_microxrcedds", "humble"),
            Repository("rosidl", "https://github.com/ros2/rosidl", "humble"),
            Repository("rmw", "https://github.com/ros2/rmw", "humble"),
            Repository("rcl_interfaces", "https://github.com/ros2/rcl_interfaces", "humble"),
            Repository("rosidl_defaults", "https://github.com/ros2/rosidl_defaults", "humble"),
            Repository("unique_identifier_msgs", "https://github.com/ros2/unique_identifier_msgs", "humble"),
            Repository("common_interfaces", "https://github.com/ros2/common_interfaces", "humble"),
            Repository("test_interface_files", "https://github.com/ros2/test_interface_files", "humble"),
            Repository("rmw_implementation", "https://github.com/ros2/rmw_implementation", "humble"),
            Repository("rcl_logging", "https://github.com/ros2/rcl_logging", "humble"),
            Repository("ros2_tracing", "https://github.com/ros2/ros2_tracing", "humble"),
        ],
        'iron': [
            Repository("micro-CDR", "https://github.com/eProsima/micro-CDR", "iron", "ros2"),
            Repository("Micro-XRCE-DDS-Client", "https://github.com/eProsima/Micro-XRCE-DDS-Client", "iron", "ros2"),
            Repository("rcl", "https://github.com/micro-ROS/rcl", "iron"),
            Repository("rclc", "https://github.com/ros2/rclc", "iron"),
            Repository("micro_ros_utilities", "https://github.com/micro-ROS/micro_ros_utilities", "iron"),
            Repository("rcutils", "https://github.com/micro-ROS/rcutils", "iron"),
            Repository("micro_ros_msgs", "https://github.com/micro-ROS/micro_ros_msgs", "iron"),
            Repository("rmw-microxrcedds", "https://github.com/micro-ROS/rmw-microxrcedds", "iron"),
            Repository("rosidl_typesupport", "https://github.com/micro-ROS/rosidl_typesupport", "iron"),
            Repository("rosidl_typesupport_microxrcedds", "https://github.com/micro-ROS/rosidl_typesupport_microxrcedds", "iron"),
            Repository("rosidl", "https://github.com/ros2/rosidl", "iron"),
            Repository("rosidl_dynamic_typesupport", "https://github.com/ros2/rosidl_dynamic_typesupport", "iron"),
            Repository("rosidl_core", "https://github.com/ros2/rosidl_core", "iron"),
            Repository("rmw", "https://github.com/ros2/rmw", "iron"),
            Repository("rcl_interfaces", "https://github.com/ros2/rcl_interfaces", "iron"),
            Repository("rosidl_defaults", "https://github.com/ros2/rosidl_defaults", "iron"),
            Repository("unique_identifier_msgs", "https://github.com/ros2/unique_identifier_msgs", "iron"),
            Repository("common_interfaces", "https://github.com/ros2/common_interfaces", "iron"),
            Repository("test_interface_files", "https://github.com/ros2/test_interface_files", "iron"),
            Repository("rmw_implementation", "https://github.com/ros2/rmw_implementation", "iron"),
            Repository("rcl_logging", "https://github.com/ros2/rcl_logging", "iron"),
            Repository("ros2_tracing", "https://github.com/ros2/ros2_tracing", "iron"),
        ],
        'jazzy': [
            Repository("micro-CDR", "https://github.com/eProsima/micro-CDR", "jazzy", "ros2"),
            Repository("Micro-XRCE-DDS-Client", "https://github.com/eProsima/Micro-XRCE-DDS-Client", "jazzy", "ros2"),
            Repository("rcl", "https://github.com/micro-ROS/rcl", "jazzy"),
            Repository("rclc", "https://github.com/ros2/rclc", "jazzy"),
            Repository("micro_ros_utilities", "https://github.com/micro-ROS/micro_ros_utilities", "jazzy"),
            Repository("rcutils", "https://github.com/micro-ROS/rcutils", "jazzy"),
            Repository("micro_ros_msgs", "https://github.com/micro-ROS/micro_ros_msgs", "jazzy"),
            Repository("rmw-microxrcedds", "https://github.com/micro-ROS/rmw-microxrcedds", "jazzy"),
            Repository("rosidl_typesupport", "https://github.com/micro-ROS/rosidl_typesupport", "jazzy"),
            Repository("rosidl_typesupport_microxrcedds", "https://github.com/micro-ROS/rosidl_typesupport_microxrcedds", "jazzy"),
            Repository("rosidl", "https://github.com/ros2/rosidl", "jazzy"),
            Repository("rosidl_dynamic_typesupport", "https://github.com/ros2/rosidl_dynamic_typesupport", "jazzy"),
            Repository("rosidl_core", "https://github.com/ros2/rosidl_core", "jazzy"),
            Repository("rmw", "https://github.com/ros2/rmw", "jazzy"),
            Repository("rcl_interfaces", "https://github.com/ros2/rcl_interfaces", "jazzy"),
            Repository("rosidl_defaults", "https://github.com/ros2/rosidl_defaults", "jazzy"),
            Repository("unique_identifier_msgs", "https://github.com/ros2/unique_identifier_msgs", "jazzy"),
            Repository("common_interfaces", "https://github.com/ros2/common_interfaces", "jazzy"),
            Repository("test_interface_files", "https://github.com/ros2/test_interface_files", "jazzy"),
            Repository("rmw_implementation", "https://github.com/ros2/rmw_implementation", "jazzy"),
            Repository("rcl_logging", "https://github.com/ros2/rcl_logging", "jazzy"),
            Repository("ros2_tracing", "https://github.com/ros2/ros2_tracing", "jazzy"),
        ],
        'rolling': [
            Repository("micro-CDR", "https://github.com/eProsima/micro-CDR", "rolling", "ros2"),
            Repository("Micro-XRCE-DDS-Client", "https://github.com/eProsima/Micro-XRCE-DDS-Client", "rolling", "ros2"),
            Repository("rcl", "https://github.com/micro-ROS/rcl", "rolling"),
            Repository("rclc", "https://github.com/ros2/rclc", "rolling"),
            Repository("micro_ros_utilities", "https://github.com/micro-ROS/micro_ros_utilities", "rolling"),
            Repository("rcutils", "https://github.com/micro-ROS/rcutils", "rolling"),
            Repository("micro_ros_msgs", "https://github.com/micro-ROS/micro_ros_msgs", "rolling"),
            Repository("rmw-microxrcedds", "https://github.com/micro-ROS/rmw-microxrcedds", "rolling"),
            Repository("rosidl_typesupport", "https://github.com/micro-ROS/rosidl_typesupport", "rolling"),
            Repository("rosidl_typesupport_microxrcedds", "https://github.com/micro-ROS/rosidl_typesupport_microxrcedds", "rolling"),
            Repository("rosidl", "https://github.com/ros2/rosidl", "rolling"),
            Repository("rosidl_dynamic_typesupport", "https://github.com/ros2/rosidl_dynamic_typesupport", "rolling"),
            Repository("rosidl_core", "https://github.com/ros2/rosidl_core", "rolling"),
            Repository("rmw", "https://github.com/ros2/rmw", "rolling"),
            Repository("rcl_interfaces", "https://github.com/ros2/rcl_interfaces", "rolling"),
            Repository("rosidl_defaults", "https://github.com/ros2/rosidl_defaults", "rolling"),
            Repository("unique_identifier_msgs", "https://github.com/ros2/unique_identifier_msgs", "rolling"),
            Repository("common_interfaces", "https://github.com/ros2/common_interfaces", "rolling"),
            Repository("test_interface_files", "https://github.com/ros2/test_interface_files", "rolling"),
            Repository("rmw_implementation", "https://github.com/ros2/rmw_implementation", "rolling"),
            Repository("rcl_logging", "https://github.com/ros2/rcl_logging", "rolling"),
            Repository("ros2_tracing", "https://github.com/ros2/ros2_tracing", "rolling"),
        ]
    }

    ignore_packages = {
        'humble': ['rcl_logging_log4cxx', 'rcl_logging_spdlog', 'rcl_yaml_param_parser', 'rclc_examples'],
        'iron': ['test_tracetools', 'rcl_logging_spdlog', 'rcl_yaml_param_parser', 'rclc_examples'],
        'jazzy': ['test_tracetools', 'rcl_logging_spdlog', 'rcl_yaml_param_parser', 'rclc_examples','lttngpy'],
        'rolling': ['test_tracetools', 'rcl_logging_spdlog', 'rcl_yaml_param_parser', 'rclc_examples','lttngpy']
    }
