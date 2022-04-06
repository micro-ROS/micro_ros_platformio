import os
import json
import shutil
import xml.etree.ElementTree as xml_parser

class Package:
    def __init__(self, name, path):
        self.name = name
        self.path = path

    def ignore(self):
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
        os.system("git clone -b {} {} {} > /dev/null 2>&1".format(self.branch, self.url, self.path))

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

class CMakeToolchain:
    def __init__(self, path, cc, cxx, cflags, cxxflags):
        cmake_toolchain = """
            include(CMakeForceCompiler)
            set(CMAKE_SYSTEM_NAME Generic)

            set(CMAKE_CROSSCOMPILING 1)
            set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

            SET (CMAKE_C_COMPILER_WORKS 1)
            SET (CMAKE_CXX_COMPILER_WORKS 1)

            set(CMAKE_C_COMPILER {C_COMPILER})
            set(CMAKE_CXX_COMPILER {CXX_COMPILER})

            set(CMAKE_C_FLAGS_INIT "{C_FLAGS}" CACHE STRING "" FORCE)
            set(CMAKE_CXX_FLAGS_INIT "{CXX_FLAGS}" CACHE STRING "" FORCE)

            set(__BIG_ENDIAN__ 0)"""

        cmake_toolchain = cmake_toolchain.format(C_COMPILER=cc, CXX_COMPILER=cxx, C_FLAGS=cflags, CXX_FLAGS=cxxflags)

        with open(path, "w") as file:
            file.write(cmake_toolchain)

        self.path = os.path.realpath(file.name)

class Build:
    def __init__(self, build_folder):
        self.build_folder = build_folder
        self.dev_environment = [
            Repository("ament_cmake", "https://github.com/ament/ament_cmake", "galactic"),
            Repository("ament_lint", "https://github.com/ament/ament_lint", "galactic"),
            Repository("ament_package", "https://github.com/ament/ament_package", "galactic"),
            Repository("googletest", "https://github.com/ament/googletest", "galactic"),
            Repository("ament_cmake_ros", "https://github.com/ros2/ament_cmake_ros", "galactic"),
            Repository("ament_index", "https://github.com/ament/ament_index", "galactic")
        ]

        self.mcu_environment = [
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
        ]

        self.dev_packages = []
        self.mcu_packages = []

        self.dev_folder = self.build_folder + '/dev'
        self.dev_src_folder = self.dev_folder + '/src'
        self.mcu_folder = self.build_folder + '/mcu'
        self.mcu_src_folder = self.mcu_folder + '/src'

        self.library = None
        self.includes = None
        self.library_path = None
        self.library_name = None

    def run(self, meta, toolchain):
        self.download_dev_environment()
        self.build_dev_environment()
        self.download_mcu_environment()
        self.build_mcu_environment(meta, toolchain)
        self.package_mcu_library()

    def ignore_package(self, name):
        for p in self.mcu_packages:
            if p.name == name:
                p.ignore()

    def download_dev_environment(self):
        if os.path.exists(self.dev_src_folder):
            print("micro-ROS dev already downloaded")
            return

        os.makedirs(self.dev_src_folder, exist_ok=True)
        print("Downloading micro-ROS dev dependencies")
        for repo in self.dev_environment:
            repo.clone(self.dev_src_folder)
            print("\t - Downloaded {}".format(repo.name))
            self.dev_packages.extend(repo.get_packages())

    def build_dev_environment(self):
        if os.path.exists(self.dev_folder + '/build'):
            print("micro-ROS dev already built")
            return

        print("Building micro-ROS dev dependencies")
        os.system("cd {} && colcon build --cmake-args -DBUILD_TESTING=OFF > /dev/null 2>&1".format(self.dev_folder))

    def download_mcu_environment(self):
        if os.path.exists(self.mcu_src_folder):
            print("micro-ROS already downloaded")
            return

        os.makedirs(self.mcu_src_folder)
        print("Downloading micro-ROS library")
        for repo in self.mcu_environment:
            repo.clone(self.mcu_src_folder)
            self.mcu_packages.extend(repo.get_packages())
            for package in repo.get_packages():
                print('\t - Downloaded {}'.format(package.name))

    def build_mcu_environment(self, meta_file, toolchain_file):
        if os.path.exists(self.mcu_folder + '/build'):
            print("micro-ROS already built")
            return

        self.ignore_package('rcl_logging_log4cxx')
        self.ignore_package('rcl_logging_spdlog')
        self.ignore_package('rcl_yaml_param_parser')
        self.ignore_package('rclc_examples')

        print("Building micro-ROS library")

        colcon_command = 'colcon build --merge-install --packages-ignore-regex=.*_cpp --metas {} --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=OFF  -DTHIRDPARTY=ON  -DBUILD_SHARED_LIBS=OFF  -DBUILD_TESTING=OFF  -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE={}'.format(meta_file, toolchain_file)
        os.system("cd {} && . {}/install/setup.sh && {} > /dev/null 2>&1".format(self.mcu_folder, self.dev_folder, colcon_command))

    def package_mcu_library(self):
        if self.library_path is not None:
            return

        shutil.rmtree(self.build_folder + "/aux", ignore_errors=True)
        shutil.rmtree(self.build_folder + "/libmicroros", ignore_errors=True)
        os.makedirs(self.build_folder + "/aux", exist_ok=True)
        os.makedirs(self.build_folder + "/libmicroros", exist_ok=True)
        for root, dirs, files in os.walk(self.mcu_folder + "/install/lib"):
            for f in files:
                if f.endswith('.a'):
                    os.makedirs(self.build_folder + "/aux/naming", exist_ok=True)
                    os.chdir(self.build_folder + "/aux/naming")
                    os.system("ar x {}".format(root + "/" + f))
                    for obj in [x for x in os.listdir() if x.endswith('obj')]:
                        os.rename(obj, '../' + f.split('.')[0] + "__" + obj)
        os.chdir(self.build_folder + "/aux")
        os.system('ar rc libmicroros.a $(ls *.o *.obj 2> /dev/null); rm *.o *.obj 2> /dev/null; ranlib libmicroros.a')
        os.rename('libmicroros.a', '../libmicroros/libmicroros.a')
        shutil.rmtree(self.build_folder + "/aux", ignore_errors=True)
        shutil.copytree(self.build_folder + "/mcu/install/include", self.build_folder + "/libmicroros/include")
        self.library = self.build_folder + "/libmicroros/libmicroros.a"
        self.library_path = self.build_folder + "/libmicroros"
        self.includes = self.build_folder + "/libmicroros/include"
        self.library_name = "microros"

