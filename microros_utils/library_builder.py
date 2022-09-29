import os, sys
import yaml
import shutil

from .utils import run_cmd
from .repositories import Repository, Sources

class CMakeToolchain:
    def __init__(self, path, cc, cxx, ar, cflags, cxxflags):
        cmake_toolchain = """include(CMakeForceCompiler)
set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

SET (CMAKE_C_COMPILER_WORKS 1)
SET (CMAKE_CXX_COMPILER_WORKS 1)

set(CMAKE_C_COMPILER {C_COMPILER})
set(CMAKE_CXX_COMPILER {CXX_COMPILER})
set(CMAKE_AR {AR_COMPILER})

set(CMAKE_C_FLAGS_INIT "{C_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "{CXX_FLAGS}" CACHE STRING "" FORCE)

set(__BIG_ENDIAN__ 0)"""

        cmake_toolchain = cmake_toolchain.format(C_COMPILER=cc, CXX_COMPILER=cxx, AR_COMPILER=ar, C_FLAGS=cflags, CXX_FLAGS=cxxflags)

        with open(path, "w") as file:
            file.write(cmake_toolchain)

        self.path = os.path.realpath(file.name)

class Build:
    def __init__(self, library_folder, packages_folder, distro):
        self.library_folder = library_folder
        self.packages_folder = packages_folder
        self.build_folder = library_folder + "/build"
        self.distro = distro

        self.dev_packages = []
        self.mcu_packages = []

        self.dev_folder = self.build_folder + '/dev'
        self.dev_src_folder = self.dev_folder + '/src'
        self.mcu_folder = self.build_folder + '/mcu'
        self.mcu_src_folder = self.mcu_folder + '/src'

        self.library_path = library_folder + '/libmicroros'
        self.library = self.library_path + "/libmicroros.a"
        self.includes = self.library_path+ '/include'
        self.library_name = "microros"
        self.env = {}

    def run(self, meta, toolchain, user_meta = ""):
        if os.path.exists(self.library_path):
            print("micro-ROS already built")
            return

        self.check_env()
        self.download_dev_environment()
        self.build_dev_environment()
        self.download_mcu_environment()
        self.build_mcu_environment(meta, toolchain, user_meta)
        self.package_mcu_library()

        # Delete build folders
        shutil.rmtree(self.build_folder, ignore_errors=True)

    def ignore_package(self, name):
        for p in self.mcu_packages:
            if p.name == name:
                p.ignore()

    def check_env(self):
        ROS_DISTRO = os.getenv('ROS_DISTRO')

        if (ROS_DISTRO):
            PATH = os.getenv('PATH')
            os.environ['PATH'] = PATH.replace('/opt/ros/{}/bin:'.format(ROS_DISTRO), '')
            os.environ.pop('AMENT_PREFIX_PATH', None)

        RMW_IMPLEMENTATION = os.getenv('RMW_IMPLEMENTATION')

        if (RMW_IMPLEMENTATION):
            os.environ['RMW_IMPLEMENTATION'] = "rmw_microxrcedds"

        self.env = os.environ.copy()

    def download_dev_environment(self):
        shutil.rmtree(self.dev_src_folder, ignore_errors=True)
        os.makedirs(self.dev_src_folder)
        print("Downloading micro-ROS dev dependencies")
        for repo in Sources.dev_environments[self.distro]:
            repo.clone(self.dev_src_folder)
            print("\t - Downloaded {}".format(repo.name))
            self.dev_packages.extend(repo.get_packages())

    def build_dev_environment(self):
        print("Building micro-ROS dev dependencies")
        command = "cd {} && colcon build --cmake-args -DBUILD_TESTING=OFF".format(self.dev_folder)
        result = run_cmd(command, env=self.env)

        if 0 != result.returncode:
            print("Build dev micro-ROS environment failed: \n {}".format(result.stderr.decode("utf-8")))
            sys.exit(1)

    def download_mcu_environment(self):
        shutil.rmtree(self.mcu_src_folder, ignore_errors=True)
        os.makedirs(self.mcu_src_folder)
        print("Downloading micro-ROS library")
        for repo in Sources.mcu_environments[self.distro]:
            repo.clone(self.mcu_src_folder)
            self.mcu_packages.extend(repo.get_packages())
            for package in repo.get_packages():
                if package.name in Sources.ignore_packages[self.distro] or package.name.endswith("_cpp"):
                    package.ignore()

                print('\t - Downloaded {}{}'.format(package.name, " (ignored)" if package.ignored else ""))

        self.download_extra_packages()

    def download_extra_packages(self):
        if not os.path.exists(self.packages_folder):
            print("\t - Extra packages folder not found, skipping...")
            return

        print("Checking extra packages")

        # Load and clone repositories from extra_packages.repos file
        extra_repos = self.get_repositories_from_yaml("{}/extra_packages.repos".format(self.packages_folder))
        for repo_name in extra_repos:
            repo_values = extra_repos[repo_name]
            version = repo_values['version'] if 'version' in repo_values else None
            Repository(repo_name, repo_values['url'], self.distro, version).clone(self.mcu_src_folder)
            print("\t - Downloaded {}".format(repo_name))

        extra_folders = os.listdir(self.packages_folder)
        if 'extra_packages.repos' in extra_folders:
            extra_folders.remove('extra_packages.repos')

        for folder in extra_folders:
            print("\t - Adding {}".format(folder))

        shutil.copytree(self.packages_folder, self.mcu_src_folder, ignore=shutil.ignore_patterns('extra_packages.repos'), dirs_exist_ok=True)

    def get_repositories_from_yaml(self, yaml_file):
        repos = {}
        try:
            with open(yaml_file, 'r') as repos_file:
                root = yaml.safe_load(repos_file)
                repositories = root['repositories']

            if repositories:
                for path in repositories:
                    repo = {}
                    attributes = repositories[path]
                    try:
                        repo['type'] = attributes['type']
                        repo['url'] = attributes['url']
                        if 'version' in attributes:
                            repo['version'] = attributes['version']
                    except KeyError as e:
                        continue
                    repos[path] = repo
        except (yaml.YAMLError, KeyError, TypeError) as e:
            print("Error on {}: {}".format(yaml_file, e))
        finally:
            return repos

    def build_mcu_environment(self, meta_file, toolchain_file, user_meta = ""):
        print("Building micro-ROS library")

        common_meta_path = self.library_folder + '/metas/common.meta'
        colcon_command = 'colcon build --merge-install --packages-ignore-regex=.*_cpp --metas {} {} {} --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=OFF  -DTHIRDPARTY=ON  -DBUILD_SHARED_LIBS=OFF  -DBUILD_TESTING=OFF  -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE={}'.format(common_meta_path, meta_file, user_meta, toolchain_file)
        command = "cd {} && . {}/install/setup.sh && {}".format(self.mcu_folder, self.dev_folder, colcon_command)
        result = run_cmd(command, env=self.env)

        if 0 != result.returncode:
            print("Build mcu micro-ROS environment failed: \n{}".format(result.stderr.decode("utf-8")))
            sys.exit(1)

    def package_mcu_library(self):
        aux_folder = self.build_folder + "/aux"

        shutil.rmtree(aux_folder, ignore_errors=True)
        shutil.rmtree(self.library_path, ignore_errors=True)
        os.makedirs(aux_folder, exist_ok=True)
        os.makedirs(self.library_path, exist_ok=True)
        for root, dirs, files in os.walk(self.mcu_folder + "/install/lib"):
            for f in files:
                if f.endswith('.a'):
                    os.makedirs(aux_folder + "/naming", exist_ok=True)
                    os.chdir(aux_folder + "/naming")
                    os.system("ar x {}".format(root + "/" + f))
                    for obj in [x for x in os.listdir() if x.endswith('obj')]:
                        os.rename(obj, '../' + f.split('.')[0] + "__" + obj)

        os.chdir(aux_folder)
        command = "ar rc libmicroros.a $(ls *.o *.obj 2> /dev/null); rm *.o *.obj 2> /dev/null; ranlib libmicroros.a"
        result = run_cmd(command)

        if 0 != result.returncode:
            print("micro-ROS static library build failed: \n{}".format(result.stderr.decode("utf-8")))
            sys.exit(1)

        os.rename('libmicroros.a', self.library)

        # Copy includes
        shutil.copytree(self.build_folder + "/mcu/install/include", self.includes)

        # Fix include paths
        if self.distro not in ["galactic", "foxy"]:
            include_folders = os.listdir(self.includes)

            for folder in include_folders:
                folder_path = self.includes + "/{}".format(folder)
                repeated_path = folder_path + "/{}".format(folder)

                if os.path.exists(repeated_path):
                    shutil.copytree(repeated_path, folder_path, copy_function=shutil.move, dirs_exist_ok=True)
                    shutil.rmtree(repeated_path)
