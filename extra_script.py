Import("env")
from glob import glob
import os
import microros_builder.library_builder as library_builder

# print(env)
# print(env.Dump())

# Install dependencies
# TODO(pgarrido): Check if they are already installed
env.Execute(
    env.VerboseAction(
        '$PYTHONEXE -m pip install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources setuptools',
        "Installing micro-ROS build system dependencies",
    )
)

project_options = env.GetProjectConfig().items(env=env["PIOENV"], as_dict=True)

global_env = DefaultEnvironment()

# Do not include the transport folder yet
env['SRC_FILTER'] += '-<build/include/*>'

# Retrieve the required transport
if 'microros_version' in project_options:
    microros_version = project_options['microros_version']
else:
    microros_version = 'galactic'

# Retrieve the required transport
if 'microros_transport' in project_options:
    microros_transport = project_options['microros_transport']
else:
    microros_transport = 'serial'

board = env['BOARD']
framework = env['PIOFRAMEWORK']

print("Configuring {} with transport {}".format(board, microros_transport))

main_path = os.path.realpath(".")
library_path = main_path + "/build"

cmake_toolchain = library_builder.CMakeToolchain(
    main_path + "/platformio_toolchain.cmake",
    env['CC'],
    env['CXX'],
    "{} {} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='".format(' '.join(env['CFLAGS']), ' '.join(env['CCFLAGS'])),
    "{} {} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='".format(' '.join(env['CXXFLAGS']), ' '.join(env['CCFLAGS']))
)

builder = library_builder.Build(library_path)
builder.run('/home/pgarrido/dev/micro-ROS/micro_ros_platformio/meta_files/colcon.meta', cmake_toolchain.path)

# Add microros lib
if (board == "portenta_h7_m7" or board == "nanorp2040connect"):
    # Linker fix for galactic
    global_env["_LIBFLAGS"] =  ('-Wl,--start-group -Wl,--whole-archive '
                '${_stripixes(LIBLINKPREFIX, LIBS, LIBLINKSUFFIX, LIBPREFIXES, '
                'LIBSUFFIXES, __env__)} -Wl,--no-whole-archive -lstdc++ '
                '-lsupc++ -lm -lc -lgcc -lnosys -l{} -Wl,--end-group'.format(builder.library_name))
else:
    global_env['LIBS'].append(builder.library_name)

global_env['LIBPATH'].append(builder.library_path)

global_env['_CPPDEFFLAGS'] += ' -DCLOCK_MONOTONIC=0 '
# global_env['_CPPDEFFLAGS'] += ' -DCLOCK_MONOTONIC=0 -D__attribute__\(x\)=\'\' '
global_env['_CPPDEFFLAGS'] += ' -I{}/build/libmicroros/include '.format(main_path)
if 'arduino' in framework:
    pass
    global_env['_CPPDEFFLAGS'] += ' -I{}/arduino'.format(main_path)





























# # Use this to select micro-ROS branch

# # main_path = os.path.realpath("../../")
# # library_path = main_path + "/include"
# # meta_folder = main_path + "/extras/meta_files/"

# # boards_toolchain = {
# #         "portenta_h7_m7" : "colcon.meta",
# #         "nanorp2040connect" : "colcon_verylowmem.meta",
# #         "teensy41" : "colcon.meta",
# #         "teensy40" : "colcon.meta",
# #         "teensy36" : "colcon_lowmem.meta",
# #         "teensy35" : "colcon_lowmem.meta",
# #         "teensy32" : "colcon_lowmem.meta",
# #         "teensy31" : "colcon_lowmem.meta",
# #         "esp32dev" : "colcon.meta",
# #         "olimex_e407" :  "colcon.meta",
# #         "due" : "colcon_verylowmem.meta",
# #         "zero" : "colcon_verylowmem.meta"
# #     }

# # cmake_toolchain = """
# #     include(CMakeForceCompiler)
# #     set(CMAKE_SYSTEM_NAME Generic)

# #     set(CMAKE_CROSSCOMPILING 1)
# #     set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# #     SET (CMAKE_C_COMPILER_WORKS 1)
# #     SET (CMAKE_CXX_COMPILER_WORKS 1)

# #     set(CMAKE_C_COMPILER {C_COMPILER})
# #     set(CMAKE_CXX_COMPILER {CXX_COMPILER})

# #     set(CMAKE_C_FLAGS_INIT "{C_FLAGS}" CACHE STRING "" FORCE)
# #     set(CMAKE_CXX_FLAGS_INIT "{CXX_FLAGS}" CACHE STRING "" FORCE)

# #     set(__BIG_ENDIAN__ 0)"""

# # try:
# #     global_env = DefaultEnvironment()

# #     # Check used board
# #     board = global_env['BOARD']

# #     # Add microros lib
# #     if (board == "portenta_h7_m7" or board == "nanorp2040connect"):
# #         # Linker fix for galactic)
# #         global_env["_LIBFLAGS"] =  ('-Wl,--start-group -Wl,--whole-archive '
# #                     '${_stripixes(LIBLINKPREFIX, LIBS, LIBLINKSUFFIX, LIBPREFIXES, '
# #                     'LIBSUFFIXES, __env__)} -Wl,--no-whole-archive -lstdc++ '
# #                     '-lsupc++ -lm -lc -lgcc -lnosys -lmicroros -Wl,--end-group')
# #     else:
# #         global_env['LIBS'].append("microros")

# #     # Add microros lib path
# #     global_env['LIBPATH'].append(library_path)
# # except (KeyError):
# #     print("Board {} not supported, available boards: {}".format(board, boards_toolchain.keys()))
# #     sys.exit(1)

# # # Check if library is already built
# # if os.path.isfile(library_path + '/libmicroros.a'):
# #     print("micro-ROS library already built")
# # else:
# #     # Install build dependencies on platformIO environment
# #     env.Execute(
# #             env.VerboseAction(
# #                 '$PYTHONEXE -m pip install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources setuptools',
# #                 "Installing micro-ROS build system dependencies",
# #             )
# #         )

# #     C_COMPILER = global_env['CC']
# #     CXX_COMPILER = global_env['CXX']
# #     FLAGS = ' '.join(global_env['CCFLAGS'])
# #     C_FLAGS = "{CFLAGS} {FLAGS} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='".format(CFLAGS=' '.join(global_env['CFLAGS']), FLAGS=FLAGS)
# #     CXX_FLAGS = "{CXXFLAGS} {FLAGS} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='".format(CXXFLAGS=' '.join(global_env['CXXFLAGS']), FLAGS=FLAGS)

# #     print("Using compiler: {}, with:".format(C_COMPILER))
# #     print("-- C flags: {}".format(C_FLAGS))
# #     print("-- CXX flags: {}".format(CXX_FLAGS))

# #     # Set toolchain options
# #     cmake_toolchain = cmake_toolchain.format(C_COMPILER=C_COMPILER, CXX_COMPILER=CXX_COMPILER, C_FLAGS=C_FLAGS, CXX_FLAGS=CXX_FLAGS)

# #     with open("platformIO_toolchain.cmake", "w") as file:
# #         file.write(cmake_toolchain)
# #         toolchain_path = os.path.realpath(file.name)

# #     # Build microros lib for target
# #     meta_file_path = meta_folder + boards_toolchain[board]
# #     build_command = "bash libmicroros.sh {TOOLCHAIN} {META_FILE}".format(TOOLCHAIN=toolchain_path, META_FILE=meta_file_path)
# #     make_process = subprocess.Popen(build_command, shell=True, stderr=subprocess.STDOUT)

# #     if make_process.wait() != 0:
# #         print("micro-ROS build failed")
# #         sys.exit(1)


