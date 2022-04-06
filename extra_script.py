Import("env")
import os, sys
import microros_utils.library_builder as library_builder

# Install dependencies
# TODO(pgarrido): Check if they are already installed
env.Execute(
    env.VerboseAction(
        '$PYTHONEXE -m pip install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources setuptools',
        "Installing micro-ROS build system dependencies",
    )
)

# Board metas
boards_metas = {
        "portenta_h7_m7" : "colcon.meta",
        "nanorp2040connect" : "colcon_verylowmem.meta",
        "teensy41" : "colcon.meta",
        "teensy40" : "colcon.meta",
        "teensy36" : "colcon_lowmem.meta",
        "teensy35" : "colcon_lowmem.meta",
        "teensy32" : "colcon_lowmem.meta",
        "teensy31" : "colcon_lowmem.meta",
        "esp32dev" : "colcon.meta",
        "olimex_e407" :  "colcon.meta",
        "due" : "colcon_verylowmem.meta",
        "zero" : "colcon_verylowmem.meta"
    }

project_options = env.GetProjectConfig().items(env=env["PIOENV"], as_dict=True)

global_env = DefaultEnvironment()

# Do not include the transport folder yet
env['SRC_FILTER'] += ' -<build/include/*>'

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
framework = env['PIOFRAMEWORK'][0]

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
builder.run('{}/metas/{}'.format(main_path, boards_metas[board]), cmake_toolchain.path)

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
global_env['_CPPDEFFLAGS'] += ' -I{}/src '.format(main_path)

if 'arduino' == framework:
    global_env['_CPPDEFFLAGS'] += ' -I{}/arduino'.format(main_path)
    env['SRC_FILTER'] += ' +<src/arduino/clock_gettime.cpp>'

env['SRC_FILTER'] += ' +<src/{}/{}/transport.cpp>'.format(framework,microros_transport)
