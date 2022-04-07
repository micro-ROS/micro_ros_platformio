Import("env")
import os, sys
import microros_utils.library_builder as library_builder

##############################
#### Install dependencies ####
##############################

pip_packages = [x.split("==")[0] for x in os.popen('{} -m pip freeze'.format(env['PYTHONEXE'])).read().split('\n')]
required_packages = ["catkin-pkg", "lark-parser", "empy", "colcon-common-extensions", "importlib-resources"]
if all([x in pip_packages for x in required_packages]):
    print("All required Python pip packages are installed")

for p in [x for x in required_packages if x not in pip_packages]:
    print('Installing {} with pip at PlatformIO environment'.format(p))
    env.Execute('$PYTHONEXE -m pip install {}'.format(p))

##########################
#### Global variables ####
##########################

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
board = env['BOARD']
framework = env['PIOFRAMEWORK'][0]
main_path = os.path.realpath(".")

selected_board_meta = boards_metas[board] if board in boards_metas else "colcon.meta"

# Retrieve the required transport
microros_version = project_options['microros_version'] if 'microros_version' in project_options else 'galactic'

# Retrieve the required transport
microros_transport = project_options['microros_transport'] if 'microros_transport' in project_options else 'serial'

# Retrieve the user meta
microros_user_meta = project_options['microros_user_meta'] if 'microros_user_meta' in project_options else ''

# Do not include build folder
env['SRC_FILTER'] += ' -<build/include/*>'


#################################
#### Build micro-ROS library ####
#################################

print("Configuring {} with transport {}".format(board, microros_transport))

cmake_toolchain = library_builder.CMakeToolchain(
    main_path + "/platformio_toolchain.cmake",
    env['CC'],
    env['CXX'],
    env['AR'],
    "{} {} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='".format(' '.join(env['CFLAGS']), ' '.join(env['CCFLAGS'])),
    "{} {} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='".format(' '.join(env['CXXFLAGS']), ' '.join(env['CCFLAGS']))
)

builder = library_builder.Build(main_path)
builder.run('{}/metas/{}'.format(main_path, selected_board_meta), cmake_toolchain.path, microros_user_meta)

#######################################################
#### Add micro-ROS library/includes to environment ####
#######################################################

# Add library
if (board == "portenta_h7_m7" or board == "nanorp2040connect"):
    # Workaround for including the library in the linker group
    #   This solves a problem with duplicated symbols in Galactic
    global_env["_LIBFLAGS"] = "-Wl,--start-group " + global_env["_LIBFLAGS"] + " -l{} -Wl,--end-group".format(builder.library_name)
else:
    global_env['LIBS'].append(builder.library_name)

# Add library path
global_env['LIBPATH'].append(builder.library_path)

# Add required defines
global_env['_CPPDEFFLAGS'] += ' -DCLOCK_MONOTONIC=0 '

# Add micro-ROS include path
global_env['_CPPDEFFLAGS'] += ' -I{}/build/libmicroros/include '.format(main_path)

# Add micro-ROS include path to library include path
env['_CPPDEFFLAGS'] += ' -I{}/build/libmicroros/include '.format(main_path)

# Add platformio library general include path
global_env['_CPPDEFFLAGS'] += ' -I{}/platform_code '.format(main_path)
global_env['_CPPDEFFLAGS'] += ' -I{}/platform_code/{}/{} '.format(main_path, framework, microros_transport)

# Add platformio library general to library include path
env['_CPPDEFFLAGS'] += ' -I{}/platform_code '.format(main_path)
env['_CPPDEFFLAGS'] += ' -I{}/platform_code/{}/{} '.format(main_path, framework, microros_transport)

# Add micro-ROS defines to user application
global_env['_CPPDEFFLAGS'] += ' -DMICRO_ROS_TRANSPORT_{} '.format(microros_transport.upper())
global_env['_CPPDEFFLAGS'] += ' -DMICRO_ROS_DISTRO_ '.format(microros_version.upper())

# Add platformio library for Arduino framework
if 'arduino' == framework:
    # Include path for Arduino framework
    global_env['_CPPDEFFLAGS'] += ' -I{}/arduino'.format(main_path)
    # Clock implementation for Arduino framework
    env['SRC_FILTER'] += ' +<platform_code/arduino/clock_gettime.cpp>'

# Add transport sources according to the framework and the transport
env['SRC_FILTER'] += ' +<platform_code/{}/{}/micro_ros_transport.cpp>'.format(framework, microros_transport)
