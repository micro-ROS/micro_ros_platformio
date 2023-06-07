Import("projenv")
Import("env")

import os
import shutil

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
    "zero" : "colcon_verylowmem.meta",
    "pico": "colcon.meta"

}

project_options = env.GetProjectConfig().items(env=env["PIOENV"], as_dict=True)
main_path = os.path.realpath(".")
global_env = DefaultEnvironment()
board = env['BOARD']
framework = env['PIOFRAMEWORK'][0]
extra_packages_path = "{}/extra_packages".format(env['PROJECT_DIR'])

selected_board_meta = boards_metas[board] if board in boards_metas else "colcon.meta"

# Retrieve the required transport. Default iron
microros_distro = global_env.BoardConfig().get("microros_distro", "iron")

# Retrieve the required transport. Default serial
microros_transport = global_env.BoardConfig().get("microros_transport", "serial")

# Retrieve the user meta. Default none
microros_user_meta = "{}/{}".format(env['PROJECT_DIR'], global_env.BoardConfig().get("microros_user_meta", ""))

# Do not include build folder
env['SRC_FILTER'] += ' -<build/include/*>'

################################
#### Library custom targets ####
################################

def clean_microros_callback(*args, **kwargs):
    library_path = main_path + '/libmicroros'
    build_path = main_path + '/build'

    # Delete library and build folders
    shutil.rmtree(library_path, ignore_errors=True)
    shutil.rmtree(build_path, ignore_errors=True)

    print("micro-ROS library cleaned!")
    os._exit(0)

global_env.AddCustomTarget("clean_microros", None, clean_microros_callback, title="Clean Micro-ROS", description="Clean Micro-ROS build environment")

def build_microros(*args, **kwargs):
    ##############################
    #### Install dependencies ####
    ##############################

    pip_packages = [x.split("==")[0] for x in os.popen('{} -m pip freeze'.format(env['PYTHONEXE'])).read().split('\n')]
    required_packages = ["catkin-pkg", "lark-parser", "empy", "colcon-common-extensions", "importlib-resources", "pyyaml", "pytz", "markupsafe==2.0.1"]
    if all([x in pip_packages for x in required_packages]):
        print("All required Python pip packages are installed")

    for p in [x for x in required_packages if x not in pip_packages]:
        print('Installing {} with pip at PlatformIO environment'.format(p))
        env.Execute('$PYTHONEXE -m pip install {}'.format(p))

    import microros_utils.library_builder as library_builder

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

    python_env_path = env['PROJECT_CORE_DIR'] + "/penv/bin/activate"
    builder = library_builder.Build(library_folder=main_path, packages_folder=extra_packages_path, distro=microros_distro, python_env=python_env_path)
    builder.run('{}/metas/{}'.format(main_path, selected_board_meta), cmake_toolchain.path, microros_user_meta)

    #######################################################
    #### Add micro-ROS library/includes to environment ####
    #######################################################

    # Add library
    if (board == "portenta_h7_m7" or board == "nanorp2040connect" or board == "pico"):
        # Workaround for including the library in the linker group
        #   This solves a problem with duplicated symbols in Galactic
        global_env["_LIBFLAGS"] = "-Wl,--start-group " + global_env["_LIBFLAGS"] + " -l{} -Wl,--end-group".format(builder.library_name)
    else:
        global_env.Append(LIBS=[builder.library_name])

    # Add library path
    global_env.Append(LIBPATH=[builder.library_path])

def update_env():
    # Add required defines
    global_env.Append(CPPDEFINES=[("CLOCK_MONOTONIC", 1)])

    # Add micro-ROS include path
    global_env.Append(CPPPATH=[main_path + "/libmicroros/include"])

    # Add micro-ROS include path to library include path
    env.Append(CPPPATH=[main_path + "/libmicroros/include"])

    # Add platformio library general include path
    global_env.Append(CPPPATH=[
        main_path + "/platform_code",
        main_path + "/platform_code/{}/{}".format(framework, microros_transport)])

    # Add platformio library general to library include path
    env.Append(CPPPATH=[
        main_path + "/platform_code",
        main_path + "/platform_code/{}/{}".format(framework, microros_transport)])

    if (board == "teensy31" or board == "teensy35" or board == "teensy36"):
        projenv.Append(LINKFLAGS=["--specs=nosys.specs"])

    # Add micro-ROS defines to user application
    projenv.Append(CPPDEFINES=[('MICRO_ROS_TRANSPORT_{}_{}'.format(framework.upper(), microros_transport.upper()), 1)])
    projenv.Append(CPPDEFINES=[('MICRO_ROS_DISTRO_{} '.format(microros_distro.upper()), 1)])

    # Include path for framework
    global_env.Append(CPPPATH=[main_path + "/platform_code/{}".format(framework)])

    # Add clock implementation
    env['SRC_FILTER'] += ' +<platform_code/{}/clock_gettime.cpp>'.format(framework)

    # Add transport sources according to the framework and the transport
    env['SRC_FILTER'] += ' +<platform_code/{}/{}/micro_ros_transport.cpp>'.format(framework, microros_transport)


from SCons.Script import COMMAND_LINE_TARGETS

# Do not build library on clean_microros target or when IDE fetches C/C++ project metadata
if set(["clean_microros", "_idedata", "idedata"]).isdisjoint(set(COMMAND_LINE_TARGETS)):
    build_microros()

update_env()