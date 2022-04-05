Import('env')
import subprocess
import os, sys

main_path = os.path.realpath("../../")
library_path = main_path + "/include"
meta_folder = main_path + "/extras/meta_files/"

boards_toolchain = {
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

try:
    global_env = DefaultEnvironment()

    # Check used board
    board = global_env['BOARD']

    # Add microros lib
    if (board == "portenta_h7_m7" or board == "nanorp2040connect"):
        # Linker fix for galactic)
        global_env["_LIBFLAGS"] =  ('-Wl,--start-group -Wl,--whole-archive '
                    '${_stripixes(LIBLINKPREFIX, LIBS, LIBLINKSUFFIX, LIBPREFIXES, '
                    'LIBSUFFIXES, __env__)} -Wl,--no-whole-archive -lstdc++ '
                    '-lsupc++ -lm -lc -lgcc -lnosys -lmicroros -Wl,--end-group')
    else:
        global_env['LIBS'].append("microros")

    # Add microros lib path
    global_env['LIBPATH'].append(library_path)
except (KeyError):
    print("Board {} not supported, available boards: {}".format(board, boards_toolchain.keys()))
    sys.exit(1)

# Check if library is already built
if os.path.isfile(library_path + '/libmicroros.a'):
    print("micro-ROS library already built")
else:
    # Install build dependencies on platformIO environment
    env.Execute(
            env.VerboseAction(
                'pip3 install catkin_pkg lark-parser empy colcon-common-extensions markupsafe==2.0.1 vcstool pytz',
                "Installing micro-ROS build system dependencies",
            )
        )

    C_COMPILER = global_env['CC']
    CXX_COMPILER = global_env['CXX']
    FLAGS = ' '.join(global_env['CCFLAGS'])
    C_FLAGS = "{CFLAGS} {FLAGS} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='".format(CFLAGS=' '.join(global_env['CFLAGS']), FLAGS=FLAGS)
    CXX_FLAGS = "{CXXFLAGS} {FLAGS} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='".format(CXXFLAGS=' '.join(global_env['CXXFLAGS']), FLAGS=FLAGS)

    print("Using compiler: {}, with:".format(C_COMPILER))
    print("-- C flags: {}".format(C_FLAGS))
    print("-- CXX flags: {}".format(CXX_FLAGS))

    # Set toolchain options
    cmake_toolchain = cmake_toolchain.format(C_COMPILER=C_COMPILER, CXX_COMPILER=CXX_COMPILER, C_FLAGS=C_FLAGS, CXX_FLAGS=CXX_FLAGS)

    with open("platformIO_toolchain.cmake", "w") as file:
        file.write(cmake_toolchain)
        toolchain_path = os.path.realpath(file.name)

    # Build microros lib for target
    meta_file_path = meta_folder + boards_toolchain[board]
    build_command = "bash libmicroros.sh {TOOLCHAIN} {META_FILE}".format(TOOLCHAIN=toolchain_path, META_FILE=meta_file_path)
    make_process = subprocess.Popen(build_command, shell=True, stderr=subprocess.STDOUT)

    if make_process.wait() != 0:
        print("micro-ROS build failed")
        sys.exit(1)


