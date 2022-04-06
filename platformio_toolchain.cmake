
            include(CMakeForceCompiler)
            set(CMAKE_SYSTEM_NAME Generic)

            set(CMAKE_CROSSCOMPILING 1)
            set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

            SET (CMAKE_C_COMPILER_WORKS 1)
            SET (CMAKE_CXX_COMPILER_WORKS 1)

            set(CMAKE_C_COMPILER arm-none-eabi-gcc)
            set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

            set(CMAKE_C_FLAGS_INIT " -Wall -ffunction-sections -fdata-sections -mthumb -mcpu=cortex-m7 -nostdlib -fsingle-precision-constant -mfloat-abi=hard -mfpu=fpv5-d16 -O2 -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)
            set(CMAKE_CXX_FLAGS_INIT "-fno-exceptions -felide-constructors -fno-rtti -std=gnu++14 -Wno-error=narrowing -fpermissive -fno-threadsafe-statics -Wall -ffunction-sections -fdata-sections -mthumb -mcpu=cortex-m7 -nostdlib -fsingle-precision-constant -mfloat-abi=hard -mfpu=fpv5-d16 -O2 -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)

            set(__BIG_ENDIAN__ 0)