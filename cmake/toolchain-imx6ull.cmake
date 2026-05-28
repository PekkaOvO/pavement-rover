set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# abseil auto-detects x86 AES/SSE flags which ARM compiler doesn't support
set(ABSL_RANDOM_RANDEN_COPTS "" CACHE STRING "abseil random copts (empty for ARM)")
