cmake_minimum_required(VERSION 3.16)
project(wearable_imu_host_tests C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror -Wconversion)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
                    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)

include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG v2.6.0)
FetchContent_MakeAvailable(unity)

set(ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)

add_executable(test_bus_mock test_bus_mock.cpp)
target_include_directories(test_bus_mock PRIVATE
    ${ROOT}/components/hal_port/include
    ${ROOT}/test/mocks)
target_link_libraries(test_bus_mock PRIVATE unity)

enable_testing()
add_test(NAME bus_mock COMMAND test_bus_mock)