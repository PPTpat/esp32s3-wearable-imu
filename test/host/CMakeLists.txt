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

find_package(Threads REQUIRED)

set(ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(HAL_INC ${ROOT}/components/hal_port/include)
set(IMU_INC ${ROOT}/components/lsm6dsox/include)
set(ACQ_INC ${ROOT}/components/acquisition/include)
set(UTIL_INC ${ROOT}/components/util/include)
set(PROTO_INC ${ROOT}/components/protocol/include)
set(DSP_INC ${ROOT}/components/dsp/include)
set(MOCK_INC ${ROOT}/test/mocks)

# --- HAL bus mock ---
add_executable(test_bus_mock test_bus_mock.cpp)
target_include_directories(test_bus_mock PRIVATE ${HAL_INC} ${MOCK_INC})
target_link_libraries(test_bus_mock PRIVATE unity)

# --- LSM6DSOX driver ---
add_executable(test_lsm6dsox
    test_lsm6dsox.cpp
    ${ROOT}/components/lsm6dsox/src/lsm6dsox.cpp)
target_include_directories(test_lsm6dsox PRIVATE ${HAL_INC} ${IMU_INC} ${MOCK_INC})
target_link_libraries(test_lsm6dsox PRIVATE unity)

# --- SPSC ring buffer ---
add_executable(test_spsc_ring test_spsc_ring.cpp)
target_include_directories(test_spsc_ring PRIVATE ${UTIL_INC})
target_link_libraries(test_spsc_ring PRIVATE unity Threads::Threads)

# --- Sample assembler ---
add_executable(test_sample_assembler
    test_sample_assembler.cpp
    ${ROOT}/components/acquisition/src/sample_assembler.cpp)
target_include_directories(test_sample_assembler PRIVATE ${HAL_INC} ${IMU_INC} ${ACQ_INC})
target_link_libraries(test_sample_assembler PRIVATE unity)

# --- Protocol: CRC16 ---
add_executable(test_crc16
    test_crc16.cpp
    ${ROOT}/components/protocol/src/crc16.cpp)
target_include_directories(test_crc16 PRIVATE ${PROTO_INC})
target_link_libraries(test_crc16 PRIVATE unity)

# --- Protocol: frames, info, commands ---
add_executable(test_frame
    test_frame.cpp
    ${ROOT}/components/protocol/src/frame.cpp
    ${ROOT}/components/protocol/src/crc16.cpp)
target_include_directories(test_frame PRIVATE ${PROTO_INC} ${ACQ_INC})
target_link_libraries(test_frame PRIVATE unity)

# --- DSP library (shared by the DSP tests and the replay tool) ---
add_library(dsp_host STATIC
    ${ROOT}/components/dsp/src/madgwick.cpp
    ${ROOT}/components/dsp/src/step_counter.cpp
    ${ROOT}/components/dsp/src/fall_detector.cpp)
target_include_directories(dsp_host PUBLIC ${DSP_INC})

add_executable(test_madgwick test_madgwick.cpp)
target_link_libraries(test_madgwick PRIVATE dsp_host unity)

add_executable(test_step_counter test_step_counter.cpp)
target_link_libraries(test_step_counter PRIVATE dsp_host unity)

add_executable(test_fall_detector test_fall_detector.cpp)
target_link_libraries(test_fall_detector PRIVATE dsp_host unity)

# --- Replay tool: runs a recorded CSV through the DSP (not a test) ---
add_executable(replay_csv replay_csv.cpp)
target_link_libraries(replay_csv PRIVATE dsp_host)

enable_testing()
add_test(NAME bus_mock COMMAND test_bus_mock)
add_test(NAME lsm6dsox COMMAND test_lsm6dsox)
add_test(NAME spsc_ring COMMAND test_spsc_ring)
add_test(NAME sample_assembler COMMAND test_sample_assembler)
add_test(NAME crc16 COMMAND test_crc16)
add_test(NAME frame COMMAND test_frame)
add_test(NAME madgwick COMMAND test_madgwick)
add_test(NAME step_counter COMMAND test_step_counter)
add_test(NAME fall_detector COMMAND test_fall_detector)