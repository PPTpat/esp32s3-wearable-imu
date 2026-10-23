idf_component_register(
    SRCS "src/sample_assembler.cpp" "src/acquisition.cpp"
    INCLUDE_DIRS "include"
    REQUIRES lsm6dsox util driver freertos log
    )