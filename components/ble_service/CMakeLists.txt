idf_component_register(
    SRCS "src/ble_service.cpp"
    INCLUDE_DIRS "include"
    REQUIRES protocol acquisition util bt freertos log
    )