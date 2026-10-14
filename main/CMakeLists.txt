idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES hal_port lsm6dsox acquisition util
    )