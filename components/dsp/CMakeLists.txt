# Pure signal processing, no ESP-IDF dependencies (host-testable).
idf_component_register(
    SRCS "src/madgwick.cpp" "src/step_counter.cpp" "src/fall_detector.cpp"
    INCLUDE_DIRS "include"
    )