cmake_minimum_required(VERSION 3.5)
project(pulsar_hardware)

# If this is not existant then this has been run by catkin.
# Otherwise we are safe to build
if($ENV{BUILD_ESP32})
    # Build with esp-idf
    include($ENV{IDF_PATH}/tools/cmake/idf.cmake)

    idf_build_process(esp32
        COMPONENTS esp32 freertos esptool_py
        SDKCONFIG ${CMAKE_BINARY_DIR}/sdkconfig
        BUILD_DIR ${CMAKE_BINARY_DIR}
    )

    include_directories("include/pulsar_hardware")

    add_library(pulsar_led
        "src/pulsar_hardware/led/led.cpp"
    )

    target_link_libraries(pulsar_led
        idf::esp32
    )

    add_library(pulsar_i2c
        "src/pulsar_hardware/i2c/i2c.cpp"
    )

    target_link_libraries(pulsar_i2c
        idf::esp32
    )

    add_library(pulsar_motor_driver
        "src/pulsar_hardware/motor_driver/motor_driver.cpp"
    )

    target_link_libraries(pulsar_motor_driver
        idf::esp32
    )

    add_executable(blink.elf
        "src/pulsar_hardware/led/blink.cpp"
    )

    target_link_libraries(blink.elf
        pulsar_led
        idf::esp32
        idf::freertos
        idf::spi_flash
    )

    idf_build_executable(blink.elf)

    add_executable(i2c.elf
        "src/pulsar_hardware/i2c/i2c_test.cpp"
    )

    target_link_libraries(i2c.elf
        pulsar_led
        pulsar_i2c
        idf::esp32
        idf::freertos
        idf::spi_flash
    )

    idf_build_executable(i2c.elf)

    add_executable(motor.elf
        "src/pulsar_hardware/motor_driver/motor_test.cpp"
    )

    target_link_libraries(motor.elf
        pulsar_motor_driver
        idf::esp32
        idf::freertos
        idf::spi_flash
    )

    idf_build_executable(motor.elf)

    set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
else()
    find_package(catkin REQUIRED COMPONENTS pulsar_lib)

    catkin_package(
      INCLUDE_DIRS include
      CATKIN_DEPENDS pulsar_lib
    )

    # Run the script that builds this package with esp-idf
    message("==================================")
    message("Running the esp32 build process...")
    message("==================================")
    execute_process(COMMAND bash -c "${CMAKE_CURRENT_SOURCE_DIR}/build_esp.sh ${CMAKE_BINARY_DIR}/ ${CMAKE_CURRENT_SOURCE_DIR}/ ${CMAKE_CURRENT_SOURCE_DIR}/../esp-idf/")
endif()
