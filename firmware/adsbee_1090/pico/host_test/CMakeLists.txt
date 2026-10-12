enable_testing()
add_executable(host_test main.cc)
add_executable(host_test_fpu main.cc)

find_package(Threads REQUIRED)

set(CMAKE_BUILD_TYPE "Debug") # Now that we've identified we're running tests, revert to an actual build config.

add_compile_options(
    -Wall                # Include all warnings.
    -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
    -Wno-unused-function # we have some for the docs that aren't called
    -Wno-maybe-uninitialized
    -ftrapv # Trap on signed integer overflow (undefined behavior).
)

# Test: Pull in google test library
add_library(libgtest SHARED IMPORTED)
set_target_properties(libgtest PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/../../modules/googletest/build/lib/libgtest.so)
target_link_libraries(host_test PRIVATE libgtest Threads::Threads)
target_link_libraries(host_test_fpu PRIVATE libgtest Threads::Threads)
target_compile_definitions(host_test_fpu PRIVATE HAS_FPU)
target_compile_definitions(host_test PRIVATE SETTINGS_BLOBS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/settings_blobs")
target_compile_definitions(host_test_fpu PRIVATE SETTINGS_BLOBS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/settings_blobs")
# MAVLink send helpers need the firmware's comms_manager; tests only exercise message packing.
target_compile_definitions(host_test PRIVATE MAVLINK_NO_CONVENIENCE_FUNCTIONS)
target_compile_definitions(host_test_fpu PRIVATE MAVLINK_NO_CONVENIENCE_FUNCTIONS)

add_subdirectory(${ADSBEE_COMMON_DIR} ${CMAKE_BINARY_DIR}/adsbee_common)
add_subdirectory(${CMAKE_SOURCE_DIR}/bootloader ${CMAKE_BINARY_DIR}/bootloader)
add_subdirectory(${CMAKE_SOURCE_DIR}/application ${CMAKE_BINARY_DIR}/application)

foreach(_test_tgt IN ITEMS host_test host_test_fpu)
    target_sources(${_test_tgt} PRIVATE
        ${ADSBEE_MODULES_DIR}/cppAT/src/cpp_at.cc
        ${ADSBEE_MODULES_DIR}/cppAT/test/test_cpp_at.cc
        hal.cc
        main.cc
        settings.cc
        # test_ads_b_decoder.cc
        test_aircraft_dictionary_mode_s.cc
        test_aircraft_dictionary_allocator.cc
        test_buffer_utils.cc
        # test_adsbee.cc
        test_composite_array.cc
        test_cpu_utils.cc
        test_crc.cc
        test_data_structures.cc
        test_decode_utils.cc
        test_fixedmath.cc
        test_geo_utils.cc
        test_nasa_cpr.cc
        test_mode_s_packet_decoder.cc
        test_mode_s_single_bit_correction.cc
        test_mode_s_single_bit_filter.cc
        test_nmea_parser.cc
        test_pps_clock.cc
        test_flarm.cc
        ${CMAKE_SOURCE_DIR}/application/peripherals/gnss/nmea_parser.cc
        test_platform.cc
        test_reporting_aircraftjson.cc
        test_reporting_beast.cc
        test_reporting_csbee.cc
        test_reporting_gdl90.cc
        test_reporting_mavlink.cc
        test_reporting_raw.cc
        test_settings.cc
        test_settings_release_blobs.cc
        test_spi_coprocessor.cc
        test_spi_link_gate.cc
        test_at_console_guard.cc
        test_at_console_input.cc
        test_wifi_network_selector.cc
        test_log_message.cc
        test_squitter_packets.cc
        test_transponder_packet.cc
        test_unit_conversions.cc
        test_pfb_queue_thread_safety.cc
        test_mode_s_soft_demod.cc
        ${CMAKE_SOURCE_DIR}/application/mode_s_soft_demod.cc
        test_r1090_rssi.cc
    )

    target_include_directories(${_test_tgt} PRIVATE
        ${ADSBEE_COMMON_DIR}
        ${ADSBEE_MODULES_DIR}/googletest
        ${ADSBEE_MODULES_DIR}/cppAT/settings # use default settings for testing
        ${ADSBEE_MODULES_DIR}/cppAT/src
        ${CMAKE_SOURCE_DIR}/application/peripherals/gnss
        .
        mocks
    )
endforeach()

# Shared JSON must compile for products with no FLARM include path.
add_library(aircraftjson_header_check OBJECT test_aircraftjson_header.cc)
get_target_property(json_header_includes host_test INCLUDE_DIRECTORIES)
list(FILTER json_header_includes EXCLUDE REGEX "/flarm(/|$)")
target_include_directories(aircraftjson_header_check PRIVATE ${json_header_includes})
target_compile_definitions(aircraftjson_header_check PRIVATE ON_HOST)

add_subdirectory(uat)

# Make test runnable with "make test".
add_test(NAME build_profiles COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/profiles/test_profiles.py)

include(GoogleTest)
foreach(_test_tgt IN ITEMS host_test host_test_fpu)
    add_dependencies(${_test_tgt} aircraftjson_header_check)
    gtest_discover_tests(${_test_tgt}
        TEST_PREFIX "${_test_tgt}."
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        PROPERTIES
            TIMEOUT 30
            DISCOVERY_TIMEOUT 10
    )
endforeach()
# Compile both translation units with the same real host mutex layout.
add_executable(flarm_snapshot_concurrency_test test_flarm_snapshot_concurrency.cc
    ${ADSBEE_COMMON_DIR}/flarm/flarm_snapshot.cpp)
target_include_directories(flarm_snapshot_concurrency_test PRIVATE
    ${ADSBEE_COMMON_DIR}/flarm ${ADSBEE_COMMON_DIR}/utils ${ADSBEE_MODULES_DIR}/googletest/googletest/include)
target_compile_options(flarm_snapshot_concurrency_test PRIVATE
    -include ${CMAKE_CURRENT_SOURCE_DIR}/mocks/flarm_test_mutex.hh)
target_link_libraries(flarm_snapshot_concurrency_test PRIVATE libgtest Threads::Threads)
add_dependencies(host_test flarm_snapshot_concurrency_test)
add_dependencies(host_test_fpu flarm_snapshot_concurrency_test)
gtest_discover_tests(flarm_snapshot_concurrency_test TEST_PREFIX "flarm_concurrency.")
