set(COMMON_DIR "../../../common")
set(MODULES_DIR "../../../modules")

idf_component_register(
    SRC_DIRS
        "."
        "${MODULES_DIR}/opendroneid-core-c/libopendroneid"  # Broadcast Remote ID decoder (opendroneid.c).
        "${COMMON_DIR}/flarm"
        "${COMMON_DIR}/adsb"
        "${COMMON_DIR}/auto_update"
        "${COMMON_DIR}/swarm"
        "${COMMON_DIR}/adsb/nasa_cpr"
        "${COMMON_DIR}/adsb/nasa_cpr/airborne"
        "${COMMON_DIR}/adsb/nasa_cpr/coarse"
        "${COMMON_DIR}/adsb/nasa_cpr/surface"
        "${COMMON_DIR}/adsb/nasa_cpr/intent"
        # "${COMMON_DIR}/comms" # Currently no src files in here.
        "${COMMON_DIR}/coprocessor"
        "${COMMON_DIR}/utils"
        "${COMMON_DIR}/utils/crc"
        "${COMMON_DIR}/utils/fec"
        "${COMMON_DIR}/utils/geo"
        "${COMMON_DIR}/comms"
        "${COMMON_DIR}/comms/asterix"
        "${COMMON_DIR}/comms/beast"
        "${COMMON_DIR}/comms/gdl90"
        "${COMMON_DIR}/comms/json"
        "${COMMON_DIR}/comms/mavlink"
        "${COMMON_DIR}/comms/raw"
        "${COMMON_DIR}/settings"
        "auto_update"
        "comms"
        "swarm"
        "peripherals/pico"
        "remote_id"
        "server"
        "target_test"
    INCLUDE_DIRS
        "."
        "${COMMON_DIR}"
        "${MODULES_DIR}/opendroneid-core-c/libopendroneid"  # opendroneid.h for Remote ID decode.
        "${COMMON_DIR}/flarm"
        "${COMMON_DIR}/adsb"
        "${COMMON_DIR}/auto_update"
        "${COMMON_DIR}/swarm"
        "${COMMON_DIR}/adsb/nasa_cpr"
        "${COMMON_DIR}/comms"
        "${COMMON_DIR}/comms/aircraftjson"
        "${COMMON_DIR}/comms/asterix"
        "${COMMON_DIR}/comms/beast"
        "${COMMON_DIR}/comms/csbee"
        "${COMMON_DIR}/comms/gdl90"
        "${COMMON_DIR}/comms/json"
        "${COMMON_DIR}/comms/mavlink"
        "${COMMON_DIR}/comms/raw"
        "${COMMON_DIR}/comms/uat_uplink"
        "${COMMON_DIR}/coprocessor"
        "${COMMON_DIR}/utils"
        "${COMMON_DIR}/utils/crc"
        "${COMMON_DIR}/utils/fec"
        "${COMMON_DIR}/utils/geo"
        "${COMMON_DIR}/settings"
        "auto_update"
        "comms"
        "swarm"
        "peripherals/pico"
        "remote_id"
        "server"
        "target_test"
    # EMBED_FILES keeps the file at its exact length, which is what the server sends. EMBED_TXTFILES would append a
    # NUL terminator that then reached the browser. Text assets are gzipped below.
    EMBED_FILES
        "server/web/favicon.png"
)

# Text web assets are served pre-compressed (Content-Encoding: gzip), roughly a quarter of the flash and airtime.
set(WEB_GZ_DIR "${CMAKE_CURRENT_BINARY_DIR}/web")
file(MAKE_DIRECTORY "${WEB_GZ_DIR}")
foreach(asset index.html adsbee.js settings.js ota_updater.js firmware_release.js fisb.js fisb_stations.js)
    set(src "${CMAKE_CURRENT_SOURCE_DIR}/server/web/${asset}")
    set(gz "${WEB_GZ_DIR}/${asset}.gz")
    add_custom_command(OUTPUT "${gz}" COMMAND gzip -9 -n -c "${src}" > "${gz}" DEPENDS "${src}" VERBATIM)
    target_add_binary_data(${COMPONENT_LIB} "${gz}" BINARY DEPENDS "${gz}")
endforeach()

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-packed-bitfield-compat)

# Drop the Open Drone ID library's printf/debug helpers (we only use the decode path). Saves flash and avoids stdio.
target_compile_definitions(${COMPONENT_LIB} PRIVATE ODID_DISABLE_PRINTF)

# Internal RAM diagnostics for memory tuning (see heap_diagnostics.hh): idf.py -DHEAP_DIAGNOSTICS=1 build
if(HEAP_DIAGNOSTICS)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE HEAP_DIAGNOSTICS)
endif()

# Bench builds of the automatic updater (auto_update/auto_update_manager.hh): another service address, a first check
# after N seconds, and a CA file named auto_update_test_ca.pem to trust instead of the certificate bundle.
#   idf.py -DAUTO_UPDATE_API_BASE=https://10.0.0.2:8443/api/fw/v1/ -DAUTO_UPDATE_FIRST_CHECK_S=60 \
#          -DAUTO_UPDATE_TEST_CA=/path/to/auto_update_test_ca.pem build
if(AUTO_UPDATE_API_BASE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE AUTO_UPDATE_API_BASE="${AUTO_UPDATE_API_BASE}")
endif()
if(AUTO_UPDATE_FIRST_CHECK_S)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE AUTO_UPDATE_FIRST_CHECK_S=${AUTO_UPDATE_FIRST_CHECK_S})
endif()
if(AUTO_UPDATE_TEST_CA)
    target_add_binary_data(${COMPONENT_LIB} "${AUTO_UPDATE_TEST_CA}" TEXT)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE AUTO_UPDATE_TEST_CA)
endif()

message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")