set(COMMON_DIR "../../../common")
set(MODULES_DIR "../../../modules")

idf_component_register(
    SRC_DIRS
        "."
        "${MODULES_DIR}/opendroneid-core-c/libopendroneid"  # Broadcast Remote ID decoder (opendroneid.c).
        "${COMMON_DIR}/flarm"
        "${COMMON_DIR}/adsb"
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
        "${COMMON_DIR}/comms/beast"
        "${COMMON_DIR}/comms/gdl90"
        "${COMMON_DIR}/comms/json"
        "${COMMON_DIR}/comms/mavlink"
        "${COMMON_DIR}/comms/raw"
        "${COMMON_DIR}/settings"
        "comms"
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
        "${COMMON_DIR}/adsb/nasa_cpr"
        "${COMMON_DIR}/comms"
        "${COMMON_DIR}/comms/aircraftjson"
        "${COMMON_DIR}/comms/beast"
        "${COMMON_DIR}/comms/csbee"
        "${COMMON_DIR}/comms/gdl90"
        "${COMMON_DIR}/comms/json"
        "${COMMON_DIR}/comms/mavlink"
        "${COMMON_DIR}/comms/raw"
        "${COMMON_DIR}/coprocessor"
        "${COMMON_DIR}/utils"
        "${COMMON_DIR}/utils/crc"
        "${COMMON_DIR}/utils/fec"
        "${COMMON_DIR}/utils/geo"
        "${COMMON_DIR}/settings"
        "comms"
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
foreach(asset index.html adsbee.js settings.js ota_updater.js firmware_release.js)
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

message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")