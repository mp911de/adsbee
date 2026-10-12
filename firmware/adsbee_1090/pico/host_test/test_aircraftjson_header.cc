#if __has_include("flarm_reports.hh")
#error "This check must compile without FLARM headers"
#endif
#include "aircraftjson_utils.hh"

void CheckAircraftJSONHeader(const AircraftDictionary& dictionary) {
    char buffer[1024];
    uint32_t uids[4];
    uint16_t next = 0;
    uint16_t num_uids = ListAircraftJSONLiveMapUIDs(dictionary, 1000, uids, 4);
    WriteAircraftJSONLiveMapArray(dictionary, 1000, uids, num_uids, next, buffer, sizeof(buffer));
}
