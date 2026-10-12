#if __has_include("flarm_reports.hh")
#error "This check must compile without FLARM headers"
#endif
#include "aircraftjson_utils.hh"

void CheckAircraftJSONHeader(const AircraftDictionary& dictionary) {
    char buffer[1024];
    WriteAircraftJSONLiveMapArrays(dictionary, 1000, buffer, sizeof(buffer), [](const char*, uint16_t) {});
}
