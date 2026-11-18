#pragma once

#include "m10_profile.hh"

#ifdef AIRGREP
#include "airgrep_m10_profile.hh"

inline constexpr const M10Profile& kM10ActiveProfile = airgrep::kM10Profile;
#else
#include "ublox_max_m10_defaults.hh"

// The receiver's defaults as a known baseline, then what this firmware sets differently. The defaults table
// leaves those items out ("APP OVERRIDE"), so no item is written twice.
inline void WriteM10DefaultProfile(const M10Options& options, M10ConfigBatch& batch) {
    for (const UbloxCfgDefault& item : kUbloxConfigDefaults) batch.Add(item.key_id, item.value);
    WriteMessageSelection(options.diagnostics, [&](uint32_t key, uint8_t rate) { batch.Add(key, rate); });
    batch.Add(M10Config::kCfgItfmAntSetting, M10Config::AntennaSetting(options.antenna));
    // TEMPORARY: NAV-SAT feeds DebugDumpModuleStatus (remove with the rest of the debug instrumentation).
    batch.Add(M10Config::kCfgMsgoutUbxNavSatUart1, 1);
    // AssistNow Autonomous: the module predicts orbits itself, which shortens the time to a fix.
    batch.Add(M10Config::kCfgAnaUseAna, 1);
}

inline constexpr M10Profile kM10ActiveProfile{WriteM10DefaultProfile,
                                              M10Config::kCfgLayerRam | M10Config::kCfgLayerBbr};
#endif
