#pragma once

#include "m10_profile.hh"

#ifdef AIRGREP
#include "airgrep_m10_profile.hh"

inline constexpr const M10Profile& kM10ActiveProfile = airgrep::kM10Profile;
#else

// NMEA position and time at 1 Hz with the status messages, SBAS, and the antenna supervisor on PIO 3 (short) and
// PIO 2 (open) when the antenna is supervised.
inline void WriteM10DefaultProfile(const M10Options& options, M10ConfigBatch& batch) {
    using Cfg = M10Config;
    const bool supervised = options.antenna == GNSSSettingsOptions::kGNSSAntennaActiveSupervised;
    batch.Add(Cfg::kCfgUart1OutNmea, 1);
    batch.Add(Cfg::kCfgUart1OutUbx, 1);
    batch.Add(Cfg::kCfgRateMeas, 1000);
    batch.Add(Cfg::kCfgRateNav, 1);
    WriteMessageSelection(options.diagnostics, [&](uint32_t key, uint8_t rate) { batch.Add(key, rate); });
    batch.Add(Cfg::kCfgMsgoutUbxNavSatUart1, Cfg::kDiagnosticsEpochDivider);
    batch.Add(Cfg::kCfgSignalSbas, 1);
    batch.Add(Cfg::kCfgItfmAntSetting, Cfg::AntennaSetting(options.antenna));
    batch.Add(Cfg::kCfgHwAntCfgVoltCtrl, supervised);
    batch.Add(Cfg::kCfgHwAntCfgShortDet, supervised);
    batch.Add(Cfg::kCfgHwAntCfgOpenDet, supervised);
    batch.Add(Cfg::kCfgHwAntCfgPowerDown, supervised);
    batch.Add(Cfg::kCfgHwAntCfgRecover, supervised);
    if (supervised) {
        batch.Add(Cfg::kCfgHwAntSupShortPin, 3);
        batch.Add(Cfg::kCfgHwAntSupOpenPin, 2);
    }
}

inline constexpr M10Profile kM10ActiveProfile{WriteM10DefaultProfile, M10Config::kCfgLayerRam};
#endif
