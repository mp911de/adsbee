#pragma once

#include "gnss_receiver.hh"
#include "hal.hh"  // For get_time_since_boot_ms().
#include "ublox_max_m10_telemetry.hh"
#include "ubx_parser.hh"
#ifdef AIRGREP
#include "../../../profiles/airgrep_gnss.hh"
#endif
#include "ublox_max_m10_defaults.hh"  // For kUbloxConfigDefaults[] (auto-generated).

/**
 * Driver for the u-blox MAX-M10(S) GNSS module.
 *
 * Implements the receiver-specific pieces on top of GNSSReceiver:
 *   - The UBX configuration interface (UBX-CFG-VALSET) with a selectable storage layer.
 *   - Exposed config-set helpers so any configuration item can be changed.
 *   - An init sequence that programs the full firmware default configuration into RAM + BBR as a
 *     known-good baseline (V_BCKP is an always-on battery-derived rail, so BBR persists), then
 *     applies the application-specific overrides we need (e.g. enabling GGA/RMC output).
 */
class UbloxMAXM10 : public GNSSReceiver, private UbxParser::Listener, private M10Diagnostics::Listener {
   public:
    // Settle floor after power-on before we start probing. u-blox does not publish a fixed
    // Vcc-to-UART-ready time for the M10 (only TTFF, which is fix time, not UART readiness)
    static constexpr uint32_t kAckTimeoutMs = 250;

    // Liveness probe (UBX-MON-VER poll) timeout, per attempt. A u-blox-reported data point shows some
    // modules are still not ready to answer at 300 ms after startup, so allow a bit more headroom.
    static constexpr uint32_t kProbeTimeoutMs = 500;

    // diagnosstics timeout
    static constexpr uint32_t kDiagnosticsTimeoutMs = 1500;

    // M10 factory default, used to recover a module that does not answer at the configured rate.
    static constexpr uint32_t kSafeBaudrate = 9600;
    static constexpr uint32_t kBaudrateSwitchSettleMs = 100;  // Integration manual: about 100 ms before input at the new rate.

    // Overall wall-clock budget for the blocking config pass (SyncConfigDefaults).
    static constexpr uint32_t kInitConfigBudgetMs = 10000;

    // UBX protocol constants.
    static constexpr uint8_t kUbxClassCfg = 0x06;
    static constexpr uint8_t kUbxClassAck = 0x05;
    static constexpr uint8_t kUbxClassMon = 0x0A;
    static constexpr uint8_t kUbxClassNav = 0x01;
    static constexpr uint8_t kUbxIdCfgValset = 0x8A;
    static constexpr uint8_t kUbxIdCfgValget = 0x8B;
    static constexpr uint8_t kUbxIdMonVer = 0x04;
    static constexpr uint8_t kUbxIdMonComms = 0x36;
    static constexpr uint8_t kUbxIdMonHw3 = 0x37;
    static constexpr uint8_t kUbxIdMonRf = 0x38;   // UBX-MON-RF: per-RF-block antenna/AGC/noise status.
    static constexpr uint8_t kUbxIdNavSat = 0x35;  // UBX-NAV-SAT: per-satellite C/N0 + count.
    static constexpr uint8_t kUbxIdAckAck = 0x01;
    static constexpr uint8_t kUbxIdAckNak = 0x00;
    // Sentinel for WaitForUbxMessage to match any message id within the requested class. No real
    // UBX message id uses 0xFF.
    static constexpr uint8_t kUbxIdWildcard = 0xFF;

    // VALSET "layers" bitfield (a config-set may target multiple layers at once).
    enum CfgLayer : uint8_t {
        kCfgLayerRam = 0x01,    // Effective immediately.
        kCfgLayerBbr = 0x02,    // Battery-backed RAM; effective on restart, persists with V_BCKP.
        kCfgLayerFlash = 0x04,  // External flash (if present)

        kCfgGetLayerRam = 0x0,       // RAM layer
        kCfgGetLayerBbr = 0x01,      // Battery-backed RAM
        kCfgGetLayerFlash = 0x02,    // External flash.
        kCfgGetLayerDefault = 0x07,  // default layer
    };

    // Selected commonly-used configuration key IDs. The full default catalog lives in
    // ublox_max_m10_defaults.hh; these named constants are for the overrides we apply explicitly.
    // Size of each value is encoded in bits 30..28 of the key id.
    static constexpr uint32_t kCfgUart1Baudrate = 0x40520001;          // U4
    static constexpr uint32_t kCfgMsgoutNmeaGgaUart1 = 0x209100bb;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaRmcUart1 = 0x209100ac;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaGsaUart1 = 0x209100c0;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaGsvUart1 = 0x209100c5;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaGllUart1 = 0x209100ca;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaVtgUart1 = 0x209100b1;     // U1
    static constexpr uint32_t kCfgNmeaProtver = 0x20930001;            // E1
    static constexpr uint32_t kCfgAnaUseAna = 0x10230001;             // L: AssistNow Autonomous enable
    static constexpr uint32_t kCfgInfmsgUbxUart1 = 0x20920002;
    static constexpr uint32_t kCfgRateMeas = 0x30210001;
    static constexpr uint32_t kCfgRateNav = 0x30210002;
    static constexpr uint32_t kCfgSignalSbas = 0x10310020;
    static constexpr uint32_t kCfgSignalSbasL1Ca = 0x10310005;
    static constexpr uint32_t kCfgMsgoutUbxMonRfUart1 = 0x2091035a;    // U1
    static constexpr uint32_t kCfgMsgoutUbxNavSbasUart1 = 0x2091006b;  // U1
    static constexpr uint32_t kCfgI2cEnabled = 0x10510003;             // L
    static constexpr uint32_t kCfgItfmAntSetting = 0x20410010;         // E1: 0 unknown, 1 passive, 2 active
    static constexpr uint32_t kCfgHwAntCfgVoltCtrl = 0x10a3002e;       // L
    static constexpr uint32_t kCfgHwAntCfgShortDet = 0x10a3002f;       // L
    static constexpr uint32_t kCfgHwAntCfgOpenDet = 0x10a30031;        // L
    static constexpr uint32_t kCfgHwAntSupShortPin = 0x20a30037;       // U1, PIO number
    static constexpr uint32_t kCfgHwAntSupOpenPin = 0x20a30038;        // U1, PIO number
    // Diagnostic streams on UART1, every Nth navigation epoch.
    static constexpr uint8_t kDiagnosticsEpochDivider = 5;
    // TEMPORARY diagnostic (remove with the rest of the GNSS debug instrumentation).
    static constexpr uint32_t kCfgMsgoutUbxNavSatUart1 = 0x20910016;  // U1

    // The latest record of one diagnostic kind and when it arrived.
    template <typename T>
    struct Latest {
        bool valid = false;
        uint32_t received_ms = 0;
        T value;
        // A record this old no longer describes the module, e.g. after it stopped sending.
        bool IsFresh(uint32_t now_ms) const { return valid && now_ms - received_ms <= kStatusMaxAgeMs; }
    };
    static constexpr uint32_t kStatusMaxAgeMs = 15000;

    // Text from UBX-MON-VER. Empty until the module answers a probe.
    struct Version {
        char module[27] = "", software[31] = "", hardware[11] = "";
    };
    UbloxMAXM10(GNSSReceiverConfig config_in) : GNSSReceiver(config_in, BSP::kGNSSModuleUbloxMAXM10) {}

    M10Telemetry* Telemetry() override { return &telemetry_; }
    UbloxMAXM10* M10() override { return this; }
    void SetModuleBaudrate(uint32_t baudrate) override { SendBaudrate(baudrate, kCfgLayerRam | kCfgLayerBbr); }
    const Latest<M10Diagnostics::RfStatus>& rf() const { return rf_; }
    const Latest<M10Diagnostics::SbasStatus>& sbas() const { return sbas_; }
    const Latest<M10Diagnostics::SatelliteSummary>& satellites() const { return satellites_; }
    const Version& version() const { return version_; }
    // True if BBR held CFG-SIGNAL-SBAS_ENA at init, so the module kept its configuration across power-off.
    bool sbas_known_configured() const { return sbas_known_configured_; }
    // Request one MON-COMMS or MON-HW3 record and wait up to kProbeTimeoutMs for it.
    bool PollComms(M10Diagnostics::CommsStatus& comms);
    bool PollHw3(M10Diagnostics::Hw3Status& hw3);

    /**
     * Set a configuration item via UBX-CFG-VALSET.
     * @param[in] key_id 32-bit configuration key ID (size encoded in bits 30..28).
     * @param[in] value Raw little-endian value to store (interpret per the item's type).
     * @param[in] value_size_bytes Number of value bytes (1, 2, 4, or 8).
     * @param[in] layers Bitwise-OR of CfgLayer values selecting where to store the value.
     * @retval True if the module acknowledged (UBX-ACK-ACK), false on NAK/timeout.
     */
    bool CfgValSet(uint32_t key_id, uint64_t value, uint8_t value_size_bytes, uint8_t layers);

    /**
     * Read a configuration item's current value via UBX-CFG-VALGET (from the RAM layer).
     * @param[in] key_id 32-bit configuration key ID (size encoded in bits 30..28).
     * @param[in] value_size_bytes Number of value bytes expected (1, 2, 4, or 8).
     * @param[out] value_out Receives the little-endian value the module reported.
    * @param[in] layer layer from which the configuration items should be retrieved.
     * @retval True if the module returned this key's value, false on timeout / not present.
     */
    bool CfgValGet(uint32_t key_id, uint8_t value_size_bytes, uint64_t& value_out, uint8_t layer);

    // Typed convenience wrappers around CfgValSet.
    bool CfgValSetU1(uint32_t key, uint8_t v, uint8_t layers) { return CfgValSet(key, v, 1, layers); }
    bool CfgValSetU2(uint32_t key, uint16_t v, uint8_t layers) { return CfgValSet(key, v, 2, layers); }
    bool CfgValSetU4(uint32_t key, uint32_t v, uint8_t layers) { return CfgValSet(key, v, 4, layers); }
    bool CfgValSetU8(uint32_t key, uint64_t v, uint8_t layers) { return CfgValSet(key, v, 8, layers); }
    bool CfgValSetL(uint32_t key, bool v, uint8_t layers) { return CfgValSet(key, v ? 1 : 0, 1, layers); }
    bool CfgValSetE1(uint32_t key, uint8_t v, uint8_t layers) { return CfgValSet(key, v, 1, layers); }

   protected:
    void IngestModuleByte(uint8_t byte, uint32_t now_ms) override { ubx_parser_.IngestByte(byte, now_ms); }
    void DiscardModuleStream() override;
    M10Telemetry telemetry_;
    UbxParser ubx_parser_{*this};
    // Probe/bring up the UART at the user-configured/persisted baud rate, not the module's factory
    // default. Once a module has been configured (its baud persists in battery-backed RAM), it keeps
    // running at whatever rate it was last set to -- using the factory default here instead would
    // mismatch the module's actual running baud and break comms after the first successful config.
    // Mirrors GenericGNSSReceiver::GetDefaultBaudrate().
    uint32_t GetDefaultBaudrate() const override {
        return settings_manager.settings.baud_rates[SettingsManager::kGNSSUART];
    }
    bool SendInitCommands() override;
    void ResendRuntimeConfig() override;

    // TEMPORARY debug hook (remove with the rest of the GNSS debug instrumentation): prints the
    // latest UBX-MON-RF and UBX-NAV-SAT snapshot.
    void DebugDumpModuleStatus() override;

   private:
#ifdef AIRGREP
    static constexpr uint8_t kMessageBatchSize = 32;
    static constexpr uint32_t kMessageReplyTimeoutMs = 500;
    static constexpr uint32_t kBootstrapReplyTimeoutMs = 1500;
    bool ConfigureAirgrep(uint8_t layers);
    bool ConfigureAirgrepMessages(uint8_t layers);
    bool ApplyMessageBatch(const airgrep::GNSSMessageRate* messages, uint8_t count, uint8_t layers,
                           uint32_t timeout_ms = kMessageReplyTimeoutMs);
    bool VerifyMessageBatch(const airgrep::GNSSMessageRate* messages, uint8_t count, uint32_t timeout_ms);
#endif
    // Build and transmit a raw UBX frame (sync + class/id + length + payload + checksum).
    void SendUbxFrame(uint8_t msg_class, uint8_t msg_id, const uint8_t* payload, uint16_t payload_len);

    // Blocking UBX receive: pumps the incoming byte stream (up to timeout_ms) through ubx_parser_
    // until a frame matching want_class/want_id arrives. Optionally copies up to out_cap payload bytes
    // into out_payload and reports the true payload length. NMEA and diagnostics keep flowing meanwhile.
    // @param[out] is_ack Optional; for ACK/NAK frames, set true on ACK-ACK and false on ACK-NAK.
    // @param[out] config_nak Optional; also accepts an ACK/NAK for want_class/want_id and reports NAK.
    // @retval True if a matching frame with a valid checksum arrived before the timeout, false otherwise.
    bool ScanForUbxMessage(uint8_t want_class, uint8_t want_id, uint32_t timeout_ms, uint8_t* out_payload,
                           uint16_t out_cap, uint16_t* out_len, bool* is_ack, bool* config_nak = nullptr);

    // Thin wrapper: wait for a matching UBX frame without capturing its payload.
    bool WaitForUbxMessage(uint8_t want_class, uint8_t want_id, uint32_t timeout_ms, bool* is_ack = nullptr);

    // Wait (up to kAckTimeoutMs) for a UBX-ACK-ACK/NAK for the given message class/id.
    // @retval True on ACK-ACK, false on ACK-NAK or timeout.
    bool WaitForAck(uint8_t acked_class, uint8_t acked_id);

    // Poll UBX-MON-VER once at the current host rate.
    bool Probe();
    void SetHostBaudrate(uint32_t baudrate);
    // Tell the module its new UART1 rate at the current host rate. The acknowledgment is not awaited: it may
    // arrive at either rate.
    void SendBaudrate(uint32_t baudrate, uint8_t layers);

    // Find the module at the configured rate, else at kSafeBaudrate and move it to the configured rate.
    // @retval True if the module answers, at the configured rate or at kSafeBaudrate.
    bool ConnectAtConfiguredBaudrate();

    // Read-modify-write the default configuration table into RAM + BBR: for each key, read the
    // module's current value (UBX-CFG-VALGET) and only write it (UBX-CFG-VALSET) if it differs.
    // On an already-provisioned module nothing is written, so the live fix and BBR-held
    // ephemeris/almanac survive and the module hot-starts. Pokes the watchdog each iteration.
    // @param[in] deadline_ms Absolute get_time_since_boot_ms() timestamp at which to abort the pass so
    //                        an unresponsive module can't stall init past the watchdog window.
    // @retval Number of keys written (0 means "already fully configured"), or a negative value if the
    //         deadline was exceeded (module stopped responding mid-pass).
    int32_t SyncConfigDefaults(uint32_t deadline_ms);

    // Enable exactly the NMEA sentences we consume (GGA + RMC) and silence the rest, on UART1.
    // Cheap and idempotent; used both during init and after a UART handover (ResendRuntimeConfig).
    // @param[in] layers CfgLayer bitfield to target.
    void ApplyRuntimeMessageConfig(uint8_t layers);
#ifndef AIRGREP
    // MON-RF and NAV-SBAS every kDiagnosticsEpochDivider epochs, INF messages per the diagnostics setting.
    void ApplyDiagnosticStreams(uint8_t layers);
#endif

    // One pending ScanForUbxMessage, matched by OnUbxMessage.
    struct Scan {
        bool active = false, done = false;
        uint8_t cls = 0, id = 0;
        uint8_t* out_payload = nullptr;
        uint16_t out_cap = 0;
        uint16_t* out_len = nullptr;
        bool* is_ack = nullptr;
        bool* config_nak = nullptr;
    };
    Scan scan_;
    void OnUbxMessage(const UbxMessage& message) override;
    bool MatchScan(const UbxMessage& message);

    void OnInfo(bool error, const char* text, uint16_t length, bool truncated) override {
        telemetry_.OnInfo(error, text, length, truncated);
    }
    void OnRf(const M10Diagnostics::RfStatus& rf) override;
    void OnSbas(const M10Diagnostics::SbasStatus& sbas) override;
    void OnComms(const M10Diagnostics::CommsStatus& comms) override;
    void OnHw3(const M10Diagnostics::Hw3Status& hw3) override;
    void OnSatellites(const M10Diagnostics::SatelliteSummary& satellites) override;
    template <typename T>
    static void Store(Latest<T>& latest, const T& value) {
        latest.value = value;
        latest.received_ms = get_time_since_boot_ms();
        latest.valid = true;
    }

    Latest<M10Diagnostics::RfStatus> rf_;
    Latest<M10Diagnostics::SbasStatus> sbas_;
    Latest<M10Diagnostics::CommsStatus> comms_;
    Latest<M10Diagnostics::Hw3Status> hw3_;
    Latest<M10Diagnostics::SatelliteSummary> satellites_;
    Version version_;
    bool sbas_known_configured_ = false;

    // Antenna mode from settings: ANTSETTING for the jamming detector and, in the airgrep profile, the supervisor.
    bool ConfigureAntenna(uint8_t layers);
};
