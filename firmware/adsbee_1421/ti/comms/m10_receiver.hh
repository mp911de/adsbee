#pragma once

#include "settings.hh"
#include "ublox_max_m10_config.hh"
#include "ublox_max_m10_diagnostics.hh"
#ifdef AIRGREP
#include "airgrep_m10_readback.hh"
#endif

class GNSSReceiver;

class M10Receiver : private M10Config, private UbxParser::Listener, private M10Diagnostics::Listener {
   public:
    explicit M10Receiver(GNSSReceiver& port) : port_(port), parser_(*this) {}

    void Reset();
    void Configure(SettingsManager::GNSSAntennaMode antenna, bool diagnostics, uint32_t baud);
    bool ChangeBaudrate(uint32_t baud);
    void Update(uint64_t now_us);
    bool Ingest(uint8_t byte, uint32_t now_ms);
    void Resync() { parser_.Resync(); }
    bool IsHealthy() const { return phase_ == Phase::ready; }
    const auto& version() const { return version_; }
    const auto& rf() const { return rf_; }
    const auto& satellites() const { return satellites_; }
    const auto& sbas() const { return sbas_; }
    bool sbas_known_configured() const { return sbas_in_bbr_; }
    bool PollComms(M10Diagnostics::CommsStatus& out);
    bool PollHw3(M10Diagnostics::Hw3Status& out);

   private:
    enum class Phase {
        probe, await_probe, next_rate, set_baud, drain_baud, switch_baud,
        check_baud, await_saved_rate, save_baud, await_baud_ack, verify_baud, await_verified_rate,
        read_backup, configure, await_ack,
#ifdef AIRGREP
        profile_set, await_profile_ack, profile_get, await_profile_values,
#endif
        ready
    };
    GNSSReceiver& port_;
    UbxParser parser_;
    Phase phase_ = Phase::probe;
    uint64_t next_action_us_ = 0;
    uint32_t target_baud_ = 9600;
    bool baud_confirmed_ = false;
    bool baud_rejected_ = false;
    uint8_t probe_index_ = 0;
    uint32_t receive_ms_ = 0;
    bool diagnostics_ = false;
    bool sbas_in_bbr_ = false;
    SettingsManager::GNSSAntennaMode antenna_ = SettingsManager::kGNSSAntennaUnknown;
    M10Diagnostics::Version version_;
    M10Diagnostics::Latest<M10Diagnostics::RfStatus> rf_;
    M10Diagnostics::Latest<M10Diagnostics::SatelliteSummary> satellites_;
    M10Diagnostics::Latest<M10Diagnostics::SbasStatus> sbas_;
    M10Diagnostics::Latest<M10Diagnostics::CommsStatus> comms_;
    M10Diagnostics::Latest<M10Diagnostics::Hw3Status> hw3_;

    bool SendConfiguration();
    bool SendBaud(uint8_t layers);
    // Request one key from battery-backed RAM.
    bool RequestBackupValue(uint32_t key);
    void Retry();
    void RejectBaud();
#ifdef AIRGREP
    static constexpr uint8_t kMessageBatch = 16;
    uint8_t profile_index_ = 0;
    uint8_t batch_count_ = 0;
    uint8_t single_key_until_ = 0;
    uint8_t LoadBatch(airgrep::GNSSMessageRate (&entries)[kMessageBatch]) const;
    bool SendProfile(bool readback);
    void NextBatch();
    void RejectBatch();
#endif
    bool Poll(uint8_t id, bool& received);
    void OnUbxMessage(const UbxMessage& msg) override;
    void OnVersion(const M10Diagnostics::Version& version) override;
    void OnInfo(bool error, const char* text, uint16_t length, bool truncated) override;
    void OnRf(const M10Diagnostics::RfStatus& rf) override;
    void OnSatellites(const M10Diagnostics::SatelliteSummary& v) override { satellites_ = {true, receive_ms_, v}; }
    void OnSbas(const M10Diagnostics::SbasStatus& v) override { sbas_ = {true, receive_ms_, v}; }
    void OnComms(const M10Diagnostics::CommsStatus& v) override { comms_ = {true, receive_ms_, v}; }
    void OnHw3(const M10Diagnostics::Hw3Status& v) override { hw3_ = {true, receive_ms_, v}; }
};
