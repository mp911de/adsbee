#pragma once

#include "m10_profile.hh"
#include "ublox_max_m10_diagnostics.hh"

// The UART a MAX-M10 is connected to, as the product drives it.
class M10Port {
   public:
    virtual ~M10Port() = default;
    // Send one UBX frame. False while the port cannot take it; the caller tries again later.
    virtual bool SendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t length) = 0;
    // Move the host UART to baud once everything sent has left it. False while it cannot switch yet.
    virtual bool SwitchBaudrate(uint32_t baud) = 0;
    virtual bool Transmitting() const = 0;
    virtual uint32_t baudrate() const = 0;
    // Feed received bytes to M10Receiver::Ingest() while a call waits for the receiver.
    // @retval False to end the wait.
    virtual bool Service() = 0;
};

// Finds a MAX-M10 on its UART, moves it to the wanted rate and keeps it configured to a profile. Nothing blocks
// except ChangeBaudrate() and the polls.
//
// Each batch of the profile is read back first and written only if it differs, so a receiver that already holds
// the profile is left alone and keeps its fix.
class M10Receiver : private M10Config, private UbxParser::Listener, private M10Diagnostics::Listener {
   public:
    static constexpr uint16_t kMaxPayload = 256;

    // max_payload: longest UBX payload the port sends in one frame, at most kMaxPayload.
    M10Receiver(M10Port& port, const M10Profile& profile, uint16_t max_payload = kMaxPayload)
        : port_(port), profile_(profile), max_payload_(max_payload < kMaxPayload ? max_payload : kMaxPayload) {}

    // Also receives the INF messages and RF status.
    void SetObserver(M10Diagnostics::Listener* observer) { observer_ = observer; }

    // Forget the receiver and find it again.
    void Reset();
    // Starts over if anything changed.
    void Configure(const M10Options& options, uint32_t baud);
    // Move receiver and host UART to baud and store it in battery-backed RAM. Waits up to six seconds.
    // @retval False if the receiver did not confirm the rate. It is then searched again at the previous rate.
    bool ChangeBaudrate(uint32_t baud);
    void Update(uint64_t now_us);
    // @retval True if the byte is not part of a UBX frame.
    bool Ingest(uint8_t byte, uint32_t now_ms);
    void Resync() { parser_.Resync(); }
    // True once the receiver holds the profile and while it keeps sending UBX-MON-RF.
    bool IsHealthy() const { return phase_ == Phase::ready; }

    const M10Diagnostics::Version& version() const { return version_; }
    const M10Diagnostics::Latest<M10Diagnostics::RfStatus>& rf() const { return rf_; }
    const M10Diagnostics::Latest<M10Diagnostics::SatelliteSummary>& satellites() const { return satellites_; }
    const M10Diagnostics::Latest<M10Diagnostics::SbasStatus>& sbas() const { return sbas_; }
    // True if battery-backed RAM held CFG-SIGNAL-SBAS_ENA when the receiver was found: it kept its configuration
    // across power-off.
    bool sbas_known_configured() const { return sbas_in_bbr_; }
    // Request one record and wait up to 1.5 seconds for it.
    bool PollComms(M10Diagnostics::CommsStatus& out);
    bool PollHw3(M10Diagnostics::Hw3Status& out);

   private:
    enum class Phase : uint8_t {
        probe, await_probe, next_rate, set_baud, drain_baud, switch_baud,
        check_baud, await_saved_rate, save_baud, await_baud_ack, verify_baud, await_verified_rate,
        read_backup, batch_get, await_batch_values, batch_set, await_batch_ack,
        ready
    };

    bool SendBaud(uint8_t layers);
    // Request one key from battery-backed RAM.
    bool RequestBackupValue(uint32_t key);
    void Retry();
    void RejectBaud();

    M10ConfigBatch::Window BatchWindow() const;
    bool SendBatch();
    void OnBatchValues(const UbxMessage& msg);
    void NextBatch();
    void RejectBatch();
    void BecomeReady();

    bool Poll(uint8_t id, bool& received);
    void OnUbxMessage(const UbxMessage& msg) override;
    void OnVersion(const M10Diagnostics::Version& version) override;
    void OnInfo(bool error, const char* text, uint16_t length, bool truncated) override;
    void OnRf(const M10Diagnostics::RfStatus& rf) override;
    void OnSatellites(const M10Diagnostics::SatelliteSummary& v) override { satellites_ = {true, receive_ms_, v}; }
    void OnSbas(const M10Diagnostics::SbasStatus& v) override { sbas_ = {true, receive_ms_, v}; }
    void OnComms(const M10Diagnostics::CommsStatus& v) override { comms_ = {true, receive_ms_, v}; }
    void OnHw3(const M10Diagnostics::Hw3Status& v) override { hw3_ = {true, receive_ms_, v}; }

    M10Port& port_;
    const M10Profile& profile_;
    const uint16_t max_payload_;
    M10Diagnostics::Listener* observer_ = nullptr;
    UbxParser parser_{*this};
    Phase phase_ = Phase::probe;
    uint64_t next_action_us_ = 0;
    uint32_t target_baud_ = 9600;
    bool baud_confirmed_ = false;
    bool baud_rejected_ = false;
    uint8_t probe_index_ = 0;
    uint32_t receive_ms_ = 0;
    M10Options options_;

    // The batch in progress: profile items [batch_first_, batch_first_ + batch_count_).
    uint16_t batch_first_ = 0;
    uint16_t batch_total_ = 0;
    uint8_t batch_count_ = 0;
    bool batch_written_ = false;
    // Items below this index go one per batch, to find the item that made the receiver reject a batch.
    uint16_t single_item_until_ = 0;

    bool sbas_in_bbr_ = false;
    M10Diagnostics::Version version_;
    M10Diagnostics::Latest<M10Diagnostics::RfStatus> rf_;
    M10Diagnostics::Latest<M10Diagnostics::SatelliteSummary> satellites_;
    M10Diagnostics::Latest<M10Diagnostics::SbasStatus> sbas_;
    M10Diagnostics::Latest<M10Diagnostics::CommsStatus> comms_;
    M10Diagnostics::Latest<M10Diagnostics::Hw3Status> hw3_;
};
