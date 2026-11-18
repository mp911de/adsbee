#pragma once

#include <ti/drivers/UART2.h>

#include "gnss_navigation.hh"
#include "m10_active_profile.hh"
#include "m10_receiver.hh"
#include "settings.hh"

class GNSSReceiver : private M10Port {
   public:
    explicit GNSSReceiver(bool (*service_during_poll)())
        : service_during_poll_(service_during_poll), m10_(*this, kM10ActiveProfile, kMaxTxFrameSize - 8) {}
    bool Configure(bool enabled, SettingsManager::GNSSReceiverType type);
    bool SetBaudrate(uint32_t baud);
    void Update();
    // Returns false while the last configuration frame is still on the wire.
    bool Stop();
    const NMEAParser::GNSSFix* GetFix() const;
    bool HasValidFix() const;
    const NMEAParser::GNSSFix& fix() const { return navigation_.fix(); }
    uint32_t pps_count() const;
    M10Receiver* M10() { return type_ == SettingsManager::kGNSSReceiverUBXMIA ? &m10_ : nullptr; }
    bool IsHealthy() const;
    bool GetUTCUnixMs(uint64_t& utc_ms) const;
    bool GetFixUTCUnixMs(uint64_t& utc_ms) const;

   private:
    static constexpr size_t kMaxTxFrameSize = 144;  // Fits one nonblocking UART1 write. A UBX frame adds 8 bytes.
    bool Open(uint32_t baud);
    bool OpenUART(uint32_t baud);
    bool SwitchBaudrate(uint32_t baud) override;
    bool Transmitting() const override;
    uint32_t baudrate() const override { return baud_; }
    bool Service() override;
    bool ApplySettings(bool enabled, SettingsManager::GNSSReceiverType type);
    bool Read();
    bool SendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t length) override;
    void NotifyFix();
    bool (*service_during_poll_)();
    UART2_Handle uart_ = nullptr;
    uint32_t baud_ = 0;
    uint32_t poll_interval_us_ = 5000;
    SettingsManager::GNSSReceiverType type_ = SettingsManager::kGNSSReceiverNone;
    M10Receiver m10_;
    GNSSNavigation navigation_;
    uint64_t last_poll_us_ = 0;
    uint64_t last_drain_us_ = 0;
    uint64_t next_notification_us_ = 0;
    bool notified_fix_ = false;
};

extern GNSSReceiver gnss_receiver;
extern GNSSReceiver* gnss;
