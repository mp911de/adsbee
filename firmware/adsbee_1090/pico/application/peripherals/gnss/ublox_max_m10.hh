#pragma once

#include "gnss_receiver.hh"
#include "m10_active_profile.hh"
#include "m10_receiver.hh"
#include "ublox_max_m10_telemetry.hh"

/**
 * Driver for the u-blox MAX-M10(S) GNSS module.
 *
 * Connects M10Receiver to the GNSS UART. M10Receiver finds the module, moves it to the configured baud rate and
 * keeps it configured to this build's profile (m10_active_profile.hh), without blocking the main loop.
 */
class UbloxMAXM10 : public GNSSReceiver, private M10Port {
   public:
    UbloxMAXM10(GNSSReceiverConfig config_in) : GNSSReceiver(config_in, BSP::kGNSSModuleUbloxMAXM10) {
        receiver_.SetObserver(&telemetry_);
    }

    M10Telemetry* Telemetry() override { return &telemetry_; }
    M10Receiver* M10() override { return &receiver_; }
    bool SetModuleBaudrate(uint32_t baudrate) override;

   protected:
    // The user-configured rate. The module keeps its rate in battery-backed RAM, so it is not at its factory
    // default once it was configured.
    uint32_t GetDefaultBaudrate() const override {
        return settings_manager.settings.baud_rates[SettingsManager::kGNSSUART];
    }
    void StartModule() override;
    bool UpdateModule() override;
    void IngestModuleByte(uint8_t byte, uint32_t now_ms) override { receiver_.Ingest(byte, now_ms); }
    void DiscardModuleStream() override;

    // TEMPORARY debug hook (remove with the rest of the GNSS debug instrumentation): prints the
    // latest UBX-MON-RF and UBX-NAV-SAT snapshot.
    void DebugDumpModuleStatus() override;

   private:
    bool SendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t length) override;
    bool SwitchBaudrate(uint32_t baud) override;
    bool Transmitting() const override;
    uint32_t baudrate() const override { return host_baud_; }
    bool Service() override;

    M10Telemetry telemetry_;
    M10Receiver receiver_{*this, kM10ActiveProfile};
    uint32_t host_baud_ = 0;
};
