#include "ublox_max_m10.hh"

#include "comms.hh"  // For comms_manager (UART I/O) and CONSOLE_* logging.
#include "hal.hh"
#include "hardware/uart.h"
#include "hardware/watchdog.h"

bool UbloxMAXM10::SetModuleBaudrate(uint32_t baudrate) {
    // A module that is off or handed over is found at the new rate when it starts.
    if (!active_ || suspended_ || initializing_) return true;
    return receiver_.ChangeBaudrate(baudrate);
}

void UbloxMAXM10::StartModule() {
    host_baud_ = GetDefaultBaudrate();  // ClaimUart() opened the UART at this rate.
    receiver_.Reset();
}

bool UbloxMAXM10::UpdateModule() {
    const auto& settings = settings_manager.settings;
    receiver_.Configure({settings.gnss_antenna, settings.gnss_diagnostics}, GetDefaultBaudrate());
    receiver_.Update(get_time_since_boot_us());
    return receiver_.IsHealthy();
}

void UbloxMAXM10::DiscardModuleStream() {
    receiver_.Resync();
    telemetry_.Reset();
}

bool UbloxMAXM10::SendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t length) {
    WriteUbxFrame(cls, id, payload, length, [](uint8_t byte) {
        comms_manager.iface_putc(SettingsManager::kGNSSUART, static_cast<char>(byte));
    });
    return true;
}

bool UbloxMAXM10::SwitchBaudrate(uint32_t baud) {
    uart_tx_wait_blocking(config_.uart_handle);
    uart_set_baudrate(config_.uart_handle, baud);
    host_baud_ = baud;
    // Bytes received at the previous rate are garbage.
    PollUartIntoRxBuffer();
    char c;
    while (ReadBufferedByte(c)) {
    }
    receiver_.Resync();
    return true;
}

bool UbloxMAXM10::Transmitting() const { return uart_get_hw(config_.uart_handle)->fr & UART_UARTFR_BUSY_BITS; }

bool UbloxMAXM10::Service() {
    watchdog_update();
    IngestRx(get_time_since_boot_ms());
    return active_ && !suspended_;
}

void UbloxMAXM10::DebugDumpModuleStatus() {
    // TEMPORARY: print the latest MON-RF + NAV-SAT snapshot (remove with the rest of the
    // GNSS debug instrumentation).
    if (receiver_.rf().valid) {
        const M10Diagnostics::RfStatus& rf = receiver_.rf().value;
        CONSOLE_INFO("UbloxMAXM10::DebugDumpModuleStatus",
                     "GNSS RF: jam=%u antStatus=%u(%s) antPower=%u(%s) noise=%u agc=%u(/8191) magI=%u magQ=%u\r\n",
                     rf.jamming, rf.antenna_status, M10Strings::AntennaStatus(rf.antenna_status), rf.antenna_power,
                     M10Strings::AntennaPower(rf.antenna_power), rf.noise, rf.agc, rf.magnitude_i, rf.magnitude_q);
    } else {
        CONSOLE_INFO("UbloxMAXM10::DebugDumpModuleStatus", "GNSS RF: waiting for UBX-MON-RF frame...\r\n");
    }
    if (receiver_.satellites().valid) {
        CONSOLE_INFO("UbloxMAXM10::DebugDumpModuleStatus", "GNSS SAT: numSvs=%u maxCno=%u dBHz\r\n",
                     receiver_.satellites().value.count, receiver_.satellites().value.max_cno_dbhz);
    } else {
        CONSOLE_INFO("UbloxMAXM10::DebugDumpModuleStatus", "GNSS SAT: waiting for UBX-NAV-SAT frame...\r\n");
    }
}
