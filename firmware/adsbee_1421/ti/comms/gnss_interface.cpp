#include "gnss_interface.hh"

#include <ti/drivers/GPIO.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/devices/cc13x4_cc26x4/driverlib/uart.h>
#include <ti/devices/cc13x4_cc26x4/inc/hw_memmap.h>

#include "hal.hh"
#include "comms.hh"
#include "ti_drivers_config.h"
#include "utc_time.hh"

GNSSReceiver* gnss = nullptr;

namespace {
volatile uint64_t last_pps_us;
volatile uint32_t pulse_count;
volatile bool tx_active;
volatile bool rx_overrun;

void OnPPS(uint_least8_t) {
    last_pps_us = get_time_since_boot_us();
    pulse_count = pulse_count + 1;
}

void OnUARTEvent(UART2_Handle, uint32_t event, uint32_t, void*) {
    if (event & UART2_EVENT_OVERRUN) rx_overrun = true;
    if (event & UART2_EVENT_TX_BEGIN) tx_active = true;
    if (event & UART2_EVENT_TX_FINISHED) tx_active = false;
}

}  // namespace

bool GNSSReceiver::OpenUART(uint32_t baud) {
    UART2_Params params;
    UART2_Params_init(&params);
    params.baudRate = baud;
    params.readMode = UART2_Mode_NONBLOCKING;
    params.writeMode = UART2_Mode_NONBLOCKING;
    params.eventMask = UART2_EVENT_TX_BEGIN | UART2_EVENT_TX_FINISHED | UART2_EVENT_OVERRUN;
    params.eventCallback = OnUARTEvent;
    uart_ = UART2_open(GNSS_UART, &params);
    if (!uart_) return false;
    rx_overrun = false;
    UART2_rxEnable(uart_);
    baud_ = baud;
    // Poll before half a 64-byte read has arrived. The RX ring holds 256 8N1 characters.
    poll_interval_us_ = 320000000u / baud;
    if (poll_interval_us_ > 5000) poll_interval_us_ = 5000;
    return true;
}

bool GNSSReceiver::Open(uint32_t baud) {
    if (!OpenUART(baud)) return false;
    // TIMEPULSE also feeds the MAX-M10S SAFEBOOT_N strap. Never pull it low.
    GPIO_setConfig(GNSS_PPS, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
    GPIO_setCallback(GNSS_PPS, OnPPS);
    GPIO_clearInt(GNSS_PPS);
    GPIO_enableInt(GNSS_PPS);
    gnss = this;
    return true;
}

bool GNSSReceiver::SwitchBaudrate(uint32_t baud) {
    if (baud_ == baud && uart_) return true;
    if (tx_active || !uart_) return false;
    const uint32_t previous = baud_;
    UART2_close(uart_);
    uart_ = nullptr;
    navigation_ = {};
    m10_.Resync();
    last_poll_us_ = last_drain_us_ = 0;
    if (OpenUART(baud)) return true;
    if (!OpenUART(previous)) Stop();
    return false;
}

bool GNSSReceiver::SetBaudrate(uint32_t baud) {
    if (!SettingsManager::IsGNSSBaudRate(baud)) return false;
    if (!gnss) return true;
    if (!M10()) return SwitchBaudrate(baud);
    return m10_.ChangeBaudrate(baud);
}

bool GNSSReceiver::Transmitting() const { return tx_active; }

bool GNSSReceiver::Stop() {
    if (tx_active) return false;
    gnss = nullptr;
    GPIO_disableInt(GNSS_PPS);
    GPIO_setCallback(GNSS_PPS, nullptr);
    if (uart_) {
        UART2_close(uart_);
        uart_ = nullptr;
    }
    GPIO_setConfig(GNSS_TX, GPIO_CFG_NO_DIR);
    GPIO_setConfig(GNSS_RX, GPIO_CFG_NO_DIR);
    GPIO_setConfig(GNSS_PPS, GPIO_CFG_NO_DIR);
    last_pps_us = 0;
    pulse_count = 0;
    navigation_ = {};
    m10_.Reset();
    type_ = SettingsManager::kGNSSReceiverNone;
    baud_ = 0;
    last_poll_us_ = 0;
    last_drain_us_ = 0;
    next_notification_us_ = 0;
    notified_fix_ = false;
    return true;
}

bool GNSSReceiver::Configure(bool enabled, SettingsManager::GNSSReceiverType type) {
    const uintptr_t key = HwiP_disable();
    pulse_count = 0;
    last_pps_us = 0;
    HwiP_restore(key);
    navigation_ = {};
    m10_.Reset();
    if (uart_) UART2_flushRx(uart_);
    notified_fix_ = false;
    next_notification_us_ = 0;
    return ApplySettings(enabled, type);
}

bool GNSSReceiver::ApplySettings(bool enabled, SettingsManager::GNSSReceiverType type) {
#ifdef AIRGREP
    if (enabled && !uart_ && get_time_since_boot_us() < 1000000) return true;
#endif
    const auto requested = enabled ? type : SettingsManager::kGNSSReceiverNone;
    if (requested != type_) {
        gnss = nullptr;
        if (uart_ && !Stop()) return true;  // Update retries after the last stop bit.
        if (requested != SettingsManager::kGNSSReceiverNone && !Open(settings_manager.settings.gnss_baud_rate)) return false;
        type_ = requested;
    }
    if (uart_) gnss = this;
    const auto& settings = settings_manager.settings;
    if (M10()) m10_.Configure({settings.gnss_antenna, settings.gnss_diagnostics}, settings.gnss_baud_rate);
    if (type_ == SettingsManager::kGNSSReceiverGeneric && !SwitchBaudrate(settings.gnss_baud_rate)) return false;
    return true;
}

void GNSSReceiver::Update() {
    const auto& settings = settings_manager.settings;
    if (!ApplySettings(settings.gnss_enabled, settings.gnss_receiver_type) || !uart_ ||
        !settings.gnss_enabled || type_ != settings.gnss_receiver_type) return;
    if (!Read()) return;
    if (M10()) m10_.Update(get_time_since_boot_us());
    if (settings.gnss_notify) NotifyFix();
}

bool GNSSReceiver::Read() {
    const uint64_t now_us = get_time_since_boot_us();
    if (now_us - last_poll_us_ < poll_interval_us_) return false;
    const uintptr_t key = HwiP_disable();
    const uint64_t edge_us = last_pps_us;
    const bool overrun = rx_overrun;
    rx_overrun = false;
    HwiP_restore(key);
    char bytes[64];
    size_t count = 0;
    UART2_read(uart_, bytes, sizeof(bytes), &count);
    if (overrun || (count && last_poll_us_ && now_us - last_poll_us_ >= 250000)) {
        // Queued data after a main-loop stall can belong to an older PPS.
        UART2_flushRx(uart_);
        navigation_ = {};
        m10_.Resync();
        count = 0;
    }
    last_poll_us_ = count == sizeof(bytes) ? now_us - poll_interval_us_ : now_us;
    for (size_t i = 0; i < count; ++i) {
        if (!M10() || m10_.Ingest(bytes[i], now_us / 1000)) {
            navigation_.Ingest(bytes[i], now_us, last_drain_us_, edge_us);
        }
    }
    if (count < sizeof(bytes)) last_drain_us_ = now_us;
    return true;
}

bool GNSSReceiver::Service() {
    if (!service_during_poll_() || !uart_) return false;
    Read();
    return true;
}

bool GNSSReceiver::SendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t length) {
    uint8_t frame[kMaxTxFrameSize];
    if (!M10() || tx_active || length > sizeof(frame) - 8) return false;
    size_t size = 0;
    WriteUbxFrame(cls, id, payload, length, [&](uint8_t byte) { frame[size++] = byte; });
    size_t written = 0;
    // A stale EOT would make the UART ISR wait for an entire frame with interrupts masked.
    UARTIntClear(UART1_BASE, UART_INT_EOT);
    return UART2_write(uart_, frame, size, &written) == UART2_STATUS_SUCCESS && written == size;
}

void GNSSReceiver::NotifyFix() {
    const bool valid = HasValidFix();
    const uint64_t now_us = get_time_since_boot_us();
    if (valid == notified_fix_ || now_us < next_notification_us_) return;
    char utc[9] = "--:--:--";
    uint64_t utc_ms;
    if (GetFixUTCUnixMs(utc_ms)) FormatUTC(utc_ms, "%H:%M:%S", utc, sizeof(utc));
    const auto& f = fix();
    CONSOLE_INFO("GNSSReceiver::Update", "GNSS_FIX=%d,%.6f,%.6f,%ld,%.1f,%ld,%u,%s,%lu", valid,
                 f.latitude_deg, f.longitude_deg, static_cast<long>(f.altitude_ft), f.heading_deg,
                 static_cast<long>(f.speed_kts), f.num_satellites, utc, static_cast<unsigned long>(pps_count()));
    notified_fix_ = valid;
    next_notification_us_ = now_us + 1000000;
}

uint32_t GNSSReceiver::pps_count() const { return pulse_count; }

bool GNSSReceiver::IsHealthy() const {
    return gnss == this && (type_ == SettingsManager::kGNSSReceiverGeneric || m10_.IsHealthy());
}

bool GNSSReceiver::HasValidFix() const {
    return gnss == this && fix().IsFresh(get_time_since_boot_us() / 1000);
}

bool GNSSReceiver::GetFixUTCUnixMs(uint64_t& utc_ms) const {
    return IsHealthy() && navigation_.GetFixUTCUnixMs(get_time_since_boot_us(), utc_ms);
}

const NMEAParser::GNSSFix* GNSSReceiver::GetFix() const {
    return IsHealthy() ? navigation_.GetFix(get_time_since_boot_us()) : nullptr;
}

bool GNSSReceiver::GetUTCUnixMs(uint64_t& utc_ms) const {
    return IsHealthy() && navigation_.GetUTCUnixMs(get_time_since_boot_us(), utc_ms);
}
