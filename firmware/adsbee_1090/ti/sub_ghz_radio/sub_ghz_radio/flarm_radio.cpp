#include "flarm_radio.hh"
#include "bsp.hh"
#include "hal.hh"
#include "object_dictionary.hh"
#include "smartrf/smartrf_settings.h"
#include "flarm_transport.hh"
extern "C" {
#include <ti/drivers/GPIO.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/rf_data_entry.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/rf_prop_mailbox.h>
}
namespace {
constexpr unsigned kEntryBytes = (sizeof(rfc_dataEntryGeneral_t) + 57 + 3) & ~3u;
alignas(4) uint8_t entries[4][kEntryBytes] = {};
dataQueue_t queue = {};
rfc_dataEntryGeneral_t* read_entry;
uint32_t sequence = 0;
FlarmPhaseEstimator phase_estimator;
void Receive(RF_Handle, RF_CmdHandle, RF_EventMask events) {
    if (!(events & RF_EventRxEntryDone)) return;
    for (unsigned i = 0; i < 4 && read_entry->status == DATA_ENTRY_FINISHED; ++i) {
        const uint8_t* bytes = &read_entry->data;
        if (bytes[0] == 56) {
            FlarmRadioPacket packet;
            if (DecodeFlarmRadioFrame(bytes + 1, 54, packet.payload)) {
                packet.sequence = ++sequence;
                packet.age_ms = get_time_since_boot_ms();
                packet.rssi_dbm = static_cast<int8_t>(bytes[55]);
                uintptr_t key = HwiP_disable();
                phase_estimator.Record(packet.age_ms, subg_radio.channel());
                if (!object_dictionary.flarm_packet_queue.Enqueue(packet)) ++object_dictionary.flarm_packets_dropped;
                HwiP_restore(key);
            }
        }
        read_entry->status = DATA_ENTRY_PENDING;
        read_entry = reinterpret_cast<rfc_dataEntryGeneral_t*>(read_entry->pNextEntry);
    }
}
}

void SubGHzRadio::SetBiasTeeEnable(bool enabled) { GPIO_write(bsp.kSubGBiasTeeEnablePin, !enabled); }
bool SubGHzRadio::Init() {
    for (unsigned i = 0; i < 4; ++i) {
        auto* e = reinterpret_cast<rfc_dataEntryGeneral_t*>(entries[i]);
        e->pNextEntry = entries[(i + 1) % 4];
        e->status = DATA_ENTRY_PENDING;
        e->config.type = DATA_ENTRY_TYPE_GEN;
        e->config.lenSz = 1;
        e->length = 57;
    }
    queue.pCurrEntry = entries[0];
    read_entry = reinterpret_cast<rfc_dataEntryGeneral_t*>(entries[0]);
    RF_cmdPropRxAdv.pQueue = &queue;
    RF_Params params;
    RF_Params_init(&params);
    rf_handle_ = RF_open(&rf_object_, &RF_prop, reinterpret_cast<RF_RadioSetup*>(&RF_cmdPropRadioDivSetup), &params);
    return rf_handle_ && (!rx_enabled || Start(1));
}
bool SubGHzRadio::Start(uint16_t channel) {
    if (rx_command_ >= 0) {
        RF_cancelCmd(rf_handle_, rx_command_, 0);
        RF_pendCmd(rf_handle_, rx_command_, RF_EventLastCmdDone);
        rx_command_ = RF_ALLOC_ERROR;
    }
    // Aborted packets leave unfinished queue entries; discard them only after RF has stopped.
    for (auto& bytes : entries) reinterpret_cast<rfc_dataEntryGeneral_t*>(bytes)->status = DATA_ENTRY_PENDING;
    queue.pCurrEntry = entries[0];
    read_entry = reinterpret_cast<rfc_dataEntryGeneral_t*>(entries[0]);
    RF_cmdFs.fractFreq = channel == 1 ? 13107 : 26214;
    if (RF_runCmd(rf_handle_, reinterpret_cast<RF_Op*>(&RF_cmdFs), RF_PriorityNormal, nullptr, 0) != RF_EventLastCmdDone ||
        RF_cmdFs.status != DONE_OK) return false;
    RF_cmdPropRxAdv.status = 0;
    rx_command_ = RF_postCmd(rf_handle_, reinterpret_cast<RF_Op*>(&RF_cmdPropRxAdv), RF_PriorityNormal, Receive,
                            RF_EventRxEntryDone | RF_EventLastCmdDone);
    channel_ = channel;
    return rx_command_ >= 0;
}
bool SubGHzRadio::Update() {
    if (!rf_handle_) return false;
    if (!rx_enabled) {
        if (rx_command_ >= 0) {
            RF_cancelCmd(rf_handle_, rx_command_, 0);
            RF_pendCmd(rf_handle_, rx_command_, RF_EventLastCmdDone);
            rx_command_ = RF_ALLOC_ERROR;
        }
        return true;
    }
    uintptr_t key = HwiP_disable();
    uint16_t channel = phase_estimator.Channel(get_time_since_boot_ms());
    HwiP_restore(key);
    uint16_t status = reinterpret_cast<volatile RF_Op*>(&RF_cmdPropRxAdv)->status;
    if (rx_command_ < 0 || channel != channel_ || (status & 0xF000) == 0x3000) return Start(channel);
    return true;
}
bool SubGHzRadio::DeInit() {
    if (!rf_handle_) return true;
    rx_enabled = false;
    Update();
    RF_close(rf_handle_);
    rf_handle_ = nullptr;
    return true;
}
