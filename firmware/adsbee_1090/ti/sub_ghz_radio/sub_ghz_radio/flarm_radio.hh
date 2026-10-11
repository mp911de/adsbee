#pragma once
#include <cstdint>
extern "C" {
#include <ti/drivers/rf/RF.h>
}
class SubGHzRadio {
public:
    struct SubGHzRadioConfig {};
    explicit SubGHzRadio(SubGHzRadioConfig) {}
    bool Init();
    bool DeInit();
    bool Update();
    void SetBiasTeeEnable(bool enabled);
    uint16_t channel() const { return channel_; }
    bool rx_enabled = true;
private:
    RF_Object rf_object_ = {};
    RF_Handle rf_handle_ = nullptr;
    RF_CmdHandle rx_command_ = RF_ALLOC_ERROR;
    uint16_t channel_ = 0;
    bool Start(uint16_t channel);
};
extern SubGHzRadio subg_radio;
