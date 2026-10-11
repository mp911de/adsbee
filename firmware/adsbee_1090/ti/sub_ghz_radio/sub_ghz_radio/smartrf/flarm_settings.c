// TI CC1312R setting_tc137: 100 ksymbol/s, 50 kHz deviation, 195.9 kHz RX bandwidth.
#include "smartrf_settings.h"
#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_prop.h)

RF_Mode RF_prop = {.rfMode = RF_MODE_AUTO, .cpePatchFxn = &rf_patch_cpe_prop};
uint32_t pOverrides[] = {
    0x00F788D3,
    0x4001405D,
    0x08141131,
    0x0A480583,
    0x000005A3,
    0x7AB80603,
    0x00000623,
    0x000C84A3,
    0x000B84B3,
    ADI_2HALFREG_OVERRIDE(0,16,0x8,0x8,17,0x1,0x1),
    HW_REG_OVERRIDE(0x6028,0x001A),
    HW_REG_OVERRIDE(0x609C,0x001A),
    0x000188A3,
    ADI_HALFREG_OVERRIDE(0,61,0xF,0xA),
    ADI_REG_OVERRIDE(0,12,0xF8),
    0xFFFFFFFF
};
rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup = {
    .commandNo = CMD_PROP_RADIO_DIV_SETUP,
    .status = 0,
    .pNextOp = 0,
    .startTime = 0,
    .startTrigger.triggerType = 0,
    .startTrigger.bEnaCmd = 0,
    .startTrigger.triggerNo = 0,
    .startTrigger.pastTrig = 0,
    .condition.rule = 1,
    .condition.nSkip = 0,
    .modulation.modType = 1,
    .modulation.deviation = 200,
    .symbolRate.preScale = 15,
    .symbolRate.rateWord = 65536,
    .symbolRate.decimMode = 0,
    .rxBw = 86,
    .preamConf.nPreamBytes = 0,
    .preamConf.preamMode = 0,
    .formatConf.nSwBits = 32,
    .formatConf.bBitReversal = 0,
    .formatConf.bMsbFirst = 1,
    .formatConf.fecMode = 0,
    .formatConf.whitenMode = 0,
    .config.frontEndMode = 0,
    .config.biasMode = 1,
    .config.analogCfgMode = 0,
    .config.bNoFsPowerUp = 0,
    .txPower = 0x9F3F,
    .pRegOverride = pOverrides,
    .centerFreq = 868,
    .intFreq = 0x8CC,
    .loDivider = 5,
};
rfc_CMD_FS_t RF_cmdFs = {
    .commandNo = CMD_FS, .condition.rule = 1,
    .frequency = 868, .fractFreq = 13107,
};
rfc_CMD_PROP_RX_ADV_t RF_cmdPropRxAdv = {
    .commandNo = CMD_PROP_RX_ADV, .condition.rule = 1,
    .pktConf.bRepeatOk = 1, .pktConf.bRepeatNok = 1,
    .rxConf.bAppendRssi = 1, .rxConf.bAppendStatus = 1,
    .syncWord0 = 0xA5A95566, .syncWord1 = 0x5A56AA99,
    .maxPktLen = 54, .endTrigger.triggerType = TRIG_NEVER,
};
