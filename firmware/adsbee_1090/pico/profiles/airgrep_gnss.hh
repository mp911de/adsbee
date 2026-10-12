#pragma once

#include <cstdint>

namespace airgrep {

struct GNSSMessageRate {
    uint32_t key;
    uint8_t rate;
};

// UART1 output selection for M10 SPG 5.10. Rejected disabled outputs are optional.
inline constexpr GNSSMessageRate kGNSSMessages[] = {
    {0x209100a7, 0},  // NMEA_ID_DTM
    {0x209100de, 0},  // NMEA_ID_GBS
    {0x209100bb, 1},  // NMEA_ID_GGA
    {0x209100ca, 0},  // NMEA_ID_GLL
    {0x209100b6, 0},  // NMEA_ID_GNS
    {0x209100cf, 0},  // NMEA_ID_GRS
    {0x209100c0, 0},  // NMEA_ID_GSA
    {0x209100d4, 0},  // NMEA_ID_GST
    {0x209100c5, 0},  // NMEA_ID_GSV
    {0x20910401, 0},  // NMEA_ID_RLM
    {0x209100ac, 1},  // NMEA_ID_RMC
    {0x209100e8, 0},  // NMEA_ID_VLW
    {0x209100b1, 0},  // NMEA_ID_VTG
    {0x209100d9, 0},  // NMEA_ID_ZDA
    {0x209100ed, 0},  // PUBX_ID_POLYP
    {0x209100f2, 0},  // PUBX_ID_POLYS
    {0x209100f7, 0},  // PUBX_ID_POLYT
    {0x20910350, 0},  // UBX_MON_COMMS
    {0x209101ba, 0},  // UBX_MON_HW2
    {0x20910355, 0},  // UBX_MON_HW3
    {0x209101b5, 0},  // UBX_MON_HW
    {0x209101a6, 0},  // UBX_MON_IO
    {0x20910197, 0},  // UBX_MON_MSGPP
    {0x2091035a, 0},  // UBX_MON_RF
    {0x209101a1, 0},  // UBX_MON_RXBUF
    {0x20910188, 0},  // UBX_MON_RXR
    {0x2091038c, 0},  // UBX_MON_SPAN
    {0x2091019c, 0},  // UBX_MON_TXBUF
    {0x2091007a, 0},  // UBX_NAV_AOPSTATUS
    {0x20910066, 0},  // UBX_NAV_CLOCK
    {0x20910084, 0},  // UBX_NAV_COV
    {0x20910039, 0},  // UBX_NAV_DOP
    {0x20910160, 0},  // UBX_NAV_EOE
    {0x2091007f, 0},  // UBX_NAV_ODO
    {0x20910011, 0},  // UBX_NAV_ORB
    {0x20910416, 0},  // UBX_NAV_PL
    {0x20910025, 0},  // UBX_NAV_POSECEF
    {0x2091002a, 0},  // UBX_NAV_POSLLH
    {0x20910007, 0},  // UBX_NAV_PVT
    {0x20910016, 0},  // UBX_NAV_SAT
    {0x2091006b, 0},  // UBX_NAV_SBAS
    {0x20910346, 0},  // UBX_NAV_SIG
    {0x20910337, 0},  // UBX_NAV_SLAS
    {0x2091001b, 0},  // UBX_NAV_STATUS
    {0x20910052, 0},  // UBX_NAV_TIMEBDS
    {0x20910057, 0},  // UBX_NAV_TIMEGAL
    {0x2091004d, 0},  // UBX_NAV_TIMEGLO
    {0x20910048, 0},  // UBX_NAV_TIMEGPS
    {0x20910061, 0},  // UBX_NAV_TIMELS
    {0x20910387, 0},  // UBX_NAV_TIMEQZSS
    {0x2091005c, 0},  // UBX_NAV_TIMEUTC
    {0x2091003e, 0},  // UBX_NAV_VELECEF
    {0x20910043, 0},  // UBX_NAV_VELNED
    {0x20910644, 0},  // UBX_RXM_MEAS20
    {0x20910649, 0},  // UBX_RXM_MEAS50
    {0x2091063f, 0},  // UBX_RXM_MEASC12
    {0x2091063a, 0},  // UBX_RXM_MEASD12
    {0x20910205, 0},  // UBX_RXM_MEASX
    {0x2091025f, 0},  // UBX_RXM_RLM
    {0x20910232, 0},  // UBX_RXM_SFRBX
    {0x20910179, 0},  // UBX_TIM_TM2
    {0x2091017e, 0},  // UBX_TIM_TP
    {0x20910093, 0},  // UBX_TIM_VRFY
    {0x20920002, 0},  // INFMSG-UBX_UART1
    {0x20920007, 0},  // INFMSG-NMEA_UART1
};

}  // namespace airgrep
