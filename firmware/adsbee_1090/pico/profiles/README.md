# Build profiles

From the repository root:

```sh
ADSBEE_PROFILE=airgrep bash firmware/build.sh adsbee_1090
ADSBEE_PROFILE=airgrep bash firmware/build.sh adsbee_1090 test
ADSBEE_PROFILE=airgrep bash firmware/build.sh adsbee_1090 flash
```

| Profile | COMMS_UART | UART AT console | RP2040 release output |
|---|---|---|---|
| `default` (unset) | 115200 8N1 | disabled | `pico/build/Release/` |
| `airgrep` | 115200 8E1 | enabled | `pico/build/Release-airgrep/` |

Both use 3.3 V and no flow control. GNSS and USB are unchanged.
Debug and test builds also have separate profile directories. Flash uses the
selected profile's image. CC1312 builds also use the profile and have separate
outputs: `ti/sub_ghz_radio/build/Release-airgrep/` (or `Debug-airgrep`). The default
keeps `ti/sub_ghz_radio/build/`. RP2040 embeds the selected CC1312 binary.
ESP32 binaries remain shared. `AT+DEVICE_INFO?` identifies the RP2040 profile.

Edit `airgrep.cmake` for carrier settings. Add another named `.cmake` file for a
new profile. `profile.cmake` provides upstream defaults before applying it.
The environment is read on every CMake configure, avoiding stale profile values.

`ADSBEE_SUBGHZ_PROTOCOL` selects the CC1312 radio and decoder together. `default`
uses `UAT`; `airgrep` uses receive-only European FAMP at 868.2/868.4 MHz.
RP2040 decodes FAMP using fresh GNSS UTC and reconstructs coordinates from its GNSS
position. Without UTC, FLARM packets are discarded. Without a position fix, `#F`
coordinates are empty. Reception on the 1090U RF hardware needs bench validation.

FLARM targets are held in a separate 64-entry RAM table and expire after 60
seconds without reception. `#F` records follow `#A`/`#U` on every interface with
CSBEE enabled (`AT+PROTOCOL_OUT=<iface>,CSBEE`), including USB and the network
console. The onboard ESP32 also reports eligible FLARM positions through local
Wi-Fi GDL90. Remote feeds receive no FLARM target data. Stealth/no-track flags remain present for local collision detection.
The display must enforce privacy for identifiers and performance information.

The `#F` v1 contract is in [common/flarm/README.md](../../../common/flarm/README.md).
The [FFPP license](../../../common/flarm/FLARM_FFPP_LICENSE.md) must accompany
firmware distributions containing this decoder. No transmitter is implemented.

Profiles do not overwrite saved settings. Provision the carrier once through USB:

```text
AT+BAUD_RATE=COMMS_UART,115200
AT+PROTOCOL_OUT=COMMS_UART,CSBEE
AT+GNSS=1,GENERIC
AT+SETTINGS=SAVE
```

Use `UBX_MIA` instead of `GENERIC` for that GNSS module. Check `AT+GNSS_FIX?`
for a fresh fix and a non-empty UTC epoch field before expecting FLARM reports.

On the airgrep link, send LF or CRLF and keep one command outstanding. Replies
return over UART. CSBEE traffic starts with `#` and carries CRC16, which the
host must verify. UART receive errors discard the command through a clean LF.
Query state before retrying a missing or corrupt reply. The link carries at most
10472 bytes/s; dense-traffic throughput still needs hardware verification.

Keep carrier policy here. Generic UART console changes still touch upstream
communications code and may need reconciliation when updating upstream.

Run profile checks with `python3 firmware/adsbee_1090/pico/profiles/test_profiles.py`.
