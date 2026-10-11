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

`ADSBEE_SUBGHZ_PROTOCOL` selects the CC1312 radio and decoder together. Both
profiles currently use `UAT`. `FLARM` is reserved and fails the build until its
radio setup and decoder are implemented. Set it in `airgrep.cmake` when ready.
The corresponding C++ constants are in `receiver_config.hh`.

Profiles do not overwrite saved settings. Provision the carrier once through USB:

```text
AT+BAUD_RATE=COMMS_UART,115200
AT+PROTOCOL_OUT=COMMS_UART,CSBEE
AT+SETTINGS=SAVE
```

On the airgrep link, send LF or CRLF and keep one command outstanding. Replies
return over UART. CSBEE traffic starts with `#` and carries CRC16, which the
host must verify. UART receive errors discard the command through a clean LF.
Query state before retrying a missing or corrupt reply. The link carries at most
10472 bytes/s; dense-traffic throughput still needs hardware verification.

Keep carrier policy here. Generic UART console changes still touch upstream
communications code and may need reconciliation when updating upstream.

Run profile checks with `python3 firmware/adsbee_1090/pico/profiles/test_profiles.py`.
