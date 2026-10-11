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
selected profile's image. ESP32 and CC1312 binaries are shared because these
profiles only change RP2040 behavior. `AT+DEVICE_INFO?` identifies the profile.

Edit `airgrep.cmake` for carrier settings. Add another named `.cmake` file for a
new profile. `profile.cmake` provides upstream defaults before applying it.
The environment is read on every CMake configure, avoiding stale profile values.

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
