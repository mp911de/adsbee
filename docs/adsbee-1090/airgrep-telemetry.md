# AIRGREP telemetry protocol

Wire contract for ADSBee AIRGREP serial telemetry.

## Selection and configuration

`AT+PROTOCOL_OUT=COMMS_UART,AIRGREP` selects existing CSBee reports plus the
GNSS frames defined here. Selecting `CSBEE` preserves its existing wire format
and emits no GNSS diagnostic frames. Console output can independently select
AIRGREP through the same command.

Extend `AT+GNSS=<enabled>[,<type>[,<notify>[,<diagnostics>[,<antenna>]]]]` with a
persisted diagnostics flag, default off, and the antenna mode. An omitted argument
preserves its stored value. `AT+GNSS?` includes the flag as its fourth result and the
antenna mode as its fifth. Saving settings uses the existing settings-save command.

The M10 streams MON-RF and NAV-SBAS every fifth navigation epoch in every profile,
so antenna and SBAS status are always current. Enabling diagnostics forwards MON-RF
as MON frames and enables UBX-INF-ERROR and UBX-INF-WARNING. Errors and warnings
are forwarded on the next available main-loop pass, without waiting for MON-RF.
NOTICE is outside this message set. Disabling diagnostics retains normal AIRGREP
CSBee output. Network feeds do not support AIRGREP.

`antStatus` and `antPower` in MON frames report the antenna supervisor, which only
runs with antenna mode `ACTIVE_SUPERVISED` on a carrier wired for it. Other modes
report `DONTKNOW`.

`AT+GNSS_STATUS?` returns one line for parsers, from stored records only:

```text
GNSS_STATUS=<module>,<software>,<hardware>,<hdop>,<antenna>,<antenna_power>,<max_cno>,<seen>,<no_signal>,<searching>,<acquired>,<unusable>,<locked>,<ephemeris>,<used>,<healthy>,<unhealthy>,<sbas_mode>,<sbas_system>,<sbas_sats>,<prc_cm>,<ic_cm>
```

- `antenna` is `INIT`, `DONTKNOW`, `OK`, `SHORT` or `OPEN`. `antenna_power` is `OFF`, `ON` or `DONTKNOW`.
- `max_cno` is the best C/N0 in dBHz. `seen` to `unhealthy` count satellites.
- `no_signal`, `searching`, `acquired`, `unusable` and `locked` are the signal quality states; each
  satellite is in one.
- `prc_cm` and `ic_cm` are the first corrected satellite's pseudorange and ionosphere corrections.
- A value that is missing or older than 15 s reads `-1` or `UNKNOWN`.

`AT+GNSS_DIAG?` prints the latest RF and SBAS records
with their age and polls MON-COMMS and MON-HW3 from the module, one labeled line
each, for people rather than parsers.

```text
AT+PROTOCOL_OUT=COMMS_UART,AIRGREP
AT+GNSS=1,UBX_MIA,0,1
AT+SETTINGS=SAVE
```

## GNSS frame (airgrep profile only)

The airgrep build pushes the receiver state once per second to every AIRGREP
output, so a host does not poll `AT+GNSS_FIX?` and `AT+GNSS_STATUS?`. It is
independent of the diagnostics flag and is sent while no AT command is active.

```text
#GNSS:<fix>,<quality>,<sats>,<lat>,<lon>,<alt>,<speed>,<track>,<hdop>,<utc>,<epoch>,<antenna>,<antenna_power>,<jamming>,<max_cno>,<seen>,<no_signal>,<searching>,<acquired>,<unusable>,<locked>,<ephemeris>,<used>,<healthy>,<unhealthy>,<sbas_mode>,<sbas_system>,<sbas_sats>,<prc_cm>,<ic_cm>,<module>,<software>,<hardware>,<bbr>,<CRC16>\r\n
```

The fields from `max_cno` on carry the `AT+GNSS_STATUS?` content with the same
names, values and strings, except that a missing or stale value is an empty
field instead of `-1` or `UNKNOWN`. Text fields never contain a comma; a comma
in a module string is replaced by a space. The longest frame is 272 wire bytes,
the sender buffer holds 320.

### Time ping (airgrep profile only)

`AT+TIME_PING=<echo>` measures link latency and clock offset. `echo` is an
opaque token of 1 to 20 decimal digits chosen by the host, typically its send
timestamp. The reply is one line, then `OK`:

```text
TIME_PING=<echo>,<epoch_ms>,<pps_count>\r\n
OK\r\n
```

`echo` returns unchanged. `epoch_ms` is the PPS disciplined UTC unix time in
milliseconds when the reply was formatted, the same clock as the `#GNSS`
`epoch` field, empty when unavailable. `pps_count` is the PPS edge count as in
`AT+GNSS_FIX?`, `0` without GNSS. The handler reads no bus and logs nothing, so
the reply delay after the command is parsed is constant. `AT+TIME_PING?`,
a bare `AT+TIME_PING`, or a token with other characters return `ERROR`.

| Field | Contract |
|---|---|
| `fix` | `1` when the fix is valid and fresh, as `AT+GNSS_FIX?` reports it, else `0`. |
| `quality` | GGA fix quality, `0` no fix, `1` GPS, `2` DGPS/SBAS. |
| `sats` | Satellites used in the fix. |
| `lat`, `lon` | WGS84 degrees, six decimals, north and east positive. Empty unless `fix` is `1`. |
| `alt` | MSL altitude in feet. Empty unless `fix` is `1`. |
| `speed` | Ground speed in knots. Empty unless `fix` is `1`. |
| `track` | Course over ground in degrees true, one decimal. Empty unless `fix` is `1`. |
| `hdop` | Two decimals. Empty when GGA did not report it. |
| `utc` | `HHMMSS` of the last NMEA time. Empty until a time is received. |
| `epoch` | UTC unix milliseconds, PPS disciplined when available. Empty without a dated RMC. |
| `antenna`, `antenna_power`, `jamming` | UBX-MON-RF codes as in `MON` frames. Empty when the RF record is missing or older than 15 s. |
| `max_cno` to `unhealthy` | As in `AT+GNSS_STATUS?`. Empty when the NAV-SAT record is missing or older than 15 s. |
| `sbas_mode`, `sbas_system`, `sbas_sats`, `prc_cm`, `ic_cm` | As in `AT+GNSS_STATUS?`. All empty when NAV-SBAS is missing or stale; `prc_cm` and `ic_cm` also empty without a corrected satellite. |
| `module`, `software`, `hardware` | UBX-MON-VER strings, e.g. `MAX-M10S`, `ROM SPG 5.10 (7b202e)`. Empty until the module answered. |
| `bbr` | `OK` when BBR held the SBAS configuration at init, else empty. |
| `CRC16` | As in `#M10`, over `#` through the last comma. |

```text
#GNSS:0,0,0,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,2695\r\n
#GNSS:1,2,11,50.500000,-8.250000,1234,87,271.5,0.90,070509,1762588800123,2,1,1,47,24,5,4,3,2,10,12,11,13,1,ENABLED,EGNOS,2,-312,45,MAX-M10S,ROM SPG 5.10 (7b202e),00000A00,OK,<CRC16>\r\n
```

## Wire frame

```text
#M10:<length>:<kind>:<id>:<part>/<count>:<payload>,<CRC16>\r\n
```

| Field | Contract |
|---|---|
| `length` | Decimal transmitted payload byte count, 0 through 160, without leading zeros. |
| `kind` | `MON`, `ERR`, or `WARN`. |
| `id` | Two decimal digits, `00` through `63`. Increment per logical message and wrap to `00`. |
| `part` | Two decimal digits, `01` through `63`, identifying this fragment. |
| `count` | Two decimal digits, `01` through `63`, identifying the total fragment count. |
| `payload` | At most 160 bytes of the escaped logical payload. |
| `CRC16` | One to four uppercase hexadecimal digits, without leading zeros, as in CSBee. |
| `\r\n` | Two actual line-ending bytes, not their printed escape sequences. |

An unfragmented message uses `01/01`. Every fragment of one logical message has
the same kind, ID, and count. Part numbers increase from 1 through count.
The length excludes the prefix, headers, checksum delimiter, CRC, and line ending. No NUL byte is
transmitted.

The longest header is 23 bytes. With 160 payload bytes, `,`, four CRC digits,
and CRLF, the longest frame is **190 wire bytes**, or **191 bytes** in a
NUL-terminated receive buffer. This fits a 200-byte frame buffer.

## Payloads

`MON` contains one RF block as CSV, in this fixed order:

```text
version,blockId,jammingState,antStatus,antPower,postStatus,noisePerMS,agcCnt,cwSuppression,ofsI,magI,ofsQ,magQ
```

All fields are decimal integers, using signed values for `ofsI` and `ofsQ`.
`version` is the received UBX-MON-RF message version, currently zero.
The field meanings and units follow UBX-MON-RF. Each logical MON message contains
one block, so `nBlocks` is not repeated. The MAX-M10S reports its L1 block.

`ERR` and `WARN` contain the corresponding M10 ASCII text. Retain at most
256 source bytes per message, including an appended `[truncated]` marker when
the original exceeds that limit. The marker replaces the tail of the text.

### Escaping and fragmentation

Escape only CR as `%0D`, LF as `%0A`, `*` as `%2A`, and `%` as `%25`.
All other bytes remain unchanged. First escape the retained logical payload,
then compute its transmitted byte length, then split it into fragments of at most
160 transmitted payload bytes. Compute the fragment count from those escaped
bytes and enforce the 63-fragment limit before transmitting any fragment.
Reject an over-limit logical message rather than emitting an incomplete prefix.
Each fragment's length counts its transmitted payload bytes, including escapes.

After reassembly, decode `%HH` back to the original bytes. Fragment boundaries
may divide escape sequences or UTF-8 characters, so do not unescape or decode
individual fragments. The M10 specifies ASCII text. Future UTF-8 producers must
truncate at a character boundary; receivers validate completed UTF-8 before display
and discard invalid text.

## CRC

Checksum every transmitted byte from the initial `#` through the comma after the
payload, excluding the hexadecimal CRC and CRLF, exactly as in CSBee. Each fragment
is independently checksummed, including its kind, length, ID, part, and count.
The last comma separates the checksum; commas inside the payload remain unchanged.

Use the existing CSBee `CalculateCRC16` algorithm: polynomial `0x1021`, initial
value `0xFFFF`, no reflection, no final XOR, followed by a byte swap of the result.
Format that swapped result with `%X`, one to four uppercase hexadecimal digits.

## Examples

These examples include calculated CRCs. Printed `\r\n` represents actual CRLF.

```text
#M10:33:MON:00:01/01:0,0,1,2,1,0,50,4000,0,0,100,0,100,E6DB\r\n
```

The following two fragments reconstruct `Antenna short detected`. Short fragments
are valid; a sender normally fills fragments up to the payload limit.

```text
#M10:14:WARN:01:01/02:Antenna short ,FCF8\r\n
#M10:8:WARN:01:02/02:detected,77C\r\n
```

## Sender state and scheduling

Keep a latest pending warning and a latest pending error. A newer occurrence
replaces the pending event of its severity. Identical text received again is a
new occurrence, not a replay to suppress.

Freeze the active logical message until all its fragments have been submitted.
New arrivals update pending state, never the active fragments. Clear the emitted
event without clearing a newer replacement. With multiple AIRGREP outputs, submit
the active message to each selected output before releasing it.

One UART output owner serializes complete frames and complete AT responses.
CSBee frames and AT responses may run between M10 fragments. Two logical M10
messages must not have interleaved fragments on one output. Yield between
fragments to keep command handling responsive.

Validate incoming UBX checksums before accepting diagnostics. Disabling
diagnostics or restarting GNSS clears pending and active diagnostics. Delivery
is best effort, without acknowledgments or retransmission. Local transmission
failure drops the affected logical message.

## Receiver bounds and recovery

Keep at most one M10 assembly per input stream. Accept only valid headers,
payload lengths, CRCs, and consecutive fragment numbers. Reject a fragment whose
part exceeds count, or whose kind, ID, or count differs from its active assembly.
A valid first fragment starts a new assembly and replaces any unfinished one.
An invalid continuation discards the affected assembly. IDs are correlation
values, not persistent identities or evidence of delivery.

Bound the accumulated escaped payload independently of count at **768 bytes**,
and the decoded logical payload at **256 bytes**. A terminator, if needed, requires
one additional byte. The 63-fragment limit never authorizes allocation of 63
message buffers or storage proportional to an advertised fragment count.

With the current payload limits, a maximally escaped text message needs at most
five normally packed fragments. Reject oversized assemblies rather than growing
their storage. Discard an overlong wire line through its next newline, then
resume normal CSBee, AT, and M10 line dispatch.

Discard an incomplete assembly **two seconds after its first
fragment**, even if later fragments have arrived. Reset assembly state when the
link is reopened. A completed assembly is dispatched once and then cleared.

## References

- [Architecture decision](../adr/0001-airgrep-gnss-telemetry.md).
- [M10 SPG 5.10 interface description, UBX-MON-RF and UBX-INF](https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf).
- [Existing CRC implementation](../../firmware/common/utils/buffer_utils.cpp).
