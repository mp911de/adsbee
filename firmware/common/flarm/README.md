# FLARM local safety feed

Receive-only European FAMP v3, ported from Mark Paluch's rtl_433 `flarm.patch`.
Core algorithms retain their GPL-2.0-or-later notices. The accompanying
[FFPP license](FLARM_FFPP_LICENSE.md) applies to protocol use and must accompany
source and firmware distributions. FAMP specification: FTD-116 v1.02, sections 2-4.

CC1312 validates complete Manchester frames and RF CRC. RP2040 decrypts with
fresh GNSS UTC, resolves compressed positions and emits local CSBEE records.
FLARM targets never enter the shared aircraft dictionary. `#F` records are emitted
in the CSBEE round after the `#A`/`#U` records, on every interface with CSBEE enabled.

## CSBEE #F version 1

The identity contract below is approved for the unshipped v1; serializer and tracker
changes are pending implementation.

One ASCII line, commas, CRLF, existing CSBEE CRC16 over `#` through the comma
preceding CRC. CRC is uppercase hexadecimal, 1-4 digits. Missing fields are empty.
The first `1` is the record version; VER is the sender's FAMP version.
Only FAMP version 3 packets are decoded; other versions are dropped.

```text
#F:1,ID,ID_TYPE,FLAGS,TYPE,LAT,LON,HAE_M,TRACK_DEG,SPEED_MPS,CLIMB_MPS,TURN_DPS,MOV,URGENCY,HACC_M,VACC_M,SACC_MPS,SIL,SDA,NIC,FIX_UTC,RX_AGE_MS,VER,VER_MAX,RSSI,FPS,CRC\r\n
```

| Field | Meaning |
|---|---|
| ID_TYPE / ID | 0 ICAO, 1 locally assigned; ID is always 6 hex digits |
| FLAGS | Hexadecimal: bit 0 stealth, bit 1 no-track; other bits reserved |
| TYPE | Native FAMP aircraft type, not ADS-B emitter category |
| LAT / LON | Reconstructed WGS-84 degrees, 7 decimals; empty without a valid reference or geographically valid reconstruction |
| HAE_M | Integer meters above WGS-84 ellipsoid, not MSL |
| TRACK_DEG | True ground track, 1 decimal |
| SPEED_MPS / CLIMB_MPS | m/s, 1 decimal, positive climb upward |
| TURN_DPS | Signed degrees/second, 2 decimals |
| MOV | 0 unknown, 1 ground, 2 flying, 3 circling/maneuvering |
| URGENCY | 0 normal, 1 high, 2 Pan-Pan, 3 Mayday |
| HACC_M / VACC_M / SACC_MPS | Native 95% accuracy estimates, 1 / 2 / 1 decimals |
| SIL / SDA / NIC | Native FAMP integrity values, not synthesized ADS-B accuracy categories |
| FIX_UTC | Unix seconds of the navigation fix |
| RX_AGE_MS | Milliseconds since last accepted radio reception, not last serial report |
| VER / VER_MAX | Sender FAMP version / maximum receive version; VER_MAX=15 means receive-incapable |
| RSSI / FPS | Received dBm / valid frames since previous report |

RP2040 preserves explicit ICAO addresses and assigns other targets IDs from
`F09000` through `F093FF`. An unambiguous current/next EPRA match preserves the
same ID across rotations. Native EPRA addresses stay internal. UART and GDL90
consume the same resolved identity; consumers do not reconstruct EPRA continuity.

Records are emitted once per CSBEE round, before the `#S` footer. The 64-entry local table
expires targets after 5 seconds; when full, the oldest reception is replaced.
No-track and stealth packets remain available for local conflict detection.
Consumers must keep no-track data out of persistent logs and external feeds and
apply display privacy to identifiers and performance details.

UTC derives from dated RMC sentences, with 5-second freshness and no PPS discipline.
Without UTC the RP2040 discards packets until time becomes available.

The CC1312 starts on 868.2 MHz and infers the 1-second cycle from its own reception
times. FTD-116 2.2: Frame 1 on 868.2 MHz spans 450-820 ms, Frame 2 on 868.4 MHz
830-1200 ms, 10 ms guard between them. The radio listens on 868.2 MHz from 325 to
825 ms and on 868.4 MHz otherwise. Once receptions of the last 30 seconds spread
over at least 300 ms of a 370 ms frame, it hops. A smaller spread, contradicting
timestamps or 30 seconds without traffic return it to fixed 868.2 MHz, so a wrong
phase heals itself. Every sender transmits on both channels each second, so the
fixed channel already yields 1 Hz updates; hopping adds the second packet.
Queue overflows on the CC1312 are counted in the batch header and logged by the RP2040.
PHY values and overrides come from TI SimpleLink CC1312R `setting_tc137.json`.
Physical reception, antenna/front-end suitability and dense-traffic UART capacity
require hardware validation.
