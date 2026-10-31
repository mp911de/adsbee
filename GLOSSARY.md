# ADSBee

## Language

**AIRGREP telemetry**:
CSBee aircraft and receiver reports extended with GNSS diagnostic snapshots and events.

**Antenna mode**:
The user's declaration of the GNSS antenna: unknown, passive, active, or active supervised.

**Antenna supervisor**:
The M10 feature that switches the active antenna supply and reports an open or short circuit on it. Only meaningful in active supervised mode.

**GNSS diagnostic snapshot**:
An M10 RF status report collected and forwarded every five seconds.

**GNSS diagnostic event**:
A warning or error received from the M10. Each occurrence is distinct, even when its text repeats.

**Pending GNSS diagnostic event**:
The latest unforwarded event for one severity, warning or error. A newer event replaces it, and forwarding clears it without waiting for the next snapshot.
