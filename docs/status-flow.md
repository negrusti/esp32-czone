# Button ⇄ relay flow (PGNs and IDs)

How an MFD button press drives a relay, and how the relay state lights the
button back up. Two independent directions; both must agree on **which output**
a circuit refers to.

## IDs and numbering

| Term | Meaning |
| --- | --- |
| **Dipswitch** | This module's address (`0x18` = 24). High byte of every `channelAddress` aimed at us. |
| **Circuit ID** | A logical circuit in the ZCF (e.g. "Nav Lights"). What a button sends/represents. |
| **channelAddress** | `(dipswitch << 8) \| outputChannel`. The ZCF links a circuit to one of these. |
| **outputChannel** | Low byte of `channelAddress`. The hardware output index on the module. For a Control 1 these are `0–3` and `12–15`. |
| **DC number** | The label the tool/MFD shows (`DC1…DC8`) and our physical relay number. Derived from `outputChannel` (see below). |

**DC ⇄ channel mapping** (module type 28 / Control 1, from the tool's
`GetChannelString`):

| outputChannel | 12 | 13 | 14 | 15 | 0 | 1 | 2 | 3 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **DC / relay** | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |

`relay_for_channel()` does channel→DC; `zcf_config_channel_for_relay()` does the
inverse.

## Direction 1 — button press → relay (control)

```
MFD button (Circuit ID)
   │  PGN 65280  CZone SS Command
   │  payload: switch_id = Circuit ID, command = 0xF1/0xF2 (press) / 0x40 (release)
   ▼
firmware: zcf_config_relays_for_circuit(Circuit ID)
   │  looks up the circuit in the parsed ZCF, keeps outputs whose
   │  channelAddress high byte == our dipswitch, returns relay = DC number
   ▼
relay_controller_toggle_channel(relay)  →  physical relay switches
```

The ZCF mapping is applied **here**, once: Circuit ID → channelAddress → relay
(= DC number).

## Direction 2 — relay state → button highlight (status)

`czone_protocol_publish_relay_state(relay_mask)` broadcasts on every change. The
PGN the MFD uses to light buttons is **65284**:

```
relay_mask (bit i = relay i+1 = DC i+1)
   │  relay_mask_to_channel_bits():  for each relay on, set bit[outputChannel]
   ▼
PGN 65284  CZone Switch Bank Status
   bytes[0:1] vendor header   byte[2] dipswitch   byte[3] module type
   bytes[4:7] 32 switch bits  →  switch[outputChannel] = on/off
```

The MFD then: button → Circuit ID → channelAddress → find module by dipswitch →
read `switch[outputChannel]`. Because we keyed the bits by **outputChannel**, the
right button reflects the right relay.

Also sent, carrying the same state:

| PGN | Name | Notes |
| --- | --- | --- |
| 65284 | CZone Switch Bank Status | Drives MFD button state. Keyed by outputChannel. |
| 127501 | NMEA Binary Switch Bank Status | Same state, keyed by outputChannel; channels we don't drive = `unavailable`. |
| 130817 | CZone OI Status | One record per DC output (DC1…DC8 order). |
