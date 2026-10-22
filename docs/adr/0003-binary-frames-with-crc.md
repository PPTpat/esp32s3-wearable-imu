# ADR 0003: Compact binary frames with sequence numbers and CRC

## Status
Accepted

## Context
The stream must survive lost notifications and corrupted data, run on a small
MTU budget and be easy to decode on a phone or PC. Gaps must be attributable:
lost in the radio link, or dropped inside the firmware.

## Decision
- Fixed little-endian binary layout, version byte, CRC-16/CCITT-FALSE per frame.
- Two counters: `frame_seq` (what the link delivered) and `first_sample_seq`
  (what the sensor produced). Comparing them tells where data was lost.
- Samples carry no per-sample timestamp; time is `seq / ODR`, with the rate
  published once in a read-only Info characteristic.
- Up to 16 samples per frame (204 bytes), batched every 50 ms to keep radio
  overhead low.
- The wire format is implemented once in a host-testable component and
  cross-checked against an independent Python decoder using a shared golden frame.

## Consequences
- Time between samples is exact only if the sensor ODR is exact; hardware
  timestamps can be added in protocol version 2.
- Changing the layout requires bumping `version` and updating both decoders.