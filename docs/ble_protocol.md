# BLE protocol (version 1)

Device name: `ESP32S3-IMU`. All multi-byte values are little-endian.

## GATT service

| Characteristic | UUID | Properties | Content |
|---|---|---|---|
| Service | `458f0001-54a4-46d8-a8d8-c8f5f76846d3` | - | IMU streaming service |
| Data | `458f0002-54a4-46d8-a8d8-c8f5f76846d3` | notify | sample frames |
| Control | `458f0003-54a4-46d8-a8d8-c8f5f76846d3` | write | one-byte command |
| Info | `458f0004-54a4-46d8-a8d8-c8f5f76846d3` | read | device description |

## Session

1. Connect and negotiate the MTU (firmware asks for 247). **Streaming starts only when the usable payload (MTU - 3) can hold at least one sample (24 bytes)**, so the default MTU of 23 is not enough.
2. Read **Info** once to learn the sample rate and scales.
3. Enable notifications on **Data**.
4. Write `0x01` (Start) to **Control**. Write `0x02` (Stop) to end the stream.
5. While idle (no Start, or notifications off) the firmware discards samples, so Start always begins with fresh data.

## Info (12 bytes)

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | protocol version (1) |
| 1 | 1 | reserved (0) |
| 2 | 2 | sample rate in Hz |
| 4 | 4 | accelerometer scale, float32, mg per LSB |
| 8 | 4 | gyroscope scale, float32, mdps per LSB |

## Sample frame (Data notification)

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | version (1) |
| 1 | 1 | type (0x01 = samples) |
| 2 | 2 | `frame_seq`, increments per delivered frame, wraps at 65536 |
| 4 | 4 | `first_sample_seq` |
| 8 | 1 | `count`, 1..16 |
| 9 | 1 | reserved (0) |
| 10 | 12 x count | samples: ax, ay, az, gx, gy, gz as int16 (raw LSB) |
| 10 + 12 x count | 2 | CRC-16/CCITT-FALSE over all preceding bytes |

- Sample `i` has sequence number `first_sample_seq + i`; time is `seq / sample_rate`.
- A frame holds only consecutive samples. If samples were lost upstream, the firmware ends the frame at the gap and starts a new one.
- Maximum frame size is 204 bytes (16 samples), which fits an MTU of 247.

## Loss detection

| Symptom at the receiver | Meaning |
|---|---|
| gap in `frame_seq` | a notification was lost between firmware and client |
| `first_sample_seq` jumps, `frame_seq` continuous | samples were dropped inside the firmware (FIFO overrun or queue overflow) |
| CRC mismatch | corrupted frame, discard it |

## Commands (Control)

| Value | Command |
|---|---|
| 0x01 | Start streaming |
| 0x02 | Stop streaming |

Any other value is rejected with an ATT error.

## Throughput

At 104 Hz the payload is 104 x 12 = 1248 B/s. Frames go out every 50 ms (about 5 samples each), so the overhead is 12 bytes per frame, about 240 B/s.