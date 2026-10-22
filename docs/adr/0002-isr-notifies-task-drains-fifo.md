# ADR 0002: ISR only notifies, a task drains the FIFO

## Status
Accepted

## Context
The sensor raises INT1 when the FIFO watermark is reached. Reading the FIFO
needs I2C transactions, which block, use the I2C driver and take milliseconds.

## Decision
The INT1 ISR only calls `vTaskNotifyGiveFromISR`. A dedicated task reads the
FIFO status and all unread words, pairs gyro and accel words and pushes
samples into a lock-free SPSC queue. Consumers (BLE, DSP) read from that queue.
The task also wakes up on a 1 s timeout so a missed edge cannot stall it, and
is notified once at start-up in case INT1 is already high.

## Consequences
- Short, bounded ISR; no blocking calls in interrupt context.
- Latency is bounded by task scheduling (priority 10) plus the FIFO drain time.
- FIFO overrun and queue overflow are counted separately and reported in Stats.
- Words are read one per transaction (7 B); burst reads are a later optimisation
  that will be measured first.

## Update: polling is the default

INT1 did not trigger on the first hardware, so polling is now the default and
interrupt mode is optional (Kconfig `IMU_USE_INT1`). In polling mode the same
task wakes up every 50 ms instead of on an INT1 edge; the FIFO drain, the sample
queue and the counters are unchanged. The first end-to-end run in this mode
(104 Hz, 30 s) delivered every sample without gaps. Burst reads of several FIFO
words per I2C transaction and measured bus load are planned.