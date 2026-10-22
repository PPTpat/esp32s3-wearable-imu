# Hardware

## Wiring (Adafruit LSM6DSOX, product 4438)

| Sensor pin | ESP32-S3 | Purpose |
|---|---|---|
| VIN | 3V3 | power (3.3 V) |
| GND | GND | ground |
| SDA | GPIO 8 | I2C data |
| SCL | GPIO 9 | I2C clock |
| CS | 3V3 | tied high: selects I2C instead of SPI |
| DO | GND | I2C address 0x6A (tie to 3V3 for 0x6B) |
| I1 (INT1) | GPIO 4 | FIFO watermark interrupt (optional, see below) |

`I2`, `SCX` and `SDX` are not used. `I1` is only needed in interrupt mode and can stay unconnected. All GPIOs can be changed in `menuconfig` (*Wearable IMU*).
For address 0x6B enable *SA0 pin tied to VDD* there.

## Lesson learned: do not leave CS and DO floating

With `CS` and `DO` unconnected the sensor behaved randomly: sometimes it answered at 0x6A, sometimes nothing answered at all.
`CS` selects the interface (high = I2C) and `DO` selects the address, so both must be tied to a defined level.
Tying `CS` to 3V3 and `DO` to GND made detection reliable.

## INT1 (optional)

By default the firmware polls the FIFO every 50 ms, so INT1 is not required.
Interrupt mode can be enabled in `menuconfig` (*Use the sensor INT1 pin*), but it is **not verified on hardware**:
on my board no edge reached the ESP32-S3 on GPIO 4 or GPIO 5 (`irq` stayed at 0) and the cause is still unknown.
In that mode the firmware prints a register read-back and the INT1 pin level to help locate the problem.

## I2C load

Each FIFO word is 7 bytes, and accelerometer and gyroscope each produce one word per sample period.
I2C at 400 kHz carries roughly 40 kB/s (estimate).

| ODR (accel + gyro) | FIFO data | Approx. bus load at 400 kHz |
|---|---|---|
| 104 Hz | about 1.5 kB/s | about 4 % |
| 416 Hz | about 5.8 kB/s | about 15 % |
| 833 Hz | about 11.7 kB/s | about 30 % |

Above roughly 833 Hz the bus becomes the bottleneck and SPI would be needed.
Measured bus load will replace these estimates.

## Troubleshooting

The firmware retries sensor initialisation and, on failure, prints a diagnostic:

- the status code and the init step that failed,
- the raw ESP-IDF error of the last transaction,
- every I2C address that acknowledges.

At boot it also prints `probe 0x6A: ack N/20`. `20/20` means a healthy bus, a value in between points to a loose contact, and `0/20` means nothing answers (check power, SDA/SCL, `CS`, `DO`).