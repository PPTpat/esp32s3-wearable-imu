# Roadmap

- [x] v0.1 Project skeleton, CI, HAL bus abstraction and mock
- [x] v0.2 LSM6DSOX driver (init, config, FIFO) with host tests
- [x] v0.3 I2C bus and FIFO acquisition with SPSC queue (polling mode verified, INT1 mode optional)
- [x] v0.4 BLE GATT service and frame protocol (verified: 104 Hz, no loss in a 30 s run)
- [x] v0.5 DSP: orientation filter, step counter, fall detection (host-tested on synthetic signals, replay tool)
- [ ] v0.6 Tune DSP on real recordings, run it on the device, suppress steps on impacts, burst FIFO reads, bus load and CPU measurements, INT1 root cause
- [ ] v0.7 Power management, offline logging, calibration stored in NVS
- [ ] v0.8 OTA with rollback (needs a 16 MB partition table), watchdog, core dump
- [ ] v1.0 Dataset, activity classification, measured results in the README