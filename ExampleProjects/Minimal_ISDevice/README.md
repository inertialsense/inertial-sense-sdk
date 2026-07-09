## Build

This example communicates over a serial device and builds without `libusb` or
`libudev`. From this directory, run:

```sh
cmake -S . -B build-minimal
cmake --build build-minimal -j
```

Run it with the serial device path (the default is `/dev/ttyACM0` on Linux):

```sh
./build-minimal/ISDeviceBasics1 /dev/ttyACM0
```
