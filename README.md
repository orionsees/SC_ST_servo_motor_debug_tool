# ServoBench

Utility for Feetech SCS/STS servos live telemetry, register editing,
joint-angle control, and calibration export.

![](docs/images/debug.png)

## Run the prebuilt binary (Ubuntu)

`packaging/servobench` is a prebuilt x86-64 binary, so you can skip the build:

```bash
sudo apt install -y libqt5widgets5 libqt5serialport5 libgl1
sudo usermod -aG dialout $USER            # port access; log back in after
./packaging/servobench
```

Built against Ubuntu 22.04 (glibc 2.34), so it runs on **Ubuntu 22.04 and
newer, x86-64**. It links against the system Qt 5 rather than bundling it,
hence the `apt install` above. On anything else — older Ubuntu, ARM, another
distro with a different Qt — build from source below.

## Build

Needs Qt 5.14+ (5.15 recommended) with the SerialPort module. Portable Qt, no
platform-specific code.

### Ubuntu 24.04

```bash
sudo apt install -y qtbase5-dev libqt5serialport5-dev qtchooser g++ make
qmake ServoBench.pro
make -j"$(nproc)"
./servobench
```

Prefer building out of tree, a stray `ui_mainwindow.h` beside the sources
shadows the generated one and breaks later builds confusingly:

```bash
mkdir -p build && cd build
qmake ../ServoBench.pro && make -j"$(nproc)"
```

Clean: `rm -rf build`, or in-source `make clean` (objects, `moc_*`, `ui_*.h`)
and `make distclean` (also binary and Makefile — re-run `qmake` after).

For port access without `sudo`: `sudo usermod -aG dialout $USER`, then log back
in.

> If `qmake` resolves oddly, check `which qmake` — a conda environment on `PATH`
> shadows the system one. Use `/usr/lib/qt5/bin/qmake` to be sure.

### Windows 11

Not tested here; package names are verified against the MSYS2 repositories, but
the build has only been run on Linux.

Install [MSYS2](https://www.msys2.org/), then in the **MSYS2 MinGW 64-bit**
shell (not the plain MSYS one):

```bash
pacman -S --needed mingw-w64-x86_64-qt5-base mingw-w64-x86_64-qt5-serialport \
                   mingw-w64-x86_64-qt5-tools mingw-w64-x86_64-gcc \
                   mingw-w64-x86_64-make
qmake-qt5 ServoBench.pro
mingw32-make release -j
windeployqt-qt5 release/servobench.exe    # copies DLLs so it runs standalone
```

MSYS2 installs `qmake-qt5.exe` and `windeployqt-qt5.exe` — not `qmake` /
`windeployqt`. Alternatively use Qt Creator with the official Qt installer: open
`ServoBench.pro`, pick a Qt 5.15 kit, build.

Ports appear as `COM3`, `COM4`, … instead of `/dev/ttyUSB0`; the list populates
automatically.

### Servo board

Needs the CH340 USB-serial driver in-kernel on Linux, usually automatic on
Windows 11, otherwise from WCH.

## Features

- **Debug** — live plot of position, goal, torque, speed, current, temperature,
  voltage. Joint angle in rad and degrees from the calibrated midpoint; control
  takes a raw count or an angle. Wheel zooms time, shift+wheel zooms values,
  drag pans, double-click resets.
- **Programming** — full register map per series, EPROM-aware writes (unlock,
  write, verify by read-back, relock), register snapshot save/load as JSON.
- **Calibration** — gated on torque being released. Sets midpoint, records range
  of motion, exports `calibration.json`.
- Servo faults (register 65) are decoded and shown per servo in the servo list.


## Code layout

```
ServoBench.pro          qmake project
src/
  mainwindow.{h,cpp,ui} GUI: debug, programming, calibration tabs
  graphwidget.{h,cpp}   telemetry plot
  servo/
    scserial.{h,cpp}    serial protocol, model tables
    servo_bus.{h,cpp}   owns the port; every transaction on its own thread
    servo_driver.h      common servo interface + per-series drivers
    servo_types.h       register maps, series traits, status decoding
    sms_sts.h, scscl.h  per-series servo classes (header only)
packaging/servobench    prebuilt x86-64 binary for Ubuntu 22.04+
```

Servo I/O is blocking, so it runs on a dedicated bus thread. `ServoBus` owns the
port; with one bus thread its event queue is also the bus lock, so transactions
cannot interleave on the wire. `MainWindow` uses `runOnBus(fn)` to wait for a
result while the window keeps repainting, and `postToBus(fn)` to fire and
forget. Either way `fn` runs on the bus thread and may touch only `bus_` and its
own by-value captures — `ServoBus` asserts this in debug builds.

## Screenshots

Programming tab : register map with EPROM-aware writes:

![](docs/images/prog.png)

Calibration tab : one row per detected servo, gated on torque being disabled:

![](docs/images/calibration.png)

## Where these servos are used

Feetech STS/SCS bus servos sold by Waveshare as the ST3215/SC series have
become the default actuator for low-cost open-source robotics. They daisy-chain
up to 253 units on one half-duplex serial line and report position from a 12-bit
magnetic encoder, so a whole arm needs a single cable.

- [**SO-101 / SO-ARM101**](https://huggingface.co/docs/lerobot/so101) — the
  Hugging Face LeRobot arm, 6× STS3215 per arm. Also its SO-100 predecessor.
- [**Open Duck Mini v2**](https://github.com/apirrone/Open_Duck_Mini) — bipedal
  BDX-style droid, 16× STS3215.
- [**LeKiwi**](https://github.com/SIGRobotics-UIUC/LeKiwi) — mobile manipulator,
  SO-ARM101 on an omni-wheel base.
- [**XLeRobot**](https://github.com/Vector-Wangel/XLeRobot) and **Bambot** —
  dual-arm mobile home robots built on the same servos.
- Plus the many hobby quadrupeds and hexapods that use
  these servos for their leg joints.

The calibration export here writes LeRobot's `calibration.json` format, so
ServoBench can be used to calibrate an SO-101 or LeKiwi directly.
