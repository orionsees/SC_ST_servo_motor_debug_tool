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

> This binary predates the network link, so it has no **Link** selector. Build
> from source for that. It is deliberately not rebuilt on a newer machine:
> doing so would raise its glibc floor and drop Ubuntu 22.04.

## Build

Needs Qt 5.14+ (5.15 recommended) with the SerialPort module. Portable Qt, no
platform-specific code.

### Ubuntu 22.04 / 24.04, Debian, Raspberry Pi OS, JetPack

```bash
sudo apt install -y qtbase5-dev libqt5serialport5-dev qtchooser g++ make

mkdir -p build && cd build
qmake ../ServoBench.pro && make -j"$(nproc)"
./servobench
```

Build out of tree as above. In-source works, but a stray `ui_mainwindow.h` left
beside the sources shadows the generated one and breaks later builds
confusingly.

The terminal tool is a second project, and worth building too — it is what
serves a bus over the network, so the robot needs it even when the window does
not:

```bash
cd .. && mkdir -p build-cli && cd build-cli
qmake ../ServoBenchCli.pro && make -j"$(nproc)"
```

The same packages on the ARM boards. The window needs a desktop session
though, so on a headless Raspberry Pi or Jetson build only the terminal tool
-- see [Terminal version](#terminal-version) below.

Clean: `rm -rf build build-cli`, or in-source `make clean` (objects, `moc_*`,
`ui_*.h`) and `make distclean` (also binary and Makefile — re-run `qmake`
after).

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
- **Remote bus** — run the servos from a robot and the window from your desk.
  The robot runs `servobench-cli serve`; the window connects to it and works
  exactly as it does on a local port. See below.
- **Terminal version** — `servobench-cli`, the same three tabs in a full-screen
  terminal tool, plus one-shot commands for scripts. See below.


## Using the window

```bash
./build/servobench
```

1. Leave **Link** on `Serial`.
2. Pick the adapter in **Com**. Only ports with a USB vendor id are listed,
   which is what hides the kernel's 30-odd `ttyS*` stubs; if nothing has one --
   a motherboard COM port would not -- everything is listed instead.
3. Set **BaudR** to whatever the servos are set to. Both tools default to
   `1000000`, which is the usual factory setting for STS and SMS. A servo
   records its own rate in its Baud Rate register (address 6), so once one
   answers you can read it back there.
4. Press **Open**, then **Search**. Every ID from 0 to 253 is pinged; press the
   same button again to stop early.
5. Select a servo in the list. The Debug tab plots it, Programming shows its
   register map, Calibration is gated on torque being released.

If the port will not open, it is usually permissions:
`sudo usermod -aG dialout $USER`, then log out and back in.


## Driving a robot's servos over the network

The servos are on the robot; you would rather not be. The robot runs a server,
the window connects to it, and everything works the way it does on a local
port — the **Com** dropdown just lists the robot's serial ports instead of
yours.

### On the robot

It needs its own `servobench-cli` — build it there, as
[above](#build) (an ARM board needs no window, so the `ServoBenchCli.pro`
project alone is enough). Then:

```bash
servobench-cli serve --port /dev/ttyUSB0 --baud 1000000 --listen 0.0.0.0:5555 --token hunter2
```

It prints what it is serving, and says so if you have opened it to every
interface without a token. Leave it running; a `systemd` unit is the obvious
home for it once you are past trying it out.

### In the window

1. Set **Link** to `Network`. **Host**, **Port** and **Token** appear.
2. Fill in the robot's name or address, `5555`, and the token.
3. Press **Open**. It reaches the robot, fills **Com** with the robot's serial
   ports, starts on the one the server was told to hold, and opens it.
4. The line underneath reads `Connected to robot.local - 4 ms round trip`.
   Every tab now works as it does locally.
5. **Close** shuts the port and lets go of the link.

**Link**, **Host**, **Port** and **Token** are editable whenever the port is
closed, so correcting a typo is pressing **Open** again — it reconnects with
whatever is in the fields.

### From a shell

The same server, driven by the terminal tool:

```bash
servobench-cli --host robot.local --token hunter2 scan
servobench-cli --host robot.local --token hunter2 monitor 1 --hz 50 --csv > run.csv
```

Leave `--port` out and the port the server already holds is used as it stands.

### Why not just tunnel the serial port

`ser2net` and friends move raw bytes, and the Feetech protocol is strict
request/response — so every transaction costs a network round trip. A single
telemetry sample is eight of them, and an ID scan is 254:

| operation | serial transactions | LAN, 2 ms | Wi-Fi, 30 ms |
|---|---|---|---|
| one telemetry sample | 8 | 16 ms | 240 ms |
| register write (unlock, write, relock, verify) | 4–5 | 10 ms | 150 ms |
| ID scan | 254 | 0.5 s | 7.6 s |

ServoBench cuts higher up instead: one round trip per *operation*, whatever it
costs in serial transactions. A telemetry sample is one request. So is a scan.

It also keeps the multi-step sequences on the robot. An EPROM write unlocks,
writes, relocks and verifies; over a byte tunnel a link that drops in the middle
leaves a servo with its EPROM unlocked. Here the whole sequence either runs on
the robot or does not start.

And a read that gets no answer stays distinguishable from a link that died —
over a byte tunnel both look like a servo that stopped responding, so a Wi-Fi
hiccup reads as every servo failing at once.

### Safety

One client at a time. Two would interleave packets on a bus whose protocol has
no way to tell whose reply is whose, so a second connection is refused with a
reason rather than quietly corrupting the first.

`--on-disconnect` decides what happens to the servos when the client goes away
or stops answering — the server notices within about six seconds, whether the
client closed, crashed, or drove out of Wi-Fi range:

| | |
|---|---|
| `stop` (default) | Command each servo the position it is in. Torque stays on, so an arm keeps holding itself up and anything moving comes to rest. This puts a wheel-mode servo into position mode, which is what stopping one requires. |
| `hold` | Change nothing. The last command stands — a wheel-mode servo keeps turning. |
| `release` | Torque off. An arm under gravity will fall. |

Only servos the client actually *commanded* are affected. One that was merely
read has not been disturbed, so it is left alone.

`--listen` defaults to `127.0.0.1:5555`. Loopback is deliberate: a robot's
motors should not become reachable from a conference Wi-Fi because a default
was left alone. To reach it from another machine, either pass a `--token` and
bind wider, or leave it on loopback and tunnel in, which needs no token and no
new code:

```bash
ssh -L 5555:localhost:5555 robot      # then connect the window to 127.0.0.1:5555
```

The token is a shared secret checked at the handshake, not a transport cipher —
it keeps the wrong client off the bus on a LAN you trust. For anything you do
not trust, use the tunnel.

### The wire, if you want to drive it yourself

One TCP connection carrying length-prefixed JSON: a four-byte big-endian byte
count, then that many bytes of UTF-8.

```
request  {"seq": 7, "op": "readStatus", "args": {"id": 1, "series": 2}}
reply    {"seq": 7, "ok": true, "result": {"status": {...}}}
event    {"event": "scanFound", "data": {"id": 3, "model": 777}}
```

Every request draws one reply carrying its `seq`; events are unsolicited and
carry none. Sequence 0 is reserved for an error against the connection itself,
such as being turned away because another client holds the bus.

Registers cross as a series and an address, never as a copy of the register
table, so the table that decides a register's width, its sign-magnitude
encoding and whether it needs the EPROM unlocked is always the robot's.

It is plain enough to drive from a shell with netcat, or from a Python script
that links none of this — which is the point of choosing JSON for a few hundred
bytes per sample.


## Terminal version

`servobench-cli` is the same tool without the window. It does everything the
GUI does -- live telemetry and plot, joint control, the register map, and the
calibration export -- over the same `src/servo` protocol code, so both send the
identical bytes to a servo.

It has two modes. With no command it starts a full-screen tool with the same
three tabs; with a command it does one thing and exits, which is what makes it
usable from a script or over SSH.

### Build

Qt 5 with the SerialPort module, and nothing else -- no widgets, no OpenGL, no
X. So it builds and runs on a headless board over SSH, which is the point of
having it on a robot at all. Qt 5.12 and newer; the Qt 5.14-only APIs are
deliberately avoided so the older JetPack images still compile it. Verified
here on Qt 5.15.13.

The build is the same three commands everywhere; only the dependency install
differs:

```bash
sudo apt update
sudo apt install qt6-base-dev qt6-base-dev-tools
mkdir -p build-cli && cd build-cli
qmake ../ServoBenchCli.pro && make -j"$(nproc)"
./servobench-cli --help
```

Optionally put it on your PATH:

```bash
sudo install -m 755 servobench-cli /usr/local/bin/
```

The prebuilt `packaging/servobench` is an x86-64 **GUI** binary, so every ARM
board below builds from source. Both tools can be built from the same checkout;
they share `src/servo` and do not interfere.

#### Ubuntu 22.04 / 24.04, Debian 11 / 12 (x86-64 or arm64)

```bash
sudo apt install -y qtbase5-dev libqt5serialport5-dev qtchooser g++ make
sudo usermod -aG dialout $USER        # port access; log back in afterwards
```

#### Raspberry Pi 4 (Raspberry Pi OS bullseye or bookworm, 32- or 64-bit)

Same packages as Debian above. On a 1 GB Pi use `make -j2` rather than
`-j"$(nproc)"`; four parallel g++ processes on this codebase can run it out of
memory.

The Waveshare servo board is a CH340 USB adapter, so it appears as
`/dev/ttyUSB0` -- `ch341` is in the stock Pi OS kernel, nothing to install. To
drive the bus from the Pi's own 40-pin UART instead, run `sudo raspi-config`
-> Interface Options -> Serial Port, answer **no** to the login shell and
**yes** to the hardware port, reboot, and pass `-p serial0` -- `ports` lists
USB adapters, so a name it does not print still opens fine.

#### Jetson Orin Nano (JetPack 5 or 6)

Same packages again. JetPack 6 is Ubuntu 22.04 with Qt 5.15, JetPack 5 is
Ubuntu 20.04 with Qt 5.12; both build. Check which you have with:

```bash
qmake -query QT_VERSION
```

The CH340 board is `/dev/ttyUSB0` here too. If you wire the servo bus to the
40-pin header UART instead, it is `/dev/ttyTHS1` (`ports --all` lists the
non-USB ports), and the serial-console service holds the debug UART until you
stop it:

```bash
sudo systemctl disable --now nvgetty
```

#### Other distributions

```bash
# Fedora
sudo dnf install -y qt5-qtbase-devel qt5-qtserialport-devel gcc-c++ make
# Arch
sudo pacman -S --needed qt5-base qt5-serialport gcc make
```

The Qt 5 qmake is called `qmake-qt5` on some of these. Any Linux with Qt 5 and
a serial port will do; there is no platform-specific code beyond the terminal
handling.

#### If qmake resolves oddly

`qmake -query QT_VERSION` has to print a 5.x. Two things commonly get this
wrong:

- **Qt 6 is not supported.** Both tools use Qt 5 APIs (`QRegExp` among them).
  Build with the Qt 5 qmake -- on Debian and Ubuntu that is
  `/usr/lib/qt5/bin/qmake`.
- **A conda environment on `PATH` shadows the system qmake.** Check
  `which qmake`; use the full path above to be sure.

#### Running it on a machine that does not build it

The binary links the system Qt rather than bundling it, so a machine that only
runs it needs the runtime packages, not the `-dev` ones:

```bash
sudo apt install -y libqt5core5a libqt5serialport5
```

It must also be the same architecture -- an arm64 build will not run on x86-64,
or the other way round.

#### Windows and macOS

The full-screen tool uses POSIX terminal APIs (`termios`, `ioctl`), so it is
Linux and macOS only. macOS should build with Qt 5 from Homebrew, though that
is untested here. On Windows, use the GUI -- see the MSYS2 instructions above.

### Full-screen tool

```bash
servobench-cli                 # first USB serial port found
servobench-cli -p ttyUSB0 -b 1000000
servobench-cli --ascii --plain # no braille, no colour, for a bare terminal
```

`o` opens the port, `s` searches the bus, `n` and `N` pick a servo, `t`
releases or engages torque everywhere, Tab moves between Debug, Programming and
Calibration, and `?` lists every key. The plot is braille, zooms with `+`/`-`
and `<`/`>`, pans with `h`/`l`, resets with `0`, and follows the mouse wheel and
drag where the terminal reports them (`z` turns mouse reporting off so the
terminal can select text again).

The dangerous operations ask the same three times as the window does, and
calibration is gated on torque being released on every detected servo.

### Commands

```bash
servobench-cli ports                      # serial ports, USB ones first
servobench-cli scan                       # ping the bus, list what answers
servobench-cli info 1                     # model, firmware, limits, faults
servobench-cli monitor 1 --hz 50 --csv    # telemetry to stdout
servobench-cli record 1 --file run.txt    # the GUI's CSV log format

servobench-cli read 1 "Position P Gain"   # by name, address, or prefix
servobench-cli write 1 21 24              # EPROM-aware, verified by read-back
servobench-cli dump 1                     # the whole register map
servobench-cli regs save 1 servo1.json    # snapshot, same JSON as the GUI
servobench-cli regs load 1 servo1.json    # restore, three confirmations

servobench-cli torque all off
servobench-cli pos 1 3000 --speed 600 --wait
servobench-cli angle 1 -45 --unit deg
servobench-cli sweep 1 --start 1000 --end 3000 --hold 500

servobench-cli serve -p ttyUSB0 --listen 0.0.0.0:5555 --token hunter2
servobench-cli --host robot.local --token hunter2 scan
```

`--json` on the read-only commands prints machine-readable output; `--quiet` on
`read` prints the bare value.

`--host H[:PORT]` sends any command to a machine running `serve` instead of to
a local port, and `--port` then names the serial port on *that* machine. Leave
it out and the port the server already holds is used as it stands. See
[Driving a robot's servos over the network](#driving-a-robots-servos-over-the-network).

Calibration is three steps in the window, so it is three commands here, sharing
a state file (`~/.cache/servobench/calibration-state.json`, or `--state PATH`):

```bash
servobench-cli torque all off
servobench-cli calib home                      # pose the arm first
servobench-cli calib record                    # move every joint, Ctrl-C to stop
servobench-cli calib names shoulder_pan,shoulder_lift,elbow_flex
servobench-cli calib export calibration.json
```

Anything that writes EPROM asks for confirmation; `--yes` answers in advance,
which is the only way to run those from a script.

`servobench-cli --help` lists everything, and `--help <command>` explains one.


## Code layout

```
ServoBench.pro          qmake project, the window
ServoBenchCli.pro       qmake project, the terminal tool
src/
  mainwindow.{h,cpp,ui} GUI: debug, programming, calibration tabs
  graphwidget.{h,cpp}   telemetry plot
  cli/
    main.cpp            argument parsing, then a command or the full-screen tool
    commands.{h,cpp}    the one-shot commands
    tui.{h,cpp}         full-screen tool: the same three tabs, from the keyboard
    plot.{h,cpp}        the telemetry plot in braille
    session.{h,cpp}     the bus, the servo table and the CSV recorder
    term.{h,cpp}        raw mode, key and mouse decoding, screen diffing
    calib_state.h       calibration state shared between the calib commands
  net/
    protocol.{h,cpp}    the wire: framing, ops, value codecs
    remote_bus.{h,cpp}  a bus reached over TCP -- the client half
    bus_server.{h,cpp}  serves this machine's bus to one client
  servo/
    scserial.{h,cpp}    serial protocol, model tables
    servo_bus.{h,cpp}   IServoBus, and the one on this machine's port
    servo_driver.h      common servo interface + per-series drivers
    servo_types.h       register maps, series traits, status decoding
    port_list.h         the serial ports worth offering
    sms_sts.h, scscl.h  per-series servo classes (header only)
packaging/servobench    prebuilt x86-64 binary for Ubuntu 22.04+
```

Servo I/O is blocking, so it runs on a dedicated bus thread. `ServoBus` owns the
port; with one bus thread its event queue is also the bus lock, so transactions
cannot interleave on the wire. `MainWindow` uses `runOnBus(fn)` to wait for a
result while the window keeps repainting, and `postToBus(fn)` to fire and
forget. Either way `fn` runs on the bus thread and may touch only `bus_` and its
own by-value captures — `IServoBus` asserts this in debug builds.

`IServoBus` is that whole surface as an interface, with two implementations:
`ServoBus` on this machine's serial port, and `RemoteServoBus` over a socket.
They share the thread rule, so the window and the terminal tool drive either one
without knowing which they have. The server is the same `ServoBus` again, with
`BusServer` in front of it translating the wire — so the protocol quirks that
matter (sign-magnitude, per-series endianness, the EPROM lock dance, holding
torque across a write) exist in exactly one place and cannot drift between
local and remote.

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

## Contributing

Issues and pull requests are welcome <3
