# Wild Sync

A small local web app for the **Sony ILX-LR1** with the **FE PZ 16-35mm F4 G**
power-zoom lens. It gives the four things the body itself has no controls for:

| | |
|---|---|
| **Camera control** | connect over USB, live view, exposure (mode / shutter / aperture / ISO / drive), storage destination, card + power status |
| **Intervalometer** | 1-second (or any) interval, either timed by the camera or by this app |
| **Manual focus** | MF mode, press-and-hold near/far drive, absolute focus position |
| **Optical zoom** | press-and-hold wide/tele at selectable speed, absolute zoom position |

Everything is driven through Sony's Camera Remote SDK v2.02.00. Only properties
the SDK's own support matrix lists as available on the ILX-LR1 are used.

## Build

Needs nothing but the Xcode command line tools — no CMake, no Xcode project.

**First, stage the SDK.** Sony's Camera Remote SDK is not redistributed in this
repo, so point `make sdk` at your own unpacked copy (the `RemoteCli` or
`SimpleCli` folder from the official package). It fills in `include/CRSDK/` and
`lib/`, and strips the quarantine attribute that otherwise stops dyld loading
the dylibs.

```bash
make sdk CRSDK_DIR=/path/to/unpacked/RemoteCli
make
```

`CRSDK_DIR` defaults to `../RemoteCli`, so unpacking the SDK package next to
this repo needs no argument. On the machine this was developed against it lives
at `~/Desktop/CrSDK_v2/RemoteCli`.

That compiles `build/ilxctl` and stages the SDK dylibs beside it. Tested with
SDK v2.02.00 on macOS 26 (Apple Silicon).

## Run

```bash
./build/ilxctl
```

Then open **http://127.0.0.1:8080**.

It connects to the first ILX-LR1 it finds and prints progress to the terminal.

```
--port N          HTTP port (default 8080)
--host ADDR       bind address (default 127.0.0.1)
--save-dir PATH   where frames sent to the PC are written
                  (default ~/Pictures/ILX-LR1)
--no-autoconnect  start without opening the camera
```

## Before you start

* **Quit Imaging Edge Remote** (and anything else talking to the camera). The
  SDK cannot share the body — a second client makes `Connect` fail.
* The camera's **USB Connection Mode** must be *PC Remote*.
* If the camera ever stops accepting connections (`0x00008208`,
  `CrError_Connect_TimeOut`, repeating even though the camera still enumerates
  on USB), its PTP session is wedged. Unplug the USB cable, wait a few seconds
  and plug it back in.

## Two intervalometers, and why

The **Timed by** selector picks which clock runs the sequence:

* **Camera (Interval REC)** — the body's own interval recording. Timing is
  generated inside the camera, so it is exact and keeps going even if USB
  hiccups. Interval resolution is 0.1 s. *While it is armed the body owns the
  shutter and most settings turn read-only.*
* **This app** — `ilxctl` fires each frame on a monotonic schedule. You get a
  live frame counter and can use any interval, but it depends on the USB link.
  Measured at a 1.000 s interval: 10 frames in 10.05 s.

The host loop schedules each frame against a fixed start time rather than
sleeping between frames, so a slow frame does not push the rest of the sequence
late.

## Two gotchas this app handles for you

Both cost real debugging time, so they are worth knowing about.

### 1. `EnumCameraObjects` fails with `0x8703` unless the adapters are in a bundle path

On macOS the SDK looks for its USB/IP transport adapters at
**`<executable dir>/Contents/Frameworks/CrAdapter`**, not just
`<executable dir>/CrAdapter`. With only the latter, enumeration fails instantly
with `CrError_Adaptor_Create` even though the camera is plainly on the USB bus
and every dylib loads fine on its own. Sony's own `CMakeLists.txt` quietly
populates both paths for Apple builds; the `Makefile` here does the same.

The SDK dylibs also carry a quarantine attribute when the package is downloaded,
which stops dyld from loading them. `make` strips it from the staged copies.

### 2. A "dead" shutter usually means Interval REC is armed

If the camera's built-in **Interval REC** is on, `SendCommand(Release)` returns
success and *nothing happens* — because in that mode the release **toggles the
interval sequence** instead of taking a frame. `Interval_Rec_Mode` is also
read-only while a sequence is running, so you cannot simply turn it off.

`ilxctl` handles this: taking a single frame, or starting the host
intervalometer, first stops any running sequence and disarms Interval REC.

Relatedly, the body ignores remote release entirely unless control priority
belongs to the PC, so the app sets `PriorityKeySettings` to `PC remote` on
connect. That property is not writable in the instant after `OnConnected`, so
it retries.

## Focus and zoom values

Both are normalised 0–65535/0–16384 scales, not physical units — Sony documents
`FocusPositionSetting` as a per-lens value you calibrate yourself. On the
16-35 PZ as tested:

* focus `0` = infinity end, `65535` = near end (verified: a room-distance
  subject is sharper at `0`)
* zoom `0` = wide, `16384` = full tele
* `NearFar` accepts −7…+7 (negative drives toward near), zoom speed −8…+8

The UI reads every range from the camera at runtime rather than hard-coding
them, so a different lens reports its own limits.

## HTTP API

The UI is just a client of this; it is stable enough to script against.

```
GET  /api/status                     everything, as JSON
POST /api/connect                    {}
POST /api/disconnect                 {}
POST /api/shutter                    {af}
POST /api/interval/start             {intervalSec, count, af}
POST /api/interval/stop              {}
POST /api/camera-interval/config     {intervalSec, shots, startDelaySec}
POST /api/camera-interval/arm        {armed}
POST /api/camera-interval/run        {start}
POST /api/focus/mode                 {mode}       1=MF 2=AF-S 6=DMF
POST /api/focus/drive                {step}       -7..7, 0 stops
POST /api/focus/position             {value}
POST /api/zoom/drive                 {speed}      -8..8, 0 stops
POST /api/zoom/position              {value}
POST /api/zoom/setting               {value}      1=optical only
POST /api/exposure                   {which, value}   iso|shutter|aperture|program|drive
POST /api/store                      {value}      1=PC 2=card 3=both
GET  /liveview.jpg                   current live view frame
```

Example — a 1 s sequence of 200 frames timed by the camera:

```bash
curl -X POST localhost:8080/api/camera-interval/arm    -d '{"armed":false}'
curl -X POST localhost:8080/api/camera-interval/config -d '{"intervalSec":1,"shots":200,"startDelaySec":1}'
curl -X POST localhost:8080/api/camera-interval/arm    -d '{"armed":true}'
curl -X POST localhost:8080/api/camera-interval/run    -d '{"start":true}'
```

## Layout

```
src/camera.{h,cpp}   SDK wrapper: connect, properties, focus, zoom, shooting
src/main.cpp         HTTP routes and lifecycle
src/web_ui.h         the single-page UI, embedded in the binary
include/CRSDK/       SDK headers (from the v2.02.00 package)
lib/                 SDK dylibs (from the v2.02.00 package)
third_party/         cpp-httplib
```

All SDK calls are serialised behind one recursive mutex, since the SDK is not
safe to call concurrently on a single device handle.
