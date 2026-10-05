# Third-party notices

Wild Sync's own source code and documentation are released under the MIT
licence in [`LICENSE`](LICENSE), Copyright (c) 2026 Wild Technologies. That
licence does not extend to anything listed here. This file records the
third-party components the software uses or that are kept in the repository,
under which licence, and how each is obtained. [`sbom.spdx.json`](sbom.spdx.json)
is the same list as an SPDX 2.3 document.

A version or licence is given only where it could be read from the Sony SDK
package, from a file in this repository, or from the metadata of a package
installed on the development host. Where it could not, it is recorded as "not
asserted" rather than guessed.

## Components

| Component | Version | Licence | Where it is | How it is obtained |
|---|---|---|---|---|
| Sony Camera Remote SDK | 2.02.00 | Sony's licence agreement (proprietary) | Not in the repository. Staged into `include/CRSDK/`, `lib/` and `lib-linux/`, all git-ignored | From Sony, under Sony's agreement; then `make sdk` or `make sdk-linux` |
| libusb | 1.0.26 | LGPL-2.1-or-later | Inside the Sony SDK (`CrAdapter/libusb-1.0*`) | Comes with the SDK |
| libssh2 | not asserted | BSD-3-Clause | Inside the Sony SDK (`CrAdapter/libssh2*`) | Comes with the SDK |
| OpenSSL | not asserted | Apache-2.0 | Statically linked into libraries of the Sony SDK | Comes with the SDK |
| cpp-httplib | 0.19.0 | MIT | `third_party/httplib.h`, vendored | In this repository |
| pyserial | not pinned | BSD-3-Clause | Python package, imported as `serial` | apt `python3-serial` (Jetson) or pip |
| Pillow | not pinned | MIT-CMU | Python package, imported as `PIL` | apt `python3-pil` (Jetson) or pip |
| numpy | not pinned | BSD-3-Clause AND 0BSD AND MIT AND Zlib AND CC0-1.0 | Python package | apt `python3-numpy` (Jetson) or pip |
| OpenCV (Python bindings) | not pinned | Apache-2.0 | Python package, imported as `cv2` | apt `python3-opencv` (Jetson) or pip `opencv-python-headless` |
| gpiod (libgpiod Python bindings) | v1.6.3 API | not asserted | Python package, imported as `gpiod` | apt `python3-libgpiod` (camera nodes, Jetson strobe role) |

Which Python package is used by which part of the system, and which are
installed from apt by `deploy/deploy.sh` rather than pip, is in
[`requirements.txt`](requirements.txt). Operating-system packages that
`deploy.sh` installs from apt (`build-essential`, `rsync`, `chrony`, `gpiod`,
`libusb-1.0-0`, `libxml2`) are infrastructure and are not listed above.

## Sony Camera Remote SDK

The SDK is proprietary software supplied by Sony under its own licence
agreement, which the SDK's readme says is linked from
<http://www.sony.net/CameraRemoteSDK/>. It is not redistributed with Wild Sync:
neither the headers (`include/CRSDK/`) nor the shared libraries are in the
repository or in its history. A developer obtains the package from Sony, unzips
it and the `RemoteCli.zip` inside it, and stages it with
`make sdk CRSDK_DIR=/path/to/RemoteCli` (or `make sdk-linux` for the node
libraries). The code was built against v2.02.00. `ilxctl` includes the SDK
headers, links `libCr_Core`, and the SDK loads its transport adapters from
`CrAdapter/` at run time.

The package also ships OpenCV for Sony's sample application. Wild Sync does not
stage or use that copy.

### libusb, libssh2 and OpenSSL inside the SDK

The SDK's `CrAdapter` folder carries libusb and libssh2, and OpenSSL is
statically linked into some of the SDK's libraries. They are not separate files
in this repository. The package includes their source archives under
`SourceCodeOfOpenSourceSoftware/` (`libusb.zip`, `libssh2.zip`, `openssl.zip`),
and the SDK readme says the licence conditions of libusb must be covered by
anyone who distributes an application with the SDK libraries. Anyone who
redistributes the staged libraries takes on those conditions and Sony's.

- **libusb 1.0.26** - LGPL-2.1-or-later (the `COPYING` and source file headers
  in `libusb.zip`). The same version record is in the macOS and the Linux
  aarch64 binaries.
- **libssh2** - BSD-3-Clause (`COPYING` in `libssh2.zip`). The version is not
  asserted because the package is not consistent: the source archive is 1.10.0,
  while the binaries identify themselves as 1.10.0 (macOS arm64) and 1.11.0
  (macOS x86_64, Linux aarch64).
- **OpenSSL** - Apache-2.0 (`LICENSE.txt` in `openssl.zip`). The version is not
  asserted because the package is not consistent: the source archive is 3.0.8,
  while the binaries carry the version strings 3.0.13 (libssh2) and 3.0.10 (the
  monitor-protocol library: `libmonitor_protocol` on macOS,
  `libmonitor_protocol_pf` on Linux aarch64). Only the libraries named here were
  inspected.

## cpp-httplib

`third_party/httplib.h` is cpp-httplib 0.19.0 (`CPPHTTPLIB_VERSION` in the
header), a single-header HTTP server library by Yuji Hirose, compiled into
`ilxctl`. The file is byte-identical to `SimpleCli/app/httplib.h` in Sony's
Camera Remote SDK v2.02.00 package. Its header states the following notice,
reproduced here with the standard MIT permission text:

```
Copyright (c) 2025 Yuji Hirose. All rights reserved.
MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Python packages

None of these is in the repository. They are imported by the Python code under
`rig/` and `deploy/`, and `requirements.txt` says by what and what stops
working without each. Versions are not pinned: on the Jetson and the camera
nodes they are whatever the distribution's apt repository provides for that
Ubuntu release. The licence for each was read from a package installed on the
development host, whose versions are shown; an older distribution package may
declare a different licence expression.

| Package | Development-host version | Licence source |
|---|---|---|
| pyserial | 3.5 | SPDX identifier in its source files (its metadata says only "BSD") |
| Pillow | 12.3.0 | `License-Expression` in its metadata |
| numpy | 2.5.2 | `License-Expression` in its metadata |
| opencv-python-headless | 5.0.0.93 | Its metadata (Apache 2.0). The wheel's own packaging scripts are MIT and it bundles further third-party libraries listed in its `LICENSE-3RD-PARTY.txt` |

`gpiod` is the libgpiod Python binding, written against the v1.6.3 API that
`docs/PROTOCOL.md` specifies for the nodes (Ubuntu 24.04 packages it as
`python3-libgpiod`). It was not installed on the host where this file was
written, so its licence and version are not asserted.

## Vendor documentation kept in `docs/`

Three PDFs from equipment vendors are kept in `docs/` for reference. They are
not part of Wild Sync, are not covered by its licence, and remain the
copyright of their publishers.

| File | Publisher | What it is |
|---|---|---|
| `docs/XR256_Datasheet.pdf` | Smart Vision Lights | XR256 high-speed strobe light (OverDrive) product data sheet |
| `docs/VA_Imaging_Industrial_Strobe_Controller.pdf` | VA Imaging | Industrial strobe controller page from the publisher's web site (its footer reads "(c) 2026 - VA Imaging"), saved to PDF from a browser on 2026-08-28 |
| `docs/iKonvertManual.pdf` | Digital Yacht | Instruction manual for the iKonvert NMEA 2000 gateway |

## Referenced at view time, not distributed

`docs/USER-GUIDE.html` (the field manual) loads the IBM Plex Mono, IBM Plex Sans
Condensed and IBM Plex Sans typefaces from Google Fonts when it is opened with a
network connection. The font files are not in this repository, and their licence
is not asserted here.
