// ilxctl - a small control panel for the Sony ILX-LR1.
//
// Serves a single-page UI on localhost and drives the camera over USB (or IP)
// through the Camera Remote SDK: shutter, a drift-free intervalometer, manual
// focus, and optical zoom on a power-zoom lens.

#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <thread>
#include <vector>

#include "camera.h"
#include "httplib.h"
#include "web_ui.h"

namespace {

Camera g_cam;
httplib::Server g_srv;
std::atomic<bool> g_stopping{false};
// Where the camera drops frames sent to the PC; also what the review pane lists.
std::string g_saveDir;

// Only JPEGs are offered for review - a browser cannot decode an ARW, and the
// point of the pane is looking at frames without leaving the app.
bool isJpegName(const std::string& n) {
    const std::size_t dot = n.rfind('.');
    if (dot == std::string::npos) return false;
    std::string ext = n.substr(dot);
    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return ext == ".jpg" || ext == ".jpeg";
}

// Reject anything that could climb out of the capture directory: only a plain
// basename of safe characters, and not a dotfile. A whitelist rather than a
// blacklist because req.matches[] carries the URL-decoded path, so "%00" and
// "%2f" arrive as a real NUL and slash - both fail the character test here,
// where a substring blacklist would let the NUL truncate the fopen() path.
bool safeShotName(const std::string& n) {
    if (n.empty() || n.size() >= 256 || n[0] == '.' || !isJpegName(n)) return false;
    for (const char c : n) {
        // Parentheses are allowed deliberately. When the SDK's auto-numbering
        // reaches a name that already exists it writes "ILX00183(1).JPG", and
        // rejecting those made the frames invisible to /api/shots - the camera
        // kept shooting and saving while the puller silently pulled nothing.
        // They are no more dangerous than '-': path traversal needs '/' or
        // "..", both of which are still refused, as is a leading '.'.
        const bool okc = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                         (c >= '0' && c <= '9') || c == '.' || c == '_' ||
                         c == '-' || c == '(' || c == ')';
        if (!okc) return false;
    }
    return n.find("..") == std::string::npos;
}

// Capture directory as JSON, sorted by name - which the camera numbers
// sequentially, so name order is capture order. Only names that /shot/ would
// actually serve are listed, which also keeps the JSON free of characters that
// would need escaping.
std::string shotsJson() {
    std::vector<std::pair<std::string, long long>> files;
    if (DIR* d = ::opendir(g_saveDir.c_str())) {
        while (const dirent* e = ::readdir(d)) {
            const std::string n = e->d_name;
            if (!safeShotName(n)) continue;
            struct stat st {};
            if (::stat((g_saveDir + "/" + n).c_str(), &st) != 0) continue;
            files.emplace_back(n, static_cast<long long>(st.st_size));
        }
        ::closedir(d);
    }
    std::sort(files.begin(), files.end());
    std::string out = "[";
    for (std::size_t i = 0; i < files.size(); ++i) {
        if (i) out += ',';
        out += "{\"name\":\"" + files[i].first +
               "\",\"size\":" + std::to_string(files[i].second) + "}";
    }
    return out + "]";
}

// Minimal JSON scalar extraction. The UI only ever sends flat objects of
// numbers, booleans and short strings, so a full parser would be dead weight.
bool jsonFind(const std::string& body, const std::string& key, std::string& out) {
    const std::string needle = "\"" + key + "\"";
    std::size_t p = body.find(needle);
    if (p == std::string::npos) return false;
    p = body.find(':', p + needle.size());
    if (p == std::string::npos) return false;
    ++p;
    while (p < body.size() && std::isspace(static_cast<unsigned char>(body[p]))) ++p;
    if (p >= body.size()) return false;
    if (body[p] == '"') {
        const std::size_t e = body.find('"', p + 1);
        if (e == std::string::npos) return false;
        out = body.substr(p + 1, e - p - 1);
        return true;
    }
    const std::size_t e = body.find_first_of(",}", p);
    out = body.substr(p, (e == std::string::npos ? body.size() : e) - p);
    while (!out.empty() && std::isspace(static_cast<unsigned char>(out.back()))) out.pop_back();
    return !out.empty();
}

double jsonNum(const std::string& body, const std::string& key, double dflt) {
    std::string s;
    if (!jsonFind(body, key, s)) return dflt;
    try {
        return std::stod(s);
    } catch (...) {
        return dflt;
    }
}

// Widest value accepted for a property: the largest double-exact integer.
constexpr long long kMaxJsonAbs = 1LL << 53;

// jsonNum() clamped into [lo, hi] before the integer cast. Casting an
// out-of-range double (an absurd number in a field, or "1e999" parsed to
// infinity) straight to an integer type is undefined behaviour.
long long jsonInt(const std::string& body, const std::string& key, long long dflt,
                  long long lo, long long hi) {
    const double v = jsonNum(body, key, static_cast<double>(dflt));
    if (!(v >= static_cast<double>(lo))) return lo;  // also catches NaN
    if (v >= static_cast<double>(hi)) return hi;
    return static_cast<long long>(v);
}

bool jsonBool(const std::string& body, const std::string& key, bool dflt) {
    std::string s;
    if (!jsonFind(body, key, s)) return dflt;
    return s == "true" || s == "1";
}

std::string jsonStr(const std::string& body, const std::string& key) {
    std::string s;
    jsonFind(body, key, s);
    return s;
}

void ok(httplib::Response& res, const std::string& extra = "") {
    res.set_content(extra.empty() ? "{\"ok\":true}" : "{\"ok\":true," + extra + "}",
                    "application/json");
}

void fail(httplib::Response& res, const std::string& err, int status = 400) {
    std::string e;
    for (const char c : err) {
        if (c == '"' || c == '\\') e += '\\';
        // A raw control character (a newline in an SDK message) breaks the JSON.
        e += static_cast<unsigned char>(c) < 0x20 ? ' ' : c;
    }
    res.status = status;
    res.set_content("{\"ok\":false,\"error\":\"" + e + "\"}", "application/json");
}

// Wrap a handler so an SDK-side failure becomes a clean JSON error.
template <typename Fn>
void guarded(httplib::Response& res, Fn fn) {
    std::string err;
    if (fn(err)) {
        ok(res);
    } else {
        fail(res, err);
    }
}

void onSignal(int) {
    if (g_stopping.exchange(true)) return;
    g_srv.stop();
}

}  // namespace

int main(int argc, char** argv) {
    int port = 8080;
    std::string host = "127.0.0.1";
    std::string saveDir;
    bool autoConnect = true;
    int camIndex = -1;          // --camera N, -1 = unset
    std::string camMatch;       // --match SUBSTR against id or model
    bool listOnly = false;      // --list: enumerate and exit

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--port" && i + 1 < argc) port = std::atoi(argv[++i]);
        else if (a == "--host" && i + 1 < argc) host = argv[++i];
        else if (a == "--save-dir" && i + 1 < argc) saveDir = argv[++i];
        else if (a == "--camera" && i + 1 < argc) camIndex = std::atoi(argv[++i]);
        else if (a == "--match" && i + 1 < argc) camMatch = argv[++i];
        else if (a == "--list") listOnly = true;
        else if (a == "--no-autoconnect") autoConnect = false;
        else if (a == "--help" || a == "-h") {
            std::printf(
                "ilxctl - Sony ILX-LR1 control panel\n\n"
                "  --port N          HTTP port (default 8080)\n"
                "  --host ADDR       bind address (default 127.0.0.1)\n"
                "  --save-dir PATH   where to write images the camera sends to the PC\n"
                "  --camera N        connect to enumerated camera N (default: prefer ILX-LR1)\n"
                "  --match SUBSTR    connect to the camera whose id or model contains SUBSTR\n"
                "  --list            list attached cameras and exit\n"
                "  --no-autoconnect  start without opening the camera\n");
            return 0;
        } else {
            std::fprintf(stderr, "unknown option: %s\n", a.c_str());
            return 2;
        }
    }

    if (saveDir.empty()) {
        const char* home = std::getenv("HOME");
        saveDir = std::string(home ? home : ".") + "/Pictures/ILX-LR1";
    }
    if (::mkdir(saveDir.c_str(), 0755) != 0 && errno != EEXIST) {
        std::fprintf(stderr, "warning: could not create %s\n", saveDir.c_str());
    }
    g_saveDir = saveDir;

    if (!Camera::sdkInit()) {
#ifdef __APPLE__
        std::fprintf(stderr,
                     "Failed to initialise the Camera Remote SDK.\n"
                     "Check that libCr_Core.dylib and CrAdapter/ sit next to this binary\n"
                     "and that the quarantine attribute has been removed.\n");
#else
        std::fprintf(stderr,
                     "Failed to initialise the Camera Remote SDK.\n"
                     "Check that libCr_Core.so and CrAdapter/ sit next to this binary.\n");
#endif
        return 1;
    }
    g_cam.log("Camera Remote SDK initialised");

    if (listOnly) {
        const auto cams = g_cam.enumerate(5);
        if (cams.empty()) {
            std::printf("\nNo cameras found.\n"
                        "  - USB Connection Mode must be PC Remote\n"
                        "  - nothing else may hold the camera (Imaging Edge, another ilxctl)\n\n");
        } else {
            std::printf("\n%zu camera(s):\n\n", cams.size());
            for (std::size_t i = 0; i < cams.size(); ++i) {
                std::printf("  [%zu]  %-12s  %-4s  %s\n", i, cams[i].model.c_str(),
                            cams[i].conn.c_str(), cams[i].id.c_str());
            }
            std::printf("\nClaim one with:  --match <id fragment>   (stable across replugs)\n"
                        "             or:  --camera <index>\n\n");
        }
        g_cam.disconnect();
        Camera::sdkRelease();
        return cams.empty() ? 1 : 0;
    }

    auto doConnect = [&](std::string& err) {
        const auto cams = g_cam.enumerate(3);
        if (cams.empty()) {
            err = "no camera found - check USB, that the camera is on, and that "
                  "USB Connection Mode is PC Remote";
            return false;
        }
        // With two bodies on the bench, "the first one" is not good enough: each
        // instance has to claim a specific camera and keep claiming the same one
        // across restarts. --match wins, then --camera, then prefer an ILX-LR1.
        int pick = -1;
        for (std::size_t i = 0; i < cams.size(); ++i) {
            g_cam.log("Found [" + std::to_string(i) + "] " + cams[i].model + " (" +
                      cams[i].conn + " " + cams[i].id + ")");
            if (!camMatch.empty() && pick < 0 &&
                (cams[i].id.find(camMatch) != std::string::npos ||
                 cams[i].model.find(camMatch) != std::string::npos)) {
                pick = static_cast<int>(i);
            }
        }
        if (pick < 0 && !camMatch.empty()) {
            err = "no camera matching '" + camMatch + "' among " +
                  std::to_string(cams.size()) + " found";
            return false;
        }
        if (pick < 0 && camIndex >= 0) {
            if (camIndex >= static_cast<int>(cams.size())) {
                err = "--camera " + std::to_string(camIndex) + " but only " +
                      std::to_string(cams.size()) + " camera(s) found";
                return false;
            }
            pick = camIndex;
        }
        if (pick < 0) {
            pick = 0;
            for (std::size_t i = 0; i < cams.size(); ++i)
                if (cams[i].model.find("ILX-LR1") != std::string::npos)
                    pick = static_cast<int>(i);
        }
        g_cam.log("Claiming [" + std::to_string(pick) + "] " + cams[pick].model +
                  " " + cams[pick].id);
        if (!g_cam.connect(pick, err)) return false;
        // Always give the SDK somewhere to put frames, in case the camera is
        // set to send them to the host rather than the card.
        std::string e2;
        if (!g_cam.setSaveDir(saveDir, e2)) g_cam.log("SetSaveInfo: " + e2);
        else g_cam.log("Frames sent to the PC will be saved in " + saveDir);
        return true;
    };

    // ----- routes ----------------------------------------------------------
    g_srv.Get("/", [](const httplib::Request&, httplib::Response& res) {
        res.set_content(kIndexHtml, "text/html; charset=utf-8");
    });

    g_srv.Get("/api/status", [](const httplib::Request&, httplib::Response& res) {
        res.set_content(g_cam.statusJson(), "application/json");
    });

    g_srv.Post("/api/connect", [&](const httplib::Request&, httplib::Response& res) {
        if (g_cam.isConnected()) {
            ok(res, "\"model\":\"" + g_cam.modelName() + "\"");
            return;
        }
        std::string err;
        if (doConnect(err)) ok(res, "\"model\":\"" + g_cam.modelName() + "\"");
        else fail(res, err);
    });

    g_srv.Post("/api/disconnect", [](const httplib::Request&, httplib::Response& res) {
        g_cam.disconnect();
        ok(res);
    });

    g_srv.Post("/api/shutter", [](const httplib::Request& req, httplib::Response& res) {
        const bool af = jsonBool(req.body, "af", false);
        guarded(res, [&](std::string& e) { return g_cam.shutter(af, e); });
    });

    g_srv.Post("/api/shutter/hold", [](const httplib::Request& req, httplib::Response& res) {
        const int ms = static_cast<int>(jsonInt(req.body, "ms", 1000, 0, 60000));
        guarded(res, [&](std::string& e) { return g_cam.holdShutter(ms, e); });
    });

    g_srv.Post("/api/interval/start", [](const httplib::Request& req, httplib::Response& res) {
        const double sec = jsonNum(req.body, "intervalSec", 1.0);
        const int count = static_cast<int>(jsonInt(req.body, "count", 0, 0, 1000000));
        const bool af = jsonBool(req.body, "af", false);
        guarded(res, [&](std::string& e) { return g_cam.startInterval(sec, count, af, e); });
    });

    g_srv.Post("/api/interval/stop", [](const httplib::Request&, httplib::Response& res) {
        g_cam.stopInterval();
        ok(res);
    });

    // The camera's own Interval REC ----------------------------------------
    g_srv.Post("/api/camera-interval/config",
               [](const httplib::Request& req, httplib::Response& res) {
        const double sec = jsonNum(req.body, "intervalSec", 1.0);
        const int shots = static_cast<int>(jsonInt(req.body, "shots", 0, 0, 1000000));
        const int delay = static_cast<int>(jsonInt(req.body, "startDelaySec", -1, -1, 86400));
        guarded(res, [&](std::string& e) {
            return g_cam.configureCameraInterval(sec, shots, delay, e);
        });
    });

    g_srv.Post("/api/camera-interval/arm",
               [](const httplib::Request& req, httplib::Response& res) {
        const bool armed = jsonBool(req.body, "armed", true);
        guarded(res, [&](std::string& e) { return g_cam.setCameraIntervalArmed(armed, e); });
    });

    g_srv.Post("/api/camera-interval/run",
               [](const httplib::Request& req, httplib::Response& res) {
        const bool start = jsonBool(req.body, "start", true);
        guarded(res, [&](std::string& e) { return g_cam.cameraIntervalRun(start, e); });
    });

    g_srv.Post("/api/focus/mode", [](const httplib::Request& req, httplib::Response& res) {
        const long long v = jsonInt(req.body, "mode", 1, -kMaxJsonAbs, kMaxJsonAbs);
        guarded(res, [&](std::string& e) { return g_cam.setFocusMode(v, e); });
    });

    g_srv.Post("/api/focus/drive", [](const httplib::Request& req, httplib::Response& res) {
        const int step = static_cast<int>(jsonInt(req.body, "step", 0, -1000000, 1000000));
        guarded(res, [&](std::string& e) { return g_cam.focusDrive(step, e); });
    });

    g_srv.Post("/api/focus/position", [](const httplib::Request& req, httplib::Response& res) {
        const long long v = jsonInt(req.body, "value", 0, -kMaxJsonAbs, kMaxJsonAbs);
        guarded(res, [&](std::string& e) { return g_cam.setFocusPosition(v, e); });
    });

    g_srv.Post("/api/zoom/drive", [](const httplib::Request& req, httplib::Response& res) {
        const int speed = static_cast<int>(jsonInt(req.body, "speed", 0, -1000000, 1000000));
        guarded(res, [&](std::string& e) { return g_cam.zoomDrive(speed, e); });
    });

    g_srv.Post("/api/zoom/position", [](const httplib::Request& req, httplib::Response& res) {
        const long long v = jsonInt(req.body, "value", 0, -kMaxJsonAbs, kMaxJsonAbs);
        guarded(res, [&](std::string& e) { return g_cam.setZoomPosition(v, e); });
    });

    g_srv.Post("/api/zoom/setting", [](const httplib::Request& req, httplib::Response& res) {
        const long long v = jsonInt(req.body, "value", 1, -kMaxJsonAbs, kMaxJsonAbs);
        guarded(res, [&](std::string& e) { return g_cam.setZoomSetting(v, e); });
    });

    // Sync the body's clock to the host's. With the hosts disciplined to the
    // Jetson, both cameras then stamp EXIF against the same master.
    g_srv.Post("/api/datetime", [](const httplib::Request& req, httplib::Response& res) {
        long long t = jsonInt(req.body, "epoch", 0, 0, kMaxJsonAbs);
        if (t <= 0) t = static_cast<long long>(std::time(nullptr));
        guarded(res, [&](std::string& e) { return g_cam.setDateTime(t, e); });
    });

    // Erase the card. Requires {"confirm":"format"} in the body: this endpoint
    // is one fat-fingered curl away from destroying a survey, and no automatic
    // path may ever reach it by accident.
    g_srv.Post("/api/format", [](const httplib::Request& req, httplib::Response& res) {
        if (jsonStr(req.body, "confirm") != "format") {
            res.status = 400;
            res.set_content("{\"ok\":false,\"error\":\"refusing to format without "
                            "{\\\"confirm\\\":\\\"format\\\"}\"}",
                            "application/json");
            return;
        }
        const bool quick = jsonInt(req.body, "quick", 1, 0, 1) != 0;
        guarded(res, [&](std::string& e) { return g_cam.formatMedia(quick, e); });
    });

    g_srv.Post("/api/store", [](const httplib::Request& req, httplib::Response& res) {
        // Accept "dest" (what rigd/rigcore and the tools send) with "value" as a
        // fallback for the older UI. Reading the wrong key silently fell back to
        // the default and forced the camera to card-only on every call, which
        // broke PC-save. Default to PC+card so a bare call errs toward transfer.
        const long long v = jsonInt(req.body, "dest",
                                    jsonInt(req.body, "value", 3, -kMaxJsonAbs, kMaxJsonAbs),
                                    -kMaxJsonAbs, kMaxJsonAbs);
        guarded(res, [&](std::string& e) { return g_cam.setStoreDestination(v, e); });
    });

    g_srv.Post("/api/exposure", [](const httplib::Request& req, httplib::Response& res) {
        const std::string which = jsonStr(req.body, "which");
        const long long v = jsonInt(req.body, "value", 0, -kMaxJsonAbs, kMaxJsonAbs);
        guarded(res, [&](std::string& e) { return g_cam.setExposure(which, v, e); });
    });

    // ----- captures ---------------------------------------------------------
    g_srv.Get("/api/shots", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Cache-Control", "no-store");
        res.set_content(shotsJson(), "application/json");
    });

    g_srv.Get(R"(/shot/(.+))", [](const httplib::Request& req, httplib::Response& res) {
        const std::string name = req.matches[1];
        if (!safeShotName(name)) {
            fail(res, "bad frame name", 404);
            return;
        }
        // A JPEG the camera writes is a few MB; refuse anything that would
        // stress a Pi's RAM to slurp. The cap is enforced on the read too, in
        // case the file grows between stat and fread.
        constexpr long long kMaxShotBytes = 64LL * 1024 * 1024;
        const std::string path = g_saveDir + "/" + name;
        struct stat st {};
        // lstat, not stat: a symlink named *.jpg in the save dir must not be
        // followed to serve whatever it points at. Only a real regular file.
        if (::lstat(path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
            fail(res, "no such frame", 404);
            return;
        }
        if (static_cast<long long>(st.st_size) > kMaxShotBytes) {
            fail(res, "frame too large to serve", 413);
            return;
        }
        std::FILE* f = std::fopen(path.c_str(), "rb");
        if (!f) {
            fail(res, "no such frame", 404);
            return;
        }
        std::string data;
        data.reserve(static_cast<std::size_t>(st.st_size));
        char buf[64 * 1024];
        std::size_t n;
        while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) {
            data.append(buf, n);
            if (data.size() > static_cast<std::size_t>(kMaxShotBytes)) {
                std::fclose(f);
                fail(res, "frame too large to serve", 413);
                return;
            }
        }
        std::fclose(f);
        // A given filename is written once by the camera, so it is safe to cache.
        res.set_header("Cache-Control", "max-age=86400");
        res.set_content(data, "image/jpeg");
    });

    g_srv.Get("/liveview.jpg", [](const httplib::Request&, httplib::Response& res) {
        std::vector<unsigned char> jpg;
        std::string err;
        if (!g_cam.liveViewJpeg(jpg, err)) {
            fail(res, err, 503);
            return;
        }
        res.set_header("Cache-Control", "no-store");
        res.set_content(reinterpret_cast<const char*>(jpg.data()), jpg.size(), "image/jpeg");
    });

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    if (autoConnect) {
        std::string err;
        if (!doConnect(err)) g_cam.log("Startup connect: " + err);
    }

    std::printf("\n  ILX-LR1 control panel -> http://%s:%d\n  Ctrl-C to quit\n\n",
                host.c_str(), port);
    std::fflush(stdout);

    if (!g_srv.listen(host.c_str(), port)) {
        std::fprintf(stderr, "Could not bind %s:%d (is another copy running?)\n",
                     host.c_str(), port);
        g_cam.disconnect();
        Camera::sdkRelease();
        return 1;
    }

    std::printf("\nShutting down...\n");
    g_cam.stopInterval();
    // Make sure we never leave the lens driving.
    std::string e;
    g_cam.zoomDrive(0, e);
    g_cam.focusDrive(0, e);
    g_cam.disconnect();
    Camera::sdkRelease();
    return 0;
}
