// ilxctl - a small control panel for the Sony ILX-LR1.
//
// Serves a single-page UI on localhost and drives the camera over USB (or IP)
// through the Camera Remote SDK: shutter, a drift-free intervalometer, manual
// focus, and optical zoom on a power-zoom lens.

#include <sys/stat.h>

#include <atomic>
#include <cctype>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "camera.h"
#include "httplib.h"
#include "web_ui.h"

namespace {

Camera g_cam;
httplib::Server g_srv;
std::atomic<bool> g_stopping{false};

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
        e += c;
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

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--port" && i + 1 < argc) port = std::atoi(argv[++i]);
        else if (a == "--host" && i + 1 < argc) host = argv[++i];
        else if (a == "--save-dir" && i + 1 < argc) saveDir = argv[++i];
        else if (a == "--no-autoconnect") autoConnect = false;
        else if (a == "--help" || a == "-h") {
            std::printf(
                "ilxctl - Sony ILX-LR1 control panel\n\n"
                "  --port N          HTTP port (default 8080)\n"
                "  --host ADDR       bind address (default 127.0.0.1)\n"
                "  --save-dir PATH   where to write images the camera sends to the PC\n"
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

    if (!Camera::sdkInit()) {
        std::fprintf(stderr,
                     "Failed to initialise the Camera Remote SDK.\n"
                     "Check that libCr_Core.dylib and CrAdapter/ sit next to this binary\n"
                     "and that the quarantine attribute has been removed.\n");
        return 1;
    }
    g_cam.log("Camera Remote SDK initialised");

    auto doConnect = [&](std::string& err) {
        const auto cams = g_cam.enumerate(3);
        if (cams.empty()) {
            err = "no camera found - check USB, that the camera is on, and that "
                  "USB Connection Mode is PC Remote";
            return false;
        }
        // Prefer an ILX-LR1 if several bodies are attached.
        int pick = 0;
        for (std::size_t i = 0; i < cams.size(); ++i) {
            g_cam.log("Found [" + std::to_string(i) + "] " + cams[i].model + " (" +
                      cams[i].conn + " " + cams[i].id + ")");
            if (cams[i].model.find("ILX-LR1") != std::string::npos) pick = static_cast<int>(i);
        }
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

    g_srv.Post("/api/interval/start", [](const httplib::Request& req, httplib::Response& res) {
        const double sec = jsonNum(req.body, "intervalSec", 1.0);
        const int count = static_cast<int>(jsonNum(req.body, "count", 0));
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
        const int shots = static_cast<int>(jsonNum(req.body, "shots", 0));
        const int delay = static_cast<int>(jsonNum(req.body, "startDelaySec", -1));
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
        const long long v = static_cast<long long>(jsonNum(req.body, "mode", 1));
        guarded(res, [&](std::string& e) { return g_cam.setFocusMode(v, e); });
    });

    g_srv.Post("/api/focus/drive", [](const httplib::Request& req, httplib::Response& res) {
        const int step = static_cast<int>(jsonNum(req.body, "step", 0));
        guarded(res, [&](std::string& e) { return g_cam.focusDrive(step, e); });
    });

    g_srv.Post("/api/focus/position", [](const httplib::Request& req, httplib::Response& res) {
        const long long v = static_cast<long long>(jsonNum(req.body, "value", 0));
        guarded(res, [&](std::string& e) { return g_cam.setFocusPosition(v, e); });
    });

    g_srv.Post("/api/zoom/drive", [](const httplib::Request& req, httplib::Response& res) {
        const int speed = static_cast<int>(jsonNum(req.body, "speed", 0));
        guarded(res, [&](std::string& e) { return g_cam.zoomDrive(speed, e); });
    });

    g_srv.Post("/api/zoom/position", [](const httplib::Request& req, httplib::Response& res) {
        const long long v = static_cast<long long>(jsonNum(req.body, "value", 0));
        guarded(res, [&](std::string& e) { return g_cam.setZoomPosition(v, e); });
    });

    g_srv.Post("/api/zoom/setting", [](const httplib::Request& req, httplib::Response& res) {
        const long long v = static_cast<long long>(jsonNum(req.body, "value", 1));
        guarded(res, [&](std::string& e) { return g_cam.setZoomSetting(v, e); });
    });

    g_srv.Post("/api/store", [](const httplib::Request& req, httplib::Response& res) {
        const long long v = static_cast<long long>(jsonNum(req.body, "value", 2));
        guarded(res, [&](std::string& e) { return g_cam.setStoreDestination(v, e); });
    });

    g_srv.Post("/api/exposure", [](const httplib::Request& req, httplib::Response& res) {
        const std::string which = jsonStr(req.body, "which");
        const long long v = static_cast<long long>(jsonNum(req.body, "value", 0));
        guarded(res, [&](std::string& e) { return g_cam.setExposure(which, v, e); });
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
