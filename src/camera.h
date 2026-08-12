// Camera wrapper around the Sony Camera Remote SDK, scoped to the controls the
// ILX-LR1 + FE PZ 16-35mm F4 G actually supports.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "CRSDK/CameraRemote_SDK.h"
#include "CRSDK/IDeviceCallback.h"

namespace SDK = SCRSDK;

// A property's [min, max, step] triple, as reported by the camera. Ranges are
// model- and lens-dependent, so everything the UI offers is derived from these
// rather than hard-coded.
struct PropRange {
    bool valid = false;
    long long min = 0;
    long long max = 0;
    long long step = 1;
};

// One selectable value of an enumerated property, with a human label.
struct PropChoice {
    long long value = 0;
    std::string label;
};

// Floor for the host-driven loop. A full release round trip over USB takes a
// few hundred ms, so anything below this cannot be honoured anyway - and
// accepting it would turn a mistyped field into a runaway sequence.
constexpr double kMinHostIntervalSec = 0.5;

struct CameraInfo {
    std::string model;
    std::string id;
    std::string conn;
};

// Snapshot of the intervalometer's state, safe to read from the HTTP threads.
struct IntervalStatus {
    bool running = false;
    int taken = 0;
    int target = 0;  // 0 == run until stopped
    double intervalSec = 1.0;
    bool useAf = false;
    std::string lastError;
};

class Camera : public SDK::IDeviceCallback {
public:
    Camera() = default;
    // IDeviceCallback has no virtual destructor; we never delete through it.
    ~Camera();

    static bool sdkInit();
    static void sdkRelease();

    // Discovery / lifecycle -------------------------------------------------
    std::vector<CameraInfo> enumerate(int timeoutSec = 3);
    bool connect(int index, std::string& err);
    void disconnect();
    bool isConnected() const { return m_connected.load(); }
    std::string modelName() const;

    // Aggregate state for the UI, already JSON-encoded.
    std::string statusJson();

    // Shooting --------------------------------------------------------------
    bool shutter(bool useAf, std::string& err);

    // Host-driven intervalometer: this process fires the shutter on a timer.
    // Arbitrary intervals and per-frame feedback, but it needs the USB link up.
    bool startInterval(double intervalSec, int count, bool useAf, std::string& err);
    void stopInterval();
    IntervalStatus intervalStatus();

    // The camera's own Interval REC. Timing is generated inside the body, so
    // it is exact and survives a dropped connection. While it is armed the
    // body owns the shutter: a release toggles the sequence rather than taking
    // a single frame, and most settings become read-only.
    bool configureCameraInterval(double intervalSec, int shots, int startDelaySec,
                                 std::string& err);
    bool setCameraIntervalArmed(bool armed, std::string& err);
    bool cameraIntervalRun(bool start, std::string& err);

    // Focus -----------------------------------------------------------------
    bool setFocusMode(long long mode, std::string& err);
    // NearFar drive: negative == near, positive == far, 0 == stop.
    bool focusDrive(int step, std::string& err);
    bool setFocusPosition(long long value, std::string& err);

    // Zoom ------------------------------------------------------------------
    // Zoom_Operation: negative == wide, positive == tele, 0 == stop.
    bool zoomDrive(int speed, std::string& err);
    bool setZoomPosition(long long value, std::string& err);
    bool setZoomSetting(long long value, std::string& err);

    // Exposure --------------------------------------------------------------
    bool setExposure(const std::string& which, long long value, std::string& err);

    // Where the camera writes each frame: card, host PC, or both.
    bool setStoreDestination(long long value, std::string& err);

    // Hand shutter/settings control to the PC. Without this the body ignores
    // remote release entirely.
    void claimPcPriority();

    // Live view -------------------------------------------------------------
    bool liveViewJpeg(std::vector<unsigned char>& out, std::string& err);

    // Where captured images land when the camera is set to send them to the PC.
    bool setSaveDir(const std::string& dir, std::string& err);

    // IDeviceCallback -------------------------------------------------------
    void OnConnected(SDK::DeviceConnectionVersioin version) override;
    void OnDisconnected(CrInt32u error) override;
    void OnError(CrInt32u error) override;
    void OnWarning(CrInt32u warning) override;
    void OnPropertyChanged() override {}
    void OnPropertyChangedCodes(CrInt32u num, CrInt32u* codes) override;
    void OnLvPropertyChanged() override {}
    void OnLvPropertyChangedCodes(CrInt32u num, CrInt32u* codes) override {}
    void OnCompleteDownload(CrChar* filename, CrInt32u type) override;
    void OnNotifyContentsTransfer(CrInt32u notify, SDK::CrContentHandle handle,
                                  CrChar* filename) override {}

    // Rolling log shown in the UI.
    std::vector<std::string> takeLog();
    void log(const std::string& msg);

private:
    // All SDK calls funnel through these, under m_sdkMutex.
    bool getProp(CrInt32u code, SDK::CrDeviceProperty& out);
    bool setProp(CrInt32u code, long long value, std::string& err);
    bool setPropLocked(CrInt32u code, long long value, std::string& err);
    bool sendCmd(CrInt32u cmd, SDK::CrCommandParam param, std::string& err);
    bool captureOnce(bool useAf, std::string& err);
    // Single-frame release only works with the body's Interval REC disarmed.
    bool ensureIntervalRecOff(std::string& err);
    long long readProp(CrInt32u code, long long dflt);

    void intervalLoop(double intervalSec, int count, bool useAf);

    SDK::ICrEnumCameraObjectInfo* m_enumInfo = nullptr;
    SDK::CrDeviceHandle m_handle = 0;
    std::atomic<bool> m_connected{false};
    std::string m_model;
    std::string m_id;

    mutable std::recursive_mutex m_sdkMutex;

    // Connect completion handshake.
    std::mutex m_connectMutex;
    std::condition_variable m_connectCv;
    int m_connectResult = 0;  // 0 pending, 1 ok, -1 failed
    CrInt32u m_lastError = 0;

    // Intervalometer.
    std::thread m_intervalThread;
    std::atomic<bool> m_intervalRun{false};
    std::mutex m_intervalMutex;
    IntervalStatus m_interval;

    std::mutex m_logMutex;
    std::vector<std::string> m_log;
};

// Value formatters matching Sony's own display conventions.
std::string formatFNumber(unsigned int v);
std::string formatIso(unsigned int v);
std::string formatShutterSpeed(unsigned int v);
std::string formatFocusMode(long long v);
std::string formatZoomSetting(long long v);
std::string formatExposureProgram(long long v);
std::string formatDriveMode(long long v);
std::string formatStoreDestination(long long v);
std::string formatSlotStatus(long long v);
std::string crErrorString(CrInt32u err);
