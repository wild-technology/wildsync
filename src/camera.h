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

// Defined in camera.cpp; shared with main.cpp's route builders.
std::string jsonEscapeStr(const std::string& s);

class Camera : public SDK::IDeviceCallback {
public:
    Camera() = default;
    // IDeviceCallback has no virtual destructor; we never delete through it.
    ~Camera();

    static bool sdkInit();
    static void sdkRelease();

    // Operator rule (docs/FIELD-RUN.md): this rig is ALWAYS manual focus, on
    // every path. It is enforced HERE, in the process that actually talks to
    // the SDK, not only in rigd - see Camera::setFocusMode. The escape hatch
    // for bench work is deliberately out-of-band: ilxctl --allow-autofocus,
    // which no field unit is ever started with.
    static void allowAutofocus(bool on);
    static bool autofocusAllowed();
    // The refusal text, shared with main.cpp so the door and the SDK layer
    // say the same thing.
    static std::string autofocusRefusal(const std::string& what);

    // X7: an error produced by a bounded SDK acquisition that timed out. It
    // means "this request never reached the body" - NOT "the body refused
    // it". The two must not be conflated: a refusal is evidence about the
    // CAMERA (rigcore records it as a divergence and alarms on it), this is
    // evidence about the daemon and a retry fixes it.
    static bool isBusyError(const std::string& err);

    // Which SDK call, if any, is holding the camera right now, and for how
    // long. Reads atomics only - it has to answer while the SDK mutex is
    // wedged, which is precisely when anyone asks.
    bool sdkBusyInfo(std::string& op, double& heldSec) const;

    // TEST ONLY (ilxctl --test-sdk-hold): hold the SDK mutex for ms on a
    // detached thread, imitating a body wedged inside an SDK call. There is
    // no other way to reproduce HANDOFF 2.1 off the hardware - the fakes
    // answer every property call instantly and hold no mutex.
    void holdSdkForTest(int ms);

    // Discovery / lifecycle -------------------------------------------------
    std::vector<CameraInfo> enumerate(int timeoutSec = 3);
    bool connect(int index, std::string& err,
                 SDK::CrSdkControlMode mode = SDK::CrSdkControlMode_Remote);
    // X7: returns false (with *err set to an isBusyError string) when the SDK
    // could not be taken - the session is left open rather than the worker
    // stranded. Callers that do not care may ignore both.
    bool disconnect(std::string* err = nullptr);
    SDK::CrSdkControlMode controlMode() const { return m_mode; }

    // Card drain (SDK "Remote Transfer"): list what the card holds, pull a
    // file to a directory, delete a content. Post-run only - never while a
    // survey is recording.
    struct CardEntry {
        CrInt32u contentId = 0;
        CrInt32u fileId = 0;
        CrInt32u fileNumber = 0;
        CrInt32u dirNumber = 0;
        std::string name;        // basename on the card, e.g. DSC09187.ARW
        long long size = 0;
        CrInt32u format = 0;     // CrContentsFile_FileFormat
        long long capturedUtc = 0;   // the BODY's clock (days wrong on this rig)
    };
    bool cardList(std::vector<CardEntry>& out, std::string& err, int maxNums = 4000);
    bool cardPull(CrInt32u contentId, CrInt32u fileId, const std::string& dir,
                  const std::string& fileName, long long& bytes, std::string& err,
                  int timeoutSec = 180);
    bool cardDelete(CrInt32u contentId, std::string& err);
    void OnNotifyRemoteTransferResult(CrInt32u notify, CrInt32u per, CrChar* filename) override;
    void OnNotifyRemoteTransferResult(CrInt32u notify, CrInt32u per, CrInt8u* data,
                                      CrInt64u size) override;
    void OnNotifyRemoteTransferContentsListChanged(CrInt32u notify, CrInt32u slotNumber,
                                                   CrInt32u addSize) override;
    bool cardIndexReady() const { return m_cardIndexReady.load(); }
    bool isConnected() const { return m_connected.load(); }
    // True while a connect attempt is in flight (X2/X5): the HTTP layer keys
    // its "pending, come back later" answers on this instead of piling more
    // workers onto m_sdkMutex behind a possibly-wedged SDK::Connect.
    bool isConnecting() const { return m_connecting.load(); }
    // Not const: the read is bounded on the SDK mutex (X7), which needs the
    // in-flight bookkeeping below.
    std::string modelName();

    // Aggregate state for the UI, already JSON-encoded.
    std::string statusJson();

    // Shooting --------------------------------------------------------------
    bool shutter(bool useAf, std::string& err);

    // Hold the release down for holdMs. With a continuous drive mode selected
    // the body runs the burst itself, so the whole sequence costs one Down/Up
    // pair rather than one round trip per frame.
    bool holdShutter(int holdMs, std::string& err);

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

    // Erase the card. Destructive; only ever reached from an explicit request.
    bool formatMedia(bool quick, std::string& err);

    // Set the body's clock from a UNIX epoch. EXIF timestamps are the only
    // common reference two cameras share, so they are worth disciplining to the
    // same master as the hosts.
    bool setDateTime(long long unixSeconds, std::string& err);

    // Hand shutter/settings control to the PC. Without this the body ignores
    // remote release entirely.
    void claimPcPriority();

    // "power=on, opmode=record, menu=no menu" - the body states that decide
    // whether any property is writable. Attached to every read-only error.
    std::string modeSummary();

    // mkdir -p. SetSaveInfo fails with 0x810c on a missing directory, and the
    // only symptom is that frames silently never leave the camera.
    static bool makeDirs(const std::string& dir);

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
    // X7 (wedged-SDK audit): bounded, wedge-aware acquisition of m_sdkMutex,
    // used by every HTTP-facing SDK entry point. A plain lock_guard is what
    // let one stuck body take the whole daemon down with it; the policy and
    // the two timeouts are documented at the top of camera.cpp.
    class SdkHold {
    public:
        SdkHold(Camera& cam, const char* op, std::chrono::milliseconds wait);
        ~SdkHold();
        SdkHold(const SdkHold&) = delete;
        SdkHold& operator=(const SdkHold&) = delete;
        explicit operator bool() const { return m_held; }
        // The error to hand back when the hold was refused. Never a claim
        // about the camera: isBusyError() recognises it as "not sent".
        std::string error() const;
        // cardList drops the mutex between index retries; the retake needs
        // the same bound the first take had.
        void release();
        bool retake(std::chrono::milliseconds wait);

    private:
        bool take(std::chrono::milliseconds wait);
        Camera& m_cam;
        const char* m_op;
        std::unique_lock<std::recursive_timed_mutex> m_lk;
        bool m_held = false;
    };
    void sdkEnter(const char* op);
    void sdkExit();
    // The two answers /api/status can give with no property table behind
    // them: a connect in flight, or an SDK call in the way. Same shape.
    std::string shortStatusJson(bool connected, const char* flag);
    // One place to say no to autofocus (the MF operator rule).
    bool autofocusBlocked(const std::string& what, std::string& err);

    // All SDK calls funnel through these, under m_sdkMutex.
    // getProp's `out` is only good for the scalar accessors (code, type, flags,
    // current value): its GetValues() buffer belongs to an SDK array that is
    // released before returning. Ranges go through getPropRange instead.
    bool getProp(CrInt32u code, SDK::CrDeviceProperty& out);
    bool getPropRange(CrInt32u code, PropRange& out);
    // Choice list of an array-typed property, extracted while the SDK still
    // owns the value buffer (getProp's shallow copy cannot provide it - see
    // the note above). elemBytes is the wire width, for masked comparison of
    // signed values (negative expcomp vs its UInt16 two's complement).
    bool getPropChoices(CrInt32u code, std::vector<long long>& out,
                        std::size_t& elemBytes);
    bool setProp(CrInt32u code, long long value, std::string& err);
    bool setPropLocked(CrInt32u code, long long value, std::string& err);
    // Write ignoring the property's enable flag, sweeping wire types and
    // confirming by read-back. For PriorityKeySettings, whose flag reads
    // non-writable precisely when the write is most needed. See camera.cpp.
    bool setPropForced(CrInt32u code, long long value, long long expect,
                       std::string& err);
    bool sendCmd(CrInt32u cmd, SDK::CrCommandParam param, std::string& err);
    bool captureOnce(bool useAf, std::string& err);
    // Single-frame release only works with the body's Interval REC disarmed.
    bool ensureIntervalRecOff(std::string& err);
    // Send the shutter release "up", retrying once after a beat. The release
    // must never be left latched, or a continuous drive mode runs the card flat.
    bool releaseUp(std::string& err);
    long long readProp(CrInt32u code, long long dflt);

    void intervalLoop(double intervalSec, int count, bool useAf);

    SDK::ICrEnumCameraObjectInfo* m_enumInfo = nullptr;
    SDK::CrDeviceHandle m_handle = 0;
    std::atomic<bool> m_connected{false};
    // Set when connect() gives up on a timed-out/refused attempt so a late
    // OnConnected from that same attempt is ignored rather than resurrecting a
    // dead session (connected==true while m_handle==0).
    std::atomic<bool> m_attemptAbandoned{false};
    SDK::CrSdkControlMode m_mode = SDK::CrSdkControlMode_Remote;
    // remote-transfer completion handshake
    std::mutex m_rtMutex;
    std::condition_variable m_rtCv;
    int m_rtResult = 0;          // 0 pending, 1 ok, -1 failed, -2 busy, -3 dropped
    CrInt32u m_rtPercent = 0;
    std::string m_rtFile;
    // The basename the CURRENT cardPull is waiting for (X4). The result slot
    // above is unkeyed, so a late Result_OK from a previous timed-out
    // transfer used to complete the NEXT pull's wait; the callback now drops
    // any result whose reported path does not end in this name.
    std::string m_rtWant;
    std::atomic<bool> m_cardIndexReady{false};
    // True from just before SDK::Connect until the attempt concludes; lets
    // statusJson answer without the SDK mutex while a connect blocks.
    std::atomic<bool> m_connecting{false};
    // Re-claims PC control priority after a spontaneous SDK reconnect; joined in
    // disconnect() so it never calls into a released SDK core at shutdown.
    std::thread m_priorityThread;
    std::mutex m_priorityThreadMutex;
    std::string m_model;
    std::string m_id;
    // Last save location handed to SetSaveInfo. The SDK forgets it across a
    // reconnect, so it is replayed from OnConnected.
    std::string m_saveDir;

    // Timed (X2): the HTTP-facing entry points bound their wait on this so a
    // wedged SDK call (a stuck SDK::Connect holds it until the body is
    // power-cycled) can never consume the whole httplib worker pool.
    mutable std::recursive_timed_mutex m_sdkMutex;
    // X7: who is inside the SDK, and since when. Deliberately atomics and NOT
    // guarded by m_sdkMutex - the only moment this matters is when that mutex
    // cannot be taken. Depth counts holders (nested takes by the owning
    // thread included); op/since describe the outermost one.
    std::atomic<int> m_sdkDepth{0};
    std::atomic<long long> m_sdkSinceMs{0};
    std::atomic<const char*> m_sdkOp{nullptr};

    // Connect completion handshake.
    std::mutex m_connectMutex;
    std::condition_variable m_connectCv;
    int m_connectResult = 0;  // 0 pending, 1 ok, -1 failed
    CrInt32u m_lastError = 0;

    // Format completion handshake. The SDK reports the end of a card format
    // only through OnWarning (CrWarning_Format_Complete / _Failed / _Invalid /
    // _Canceled); formatMedia arms this and waits for the callback.
    std::mutex m_fmtMutex;
    std::condition_variable m_fmtCv;
    bool m_fmtArmed = false;
    int m_fmtOutcome = 0;     // 0 pending, 1 complete, -1 failed/invalid/canceled
    CrInt32u m_fmtWarning = 0;

    // Intervalometer. m_intervalThreadMutex guards join/assign of the thread
    // object itself (two HTTP threads may race stop against stop/start); it is
    // separate from m_intervalMutex because the loop takes that one to publish
    // progress, and joining while holding it would deadlock.
    std::thread m_intervalThread;
    std::mutex m_intervalThreadMutex;
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
