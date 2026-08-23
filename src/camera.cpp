#include "camera.h"

#include <sys/stat.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "CRSDK/CrDeviceProperty.h"
#include "CRSDK/CrImageDataBlock.h"

using namespace std::chrono_literals;

namespace {

// Element width implied by a CrDataType, in bytes.
std::size_t elemSize(SDK::CrDataType t) {
    switch (t & 0x0FFF) {
        case SDK::CrDataType_UInt8: return 1;
        case SDK::CrDataType_UInt16: return 2;
        case SDK::CrDataType_UInt32: return 4;
        case SDK::CrDataType_UInt64: return 8;
        default: return 0;
    }
}

bool isSigned(SDK::CrDataType t) { return (t & SDK::CrDataType_SignBit) != 0; }

// Read one element out of a property's raw value buffer, sign-extending when
// the declared type is signed.
long long readElem(const unsigned char* buf, std::size_t idx, SDK::CrDataType t) {
    const std::size_t sz = elemSize(t);
    unsigned long long raw = 0;
    std::memcpy(&raw, buf + idx * sz, sz);
    if (isSigned(t) && sz < 8) {
        const unsigned long long signBit = 1ULL << (sz * 8 - 1);
        if (raw & signBit) raw |= ~((1ULL << (sz * 8)) - 1);
        return static_cast<long long>(raw);
    }
    return static_cast<long long>(raw);
}

// Range-typed properties report [min, max, step] in their value buffer.
PropRange rangeOf(const SDK::CrDeviceProperty& p) {
    PropRange r;
    const SDK::CrDataType t = p.GetValueType();
    const std::size_t sz = elemSize(t);
    if (sz == 0 || !(t & SDK::CrDataType_RangeBit)) return r;
    const unsigned char* buf = p.GetValues();
    const std::size_t n = buf ? p.GetValueSize() / sz : 0;
    if (n < 3) return r;
    r.min = readElem(buf, 0, t);
    r.max = readElem(buf, 1, t);
    r.step = readElem(buf, 2, t);
    if (r.step == 0) r.step = 1;
    r.valid = true;
    return r;
}

// Array-typed properties report the full set of selectable values.
std::vector<long long> choicesOf(const SDK::CrDeviceProperty& p) {
    std::vector<long long> out;
    const SDK::CrDataType t = p.GetValueType();
    const std::size_t sz = elemSize(t);
    if (sz == 0 || !(t & SDK::CrDataType_ArrayBit)) return out;
    const unsigned char* buf = p.GetValues();
    const std::size_t n = buf ? p.GetValueSize() / sz : 0;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) out.push_back(readElem(buf, i, t));
    return out;
}

std::string jsonEscape(const std::string& s) {
    std::string o;
    o.reserve(s.size() + 8);
    for (const char c : s) {
        switch (c) {
            case '"': o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\n': o += "\\n"; break;
            case '\r': o += "\\r"; break;
            case '\t': o += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char b[8];
                    std::snprintf(b, sizeof(b), "\\u%04x", c);
                    o += b;
                } else {
                    o += c;
                }
        }
    }
    return o;
}

std::string rangeJson(const PropRange& r) {
    if (!r.valid) return "null";
    std::ostringstream os;
    os << "{\"min\":" << r.min << ",\"max\":" << r.max << ",\"step\":" << r.step << "}";
    return os.str();
}

std::string timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()) % 1000;
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d", tm.tm_hour, tm.tm_min, tm.tm_sec,
                  static_cast<int>(ms.count()));
    return buf;
}

}  // namespace

std::string jsonEscapeStr(const std::string& s) { return jsonEscape(s); }

// ---------------------------------------------------------------------------
// Formatters (mirroring Sony's RemoteCli display conventions)
// ---------------------------------------------------------------------------

std::string formatFNumber(unsigned int v) {
    if (v == 0 || v == SDK::CrFnumber_Unknown) return "--";
    if (v == SDK::CrFnumber_Nothing) return "";
    std::ostringstream os;
    os << "F";
    if (v % 100) {
        os << std::fixed << std::setprecision(1) << (v / 100.0);
    } else {
        os << (v / 100);
    }
    return os.str();
}

std::string formatIso(unsigned int v) {
    const unsigned int mode = (v >> 24) & 0x0F;
    const unsigned int value = v & 0x00FFFFFF;
    std::string prefix;
    if (mode == SDK::CrISO_MultiFrameNR) prefix = "MFNR ";
    else if (mode == SDK::CrISO_MultiFrameNR_High) prefix = "MFNR-Hi ";
    if (value == SDK::CrISO_AUTO) return prefix + "ISO AUTO";
    return prefix + "ISO " + std::to_string(value);
}

std::string formatShutterSpeed(unsigned int v) {
    if (v == SDK::CrShutterSpeed_Bulb) return "Bulb";
    if (v == SDK::CrShutterSpeed_Nothing) return "--";
    const unsigned int num = (v >> 16) & 0xFFFF;
    const unsigned int den = v & 0xFFFF;
    if (den == 0) return "--";
    std::ostringstream os;
    if (num == 1) {
        os << "1/" << den;
    } else if (num % den == 0) {
        os << (num / den) << "\"";
    } else {
        os << (num / den) << "." << (num % den) << "\"";
    }
    return os.str();
}

std::string formatFocusMode(long long v) {
    switch (v) {
        case SDK::CrFocus_MF: return "MF";
        case SDK::CrFocus_AF_S: return "AF-S";
        case SDK::CrFocus_AF_C: return "AF-C";
        case SDK::CrFocus_AF_A: return "AF-A";
        case SDK::CrFocus_AF_D: return "AF-D";
        case SDK::CrFocus_DMF: return "DMF";
        case SDK::CrFocus_PF: return "PF";
        default: return "?";
    }
}

std::string formatZoomSetting(long long v) {
    switch (v) {
        case SDK::CrZoomSetting_OpticalZoomOnly: return "Optical only";
        case SDK::CrZoomSetting_SmartZoomOnly: return "Smart zoom";
        case SDK::CrZoomSetting_On_ClearImageZoom: return "Clear image zoom";
        case SDK::CrZoomSetting_On_DigitalZoom: return "Digital zoom";
        default: return "?";
    }
}

std::string formatExposureProgram(long long v) {
    switch (v) {
        case SDK::CrExposure_M_Manual: return "M";
        case SDK::CrExposure_P_Auto: return "P";
        case SDK::CrExposure_A_AperturePriority: return "A";
        case SDK::CrExposure_S_ShutterSpeedPriority: return "S";
        case SDK::CrExposure_Auto: return "Auto";
        case SDK::CrExposure_Auto_Plus: return "Auto+";
        case SDK::CrExposure_Movie_P: return "Movie P";
        case SDK::CrExposure_Movie_A: return "Movie A";
        case SDK::CrExposure_Movie_S: return "Movie S";
        case SDK::CrExposure_Movie_M: return "Movie M";
        case SDK::CrExposure_Movie_Auto: return "Movie Auto";
        default: {
            std::ostringstream os;
            os << "0x" << std::hex << v;
            return os.str();
        }
    }
}

std::string formatDriveMode(long long v) {
    switch (v) {
        case SDK::CrDrive_Single: return "Single";
        case SDK::CrDrive_Continuous_Hi: return "Cont. Hi";
        case SDK::CrDrive_Continuous_Hi_Plus: return "Cont. Hi+";
        case SDK::CrDrive_Continuous_Lo: return "Cont. Lo";
        case SDK::CrDrive_Continuous: return "Cont.";
        default: {
            std::ostringstream os;
            os << "0x" << std::hex << v;
            return os.str();
        }
    }
}

std::string formatStoreDestination(long long v) {
    switch (v) {
        case SDK::CrStillImageStoreDestination_HostPC: return "Host PC";
        case SDK::CrStillImageStoreDestination_MemoryCard: return "Memory card";
        case SDK::CrStillImageStoreDestination_HostPCAndMemoryCard: return "PC + card";
        default: return "?";
    }
}

std::string formatSlotStatus(long long v) {
    switch (v) {
        case SDK::CrSlotStatus_OK: return "OK";
        case SDK::CrSlotStatus_NoCard: return "no card";
        case SDK::CrSlotStatus_CardError: return "card error";
        case SDK::CrSlotStatus_RecognizingOrLockedError: return "locked/recognizing";
        case SDK::CrSlotStatus_DBError: return "database error";
        case SDK::CrSlotStatus_CardRecognizing: return "recognizing";
        case SDK::CrSlotStatus_CardLockedAndDBError: return "locked + DB error";
        case SDK::CrSlotStatus_DBError_CantRepairAndNeedFormat: return "needs format";
        case SDK::CrSlotStatus_CardError_ReadOnlyMedia: return "read-only";
        default: return "?";
    }
}

// Every property carries an enable flag saying whether the SDK may write it
// right now. "False" is a transient body-state lock we can often clear;
// "DisplayOnly" means this body only ever accepts the setting from its own menu.
// Telling those two apart is the difference between a software fix and a trip to
// the camera, so the flag is surfaced verbatim in every read-only error.
std::string formatEnableFlag(long long v) {
    switch (v) {
        case SDK::CrEnableValue_NotSupported: return "NotSupported(-1)";
        case SDK::CrEnableValue_False: return "False(0) - locked by camera state";
        case SDK::CrEnableValue_True: return "True(1)";
        case SDK::CrEnableValue_DisplayOnly: return "DisplayOnly(2) - body menu only";
        case SDK::CrEnableValue_SetOnly: return "SetOnly(3)";
        // Invalid/Invariable/Variable are aliases of False/True/DisplayOnly, so
        // they cannot be distinguished here and deliberately are not listed.
        default: return "flag=" + std::to_string(v);
    }
}

std::string formatPowerStatus(long long v) {
    switch (v) {
        case SDK::CrCameraPowerStatus_Off: return "off";
        case SDK::CrCameraPowerStatus_Standby: return "standby";
        case SDK::CrCameraPowerStatus_PowerOn: return "on";
        case SDK::CrCameraPowerStatus_TransitioningFromPowerOnToStandby: return "->standby";
        case SDK::CrCameraPowerStatus_TransitioningFromStandbyToPowerOn: return "->on";
        default: return "?";
    }
}

std::string formatOperatingMode(long long v) {
    switch (v) {
        case SDK::CrCameraOperatingMode_Record: return "record";
        case SDK::CrCameraOperatingMode_Playback: return "playback";
        default: return "?";
    }
}

// The LOCK switch disables the control wheel and every button except the
// shutter, so a locked body looks like a dead menu while still shooting. The
// SDK can read and clear it, which is the only way in without the menu.
std::string formatKeyLock(long long v) {
    switch (v) {
        case SDK::CrLockIndicator_Unlocked: return "unlocked";
        case SDK::CrLockIndicator_Locked: return "LOCKED";
        default: return "?";
    }
}

std::string formatMenuStatus(long long v) {
    switch (v) {
        case SDK::CrDisplayedMenuStatus_Off: return "no menu";
        case SDK::CrDisplayedMenuStatus_StatusMenu: return "status menu";
        case SDK::CrDisplayedMenuStatus_FullMenu: return "FULL MENU";
        default: return "?";
    }
}

std::string crErrorString(CrInt32u err) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "0x%08x", err);
    return buf;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

Camera::~Camera() {
    stopInterval();
    disconnect();
}

bool Camera::sdkInit() { return SDK::Init(); }
void Camera::sdkRelease() { SDK::Release(); }

void Camera::log(const std::string& msg) {
    std::lock_guard<std::mutex> lk(m_logMutex);
    m_log.push_back(timestamp() + "  " + msg);
    if (m_log.size() > 200) m_log.erase(m_log.begin(), m_log.begin() + (m_log.size() - 200));
    std::printf("[%s] %s\n", timestamp().c_str(), msg.c_str());
    std::fflush(stdout);
}

std::vector<std::string> Camera::takeLog() {
    std::lock_guard<std::mutex> lk(m_logMutex);
    return m_log;
}

std::string Camera::modelName() const {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    return m_model;
}

std::vector<CameraInfo> Camera::enumerate(int timeoutSec) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    std::vector<CameraInfo> out;
    if (m_enumInfo) {
        m_enumInfo->Release();
        m_enumInfo = nullptr;
    }
    const SDK::CrError err =
        SDK::EnumCameraObjects(&m_enumInfo, static_cast<CrInt8u>(timeoutSec));
    if (err != SDK::CrError_None || !m_enumInfo) {
        log("EnumCameraObjects found nothing (" + crErrorString(err) + ")");
        return out;
    }
    const CrInt32u n = m_enumInfo->GetCount();
    for (CrInt32u i = 0; i < n; ++i) {
        const auto* info = m_enumInfo->GetCameraObjectInfo(i);
        if (!info) continue;
        CameraInfo ci;
        ci.model = info->GetModel() ? info->GetModel() : "";
        ci.conn = info->GetConnectionTypeName() ? info->GetConnectionTypeName() : "";
        if (ci.conn == "IP") {
            ci.id = info->GetMACAddressChar() ? info->GetMACAddressChar() : "";
        } else {
            ci.id = info->GetId() ? reinterpret_cast<const char*>(info->GetId()) : "";
        }
        out.push_back(ci);
    }
    return out;
}

bool Camera::connect(int index, std::string& err, SDK::CrSdkControlMode mode) {
    std::unique_lock<std::recursive_mutex> lk(m_sdkMutex);
    if (m_handle) {
        bool pending;
        {
            std::lock_guard<std::mutex> clk(m_connectMutex);
            pending = (m_connectResult == 0);
        }
        if (m_connected || pending) {
            // A second racing /api/connect must not overwrite (and leak) the
            // handle of a connection that is already up or underway.
            err = "already connected";
            return false;
        }
        // A handle left behind by a dropped session: OnDisconnected clears
        // m_connected but keeps m_handle, so every later connect was refused
        // with "already connected" forever and only a daemon restart brought
        // the body back (rigd re-POSTed into that wall every backoff,
        // 2026-08-23). Release the stale handle and reconnect.
        log("Stale SDK handle from a dropped session - releasing before reconnect");
        SDK::Disconnect(m_handle);
        SDK::ReleaseDevice(m_handle);
        m_handle = 0;
    }
    if (!m_enumInfo || index < 0 ||
        static_cast<CrInt32u>(index) >= m_enumInfo->GetCount()) {
        err = "no camera at index " + std::to_string(index) + " (run discovery first)";
        return false;
    }
    auto* obj = const_cast<SDK::ICrCameraObjectInfo*>(m_enumInfo->GetCameraObjectInfo(index));
    m_model = obj->GetModel() ? obj->GetModel() : "";
    m_id = obj->GetId() ? reinterpret_cast<const char*>(obj->GetId()) : "";

    m_attemptAbandoned = false;
    {
        std::lock_guard<std::mutex> clk(m_connectMutex);
        m_connectResult = 0;
    }

    m_connecting = true;
    struct ConnectingGuard {
        std::atomic<bool>& f;
        ~ConnectingGuard() { f = false; }
    } connectingGuard{m_connecting};
    m_mode = mode;
    // A RemoteTransfer session must NOT auto-reconnect: the SDK's reconnect
    // (and any concurrent remote claim) tears the transfer session down a few
    // seconds in, so the card index never settles and every list answers
    // 0x8D05 forever (observed 2026-08-23). Transfer sessions are short and
    // supervised by the drain; remote sessions keep auto-reconnect.
    const SDK::CrReconnectingSet reconnect =
        (mode == SDK::CrSdkControlMode_RemoteTransfer) ? SDK::CrReconnecting_OFF
                                                       : SDK::CrReconnecting_ON;
    if (mode == SDK::CrSdkControlMode_RemoteTransfer) m_cardIndexReady = false;
    const SDK::CrError e = SDK::Connect(obj, this, &m_handle, mode, reconnect);
    if (e != SDK::CrError_None) {
        err = "Connect failed: " + crErrorString(e);
        log(err);
        return false;
    }
    // Waiting for the callback with m_sdkMutex held would stall /api/status
    // and every other SDK call for up to 15 s.
    lk.unlock();

    auto abandon = [this] {
        m_attemptAbandoned = true;
        std::lock_guard<std::recursive_mutex> relk(m_sdkMutex);
        if (m_handle) {
            SDK::Disconnect(m_handle);
            SDK::ReleaseDevice(m_handle);
            m_handle = 0;
        }
        m_connected = false;
    };

    std::unique_lock<std::mutex> clk(m_connectMutex);
    if (!m_connectCv.wait_for(clk, 15s, [this] { return m_connectResult != 0; })) {
        err = "timed out waiting for the camera to accept the connection";
        log(err);
        clk.unlock();
        abandon();
        return false;
    }
    if (m_connectResult < 0) {
        err = "camera refused the connection (" + crErrorString(m_lastError) +
              ") - is Imaging Edge or another app still holding it?";
        log(err);
        clk.unlock();
        abandon();
        return false;
    }

    clk.unlock();
    claimPcPriority();
    return true;
}

// The body silently ignores a remote shutter release while control priority
// sits with the camera, so claim it for the PC. The property list is not
// populated the instant OnConnected fires, hence the retries.
void Camera::claimPcPriority() {
    for (int attempt = 0; attempt < 10 && m_connected; ++attempt) {
        SDK::CrDeviceProperty pk;
        if (getProp(SDK::CrDeviceProperty_PriorityKeySettings, pk)) {
            if (pk.GetCurrentValue() == SDK::CrPriorityKey_PCRemote) {
                if (attempt) log("Control priority is with the PC");
                return;
            }
            std::string err;
            // Forced: see setPropForced. While priority sits with the camera
            // the body marks this property non-writable, so honouring the
            // enable flag here would make the state permanent.
            if (setPropForced(SDK::CrDeviceProperty_PriorityKeySettings,
                              SDK::CrPriorityKey_PCRemote,
                              SDK::CrPriorityKey_PCRemote, err)) {
                log("Control priority handed to the PC");
                return;
            }
            if (attempt == 9) {
                log("Could not take PC control priority: " + err +
                    " - the camera will ignore the remote shutter");
                return;
            }
        }
        std::this_thread::sleep_for(300ms);
    }
    log("Control priority could not be read - the remote shutter may not fire");
}

// One-line snapshot of the body states that gate whether properties are
// writable at all: a camera in standby, in playback, or sitting on its own menu
// reports the whole property table read-only, and from the outside that is
// indistinguishable from a broken camera. Cheap enough to attach to every
// read-only error, which is exactly where it is needed.
std::string Camera::modeSummary() {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (!m_handle) return "not connected";
    std::string s;
    auto add = [&](const char* label, CrInt32u code, std::string (*fmt)(long long)) {
        SDK::CrDeviceProperty q;
        if (!s.empty()) s += ", ";
        s += label;
        s += "=";
        s += getProp(code, q) ? fmt(static_cast<long long>(q.GetCurrentValue())) : "?";
    };
    // CameraPowerStatus and DisplayedMenuStatus are NOT supported on the
    // ILX-LR1 (confirmed against Sony's v2.02.00 per-model property matrix, and
    // measured: both read back "?"). Reporting them was worse than useless - it
    // invited a menu theory that the body cannot even confirm. These are the
    // states this body actually publishes, and each one really can stop a
    // release: playback mode, a locked body, a card fault, a caution modal.
    add("opmode", SDK::CrDeviceProperty_CameraOperatingMode, formatOperatingMode);
    add("keylock", SDK::CrDeviceProperty_BodyKeyLock, formatKeyLock);
    add("slot1", SDK::CrDeviceProperty_MediaSLOT1_Status, formatSlotStatus);
    add("program", SDK::CrDeviceProperty_ExposureProgramMode, formatExposureProgram);
    SDK::CrDeviceProperty caution;
    if (getProp(SDK::CrDeviceProperty_CameraErrorCautionStatus, caution)) {
        // CrCameraErrorCautionStatus is 1-based: NoError = 0x01, Error = 0x02.
        // Testing `!= 0` therefore reported "CAUTION ON BODY (0x00000001)" on a
        // perfectly healthy body - i.e. on EVERY read-only error this function
        // ever formatted. That sent operators hunting for a modal on a
        // screenless camera that was never there, and it is quoted in the
        // handover docs as though it were a real diagnosis. Only 0x02 (and any
        // future non-NoError value) is an actual caution.
        const long long c = static_cast<long long>(caution.GetCurrentValue());
        if (c != 0 && c != SDK::CrCameraErrorCautionStatus_NoError) {
            s += ", CAUTION ON BODY (0x" +
                 crErrorString(static_cast<CrInt32u>(c)).substr(2) + ")";
        }
    }
    return s;
}

void Camera::disconnect() {
    stopInterval();
    // Drop the flag first so any in-flight claimPcPriority loop exits promptly,
    // then join it before we touch the SDK so it can never run against a
    // released core.
    m_connected = false;
    {
        std::lock_guard<std::mutex> lk(m_priorityThreadMutex);
        if (m_priorityThread.joinable()) m_priorityThread.join();
    }
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (m_handle) {
        SDK::Disconnect(m_handle);
        SDK::ReleaseDevice(m_handle);
        m_handle = 0;
    }
    if (m_enumInfo) {
        m_enumInfo->Release();
        m_enumInfo = nullptr;
    }
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void Camera::OnConnected(SDK::DeviceConnectionVersioin) {
    if (m_attemptAbandoned) {
        // connect() already gave up on this attempt and released the handle;
        // resurrecting m_connected here would report a live camera with no
        // handle, so every op fails until a manual reconnect. Ignore it.
        log("Ignoring a late connect callback for an abandoned attempt");
        return;
    }
    m_connected = true;
    log("Connected to " + m_model);
    bool wasExplicit;
    {
        std::lock_guard<std::mutex> lk(m_connectMutex);
        wasExplicit = (m_connectResult == 0);   // an explicit connect() is waiting
        m_connectResult = 1;
        m_connectCv.notify_all();
    }
    // The SDK reconnects on its own after a USB drop (CrReconnecting_ON) and
    // fires this callback, but control priority is handed back to the camera
    // body on the drop. If we do not re-take it, the body keeps priority: the
    // remote shutter is ignored, PC-save reverts to the card, and the body's
    // own menu stays locked ("PC" shows white, not yellow). The explicit
    // connect() path already claims priority itself, so only do it here for a
    // spontaneous reconnect. Detached because this is an SDK callback and
    // claimPcPriority makes blocking SDK calls with retries.
    if (!wasExplicit) {
        std::lock_guard<std::mutex> lk(m_priorityThreadMutex);
        // Detach any prior claimer (it self-terminates once m_connected drops)
        // and track the current one so disconnect() can join it before the SDK
        // core is released.
        if (m_priorityThread.joinable()) {
            m_priorityThread.detach();
        }
        m_priorityThread = std::thread([this] {
            claimPcPriority();
            // The save location does not survive a reconnect either, and losing
            // it is silent: the camera keeps shooting and simply stops sending
            // frames to the host. Re-register whatever connect() last set.
            std::string dir;
            {
                std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
                dir = m_saveDir;
            }
            if (!dir.empty() && m_connected) {
                std::string e;
                if (setSaveDir(dir, e))
                    log("Save directory re-registered after reconnect");
                else
                    log("Could not re-register the save directory: " + e +
                        " - frames will stay on the camera");
            }
        });
    }
}

void Camera::OnDisconnected(CrInt32u error) {
    m_connected = false;
    log("Disconnected (" + crErrorString(error) + ")");
    std::lock_guard<std::mutex> lk(m_connectMutex);
    if (m_connectResult == 0) {
        m_lastError = error;
        m_connectResult = -1;
        m_connectCv.notify_all();
    }
}

void Camera::OnError(CrInt32u error) {
    log("SDK error " + crErrorString(error));
    std::lock_guard<std::mutex> lk(m_connectMutex);
    if (m_connectResult == 0) {
        m_lastError = error;
        m_connectResult = -1;
        m_connectCv.notify_all();
    }
}

void Camera::OnWarning(CrInt32u warning) {
    switch (warning) {
        case SDK::CrWarning_Connect_Reconnecting:
            log("Reconnecting...");
            return;
        // Routine "operation succeeded" notifications - every focus or zoom
        // move emits one, which would drown the log.
        case SDK::CrWarning_FocusPosition_Result_OK:
        case SDK::CrWarning_ZoomPosition_Result_OK:
            return;
        case SDK::CrWarning_File_StorageFull:
            log("Storage is full");
            return;
        default:
            log("SDK warning " + crErrorString(warning));
    }
}

void Camera::OnPropertyChangedCodes(CrInt32u, CrInt32u*) {
    // The UI polls /api/status, so there is nothing to push here.
}

// ---------------------------------------------------------------------------
// Card drain — SDK Remote Transfer
// ---------------------------------------------------------------------------
static long long captureDateToEpoch(const SDK::CrCaptureDate& d) {
    if (d.year < 1970) return 0;
    std::tm t{};
    t.tm_year = d.year - 1900;
    t.tm_mon = d.month - 1;
    t.tm_mday = d.day;
    t.tm_hour = d.hour;
    t.tm_min = d.minute;
    t.tm_sec = d.sec;
    return static_cast<long long>(timegm(&t));
}

bool Camera::cardList(std::vector<CardEntry>& out, std::string& err, int maxNums) {
    std::unique_lock<std::recursive_mutex> lk(m_sdkMutex);
    if (!m_handle) {
        err = "not connected";
        return false;
    }
    out.clear();
    // The body indexes its card by capture day: ask for the day list, then
    // the contents of each day. (Type_All with no date is refused with
    // InvalidParameter on the ILX-LR1.)
    SDK::CrCaptureDate* dates = nullptr;
    CrInt32u nd = 0;
    SDK::CrError e = SDK::GetRemoteTransferCapturedDateList(m_handle, SDK::CrSlotNumber_Slot1,
                                                            &dates, &nd);
    if (e != SDK::CrError_None) {
        err = "GetRemoteTransferCapturedDateList: " + crErrorString(e);
        return false;
    }
    std::vector<SDK::CrCaptureDate> days;
    for (CrInt32u i = 0; dates && i < nd; ++i) days.push_back(dates[i]);
    if (dates) SDK::ReleaseRemoteTransferCapturedDateList(m_handle, dates);

    for (auto& day : days) {
        SDK::CrContentsInfo* list = nullptr;
        CrInt32u n = 0;
        // The body indexes the card asynchronously after a mode switch and
        // answers 0x8D05 (GetContentsInfoListProcessing) until it is done:
        // retry, releasing the SDK mutex between tries so status keeps
        // answering.
        for (int attempt = 0; attempt < 20; ++attempt) {   // ~10 s, then report
            e = SDK::GetRemoteTransferContentsInfoList(
                m_handle, SDK::CrSlotNumber_Slot1, SDK::CrGetContentsInfoListType_Range_Day, &day,
                static_cast<CrInt32u>(maxNums), &list, &n);
            if (e != SDK::CrError_RemoteTransfer_GetContentsInfoListProcessing) break;
            lk.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            lk.lock();
            if (!m_handle) {
                err = "disconnected while listing";
                return false;
            }
        }
        if (e != SDK::CrError_None) {
            err = "GetRemoteTransferContentsInfoList(day): " + crErrorString(e);
            return false;
        }
        for (CrInt32u i = 0; list && i < n; ++i) {
            const SDK::CrContentsInfo& c = list[i];
            for (CrInt32u f = 0; c.files && f < c.filesNum; ++f) {
                const SDK::CrContentsFile& cf = c.files[f];
                CardEntry ce;
                ce.contentId = c.contentId;
                ce.fileId = cf.fileId;
                ce.fileNumber = c.fileNumber;
                ce.dirNumber = c.dirNumber;
                ce.size = static_cast<long long>(cf.fileSize);
                ce.format = cf.fileFormat;
                ce.capturedUtc = captureDateToEpoch(c.creationDatetimeUTC);
                std::string path;
                if (cf.filePath && cf.filePathLength)
                    path.assign(reinterpret_cast<const char*>(cf.filePath), cf.filePathLength);
                while (!path.empty() && path.back() == '\0') path.pop_back();
                const auto slash = path.find_last_of("/\\");
                ce.name = slash == std::string::npos ? path : path.substr(slash + 1);
                out.push_back(ce);
            }
        }
        if (list) SDK::ReleaseRemoteTransferContentsInfoList(m_handle, list);
        if (static_cast<int>(out.size()) >= maxNums) break;
    }
    return true;
}

bool Camera::cardPull(CrInt32u contentId, CrInt32u fileId, const std::string& dir,
                      const std::string& fileName, long long& bytes, std::string& err,
                      int timeoutSec) {
    {
        std::lock_guard<std::mutex> rl(m_rtMutex);
        m_rtResult = 0;
        m_rtPercent = 0;
        m_rtFile.clear();
    }
    {
        std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
        if (!m_handle) {
            err = "not connected";
            return false;
        }
        if (!makeDirs(dir)) {
            err = "could not create " + dir;
            return false;
        }
        std::string d = dir, f = fileName;
        // divisionSize is the transfer chunk; 0 is refused (InvalidParameter).
        // 8 MB balances round-trips against RAM on the Pi.
        const CrInt32u kDivision = 8u * 1024u * 1024u;
        const SDK::CrError e = SDK::GetRemoteTransferContentsDataFile(
            m_handle, SDK::CrSlotNumber_Slot1, contentId, fileId, kDivision,
            const_cast<CrChar*>(d.c_str()), const_cast<CrChar*>(f.c_str()));
        if (e != SDK::CrError_None) {
            err = "GetRemoteTransferContentsDataFile: " + crErrorString(e);
            return false;
        }
    }
    // Wait WITHOUT the SDK mutex: the transfer runs on the SDK's thread and
    // reports through OnNotifyRemoteTransferResult; /api/status must keep
    // answering meanwhile.
    std::unique_lock<std::mutex> rl(m_rtMutex);
    if (!m_rtCv.wait_for(rl, std::chrono::seconds(timeoutSec),
                         [this] { return m_rtResult != 0; })) {
        err = "transfer timed out after " + std::to_string(timeoutSec) + " s";
        return false;
    }
    if (m_rtResult != 1) {
        err = m_rtResult == -2 ? "device busy" : "transfer failed";
        return false;
    }
    const std::string path = dir + "/" + fileName;
    struct stat st{};
    if (::stat(path.c_str(), &st) != 0) {
        err = "transfer reported ok but " + path + " is missing";
        return false;
    }
    bytes = static_cast<long long>(st.st_size);
    return true;
}

bool Camera::cardDelete(CrInt32u contentId, std::string& err) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (!m_handle) {
        err = "not connected";
        return false;
    }
    const SDK::CrError e = SDK::DeleteRemoteTransferContentsFile(
        m_handle, SDK::CrSlotNumber_Slot1, contentId);
    if (e != SDK::CrError_None) {
        err = "DeleteRemoteTransferContentsFile: " + crErrorString(e);
        return false;
    }
    return true;
}

void Camera::OnNotifyRemoteTransferResult(CrInt32u notify, CrInt32u per, CrChar* filename) {
    std::lock_guard<std::mutex> rl(m_rtMutex);
    m_rtPercent = per;
    if (filename) m_rtFile = reinterpret_cast<const char*>(filename);
    if (notify == SDK::CrNotify_RemoteTransfer_Result_OK) m_rtResult = 1;
    else if (notify == SDK::CrNotify_RemoteTransfer_Result_NG) m_rtResult = -1;
    else if (notify == SDK::CrNotify_RemoteTransfer_Result_DeviceBusy) m_rtResult = -2;
    else return;                       // InProgress etc.: keep waiting
    m_rtCv.notify_all();
}

void Camera::OnNotifyRemoteTransferContentsListChanged(CrInt32u notify, CrInt32u slotNumber,
                                                       CrInt32u addSize) {
    // Changed_All = the body finished indexing the card (after a mode switch);
    // Changed_Add = new content appeared; Changed_Clear = card removed/format.
    m_cardIndexReady = (notify != SDK::CrNotify_RemoteTransfer_Changed_Clear);
    log("Card index " + std::string(notify == SDK::CrNotify_RemoteTransfer_Changed_All ? "ready" :
                                    notify == SDK::CrNotify_RemoteTransfer_Changed_Add ? "added" :
                                    notify == SDK::CrNotify_RemoteTransfer_Changed_Clear ? "cleared" : "changed") +
        " (slot " + std::to_string(slotNumber) + ", " + std::to_string(addSize) + ")");
}

void Camera::OnNotifyRemoteTransferResult(CrInt32u notify, CrInt32u per, CrInt8u*, CrInt64u) {
    OnNotifyRemoteTransferResult(notify, per, static_cast<CrChar*>(nullptr));
}

void Camera::OnCompleteDownload(CrChar* filename, CrInt32u) {
    log(std::string("Saved ") + (filename ? filename : ""));
}

// ---------------------------------------------------------------------------
// Property access
// ---------------------------------------------------------------------------

bool Camera::getProp(CrInt32u code, SDK::CrDeviceProperty& out) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (!m_handle) return false;
    SDK::CrDeviceProperty* list = nullptr;
    CrInt32 n = 0;
    const SDK::CrError e = SDK::GetSelectDeviceProperties(m_handle, 1, &code, &list, &n);
    bool ok = false;
    if (e == SDK::CrError_None && list && n >= 1) {
        // The copy is shallow: after the release below, out.GetValues() points
        // into freed memory. Callers may only use the scalar accessors.
        out = list[0];
        ok = true;
    }
    if (list) SDK::ReleaseDeviceProperties(m_handle, list);
    return ok;
}

// Range triple of a property, extracted while the SDK still owns the value
// buffer - the one thing getProp's returned copy cannot provide.
bool Camera::getPropRange(CrInt32u code, PropRange& out) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    out = PropRange{};
    if (!m_handle) return false;
    SDK::CrDeviceProperty* list = nullptr;
    CrInt32 n = 0;
    const SDK::CrError e = SDK::GetSelectDeviceProperties(m_handle, 1, &code, &list, &n);
    bool ok = false;
    if (e == SDK::CrError_None && list && n >= 1) {
        out = rangeOf(list[0]);
        ok = true;
    }
    if (list) SDK::ReleaseDeviceProperties(m_handle, list);
    return ok;
}

// Read-modify-write: the camera tells us the property's real data type, and we
// hand the same type back. Signed values are passed sign-extended to 64 bits so
// that truncation to the property's real width yields correct two's complement.
bool Camera::setPropLocked(CrInt32u code, long long value, std::string& err) {
    if (!m_handle) {
        err = "not connected";
        return false;
    }
    SDK::CrDeviceProperty cur;
    if (!getProp(code, cur)) {
        err = "property 0x" + crErrorString(code).substr(2) + " unavailable on this camera";
        return false;
    }
    if (!cur.IsSetEnableCurrentValue()) {
        // The enable flag is the only thing that distinguishes "the body is in a
        // state that temporarily locks this property" from "this body never lets
        // the SDK write it". Losing it here cost us a night of blind guessing, so
        // it goes in the message along with whatever the body says about its mode.
        err = "property is read-only right now (enableFlag=" +
              formatEnableFlag(cur.GetPropertyEnableFlag()) + "; " + modeSummary() + ")";
        return false;
    }
    // Build the write from scratch rather than reusing the read-back copy: the
    // copy's value buffer points into an array the SDK has already freed, and
    // SetDeviceProperty walking it is a use-after-free (see setDateTime).
    SDK::CrDeviceProperty prop;
    prop.SetCode(code);
    prop.SetValueType(cur.GetValueType());
    prop.SetCurrentValue(static_cast<CrInt64u>(value));
    const SDK::CrError e = SDK::SetDeviceProperty(m_handle, &prop);
    if (e != SDK::CrError_None) {
        err = "SetDeviceProperty failed: " + crErrorString(e);
        return false;
    }
    return true;
}

bool Camera::setProp(CrInt32u code, long long value, std::string& err) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    return setPropLocked(code, value, err);
}

// Write a property WITHOUT consulting its enable flag, trying each plausible
// wire type until one takes.
//
// This exists for CrDeviceProperty_PriorityKeySettings, and the reason is a
// deadlock in the obvious design: while control priority sits with the camera,
// the body reports properties as non-writable - including the priority property
// itself. Gating the write on the enable flag therefore means the one write
// that would unlock everything is the one write never attempted. Sony's own
// RemoteCli sample sets this property with no enable check at all, and the
// v2.02.00 property matrix lists it as Get/Set for the ILX-LR1, so the flag is
// simply not authoritative here.
//
// The type sweep is also from Sony's sample, which is self-inconsistent: the
// reference says UInt16Array, set_position_key_setting() sends UInt8Array, and
// continuous_shooting() sends UInt32Array. Rather than guess, try the body's
// own reported type first and fall back through the others.
bool Camera::setPropForced(CrInt32u code, long long value, long long expect,
                           std::string& err) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (!m_handle) {
        err = "not connected";
        return false;
    }
    std::vector<CrInt32u> types;
    SDK::CrDeviceProperty cur;
    if (getProp(code, cur)) types.push_back(cur.GetValueType());
    for (const CrInt32u t : {SDK::CrDataType_UInt16Array, SDK::CrDataType_UInt32Array,
                             SDK::CrDataType_UInt8Array, SDK::CrDataType_UInt16}) {
        if (std::find(types.begin(), types.end(), t) == types.end()) types.push_back(t);
    }
    for (const CrInt32u t : types) {
        SDK::CrDeviceProperty prop;
        prop.SetCode(code);
        prop.SetValueType(static_cast<SDK::CrDataType>(t));
        prop.SetCurrentValue(static_cast<CrInt64u>(value));
        const SDK::CrError e = SDK::SetDeviceProperty(m_handle, &prop);
        if (e != SDK::CrError_None) {
            err = "SetDeviceProperty failed: " + crErrorString(e);
            continue;
        }
        // SetDeviceProperty can report success while the body quietly declines,
        // so the read-back is the only real confirmation. Sony asks for a beat
        // after a settings write before trusting the next read.
        std::this_thread::sleep_for(500ms);
        SDK::CrDeviceProperty back;
        if (getProp(code, back) &&
            static_cast<long long>(back.GetCurrentValue()) == expect) {
            return true;
        }
        err = "the camera accepted the write but did not take the value";
    }
    return false;
}

bool Camera::sendCmd(CrInt32u cmd, SDK::CrCommandParam param, std::string& err) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (!m_handle) {
        err = "not connected";
        return false;
    }
    const SDK::CrError e = SDK::SendCommand(m_handle, cmd, param);
    if (e != SDK::CrError_None) {
        err = "SendCommand failed: " + crErrorString(e);
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Shooting
// ---------------------------------------------------------------------------

bool Camera::releaseUp(std::string& err) {
    if (sendCmd(SDK::CrCommandId_Release, SDK::CrCommandParam_Up, err)) return true;
    // Never leave the release latched: with a continuous drive mode selected
    // that would run the card flat. One retry after a beat.
    std::this_thread::sleep_for(100ms);
    std::string retryErr;
    return sendCmd(SDK::CrCommandId_Release, SDK::CrCommandParam_Up, retryErr);
}

bool Camera::captureOnce(bool useAf, std::string& err) {
    if (useAf) {
        // Half-press to drive AF, give it a moment to lock, then release.
        std::string ignored;
        setProp(SDK::CrDeviceProperty_S1, SDK::CrLockIndicator_Locked, ignored);
        std::this_thread::sleep_for(500ms);
    }
    bool ok = sendCmd(SDK::CrCommandId_Release, SDK::CrCommandParam_Down, err);
    if (ok) {
        std::this_thread::sleep_for(35ms);
        ok = releaseUp(err);
    }
    if (useAf) {
        // Always drop the half-press, or a failed release leaves AF latched.
        std::string ignored;
        setProp(SDK::CrDeviceProperty_S1, SDK::CrLockIndicator_Unlocked, ignored);
    }
    return ok;
}

bool Camera::holdShutter(int holdMs, std::string& err) {
    if (!isConnected()) {
        err = "not connected";
        return false;
    }
    if (m_intervalRun.load()) {
        err = "intervalometer is running - stop it first";
        return false;
    }
    if (!ensureIntervalRecOff(err)) {
        log("Hold blocked: " + err);
        return false;
    }
    if (holdMs < 0) holdMs = 0;
    if (holdMs > 60000) holdMs = 60000;  // a mistyped field should not run the card flat
    if (!sendCmd(SDK::CrCommandId_Release, SDK::CrCommandParam_Down, err)) {
        log("Hold failed: " + err);
        return false;
    }
    // sendCmd takes the SDK lock per call, so the sleep does not block status
    // polling or the live view while the burst runs. Sleep in slices so a
    // camera dropping off the bus ends the hold early, and swallow anything
    // thrown in between - from here on the Up MUST be sent, or the release
    // stays latched.
    try {
        const auto until = std::chrono::steady_clock::now() +
                           std::chrono::milliseconds(holdMs);
        while (isConnected()) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= until) break;
            const auto remaining = until - now;
            std::this_thread::sleep_for(remaining > 50ms ? 50ms : remaining);
        }
    } catch (...) {}
    std::string upErr;
    bool up = releaseUp(upErr);
    log(up ? "Shutter held " + std::to_string(holdMs) + " ms"
           : "Shutter stuck down: " + upErr);
    if (!up) err = upErr;
    return up;
}

long long Camera::readProp(CrInt32u code, long long dflt) {
    SDK::CrDeviceProperty p;
    if (!getProp(code, p)) return dflt;
    return static_cast<long long>(p.GetCurrentValue());
}

// With Interval REC armed, a release starts or stops the body's own sequence
// instead of taking one frame - which looks exactly like "the shutter does
// nothing". Disarm it first, stopping a running sequence if necessary.
bool Camera::ensureIntervalRecOff(std::string& err) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (readProp(SDK::CrDeviceProperty_Interval_Rec_Mode, SDK::CrIntervalRecMode_OFF) ==
        SDK::CrIntervalRecMode_OFF) {
        return true;
    }
    // Interval_Rec_Mode is only writable while the sequence is stopped.
    if (readProp(SDK::CrDeviceProperty_Interval_Rec_Status,
                 SDK::CrIntervalRecStatus_WaitingStart) ==
        SDK::CrIntervalRecStatus_IntervalShooting) {
        log("Stopping the camera's running interval sequence");
        std::string e2;
        sendCmd(SDK::CrCommandId_Release, SDK::CrCommandParam_Down, e2);
        std::this_thread::sleep_for(35ms);
        sendCmd(SDK::CrCommandId_Release, SDK::CrCommandParam_Up, e2);
        std::this_thread::sleep_for(1200ms);
    }
    for (int attempt = 0; attempt < 6; ++attempt) {
        if (setPropLocked(SDK::CrDeviceProperty_Interval_Rec_Mode,
                          SDK::CrIntervalRecMode_OFF, err)) {
            log("Camera Interval REC disarmed");
            std::this_thread::sleep_for(300ms);
            return true;
        }
        std::this_thread::sleep_for(400ms);
    }
    err = "the camera's built-in Interval REC is on and would not turn off (" + err +
          ") - while it is armed the shutter only starts and stops that sequence";
    return false;
}

bool Camera::shutter(bool useAf, std::string& err) {
    if (!isConnected()) {
        err = "not connected";
        return false;
    }
    if (m_intervalRun.load()) {
        err = "intervalometer is running - stop it first";
        return false;
    }
    if (!ensureIntervalRecOff(err)) {
        log("Shutter blocked: " + err);
        return false;
    }
    const bool ok = captureOnce(useAf, err);
    log(ok ? "Shutter fired" : "Shutter failed: " + err);
    return ok;
}

// --- the camera's own Interval REC -----------------------------------------

bool Camera::configureCameraInterval(double intervalSec, int shots, int startDelaySec,
                                     std::string& err) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (!isConnected()) {
        err = "not connected";
        return false;
    }
    // Settings are locked while a sequence is running.
    if (readProp(SDK::CrDeviceProperty_Interval_Rec_Status,
                 SDK::CrIntervalRecStatus_WaitingStart) ==
        SDK::CrIntervalRecStatus_IntervalShooting) {
        err = "stop the running interval sequence before changing its settings";
        return false;
    }
    // Casting a NaN or out-of-range double to an integer is undefined
    // behaviour, so pin the request to sanity before the tick conversion.
    if (!(intervalSec >= 0.1 && intervalSec <= 86400.0)) {
        err = "interval must be between 0.1s and 86400s";
        return false;
    }
    // The property is in tenths of a second, and the body publishes its own
    // min/max/step - on the ILX-LR1 that is 1.0s to 60.0s in 1.0s steps.
    // Sending anything outside that range is silently ignored, so check first.
    long long ticks = static_cast<long long>(intervalSec * 10.0 + 0.5);
    PropRange r;
    if (getPropRange(SDK::CrDeviceProperty_IntervalRec_ShootingInterval, r)) {
        if (r.valid && (ticks < r.min || ticks > r.max)) {
            std::ostringstream os;
            os << "this camera accepts intervals of " << (r.min / 10.0) << "s to "
               << (r.max / 10.0) << "s in " << (r.step / 10.0) << "s steps (asked for "
               << intervalSec << "s)";
            err = os.str();
            return false;
        }
        if (r.valid && r.step > 1) {
            ticks = r.min + ((ticks - r.min) / r.step) * r.step;  // snap to a valid step
        }
    }
    if (!setPropLocked(SDK::CrDeviceProperty_IntervalRec_ShootingInterval,
                       ticks < 1 ? 1 : ticks, err)) {
        return false;
    }
    if (shots > 0 &&
        !setPropLocked(SDK::CrDeviceProperty_IntervalRec_NumberOfShots, shots, err)) {
        return false;
    }
    if (startDelaySec >= 0) {
        std::string ignored;
        setPropLocked(SDK::CrDeviceProperty_IntervalRec_ShootingStartTime, startDelaySec,
                      ignored);
    }
    std::ostringstream os;
    os << "Camera interval configured: " << std::fixed << std::setprecision(1)
       << (ticks / 10.0) << "s"
       << (shots > 0 ? ", " + std::to_string(shots) + " shots" : "")
       << (startDelaySec >= 0 ? ", " + std::to_string(startDelaySec) + "s delay" : "");
    log(os.str());
    return true;
}

bool Camera::setCameraIntervalArmed(bool armed, std::string& err) {
    if (!armed) return ensureIntervalRecOff(err);

    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (m_intervalRun.load()) {
        err = "the host intervalometer is running - stop it first";
        return false;
    }
    // SetDeviceProperty can report success while the body quietly declines, so
    // arming is only real once it reads back as ON.
    for (int attempt = 0; attempt < 8; ++attempt) {
        setPropLocked(SDK::CrDeviceProperty_Interval_Rec_Mode, SDK::CrIntervalRecMode_ON,
                      err);
        std::this_thread::sleep_for(400ms);
        if (readProp(SDK::CrDeviceProperty_Interval_Rec_Mode, SDK::CrIntervalRecMode_OFF) ==
            SDK::CrIntervalRecMode_ON) {
            log("Camera Interval REC armed");
            return true;
        }
    }
    err = "the camera did not accept Interval REC being armed" +
          (err.empty() ? std::string() : " (" + err + ")");
    log(err);
    return false;
}

bool Camera::cameraIntervalRun(bool start, std::string& err) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (readProp(SDK::CrDeviceProperty_Interval_Rec_Mode, SDK::CrIntervalRecMode_OFF) !=
        SDK::CrIntervalRecMode_ON) {
        err = "arm the camera's Interval REC first";
        return false;
    }
    const long long status = readProp(SDK::CrDeviceProperty_Interval_Rec_Status,
                                      SDK::CrIntervalRecStatus_WaitingStart);
    const bool running = status == SDK::CrIntervalRecStatus_IntervalShooting;
    if (running == start) {
        return true;  // already in the requested state
    }
    // In Interval REC the release button toggles the sequence.
    if (!sendCmd(SDK::CrCommandId_Release, SDK::CrCommandParam_Down, err)) return false;
    std::this_thread::sleep_for(35ms);
    if (!sendCmd(SDK::CrCommandId_Release, SDK::CrCommandParam_Up, err)) return false;
    log(start ? "Camera interval sequence started" : "Camera interval sequence stopped");
    return true;
}

bool Camera::startInterval(double intervalSec, int count, bool useAf, std::string& err) {
    // Validate the request before anything else. An empty field arrives as 0,
    // and silently turning that into "as fast as possible, forever" burns
    // thousands of frames - so refuse it rather than substituting a default.
    if (!(intervalSec >= kMinHostIntervalSec && intervalSec <= 86400.0)) {
        std::ostringstream os;
        os << "interval must be between " << kMinHostIntervalSec << "s and 86400s (got "
           << intervalSec << "s)";
        err = os.str();
        return false;
    }
    if (count < 0) {
        err = "frame count cannot be negative";
        return false;
    }
    if (!isConnected()) {
        err = "not connected";
        return false;
    }
    // The host loop drives single frames, so the body must not be holding the
    // shutter for its own sequence.
    if (!ensureIntervalRecOff(err)) return false;
    if (m_intervalRun.exchange(true)) {
        err = "intervalometer already running";
        return false;
    }
    {
        std::lock_guard<std::mutex> tlk(m_intervalThreadMutex);
        if (m_intervalThread.joinable()) m_intervalThread.join();
        {
            std::lock_guard<std::mutex> lk(m_intervalMutex);
            m_interval = IntervalStatus{};
            m_interval.running = true;
            m_interval.intervalSec = intervalSec;
            m_interval.target = count;
            m_interval.useAf = useAf;
        }
        m_intervalThread = std::thread(&Camera::intervalLoop, this, intervalSec, count, useAf);
    }
    std::ostringstream os;
    os << "Intervalometer started: " << std::fixed << std::setprecision(2) << intervalSec
       << "s interval, " << (count > 0 ? std::to_string(count) + " frames" : "unlimited");
    log(os.str());
    return true;
}

void Camera::stopInterval() {
    const bool wasRunning = m_intervalRun.exchange(false);
    {
        std::lock_guard<std::mutex> tlk(m_intervalThreadMutex);
        if (m_intervalThread.joinable()) m_intervalThread.join();
    }
    if (!wasRunning) return;
    std::lock_guard<std::mutex> lk(m_intervalMutex);
    m_interval.running = false;
    log("Intervalometer stopped after " + std::to_string(m_interval.taken) + " frames");
}

// Absolute scheduling off a fixed start point, so a slow frame does not push
// every subsequent frame later (no cumulative drift).
void Camera::intervalLoop(double intervalSec, int count, bool useAf) {
    const auto start = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(intervalSec));
    int taken = 0;

    while (m_intervalRun.load()) {
        if (count > 0 && taken >= count) break;

        std::string err;
        const bool ok = captureOnce(useAf, err);
        ++taken;
        {
            std::lock_guard<std::mutex> lk(m_intervalMutex);
            m_interval.taken = taken;
            if (!ok) m_interval.lastError = err;
        }
        if (!ok) {
            log("Interval frame " + std::to_string(taken) + " failed: " + err);
            // A single hiccup should not end a long sequence; keep going unless
            // the camera has actually dropped off the bus.
            if (!isConnected()) break;
        }

        if (count > 0 && taken >= count) break;

        // Frame i is due at start + i*period; `taken` frames are done, so the
        // next one is frame index `taken`. Skip any slots already missed.
        auto next = start + period * taken;
        const auto now = std::chrono::steady_clock::now();
        if (next <= now) {
            const auto behind = now - start;
            const long long slots = behind / period;
            next = start + period * (slots + 1);
        }
        while (m_intervalRun.load() && std::chrono::steady_clock::now() < next) {
            const auto remaining = next - std::chrono::steady_clock::now();
            std::this_thread::sleep_for(remaining > 50ms ? 50ms : remaining);
        }
    }

    m_intervalRun = false;
    std::lock_guard<std::mutex> lk(m_intervalMutex);
    m_interval.running = false;
    m_interval.taken = taken;
}

IntervalStatus Camera::intervalStatus() {
    std::lock_guard<std::mutex> lk(m_intervalMutex);
    return m_interval;
}

// ---------------------------------------------------------------------------
// Focus
// ---------------------------------------------------------------------------

bool Camera::setFocusMode(long long mode, std::string& err) {
    const bool ok = setProp(SDK::CrDeviceProperty_FocusMode, mode, err);
    log(ok ? "Focus mode -> " + formatFocusMode(mode) : "Focus mode failed: " + err);
    return ok;
}

bool Camera::focusDrive(int step, std::string& err) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    // NearFar only responds in MF; nudge the camera there rather than failing
    // silently with the lens parked in AF.
    if (step != 0) {
        SDK::CrDeviceProperty fm;
        if (getProp(SDK::CrDeviceProperty_FocusMode, fm) &&
            fm.GetCurrentValue() != SDK::CrFocus_MF) {
            std::string ignored;
            setPropLocked(SDK::CrDeviceProperty_FocusMode, SDK::CrFocus_MF, ignored);
        }
    }
    return setPropLocked(SDK::CrDeviceProperty_NearFar, step, err);
}

bool Camera::setFocusPosition(long long value, std::string& err) {
    return setProp(SDK::CrDeviceProperty_FocusPositionSetting, value, err);
}

// ---------------------------------------------------------------------------
// Zoom
// ---------------------------------------------------------------------------

bool Camera::zoomDrive(int speed, std::string& err) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (speed != 0) {
        SDK::CrDeviceProperty st;
        if (getProp(SDK::CrDeviceProperty_Zoom_Operation_Status, st) &&
            st.GetCurrentValue() != SDK::CrZoomOperationEnableStatus_Enable) {
            err = "the camera reports zoom operation as disabled right now";
            return false;
        }
    }
    return setPropLocked(SDK::CrDeviceProperty_Zoom_Operation, speed, err);
}

bool Camera::setZoomPosition(long long value, std::string& err) {
    return setProp(SDK::CrDeviceProperty_ZoomPositionSetting, value, err);
}

bool Camera::setZoomSetting(long long value, std::string& err) {
    const bool ok = setProp(SDK::CrDeviceProperty_Zoom_Setting, value, err);
    log(ok ? "Zoom setting -> " + formatZoomSetting(value) : "Zoom setting failed: " + err);
    return ok;
}

// ---------------------------------------------------------------------------
// Exposure
// ---------------------------------------------------------------------------

bool Camera::setExposure(const std::string& which, long long value, std::string& err) {
    CrInt32u code = 0;
    if (which == "iso") code = SDK::CrDeviceProperty_IsoSensitivity;
    else if (which == "shutter") code = SDK::CrDeviceProperty_ShutterSpeed;
    else if (which == "aperture") code = SDK::CrDeviceProperty_FNumber;
    else if (which == "program") code = SDK::CrDeviceProperty_ExposureProgramMode;
    else if (which == "drive") code = SDK::CrDeviceProperty_DriveMode;
    // Capture format. Small JPEGs turn a 130 MB RAW into a few hundred KB, which
    // is the difference between a test sequence you can review and one that
    // saturates the link.  1=Jpeg 2=Raw 3=Raw+Jpeg / 1=L 2=M 3=S / 1=Light..4=ExFine
    else if (which == "filetype") code = SDK::CrDeviceProperty_FileType;
    else if (which == "imagesize") code = SDK::CrDeviceProperty_ImageSize;
    else if (which == "quality") code = SDK::CrDeviceProperty_StillImageQuality;
    // What gets sent to the host, independently of what is written to the card:
    // 0=Original 1=SmallSize. The card keeps full resolution either way, so this
    // buys transfer rate for live review without costing survey data.
    else if (which == "transsize") code = SDK::CrDeviceProperty_Still_Image_Trans_Size;
    // When shooting RAW+JPEG, which half reaches the host: 0=both 1=JPEG only
    // 2=RAW only. Paired with transsize this records RAW to the card while the
    // host sees only a small JPEG - full survey data, cheap live review.
    else if (which == "pcsave") code = SDK::CrDeviceProperty_RAW_J_PC_Save_Image;
    else if (which == "rawtype") code = SDK::CrDeviceProperty_RAW_FileCompressionType;
    // EV offset in thousandths (+1/3 EV = 333). UInt16 on the wire; negative
    // values ride on setPropLocked's sign extension and truncate to two's
    // complement, matching Sony's encoding. Only writable in an auto exposure
    // mode - in M the body reports it read-only and the error says so.
    else if (which == "expcomp") code = SDK::CrDeviceProperty_ExposureBiasCompensation;
    // White balance: wb_mode picks the regime (CrWhiteBalance_*; 0x0100 =
    // fixed color temperature), colortemp the Kelvin value used in that
    // regime. Both readable, so the convergence engine can verify them.
    else if (which == "wb_mode") code = SDK::CrDeviceProperty_WhiteBalance;
    else if (which == "colortemp") code = SDK::CrDeviceProperty_Colortemp;
    else {
        err = "unknown exposure control '" + which + "'";
        return false;
    }
    return setProp(code, value, err);
}

bool Camera::setDateTime(long long unixSeconds, std::string& err) {
    // CrDeviceProperty_DateTime_Settings is a UInt64 of seconds since the epoch.
    // Sony notes the displayed time follows the host's timezone, and that some
    // bodies must be set to UTC in their own menu first.
    //
    // Deliberately NOT routed through setProp(): that read-modify-writes via
    // getProp(), which shallow-copies a CrDeviceProperty and then releases the
    // SDK's array - leaving the copy's value buffer dangling. Handing that to
    // SetDeviceProperty is a use-after-free, and for this property it crashes.
    // A set-only property does not need the read, so build it outright.
    //
    // NOTE: this returns CrError_Api_InvalidCalled (0x8402) on the ILX-LR1 as of
    // firmware seen 2026-08-14 - the property is listed as supported in Sony's
    // matrix but the body rejects the write. Adding SetValueSize(8) makes the
    // SDK crash outright instead, so this form is deliberately kept: it fails
    // cleanly. Camera clocks must be set from the body's own menu; correlate
    // frames by measuring each body's constant EXIF offset instead.
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (!m_handle) {
        err = "not connected";
        return false;
    }
    SDK::CrDeviceProperty prop;
    prop.SetCode(SDK::CrDeviceProperty_DateTime_Settings);
    prop.SetValueType(SDK::CrDataType_UInt64);
    prop.SetCurrentValue(static_cast<CrInt64u>(unixSeconds));
    const SDK::CrError e = SDK::SetDeviceProperty(m_handle, &prop);
    if (e != SDK::CrError_None) {
        err = "SetDeviceProperty(DateTime) failed: " + crErrorString(e);
        log("Camera clock set failed: " + err);
        return false;
    }
    log("Camera clock set to epoch " + std::to_string(unixSeconds));
    return true;
}

bool Camera::setStoreDestination(long long value, std::string& err) {
    const bool ok = setProp(SDK::CrDeviceProperty_StillImageStoreDestination, value, err);
    log(ok ? "Store destination -> " + formatStoreDestination(value)
           : "Store destination failed: " + err);
    return ok;
}

// ---------------------------------------------------------------------------
// Live view
// ---------------------------------------------------------------------------

bool Camera::liveViewJpeg(std::vector<unsigned char>& out, std::string& err) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (!m_handle || !isConnected()) {
        err = "not connected";
        return false;
    }
    SDK::CrImageInfo info;
    SDK::CrError e = SDK::GetLiveViewImageInfo(m_handle, &info);
    if (e != SDK::CrError_None) {
        err = "GetLiveViewImageInfo: " + crErrorString(e);
        return false;
    }
    const CrInt32u bufSize = info.GetBufferSize();
    if (bufSize == 0) {
        err = "live view buffer is empty";
        return false;
    }
    // Do not "optimise" this by pointing SetData at `out` and then shrinking it:
    // GetImageData() does NOT return the pointer handed to SetData. The SDK
    // writes a header into the front of the buffer (200 bytes as measured on
    // v2.02.00, and the size is not contractual) and returns a pointer past it,
    // so the frame has to be copied from GetImageData(), not from the start of
    // the buffer. Getting this wrong serves header + a truncated JPEG, which
    // decodes nowhere and shows up as "Live view unavailable".
    std::vector<unsigned char> buf(bufSize);
    SDK::CrImageDataBlock block;
    block.SetSize(bufSize);
    block.SetData(buf.data());
    e = SDK::GetLiveViewImage(m_handle, &block);
    if (e != SDK::CrError_None) {
        err = "GetLiveViewImage: " + crErrorString(e);
        return false;
    }
    const CrInt32u imgSize = block.GetImageSize();
    if (imgSize == 0 || block.GetImageData() == nullptr || imgSize > bufSize) {
        err = "live view frame was empty";
        return false;
    }
    out.assign(block.GetImageData(), block.GetImageData() + imgSize);
    return true;
}

// Erase the card. Destructive and not undoable, so it is never called from any
// automatic path - only from an explicit operator request.
bool Camera::formatMedia(bool quick, std::string& err) {
    if (!isConnected()) {
        err = "not connected";
        return false;
    }
    const CrInt32u cmd = quick ? SDK::CrCommandId_MediaQuickFormat
                               : SDK::CrCommandId_MediaFormat;
    if (!sendCmd(cmd, SDK::CrCommandParam_Down, err)) {
        log("Card format failed: " + err);
        return false;
    }
    log(quick ? "Card quick format started" : "Card format started");
    return true;
}

bool Camera::setSaveDir(const std::string& dir, std::string& err) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (!m_handle) {
        err = "not connected";
        return false;
    }
    // The SDK will not create the destination itself: pointing SetSaveInfo at a
    // missing directory fails with 0x810c, and the only visible symptom is that
    // frames never leave the camera. A freshly provisioned node has no
    // ~/Pictures/... at all, so it shoots perfectly and transfers nothing.
    if (!makeDirs(dir)) {
        err = "could not create save directory " + dir;
        return false;
    }
    std::string path = dir;
    std::string prefix = "ILX";
    const SDK::CrError e = SDK::SetSaveInfo(m_handle, path.data(), prefix.data(),
                                            SDK::CrSETSAVEINFO_AUTO_NUMBER);
    if (e != SDK::CrError_None) {
        err = "SetSaveInfo failed: " + crErrorString(e);
        return false;
    }
    // Remember it: the SDK forgets the save location across a reconnect, and a
    // spontaneous reconnect (CrReconnecting_ON) does not go through connect().
    m_saveDir = dir;
    return true;
}

// mkdir -p, without pulling in <filesystem>.
bool Camera::makeDirs(const std::string& dir) {
    if (dir.empty()) return false;
    struct ::stat st;
    if (::stat(dir.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
    for (size_t i = 1; i <= dir.size(); ++i) {
        if (i != dir.size() && dir[i] != '/') continue;
        const std::string part = dir.substr(0, i);
        if (::mkdir(part.c_str(), 0775) != 0 && errno != EEXIST) return false;
    }
    return ::stat(dir.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

// ---------------------------------------------------------------------------
// Status snapshot
// ---------------------------------------------------------------------------

std::string Camera::statusJson() {
    // A connect in flight holds m_sdkMutex for as long as SDK::Connect takes
    // — which against a stalled PTP session is "until the body is power-
    // cycled". Taking the mutex here then froze /api/status too, and the
    // daemon looked dead (rigd: ILX_DOWN). Answer without it while a connect
    // is pending: connected:false, connecting:true, and the log.
    if (m_connecting && !m_connected) {
        {
            std::ostringstream pos;
            pos << "{\"connected\":false,\"connecting\":true,\"model\":\"\","
                   "\"id\":\"\",\"log\":[";
            const auto logLines = takeLog();
            const std::size_t from = logLines.size() > 40 ? logLines.size() - 40 : 0;
            for (std::size_t i = from; i < logLines.size(); ++i) {
                if (i > from) pos << ",";
                pos << "\"" << jsonEscape(logLines[i]) << "\"";
            }
            pos << "]}";
            return pos.str();
        }
    }
    std::string model, id;
    {
        std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
        model = m_model;
        id = m_id;
    }
    std::ostringstream os;
    os << "{";
    os << "\"connected\":" << (isConnected() ? "true" : "false");
    os << ",\"model\":\"" << jsonEscape(model) << "\"";
    os << ",\"id\":\"" << jsonEscape(id) << "\"";

    if (isConnected()) {
        // One batched read keeps the USB round-trips down while the UI polls.
        CrInt32u codes[] = {
            SDK::CrDeviceProperty_FocusMode,
            SDK::CrDeviceProperty_NearFar,
            SDK::CrDeviceProperty_FocusPositionSetting,
            SDK::CrDeviceProperty_FocusPositionCurrentValue,
            SDK::CrDeviceProperty_FocusIndication,
            SDK::CrDeviceProperty_FocusDrivingStatus,
            SDK::CrDeviceProperty_Zoom_Setting,
            SDK::CrDeviceProperty_Zoom_Operation,
            SDK::CrDeviceProperty_Zoom_Operation_Status,
            SDK::CrDeviceProperty_Zoom_Speed_Range,
            SDK::CrDeviceProperty_Zoom_Scale,
            SDK::CrDeviceProperty_Zoom_Bar_Information,
            SDK::CrDeviceProperty_Zoom_Type_Status,
            SDK::CrDeviceProperty_ZoomPositionSetting,
            SDK::CrDeviceProperty_ZoomPositionCurrentValue,
            SDK::CrDeviceProperty_ZoomDrivingStatus,
            SDK::CrDeviceProperty_IsoSensitivity,
            SDK::CrDeviceProperty_ShutterSpeed,
            SDK::CrDeviceProperty_FNumber,
            SDK::CrDeviceProperty_ExposureProgramMode,
            SDK::CrDeviceProperty_DriveMode,
            SDK::CrDeviceProperty_BatteryRemain,
            SDK::CrDeviceProperty_MediaSLOT1_RemainingNumber,
            SDK::CrDeviceProperty_MediaSLOT1_Status,
            // The three properties that distinguish "the software is broken"
            // from "the body cannot record right now". Without them a stuck
            // card write, a thermal shutdown and a live-view session that never
            // stopped all present identically: a camera that connects, answers
            // every read, fires the shutter, and silently records nothing.
            SDK::CrDeviceProperty_MediaSLOT1_WritingState,
            SDK::CrDeviceProperty_DeviceOverheatingState,
            SDK::CrDeviceProperty_LiveViewStatus,
            SDK::CrDeviceProperty_StillImageStoreDestination,
            SDK::CrDeviceProperty_PriorityKeySettings,
            SDK::CrDeviceProperty_Interval_Rec_Mode,
            SDK::CrDeviceProperty_Interval_Rec_Status,
            SDK::CrDeviceProperty_IntervalRec_ShootingInterval,
            SDK::CrDeviceProperty_IntervalRec_NumberOfShots,
            SDK::CrDeviceProperty_IntervalRec_ShootingStartTime,
            // Body state. These decide whether anything else is writable, so
            // they belong in every status poll rather than a special probe.
            SDK::CrDeviceProperty_CameraPowerStatus,
            SDK::CrDeviceProperty_CameraOperatingMode,
            SDK::CrDeviceProperty_DisplayedMenuStatus,
            SDK::CrDeviceProperty_SdkControlMode,
            // White balance: the ILX-LR1 accepts the writes (verified live
            // 2026-08-23) and answers a targeted read; it must be asked for
            // by code here or the readback stays -1 and convergence can never
            // confirm what the bodies render.
            SDK::CrDeviceProperty_WhiteBalance,
            SDK::CrDeviceProperty_Colortemp,
        };
        const CrInt32u nCodes = sizeof(codes) / sizeof(codes[0]);

        SDK::CrDeviceProperty* list = nullptr;
        CrInt32 n = 0;
        // Hold the SDK lock across read -> parse -> release: disconnect()
        // frees the device handle under this same lock, so a concurrent
        // /api/disconnect cannot pull the property array out from under the
        // parse loop below.
        std::unique_lock<std::recursive_mutex> lk(m_sdkMutex);
        if (m_handle) {
            SDK::GetSelectDeviceProperties(m_handle, nCodes, codes, &list, &n);
        }

        auto emitNum = [&](const char* key, long long v) {
            os << ",\"" << key << "\":" << v;
        };
        auto emitStr = [&](const char* key, const std::string& v) {
            os << ",\"" << key << "\":\"" << jsonEscape(v) << "\"";
        };

        // Defaults so the UI always sees every key.
        long long focusMode = 0, focusPos = 0, focusPosCur = 0, focusInd = 0, focusDriving = 0;
        long long zoomSetting = 0, zoomOpStatus = 0, zoomScale = 0, zoomBar = 0;
        long long zoomType = 0, zoomPos = 0, zoomPosCur = 0, zoomDriving = 0;
        long long iso = 0, shutter = 0, fnum = 0, program = 0, drive = 0;
        long long battery = -1, remainShots = -1, slotStatus = -1, storeDest = 0;
        long long priorityKey = 0;
        // -1 = not reported. whiteBalance is the MODE (CrWhiteBalance_*,
        // 0x0100 = fixed color temperature); colorTemp is the Kelvin value
        // that applies while the mode is ColorTemp.
        long long whiteBalance = -1, colorTemp = -1;
        // -1 = the body did not report it, which is different from "fine".
        long long overheat = -1, liveViewStatus = -1, slotWriting = -1;
        long long powerStatus = 0, opMode = 0, menuStatus = 0, sdkCtlMode = -1;
        long long ivMode = SDK::CrIntervalRecMode_OFF, ivStatus = 0;
        long long ivInterval = 0, ivShots = 0, ivStart = 0;
        PropRange ivIntervalR, ivShotsR, ivStartR;
        PropRange nearFarR, focusPosR, zoomSpeedR, zoomPosR;
        std::vector<PropChoice> isoChoices, shutterChoices, apChoices, programChoices,
            driveChoices, focusModeChoices, zoomSettingChoices, storeChoices;

        auto collect = [](const SDK::CrDeviceProperty& p,
                          std::vector<PropChoice>& dst,
                          std::string (*fmt)(long long)) {
            for (const long long v : choicesOf(p)) dst.push_back({v, fmt(v)});
        };

        // Writability of the properties we actually drive. The body reports the
        // whole table read-only whenever it is in a state that locks settings,
        // and without this the UI can only show that a write "failed" - which
        // reads as a broken camera rather than a camera on its menu.
        std::vector<std::pair<const char*, long long>> flags;
        auto flagOf = [&flags](const char* name, const SDK::CrDeviceProperty& p) {
            flags.emplace_back(name, static_cast<long long>(p.GetPropertyEnableFlag()));
        };

        for (CrInt32 i = 0; i < n; ++i) {
            const SDK::CrDeviceProperty& p = list[i];
            const long long cur = static_cast<long long>(p.GetCurrentValue());
            switch (p.GetCode()) {
                case SDK::CrDeviceProperty_IsoSensitivity: flagOf("iso", p); break;
                case SDK::CrDeviceProperty_ShutterSpeed: flagOf("shutter", p); break;
                case SDK::CrDeviceProperty_FNumber: flagOf("aperture", p); break;
                case SDK::CrDeviceProperty_FocusMode: flagOf("focusMode", p); break;
                case SDK::CrDeviceProperty_PriorityKeySettings: flagOf("priorityKey", p); break;
                case SDK::CrDeviceProperty_StillImageStoreDestination:
                    flagOf("storeDest", p);
                    break;
                case SDK::CrDeviceProperty_WhiteBalance: flagOf("whiteBalance", p); break;
                case SDK::CrDeviceProperty_Colortemp: flagOf("colorTemp", p); break;
                default: break;
            }
            switch (p.GetCode()) {
                case SDK::CrDeviceProperty_FocusMode:
                    focusMode = cur;
                    collect(p, focusModeChoices, [](long long v) { return formatFocusMode(v); });
                    break;
                case SDK::CrDeviceProperty_NearFar: nearFarR = rangeOf(p); break;
                case SDK::CrDeviceProperty_FocusPositionSetting:
                    focusPos = cur;
                    focusPosR = rangeOf(p);
                    break;
                case SDK::CrDeviceProperty_FocusPositionCurrentValue: focusPosCur = cur; break;
                case SDK::CrDeviceProperty_FocusIndication: focusInd = cur; break;
                case SDK::CrDeviceProperty_FocusDrivingStatus: focusDriving = cur; break;
                case SDK::CrDeviceProperty_Zoom_Setting:
                    zoomSetting = cur;
                    collect(p, zoomSettingChoices,
                            [](long long v) { return formatZoomSetting(v); });
                    break;
                case SDK::CrDeviceProperty_Zoom_Operation_Status: zoomOpStatus = cur; break;
                case SDK::CrDeviceProperty_Zoom_Speed_Range: zoomSpeedR = rangeOf(p); break;
                case SDK::CrDeviceProperty_Zoom_Scale: zoomScale = cur; break;
                case SDK::CrDeviceProperty_Zoom_Bar_Information: zoomBar = cur; break;
                case SDK::CrDeviceProperty_Zoom_Type_Status: zoomType = cur; break;
                case SDK::CrDeviceProperty_ZoomPositionSetting:
                    zoomPos = cur;
                    zoomPosR = rangeOf(p);
                    break;
                case SDK::CrDeviceProperty_ZoomPositionCurrentValue: zoomPosCur = cur; break;
                case SDK::CrDeviceProperty_ZoomDrivingStatus: zoomDriving = cur; break;
                case SDK::CrDeviceProperty_IsoSensitivity:
                    iso = cur;
                    for (const long long v : choicesOf(p))
                        isoChoices.push_back({v, formatIso(static_cast<unsigned int>(v))});
                    break;
                case SDK::CrDeviceProperty_ShutterSpeed:
                    shutter = cur;
                    for (const long long v : choicesOf(p))
                        shutterChoices.push_back(
                            {v, formatShutterSpeed(static_cast<unsigned int>(v))});
                    break;
                case SDK::CrDeviceProperty_FNumber:
                    fnum = cur;
                    for (const long long v : choicesOf(p))
                        apChoices.push_back({v, formatFNumber(static_cast<unsigned int>(v))});
                    break;
                case SDK::CrDeviceProperty_ExposureProgramMode:
                    program = cur;
                    collect(p, programChoices,
                            [](long long v) { return formatExposureProgram(v); });
                    break;
                case SDK::CrDeviceProperty_DriveMode:
                    drive = cur;
                    collect(p, driveChoices, [](long long v) { return formatDriveMode(v); });
                    break;
                case SDK::CrDeviceProperty_WhiteBalance: whiteBalance = cur; break;
                case SDK::CrDeviceProperty_Colortemp: colorTemp = cur; break;
                case SDK::CrDeviceProperty_BatteryRemain: battery = cur; break;
                case SDK::CrDeviceProperty_MediaSLOT1_RemainingNumber: remainShots = cur; break;
                case SDK::CrDeviceProperty_MediaSLOT1_Status: slotStatus = cur; break;
                // Whether the card is mid-write. This is NOT covered by
                // MediaSLOT1_Status, which reports the card as OK because the
                // card is recognised perfectly well - it is the WRITE that is
                // hung. A body with one frame stuck in its write buffer goes
                // busy, locks its whole property table (storeDest and the drive
                // list stop being offered at all), stops delivering to the PC
                // and refuses to format, while still answering status and still
                // firing the shutter. Every visible symptom points at the
                // software; the only honest signal is this property.
                case SDK::CrDeviceProperty_MediaSLOT1_WritingState:
                    slotWriting = cur; break;
                // A body that is cooking stops recording long before it says so
                // in any way the host can see: the shutter still fires and the
                // EXPOSURE edge still lands, but nothing is written to card or
                // delivered to the PC, and every save-related property goes
                // read-only behind a caution the screenless body cannot show.
                // That is indistinguishable from a software fault unless this
                // property is read - and it was not, which cost a session.
                case SDK::CrDeviceProperty_DeviceOverheatingState: overheat = cur; break;
                // Whether the sensor is actually streaming for live view. On a
                // sealed underwater housing, live view is pure heat with nobody
                // watching, so the rig needs to be able to SEE that it is on.
                case SDK::CrDeviceProperty_LiveViewStatus: liveViewStatus = cur; break;
                case SDK::CrDeviceProperty_PriorityKeySettings: priorityKey = cur; break;
                case SDK::CrDeviceProperty_CameraPowerStatus: powerStatus = cur; break;
                case SDK::CrDeviceProperty_CameraOperatingMode: opMode = cur; break;
                case SDK::CrDeviceProperty_DisplayedMenuStatus: menuStatus = cur; break;
                case SDK::CrDeviceProperty_SdkControlMode: sdkCtlMode = cur; break;
                case SDK::CrDeviceProperty_Interval_Rec_Mode: ivMode = cur; break;
                case SDK::CrDeviceProperty_Interval_Rec_Status: ivStatus = cur; break;
                case SDK::CrDeviceProperty_IntervalRec_ShootingInterval:
                    ivInterval = cur;
                    ivIntervalR = rangeOf(p);
                    break;
                case SDK::CrDeviceProperty_IntervalRec_NumberOfShots:
                    ivShots = cur;
                    ivShotsR = rangeOf(p);
                    break;
                case SDK::CrDeviceProperty_IntervalRec_ShootingStartTime:
                    ivStart = cur;
                    ivStartR = rangeOf(p);
                    break;
                case SDK::CrDeviceProperty_StillImageStoreDestination:
                    storeDest = cur;
                    collect(p, storeChoices,
                            [](long long v) { return formatStoreDestination(v); });
                    break;
                default: break;
            }
        }
        if (list) SDK::ReleaseDeviceProperties(m_handle, list);
        lk.unlock();

        auto emitChoices = [&](const char* key, const std::vector<PropChoice>& c) {
            os << ",\"" << key << "\":[";
            for (std::size_t i = 0; i < c.size(); ++i) {
                if (i) os << ",";
                os << "{\"v\":" << c[i].value << ",\"l\":\"" << jsonEscape(c[i].label) << "\"}";
            }
            os << "]";
        };

        emitNum("focusMode", focusMode);
        emitStr("focusModeLabel", formatFocusMode(focusMode));
        emitChoices("focusModes", focusModeChoices);
        emitNum("focusPos", focusPos);
        emitNum("focusPosCur", focusPosCur);
        os << ",\"focusPosRange\":" << rangeJson(focusPosR);
        os << ",\"nearFarRange\":" << rangeJson(nearFarR);
        emitNum("focusIndication", focusInd);
        emitNum("focusDriving", focusDriving);

        emitNum("zoomSetting", zoomSetting);
        emitStr("zoomSettingLabel", formatZoomSetting(zoomSetting));
        emitChoices("zoomSettings", zoomSettingChoices);
        emitNum("zoomOpEnabled", zoomOpStatus);
        os << ",\"zoomSpeedRange\":" << rangeJson(zoomSpeedR);
        emitNum("zoomScale", zoomScale);
        emitNum("zoomPos", zoomPos);
        emitNum("zoomPosCur", zoomPosCur);
        os << ",\"zoomPosRange\":" << rangeJson(zoomPosR);
        emitNum("zoomDriving", zoomDriving);
        // Zoom bar: total boxes (31-24), current box (23-16), position in box (15-0).
        os << ",\"zoomBar\":{\"total\":" << ((zoomBar >> 24) & 0xFF)
           << ",\"current\":" << ((zoomBar >> 16) & 0xFF)
           << ",\"pos\":" << (zoomBar & 0xFFFF) << "}";
        emitNum("zoomType", zoomType);

        emitStr("iso", formatIso(static_cast<unsigned int>(iso)));
        emitNum("isoValue", iso);
        emitChoices("isoChoices", isoChoices);
        emitStr("shutter", formatShutterSpeed(static_cast<unsigned int>(shutter)));
        emitNum("shutterValue", shutter);
        emitChoices("shutterChoices", shutterChoices);
        emitStr("aperture", formatFNumber(static_cast<unsigned int>(fnum)));
        emitNum("apertureValue", fnum);
        emitChoices("apertureChoices", apChoices);
        emitStr("program", formatExposureProgram(program));
        emitNum("programValue", program);
        emitChoices("programChoices", programChoices);
        emitStr("drive", formatDriveMode(drive));
        emitNum("driveValue", drive);
        emitChoices("driveChoices", driveChoices);
        // 0xFFFF means "not available" - the ILX-LR1 reports this when it is
        // running on external power rather than a battery.
        const bool battOk = battery >= 0 && battery <= 100;
        os << ",\"battery\":" << (battOk ? std::to_string(battery) : "null");
        os << ",\"remainingShots\":" << (remainShots >= 0 ? std::to_string(remainShots) : "null");
        emitStr("slotStatus", slotStatus >= 0 ? formatSlotStatus(slotStatus) : "--");
        // Thermal state, and whether the sensor is streaming for live view.
        // null means the body did not report the property at all - which must
        // never be shown as "not overheating".
        os << ",\"overheating\":"
           << (overheat >= 0 ? std::to_string(overheat) : "null");
        emitStr("overheatingLabel",
                overheat < 0 ? "unknown"
                : overheat == SDK::CrDeviceOverheatingState_NotOverheating ? "ok"
                : overheat == SDK::CrDeviceOverheatingState_PreOverheating ? "pre-overheat"
                : overheat == SDK::CrDeviceOverheatingState_Overheating ? "OVERHEATING"
                : "unknown");
        os << ",\"slotWriting\":"
           << (slotWriting >= 0 ? std::to_string(slotWriting) : "null");
        emitStr("slotWritingLabel",
                slotWriting < 0 ? "unknown"
                : slotWriting == SDK::CrMediaSlotWritingState_NotWriting ? "idle"
                : slotWriting == SDK::CrMediaSlotWritingState_ContentsWriting ? "WRITING"
                : "unknown");
        os << ",\"liveViewStatus\":"
           << (liveViewStatus >= 0 ? std::to_string(liveViewStatus) : "null");
        emitStr("liveViewLabel",
                liveViewStatus < 0 ? "unknown"
                : liveViewStatus == SDK::CrLiveView_NotSupport ? "not supported"
                : liveViewStatus == SDK::CrLiveView_Disable ? "disabled"
                : liveViewStatus == SDK::CrLiveView_Enable ? "streaming"
                : "unknown");
        // The body's own Interval REC. Interval is reported in tenths of a
        // second; the UI works in seconds.
        os << ",\"camIv\":{"
           << "\"armed\":" << (ivMode == SDK::CrIntervalRecMode_ON ? "true" : "false")
           << ",\"running\":"
           << (ivStatus == SDK::CrIntervalRecStatus_IntervalShooting ? "true" : "false")
           << ",\"intervalSec\":" << std::fixed << std::setprecision(1) << (ivInterval / 10.0)
           << ",\"shots\":" << ivShots << ",\"startDelaySec\":" << ivStart
           << ",\"intervalRange\":" << rangeJson(ivIntervalR)
           << ",\"shotsRange\":" << rangeJson(ivShotsR)
           << ",\"startRange\":" << rangeJson(ivStartR) << "}";

        emitNum("powerStatus", powerStatus);
        emitStr("powerLabel", formatPowerStatus(powerStatus));
        emitNum("operatingMode", opMode);
        emitStr("operatingModeLabel", formatOperatingMode(opMode));
        emitNum("menuStatus", menuStatus);
        emitStr("menuLabel", formatMenuStatus(menuStatus));
        emitNum("sdkControlMode", sdkCtlMode);
        os << ",\"writable\":{";
        for (size_t i = 0; i < flags.size(); ++i) {
            if (i) os << ",";
            os << "\"" << flags[i].first << "\":" << flags[i].second;
        }
        os << "}";
        emitNum("priorityKey", priorityKey);
        emitStr("priorityKeyLabel", priorityKey == SDK::CrPriorityKey_PCRemote ? "PC remote"
                                    : priorityKey == SDK::CrPriorityKey_CameraPosition
                                          ? "Camera position"
                                          : "--");
        emitNum("storeDest", storeDest);
        emitStr("storeDestLabel", formatStoreDestination(storeDest));
        emitChoices("storeChoices", storeChoices);
        emitNum("whiteBalance", whiteBalance);
        emitStr("whiteBalanceLabel",
                whiteBalance == 0x0000 ? "AWB"
                : whiteBalance == 0x0011 ? "Daylight"
                : whiteBalance == 0x0100 ? "Color temp"
                : whiteBalance < 0      ? "--"
                                        : "other");
        emitNum("colorTemp", colorTemp);
    }

    const IntervalStatus iv = intervalStatus();
    os << ",\"interval\":{\"running\":" << (iv.running ? "true" : "false")
       << ",\"taken\":" << iv.taken << ",\"target\":" << iv.target << ",\"intervalSec\":"
       << std::fixed << std::setprecision(2) << iv.intervalSec
       << ",\"useAf\":" << (iv.useAf ? "true" : "false")
       << ",\"lastError\":\"" << jsonEscape(iv.lastError) << "\"}";

    os << ",\"log\":[";
    const auto logLines = takeLog();
    const std::size_t from = logLines.size() > 40 ? logLines.size() - 40 : 0;
    for (std::size_t i = from; i < logLines.size(); ++i) {
        if (i > from) os << ",";
        os << "\"" << jsonEscape(logLines[i]) << "\"";
    }
    os << "]";

    os << "}";
    return os.str();
}
