#include "camera.h"

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

bool Camera::connect(int index, std::string& err) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (!m_enumInfo || index < 0 ||
        static_cast<CrInt32u>(index) >= m_enumInfo->GetCount()) {
        err = "no camera at index " + std::to_string(index) + " (run discovery first)";
        return false;
    }
    auto* obj = const_cast<SDK::ICrCameraObjectInfo*>(m_enumInfo->GetCameraObjectInfo(index));
    m_model = obj->GetModel() ? obj->GetModel() : "";
    m_id = obj->GetId() ? reinterpret_cast<const char*>(obj->GetId()) : "";

    {
        std::lock_guard<std::mutex> clk(m_connectMutex);
        m_connectResult = 0;
    }

    const SDK::CrError e = SDK::Connect(obj, this, &m_handle, SDK::CrSdkControlMode_Remote,
                                        SDK::CrReconnecting_ON);
    if (e != SDK::CrError_None) {
        err = "Connect failed: " + crErrorString(e);
        log(err);
        return false;
    }

    std::unique_lock<std::mutex> clk(m_connectMutex);
    if (!m_connectCv.wait_for(clk, 15s, [this] { return m_connectResult != 0; })) {
        err = "timed out waiting for the camera to accept the connection";
        log(err);
        return false;
    }
    if (m_connectResult < 0) {
        err = "camera refused the connection (" + crErrorString(m_lastError) +
              ") - is Imaging Edge or another app still holding it?";
        log(err);
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
    for (int attempt = 0; attempt < 10; ++attempt) {
        SDK::CrDeviceProperty pk;
        if (getProp(SDK::CrDeviceProperty_PriorityKeySettings, pk)) {
            if (pk.GetCurrentValue() == SDK::CrPriorityKey_PCRemote) {
                if (attempt) log("Control priority is with the PC");
                return;
            }
            std::string err;
            if (setProp(SDK::CrDeviceProperty_PriorityKeySettings,
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

void Camera::disconnect() {
    stopInterval();
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
    m_connected = false;
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void Camera::OnConnected(SDK::DeviceConnectionVersioin) {
    m_connected = true;
    log("Connected to " + m_model);
    std::lock_guard<std::mutex> lk(m_connectMutex);
    m_connectResult = 1;
    m_connectCv.notify_all();
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
        out = list[0];
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
    SDK::CrDeviceProperty prop;
    if (!getProp(code, prop)) {
        err = "property 0x" + crErrorString(code).substr(2) + " unavailable on this camera";
        return false;
    }
    if (!prop.IsSetEnableCurrentValue()) {
        err = "property is currently read-only (check the camera's mode)";
        return false;
    }
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

bool Camera::captureOnce(bool useAf, std::string& err) {
    if (useAf) {
        // Half-press to drive AF, give it a moment to lock, then release.
        std::string ignored;
        setProp(SDK::CrDeviceProperty_S1, SDK::CrLockIndicator_Locked, ignored);
        std::this_thread::sleep_for(500ms);
    }
    if (!sendCmd(SDK::CrCommandId_Release, SDK::CrCommandParam_Down, err)) return false;
    std::this_thread::sleep_for(35ms);
    if (!sendCmd(SDK::CrCommandId_Release, SDK::CrCommandParam_Up, err)) return false;
    if (useAf) {
        std::string ignored;
        setProp(SDK::CrDeviceProperty_S1, SDK::CrLockIndicator_Unlocked, ignored);
    }
    return true;
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
    // The property is in tenths of a second, and the body publishes its own
    // min/max/step - on the ILX-LR1 that is 1.0s to 60.0s in 1.0s steps.
    // Sending anything outside that range is silently ignored, so check first.
    long long ticks = static_cast<long long>(intervalSec * 10.0 + 0.5);
    SDK::CrDeviceProperty ivProp;
    if (getProp(SDK::CrDeviceProperty_IntervalRec_ShootingInterval, ivProp)) {
        const PropRange r = rangeOf(ivProp);
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
    if (!(intervalSec >= kMinHostIntervalSec)) {
        std::ostringstream os;
        os << "interval must be at least " << kMinHostIntervalSec << "s (got "
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
    std::ostringstream os;
    os << "Intervalometer started: " << std::fixed << std::setprecision(2) << intervalSec
       << "s interval, " << (count > 0 ? std::to_string(count) + " frames" : "unlimited");
    log(os.str());
    return true;
}

void Camera::stopInterval() {
    if (!m_intervalRun.exchange(false)) {
        if (m_intervalThread.joinable()) m_intervalThread.join();
        return;
    }
    if (m_intervalThread.joinable()) m_intervalThread.join();
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
    else {
        err = "unknown exposure control '" + which + "'";
        return false;
    }
    return setProp(code, value, err);
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
    if (imgSize == 0) {
        err = "live view frame was empty";
        return false;
    }
    out.assign(block.GetImageData(), block.GetImageData() + imgSize);
    return true;
}

bool Camera::setSaveDir(const std::string& dir, std::string& err) {
    std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
    if (!m_handle) {
        err = "not connected";
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
    return true;
}

// ---------------------------------------------------------------------------
// Status snapshot
// ---------------------------------------------------------------------------

std::string Camera::statusJson() {
    std::ostringstream os;
    os << "{";
    os << "\"connected\":" << (isConnected() ? "true" : "false");
    os << ",\"model\":\"" << jsonEscape(modelName()) << "\"";
    os << ",\"id\":\"" << jsonEscape(m_id) << "\"";

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
            SDK::CrDeviceProperty_StillImageStoreDestination,
            SDK::CrDeviceProperty_PriorityKeySettings,
            SDK::CrDeviceProperty_Interval_Rec_Mode,
            SDK::CrDeviceProperty_Interval_Rec_Status,
            SDK::CrDeviceProperty_IntervalRec_ShootingInterval,
            SDK::CrDeviceProperty_IntervalRec_NumberOfShots,
            SDK::CrDeviceProperty_IntervalRec_ShootingStartTime,
        };
        const CrInt32u nCodes = sizeof(codes) / sizeof(codes[0]);

        SDK::CrDeviceProperty* list = nullptr;
        CrInt32 n = 0;
        {
            std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
            if (m_handle) {
                SDK::GetSelectDeviceProperties(m_handle, nCodes, codes, &list, &n);
            }
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

        for (CrInt32 i = 0; i < n; ++i) {
            const SDK::CrDeviceProperty& p = list[i];
            const long long cur = static_cast<long long>(p.GetCurrentValue());
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
                case SDK::CrDeviceProperty_BatteryRemain: battery = cur; break;
                case SDK::CrDeviceProperty_MediaSLOT1_RemainingNumber: remainShots = cur; break;
                case SDK::CrDeviceProperty_MediaSLOT1_Status: slotStatus = cur; break;
                case SDK::CrDeviceProperty_PriorityKeySettings: priorityKey = cur; break;
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
        if (list) {
            std::lock_guard<std::recursive_mutex> lk(m_sdkMutex);
            if (m_handle) SDK::ReleaseDeviceProperties(m_handle, list);
        }

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

        emitNum("priorityKey", priorityKey);
        emitStr("priorityKeyLabel", priorityKey == SDK::CrPriorityKey_PCRemote ? "PC remote"
                                    : priorityKey == SDK::CrPriorityKey_CameraPosition
                                          ? "Camera position"
                                          : "--");
        emitNum("storeDest", storeDest);
        emitStr("storeDestLabel", formatStoreDestination(storeDest));
        emitChoices("storeChoices", storeChoices);
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
