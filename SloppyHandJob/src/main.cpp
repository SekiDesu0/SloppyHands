// SloppyHandJob — Control GUI for the Knuckles proxy driver
// SDL2 + Dear ImGui
// Uses SLP1 protocol (port 4242) for ESP32 finger tracking
#include <SDL.h>
#include <SDL_opengl.h>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"
#include "shared_state.h"

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <openvr.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cmath>
#include <algorithm>

#pragma comment(lib, "ws2_32.lib")

// ===========================================================================
// SLP1 Protocol (mirrors firmware/test_tracker.py)
// ===========================================================================
static constexpr uint32_t SLP1_MAGIC = 0x534C5031u; // "SLP1"
static constexpr int      SLP1_PORT  = 4242;

static constexpr uint8_t  TYPE_HELLO     = 1;
static constexpr uint8_t  TYPE_WELCOME   = 2;
static constexpr uint8_t  TYPE_DATA      = 3;
static constexpr uint8_t  TYPE_KEEPALIVE = 4;
static constexpr uint8_t  TYPE_BYE      = 5;

static constexpr uint8_t  HAND_UNKNOWN = 0;
static constexpr uint8_t  HAND_LEFT    = 1;
static constexpr uint8_t  HAND_RIGHT   = 2;

static constexpr double   KEEPALIVE_INTERVAL_S = 1.0;
static constexpr double   DEVICE_TIMEOUT_S     = 5.0;

#pragma pack(push, 1)
struct SLPHeader  { uint32_t magic; uint8_t type; uint8_t fw; uint16_t reserved; };
struct SLPHello   { SLPHeader hdr; uint8_t mac[6]; uint8_t devType; uint8_t chCount; uint8_t hand; uint8_t rsv; };
struct SLPWelcome { SLPHeader hdr; uint16_t dataPort; uint16_t keepaliveMs; };
struct SLPData     { SLPHeader hdr; uint32_t packetId; uint32_t uptime; uint16_t filtered[12]; uint16_t touch; uint16_t i2cMs; uint16_t loopMs; int8_t rssi; uint8_t rsv; };
struct SLPKeep     { SLPHeader hdr; uint32_t lastPacketId; };
#pragma pack(pop)

// ===========================================================================
// Joint keys (same order as test_tracker.py)
// 0:mid_p 1:mid_d 2:ring_p 3:ring_d 4:pinky_p 5:pinky_d
// 6:thumb_p 7:thumb_d 8:index_p 9:index_d
// ===========================================================================
static constexpr int NUM_JOINTS = 10;
static const char* JOINT_NAMES[NUM_JOINTS] = {
    "mid_p","mid_d","ring_p","ring_d","pinky_p","pinky_d",
    "thumb_p","thumb_d","index_p","index_d"
};

// Source IDs:
//   -1       = None (use fallback)
//   0-11     = ESP32 electrode channel
//   100+     = SteamVR virtual sources
static constexpr int SRC_NONE         = -1;
static constexpr int SRC_TRIGGER      = 100;
static constexpr int SRC_GRIP         = 101;
static constexpr int SRC_TRIGGER_TOUCH = 102;
static constexpr int SRC_GRIP_TOUCH    = 103;
static constexpr int SRC_A_TOUCH      = 104;
static constexpr int SRC_B_TOUCH      = 105;
static constexpr int SRC_STICK_TOUCH   = 106;
static constexpr int SRC_THUMBREST_TOUCH = 107;
static constexpr int SRC_A_CLICK      = 108;
static constexpr int SRC_B_CLICK      = 109;
static constexpr int SRC_STICK_CLICK   = 110;
static constexpr int SRC_TRIGGER_CLICK = 111;

struct SourceOption { int id; const char* label; };
static const SourceOption SOURCE_OPTIONS[] = {
    {SRC_NONE, "None"},
    {0,"Elec 0"},{1,"Elec 1"},{2,"Elec 2"},{3,"Elec 3"},{4,"Elec 4"},{5,"Elec 5"},
    {6,"Elec 6"},{7,"Elec 7"},{8,"Elec 8"},{9,"Elec 9"},{10,"Elec 10"},{11,"Elec 11"},
    {SRC_TRIGGER,       "Trigger"},
    {SRC_GRIP,          "Grip"},
    {SRC_TRIGGER_TOUCH, "Trig Touch"},
    {SRC_GRIP_TOUCH,    "Grip Touch"},
    {SRC_A_TOUCH,       "A Touch"},
    {SRC_B_TOUCH,       "B Touch"},
    {SRC_STICK_TOUCH,   "Stick Touch"},
    {SRC_THUMBREST_TOUCH,"Thumbrest"},
    {SRC_A_CLICK,       "A Click"},
    {SRC_B_CLICK,       "B Click"},
    {SRC_STICK_CLICK,   "Stick Click"},
    {SRC_TRIGGER_CLICK, "Trig Click"},
};
static constexpr int NUM_SOURCES = sizeof(SOURCE_OPTIONS)/sizeof(SOURCE_OPTIONS[0]);

// Map curl output index → proximal joint index (_p)
// curl[0]=thumb → joint 6 (thumb_p), distal = joint 7 (thumb_d)
// curl[1]=index → joint 8 (index_p), distal = joint 9 (index_d)
// curl[2]=middle → joint 0 (mid_p), distal = joint 1 (mid_d)
// curl[3]=ring → joint 2 (ring_p), distal = joint 3 (ring_d)
// curl[4]=pinky → joint 4 (pinky_p), distal = joint 5 (pinky_d)
static constexpr int CURL_TO_JOINT[5] = { 6, 8, 0, 2, 4 };

// Default maps
static int DEFAULT_MAP_RIGHT[10] = { 0,1,2,3,4,5, SRC_NONE,SRC_NONE,SRC_TRIGGER,SRC_NONE };
static int DEFAULT_MAP_LEFT[10]  = { 0,1,2,3,4,5, 6,7,8,9 };

// ===========================================================================
// Smoother (median → EMA → deadband)
// ===========================================================================
struct Smoother {
    static constexpr int MAX_WIN = 9;
    float hist[MAX_WIN] = {};
    int   count = 0;
    int   head  = 0;
    float smoothed = 0.0f;
    float last_out = 0.0f;

    void reset() { count = 0; head = 0; smoothed = 0.0f; last_out = 0.0f; }

    float update(float raw, float alpha, int window, float deadband, bool enabled) {
        if (!enabled) return raw;
        if (window > MAX_WIN) window = MAX_WIN;
        if (window < 1) window = 1;

        hist[head] = raw;
        head = (head + 1) % window;
        if (count < window) count++;

        // Insertion sort on a tiny fixed-size buffer (no heap alloc)
        float tmp[9];
        memcpy(tmp, hist, count * sizeof(float));
        for (int i = 1; i < count; i++) {
            float v = tmp[i]; int j = i - 1;
            while (j >= 0 && tmp[j] > v) { tmp[j+1] = tmp[j]; j--; }
            tmp[j+1] = v;
        }
        float median = tmp[count / 2];
        smoothed = alpha * median + (1.0f - alpha) * smoothed;
        if (fabsf(smoothed - last_out) < deadband) return last_out;
        last_out = smoothed;
        return last_out;
    }
};

// ===========================================================================
// Per-hand runtime state
// ===========================================================================
struct HandState {
    std::string name;
    bool assigned = false;
    std::string mac;
    std::string ip;
    int port = SLP1_PORT;
    uint8_t fw = 0;
    std::chrono::steady_clock::time_point last_seen;
    std::chrono::steady_clock::time_point last_keepalive;
    uint32_t last_packet_id = 0;
    uint16_t filtered[12] = {};
    uint16_t touch = 0;
    int rssi = 0;
    uint32_t uptime = 0;
    uint16_t i2c_ms = 0;
    uint16_t loop_ms = 0;

    bool isAlive() const {
        return assigned && !ip.empty() &&
               (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_seen).count() < DEVICE_TIMEOUT_S);
    }
};

// ===========================================================================
// Config
// ===========================================================================
struct Config {
    float baseline = 200.0f;
    float max_delta = 185.0f;
    float coupling = 0.4f;
    bool  smooth_enabled = true;
    float smooth_alpha = 0.35f;
    int   smooth_median = 3;
    float smooth_deadband = 0.015f;
    int   elec_map[2][10]; // [hand][joint] — source IDs
    std::string saved_mac[2]; // persistent ESP32 MAC → hand assignment
    float pivot_offset[2][3]; // [hand][xyz] — Quest→Knuckles pivot offset (meters)

    Config() {
        for (int i = 0; i < 10; i++) {
            elec_map[0][i] = DEFAULT_MAP_LEFT[i];
            elec_map[1][i] = DEFAULT_MAP_RIGHT[i];
        }
    }
};

// ===========================================================================
// Global state
// ===========================================================================
static Config g_cfg;
static HandState g_hands[2]; // [0]=left, [1]=right
static Smoother g_smoothers[2][NUM_JOINTS]; // [hand][joint]
static std::mutex g_hands_mutex;
static std::atomic<bool> g_udp_running{false};
static int g_udp_port = SLP1_PORT;

static HANDLE g_drv_shm = nullptr;
static DriverSharedState* g_drv_state = nullptr;
static bool g_driver_connected = false;
static bool g_virtual_enabled = false;
static uint32_t g_curls_seq = 0;
static float g_fused_curls[2][5] = {};
static bool g_manual_override = false;
static float g_manual_curls[2][5] = {}; // 0..1 per finger

// OpenVR client (for haptics)
static vr::IVRSystem* g_vr_system = nullptr;
static vr::TrackedDeviceIndex_t g_quest_idx_left  = vr::k_unTrackedDeviceIndexInvalid;
static vr::TrackedDeviceIndex_t g_quest_idx_right = vr::k_unTrackedDeviceIndexInvalid;
static uint32_t g_last_haptic_seq_l = 0;
static uint32_t g_last_haptic_seq_r = 0;

// ===========================================================================
// Logging
// ===========================================================================
struct LogEntry { std::string ts, text; ImVec4 color; };
static std::mutex g_log_mutex;
static std::vector<LogEntry> g_log_entries;

static void LogMsg(const char* text, ImVec4 color) {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    struct tm tmv;
    localtime_s(&tmv, &t);
    char ts[32]; strftime(ts, sizeof(ts), "%H:%M:%S", &tmv);
    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_log_entries.push_back({ ts, text, color });
    if (g_log_entries.size() > 500) g_log_entries.erase(g_log_entries.begin());
}
static void LogI(const char* fmt, ...) {
    char buf[1024]; va_list a; va_start(a, fmt); vsnprintf(buf, sizeof(buf), fmt, a); va_end(a);
    LogMsg(buf, ImVec4(0.7f, 0.8f, 1.0f, 1.0f));
}
static void LogO(const char* fmt, ...) {
    char buf[1024]; va_list a; va_start(a, fmt); vsnprintf(buf, sizeof(buf), fmt, a); va_end(a);
    LogMsg(buf, ImVec4(0.5f, 1.0f, 0.5f, 1.0f));
}
static void LogE(const char* fmt, ...) {
    char buf[1024]; va_list a; va_start(a, fmt); vsnprintf(buf, sizeof(buf), fmt, a); va_end(a);
    LogMsg(buf, ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
}

// ===========================================================================
// MAC helpers
// ===========================================================================
static std::string mac_to_str(const uint8_t* b) {
    char buf[18]; snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", b[0],b[1],b[2],b[3],b[4],b[5]);
    return buf;
}

// ===========================================================================
// Driver shared memory
// ===========================================================================
static bool ConnectDriver() {
    if (g_drv_state) return true;
    g_drv_shm = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, DriverSharedState::kShmName);
    if (!g_drv_shm) return false;
    g_drv_state = (DriverSharedState*)MapViewOfFile(g_drv_shm, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(DriverSharedState));
    if (!g_drv_state) { CloseHandle(g_drv_shm); g_drv_shm = nullptr; return false; }
    if (g_drv_state->magic != DriverSharedState::kMagic) {
        UnmapViewOfFile(g_drv_state); g_drv_state = nullptr;
        CloseHandle(g_drv_shm); g_drv_shm = nullptr; return false;
    }
    LogO("Connected to driver shared memory");
    return true;
}
static void DisconnectDriver() {
    if (g_drv_state) { UnmapViewOfFile(g_drv_state); g_drv_state = nullptr; }
    if (g_drv_shm) { CloseHandle(g_drv_shm); g_drv_shm = nullptr; }
}

// ===========================================================================
// OpenVR client (for haptics)
// ===========================================================================
static bool InitOpenVRClient() {
    if (g_vr_system) return true;
    vr::EVRInitError err = vr::VRInitError_None;
    g_vr_system = vr::VR_Init(&err, vr::VRApplication_Background);
    if (err != vr::VRInitError_None || !g_vr_system) { g_vr_system = nullptr; return false; }
    LogO("OpenVR client connected (background mode)");
    return true;
}
static void ShutdownOpenVRClient() {
    if (g_vr_system) { vr::VR_Shutdown(); g_vr_system = nullptr; }
    g_quest_idx_left = g_quest_idx_right = vr::k_unTrackedDeviceIndexInvalid;
}
static void UpdateQuestIndices() {
    if (!g_drv_state || !g_vr_system) return;
    uint32_t nl = g_drv_state->quest_left_index, nr = g_drv_state->quest_right_index;
    if (nl != g_quest_idx_left || nr != g_quest_idx_right) {
        g_quest_idx_left = nl; g_quest_idx_right = nr;
        LogI("Quest indices: L=%u R=%u", nl, nr);
    }
}
static void ForwardHaptics() {
    if (!g_drv_state || !g_vr_system) return;
    if (g_drv_state->haptic_seq_left != g_last_haptic_seq_l) {
        g_last_haptic_seq_l = g_drv_state->haptic_seq_left;
        float amp = g_drv_state->haptic_amplitude_left, dur = g_drv_state->haptic_duration_left;
        if (amp > 0.f && g_quest_idx_left != vr::k_unTrackedDeviceIndexInvalid) {
            uint32_t us = (uint32_t)(dur * 1e6f * amp); if (us > 3999) us = 3999; if (us < 500) us = 500;
            g_vr_system->TriggerHapticPulse(g_quest_idx_left, 0, us);
        }
    }
    if (g_drv_state->haptic_seq_right != g_last_haptic_seq_r) {
        g_last_haptic_seq_r = g_drv_state->haptic_seq_right;
        float amp = g_drv_state->haptic_amplitude_right, dur = g_drv_state->haptic_duration_right;
        if (amp > 0.f && g_quest_idx_right != vr::k_unTrackedDeviceIndexInvalid) {
            uint32_t us = (uint32_t)(dur * 1e6f * amp); if (us > 3999) us = 3999; if (us < 500) us = 500;
            g_vr_system->TriggerHapticPulse(g_quest_idx_right, 0, us);
        }
    }
}

// ===========================================================================
// UDP thread — SLP1 protocol
// ===========================================================================
static SOCKET g_sock = INVALID_SOCKET;

static void send_packet(const char* ip, int port, const void* data, int len) {
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &addr.sin_addr);
    sendto(g_sock, (const char*)data, len, 0, (sockaddr*)&addr, sizeof(addr));
}

static void send_welcome(HandState& hs) {
    SLPWelcome pkt = {};
    pkt.hdr.magic = SLP1_MAGIC; pkt.hdr.type = TYPE_WELCOME; pkt.hdr.fw = 3;
    pkt.dataPort = SLP1_PORT;
    pkt.keepaliveMs = (uint16_t)(KEEPALIVE_INTERVAL_S * 1000);
    send_packet(hs.ip.c_str(), hs.port, &pkt, sizeof(pkt));
    hs.last_keepalive = std::chrono::steady_clock::now();
    LogI("Sent WELCOME to %s:%d (%s)", hs.ip.c_str(), hs.port, hs.name.c_str());
}

static void send_keepalive(HandState& hs) {
    SLPKeep pkt = {};
    pkt.hdr.magic = SLP1_MAGIC; pkt.hdr.type = TYPE_KEEPALIVE; pkt.hdr.fw = 3;
    pkt.lastPacketId = hs.last_packet_id;
    send_packet(hs.ip.c_str(), hs.port, &pkt, sizeof(pkt));
    hs.last_keepalive = std::chrono::steady_clock::now();
}

static void send_bye(HandState& hs) {
    SLPKeep pkt = {};
    pkt.hdr.magic = SLP1_MAGIC; pkt.hdr.type = TYPE_BYE; pkt.hdr.fw = 3;
    send_packet(hs.ip.c_str(), hs.port, &pkt, sizeof(pkt));
}

static void UdpThreadFunc() {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) { LogE("WSAStartup failed"); return; }

    g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_sock == INVALID_SOCKET) { LogE("socket creation failed"); WSACleanup(); return; }

    BOOL reuse = TRUE;
    setsockopt(g_sock, SOL_SOCKET, SO_REUSEADDR, (char*)&reuse, sizeof(reuse));

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(g_udp_port);
    if (bind(g_sock, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        LogE("bind failed on port %d (WSA=%d)", g_udp_port, WSAGetLastError());
        closesocket(g_sock); WSACleanup(); return;
    }

    // Non-blocking
    u_long mode = 1; ioctlsocket(g_sock, FIONBIO, &mode);
    LogO("SLP1 listening on UDP %d", g_udp_port);

    char buf[1024];
    sockaddr_in from; int fromlen = sizeof(from);

    while (g_udp_running) {
        int n = recvfrom(g_sock, buf, sizeof(buf), 0, (sockaddr*)&from, &fromlen);
        if (n > 0) {
            if (n < (int)sizeof(SLPHeader)) continue;
            SLPHeader* hdr = (SLPHeader*)buf;
            if (hdr->magic != SLP1_MAGIC) continue;

            char ip[32]; inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));

            if (hdr->type == TYPE_HELLO && n >= (int)sizeof(SLPHello)) {
                SLPHello* hello = (SLPHello*)buf;
                std::string mac = mac_to_str(hello->mac);
                uint8_t hand_hint = hello->hand;

                std::lock_guard<std::mutex> lock(g_hands_mutex);
                // Check if already assigned to a slot
                bool found = false;
                for (int h = 0; h < 2; h++) {
                    if (g_hands[h].mac == mac) {
                        g_hands[h].ip = ip; g_hands[h].fw = hdr->fw;
                        g_hands[h].last_seen = std::chrono::steady_clock::now();
                        g_hands[h].last_keepalive = {}; // force WELCOME so ESP32 resumes DATA streaming
                        send_welcome(g_hands[h]);
                        found = true; LogI("HELLO from known %s (%s hand)", mac.c_str(), g_hands[h].name.c_str()); break;
                    }
                }
                if (!found) {
                    // Check saved MACs from config first (persistent assignment)
                    int slot = -1;
                    if (g_cfg.saved_mac[0] == mac) slot = 0;
                    else if (g_cfg.saved_mac[1] == mac) slot = 1;
                    else if (hand_hint == HAND_LEFT)       slot = 0;
                    else if (hand_hint == HAND_RIGHT)      slot = 1;
                    else {
                        // assign to first empty slot
                        for (int h = 0; h < 2; h++) if (!g_hands[h].assigned) { slot = h; break; }
                    }
                    if (slot >= 0 && !g_hands[slot].assigned) {
                        g_hands[slot].assigned = true;
                        g_hands[slot].mac = mac;
                        g_hands[slot].ip = ip;
                        g_hands[slot].port = SLP1_PORT;
                        g_hands[slot].fw = hdr->fw;
                        g_hands[slot].last_seen = std::chrono::steady_clock::now();
                        g_hands[slot].last_keepalive = {};
                        g_cfg.saved_mac[slot] = mac;
                        send_welcome(g_hands[slot]);
                        LogO("HELLO from %s → assigned to %s hand", mac.c_str(), g_hands[slot].name.c_str());
                    } else {
                        LogI("HELLO from %s (no free slot)", mac.c_str());
                    }
                }
            }
            else if (hdr->type == TYPE_DATA && n >= (int)sizeof(SLPData)) {
                SLPData* data = (SLPData*)buf;
                std::lock_guard<std::mutex> lock(g_hands_mutex);
                for (int h = 0; h < 2; h++) {
                    if (g_hands[h].ip == ip) {
                        g_hands[h].last_packet_id = data->packetId;
                        g_hands[h].uptime = data->uptime;
                        memcpy(g_hands[h].filtered, data->filtered, sizeof(g_hands[h].filtered));
                        g_hands[h].touch = data->touch;
                        g_hands[h].i2c_ms = data->i2cMs;
                        g_hands[h].loop_ms = data->loopMs;
                        g_hands[h].rssi = data->rssi;
                        g_hands[h].last_seen = std::chrono::steady_clock::now();
                        break;
                    }
                }
            }
        }

        // Send keepalives
        auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(g_hands_mutex);
            for (int h = 0; h < 2; h++) {
                if (g_hands[h].isAlive()) {
                    double since = std::chrono::duration<double>(now - g_hands[h].last_keepalive).count();
                    if (since >= KEEPALIVE_INTERVAL_S)
                        send_keepalive(g_hands[h]);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Send BYE to all alive hands
    {
        std::lock_guard<std::mutex> lock(g_hands_mutex);
        for (int h = 0; h < 2; h++) {
            if (g_hands[h].isAlive()) send_bye(g_hands[h]);
        }
    }
    closesocket(g_sock);
    WSACleanup();
    LogI("UDP receiver stopped");
}

// ===========================================================================
// Unified source lookup — resolves a source ID to a 0-1 curl value
// Returns -1.0f if source not available (triggers fallback)
// ===========================================================================
static float resolve_source_locked(int hand_idx, int joint_idx, const HandState& hs) {
    int src = g_cfg.elec_map[hand_idx][joint_idx];
    if (src == SRC_NONE) return -1.0f;

    // Electrode source (0-11)
    if (src >= 0 && src < 12) {
        if (!hs.isAlive()) return -1.0f;
        float raw = hs.filtered[src];
        float delta = fmaxf(0.0f, g_cfg.baseline - raw);
        float norm = fminf(1.0f, delta / fmaxf(1.0f, g_cfg.max_delta));
        return g_smoothers[hand_idx][joint_idx].update(norm, g_cfg.smooth_alpha, g_cfg.smooth_median,
                                               g_cfg.smooth_deadband, g_cfg.smooth_enabled);
    }

    // SteamVR virtual sources (100+)
    if (!g_drv_state) return -1.0f;
    bool left = (hand_idx == 0);
    switch (src) {
        case SRC_TRIGGER:        return left ? g_drv_state->trigger_left  : g_drv_state->trigger_right;
        case SRC_GRIP:           return left ? g_drv_state->grip_left     : g_drv_state->grip_right;
        case SRC_TRIGGER_TOUCH:  return (left ? g_drv_state->trigger_touch_left  : g_drv_state->trigger_touch_right)  ? 0.5f : 0.0f;
        case SRC_GRIP_TOUCH:     return (left ? g_drv_state->grip_touch_left     : g_drv_state->grip_touch_right)     ? 0.5f : 0.0f;
        case SRC_A_TOUCH:        return (left ? g_drv_state->a_touch_left  : g_drv_state->a_touch_right)  ? 0.5f : 0.0f;
        case SRC_B_TOUCH:        return (left ? g_drv_state->b_touch_left  : g_drv_state->b_touch_right)  ? 0.5f : 0.0f;
        case SRC_STICK_TOUCH:    return (left ? g_drv_state->thumbstick_touch_left : g_drv_state->thumbstick_touch_right) ? 0.5f : 0.0f;
        case SRC_THUMBREST_TOUCH:return (left ? g_drv_state->thumbrest_touch_left : g_drv_state->thumbrest_touch_right) ? 0.5f : 0.0f;
        case SRC_A_CLICK:        return (left ? g_drv_state->a_click_left  : g_drv_state->a_click_right)  ? 1.0f : 0.0f;
        case SRC_B_CLICK:        return (left ? g_drv_state->b_click_left  : g_drv_state->b_click_right)  ? 1.0f : 0.0f;
        case SRC_STICK_CLICK:    return (left ? g_drv_state->thumbstick_click_left : g_drv_state->thumbstick_click_right) ? 1.0f : 0.0f;
        case SRC_TRIGGER_CLICK:  return (left ? g_drv_state->trigger_click_left : g_drv_state->trigger_click_right) ? 1.0f : 0.0f;
    }
    return -1.0f;
}

static void FuseFingers() {
    // Snapshot hand data under lock (minimal hold time)
    HandState snap[2];
    {
        std::lock_guard<std::mutex> lock(g_hands_mutex);
        snap[0] = g_hands[0];
        snap[1] = g_hands[1];
    }

    // Fuse outside the lock so the UDP thread can keep draining packets
    // Each finger = proximal (_p, 0-50%) + distal (_d, 0-50%) with distal coupling
    // joint pairs: thumb=6/7, index=8/9, middle=0/1, ring=2/3, pinky=4/5
    // CURL_TO_JOINT[f] gives the _p joint; _d = _p + 1
    for (int h = 0; h < 2; h++) {
        float* curls = g_fused_curls[h];
        float trig = g_drv_state ? (h == 0 ? g_drv_state->trigger_left : g_drv_state->trigger_right) : 0.0f;
        float grip = g_drv_state ? (h == 0 ? g_drv_state->grip_left : g_drv_state->grip_right) : 0.0f;
        uint8_t stick_t = g_drv_state ? (h == 0 ? g_drv_state->thumbstick_touch_left : g_drv_state->thumbstick_touch_right) : 0;
        uint8_t a_t = g_drv_state ? (h == 0 ? g_drv_state->a_touch_left : g_drv_state->a_touch_right) : 0;
        uint8_t b_t = g_drv_state ? (h == 0 ? g_drv_state->b_touch_left : g_drv_state->b_touch_right) : 0;
        uint8_t tr_t = g_drv_state ? (h == 0 ? g_drv_state->thumbrest_touch_left : g_drv_state->thumbrest_touch_right) : 0;

        for (int f = 0; f < 5; f++) {
            int p_joint = CURL_TO_JOINT[f];
            int d_joint = p_joint + 1;

            float p_val = resolve_source_locked(h, p_joint, snap[h]);
            float d_val = resolve_source_locked(h, d_joint, snap[h]);

            if (p_val >= 0.0f || d_val >= 0.0f) {
                // At least one source configured — combine them
                if (p_val < 0.0f) p_val = 0.0f;
                if (d_val < 0.0f) d_val = 0.0f;
                p_val = fminf(1.0f, p_val + g_cfg.coupling * d_val);
                curls[f] = p_val * 0.5f + d_val * 0.5f;
            } else {
                // Both sources unavailable — use fallback
                switch (f) {
                    case 0: curls[0] = (stick_t || a_t || b_t || tr_t) ? 0.5f : 0.0f; break;
                    case 1: curls[1] = trig; break;
                    case 2: curls[2] = grip; break;
                    case 3: curls[3] = grip; break;
                    case 4: curls[4] = grip; break;
                }
            }
        }
    }
}

// ===========================================================================
// Write to driver
// ===========================================================================
static void WriteToDriver() {
    if (!g_drv_state) return;
    memcpy(g_drv_state->curls_left, g_fused_curls[0], sizeof(float) * 5);
    memcpy(g_drv_state->curls_right, g_fused_curls[1], sizeof(float) * 5);
    memcpy(g_drv_state->pivot_offset_left, g_cfg.pivot_offset[0], sizeof(float) * 3);
    memcpy(g_drv_state->pivot_offset_right, g_cfg.pivot_offset[1], sizeof(float) * 3);
    g_drv_state->virtual_enabled = g_virtual_enabled ? 1 : 0;
    g_drv_state->curls_sequence = ++g_curls_seq;
}

// ===========================================================================
// Config persistence (simple text format, next to exe)
// ===========================================================================
static std::string GetConfigPath() {
    char buf[1024];
    GetModuleFileNameA(NULL, buf, sizeof(buf));
    char* dot = strrchr(buf, '.');
    if (dot) *dot = '\0';
    strcat_s(buf, ".cfg");
    return buf;
}

static void SaveConfig() {
    FILE* f = nullptr;
    fopen_s(&f, GetConfigPath().c_str(), "w");
    if (!f) { LogE("SaveConfig: can't write %s", GetConfigPath().c_str()); return; }
    fprintf(f, "# SloppyHandJob config\n");
    fprintf(f, "map_left=");
    for (int j = 0; j < 10; j++) fprintf(f, "%s%d", j>0?" ":"", g_cfg.elec_map[0][j]);
    fprintf(f, "\nmap_right=");
    for (int j = 0; j < 10; j++) fprintf(f, "%s%d", j>0?" ":"", g_cfg.elec_map[1][j]);
    fprintf(f, "\nbaseline=%.0f\n", g_cfg.baseline);
    fprintf(f, "max_delta=%.0f\n", g_cfg.max_delta);
    fprintf(f, "coupling=%.2f\n", g_cfg.coupling);
    fprintf(f, "smooth_enabled=%d\n", g_cfg.smooth_enabled ? 1 : 0);
    fprintf(f, "smooth_alpha=%.2f\n", g_cfg.smooth_alpha);
    fprintf(f, "smooth_median=%d\n", g_cfg.smooth_median);
    fprintf(f, "smooth_deadband=%.3f\n", g_cfg.smooth_deadband);
    fprintf(f, "mac_left=%s\n", g_cfg.saved_mac[0].c_str());
    fprintf(f, "mac_right=%s\n", g_cfg.saved_mac[1].c_str());
    fprintf(f, "pivot_left=%.3f,%.3f,%.3f\n", g_cfg.pivot_offset[0][0], g_cfg.pivot_offset[0][1], g_cfg.pivot_offset[0][2]);
    fprintf(f, "pivot_right=%.3f,%.3f,%.3f\n", g_cfg.pivot_offset[1][0], g_cfg.pivot_offset[1][1], g_cfg.pivot_offset[1][2]);
    fclose(f);
    LogI("Config saved to %s", GetConfigPath().c_str());
}

static void LoadConfig() {
    FILE* f = nullptr;
    fopen_s(&f, GetConfigPath().c_str(), "r");
    if (!f) { LogI("No config file found, using defaults"); return; }
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        int v[10];
        if (sscanf_s(line, "map_left=%d %d %d %d %d %d %d %d %d %d",
            &v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6],&v[7],&v[8],&v[9]) == 10)
            for (int j = 0; j < 10; j++) g_cfg.elec_map[0][j] = v[j];
        else if (sscanf_s(line, "map_right=%d %d %d %d %d %d %d %d %d %d",
            &v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6],&v[7],&v[8],&v[9]) == 10)
            for (int j = 0; j < 10; j++) g_cfg.elec_map[1][j] = v[j];
        else if (sscanf_s(line, "baseline=%f", &g_cfg.baseline) == 1) {}
        else if (sscanf_s(line, "max_delta=%f", &g_cfg.max_delta) == 1) {}
        else if (sscanf_s(line, "coupling=%f", &g_cfg.coupling) == 1) {}
        else if (sscanf_s(line, "smooth_enabled=%d", &v[0]) == 1) g_cfg.smooth_enabled = (v[0] != 0);
        else if (sscanf_s(line, "smooth_alpha=%f", &g_cfg.smooth_alpha) == 1) {}
        else if (sscanf_s(line, "smooth_median=%d", &g_cfg.smooth_median) == 1) {}
        else if (sscanf_s(line, "smooth_deadband=%f", &g_cfg.smooth_deadband) == 1) {}
        else {
            char key[64] = {}, val[256] = {};
            if (sscanf_s(line, " %63[^=]=%255s", key, (unsigned)sizeof(key), val, (unsigned)sizeof(val)) == 2) {
                if (strcmp(key, "mac_left") == 0) g_cfg.saved_mac[0] = val;
                else if (strcmp(key, "mac_right") == 0) g_cfg.saved_mac[1] = val;
                else if (strcmp(key, "pivot_left") == 0) {
                    float x = 0, y = 0, z = 0;
                    if (sscanf_s(val, "%f,%f,%f", &x, &y, &z) >= 3) {
                        g_cfg.pivot_offset[0][0] = x; g_cfg.pivot_offset[0][1] = y; g_cfg.pivot_offset[0][2] = z;
                    }
                }
                else if (strcmp(key, "pivot_right") == 0) {
                    float x = 0, y = 0, z = 0;
                    if (sscanf_s(val, "%f,%f,%f", &x, &y, &z) >= 3) {
                        g_cfg.pivot_offset[1][0] = x; g_cfg.pivot_offset[1][1] = y; g_cfg.pivot_offset[1][2] = z;
                    }
                }
            }
        }
    }
    fclose(f);
    LogI("Config loaded from %s", GetConfigPath().c_str());
}

// ===========================================================================
// Main
// ===========================================================================
int main(int argc, char* argv[]) {
    LoadConfig();
    g_hands[0].name = "left";
    g_hands[1].name = "right";

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) { MessageBoxA(NULL, "SDL_Init failed", "Error", MB_OK); return 1; }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    SDL_Window* window = SDL_CreateWindow("SloppyHandJob", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 960, 640, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_GLContext gl_ctx = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_ctx);
    SDL_GL_SetSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForOpenGL(window, gl_ctx);
    ImGui_ImplOpenGL3_Init("#version 130");

    LogI("SloppyHandJob started (SLP1 protocol, port %d)", SLP1_PORT);
    InitOpenVRClient();

    bool done = false;
    while (!done) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) done = true;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window)) done = true;
        }

        if (!g_driver_connected && ConnectDriver()) g_driver_connected = true;
        if (!g_vr_system) InitOpenVRClient();
        if (g_vr_system && g_drv_state) UpdateQuestIndices();
        if (g_driver_connected) {
            if (g_manual_override) {
                for (int h = 0; h < 2; h++)
                    memcpy(g_fused_curls[h], g_manual_curls[h], sizeof(float) * 5);
            } else {
                FuseFingers();
            }
            WriteToDriver();
        }
        ForwardHaptics();

        // ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(io.DisplaySize);
        ImGui::Begin("SloppyHandJob", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

        if (ImGui::BeginTabBar("##tabs")) {

            // --- Controllers Tab ---
            if (ImGui::BeginTabItem("Controllers")) {
                ImGui::Text("Driver: %s", g_driver_connected ? "Connected" : "Not connected");
                ImGui::SameLine();
                if (!g_driver_connected && ImGui::Button("Retry")) ConnectDriver();
                ImGui::Separator();
                if (g_drv_state) {
                    ImGui::Text("Quest L: %s  R: %s", g_drv_state->quest_left_detected?"Yes":"No", g_drv_state->quest_right_detected?"Yes":"No");
                    ImGui::Text("Proxy L: %s  R: %s", g_drv_state->proxy_left_active?"Active":"Off", g_drv_state->proxy_right_active?"Off":"Off");
                }
                ImGui::Separator();
                if (ImGui::Checkbox("Enable Virtual Knuckles", &g_virtual_enabled)) {
                    LogO(g_virtual_enabled ? "Virtual controllers ENABLED" : "Virtual controllers DISABLED");
                }
                if (g_virtual_enabled) ImGui::TextColored(ImVec4(0.5f,1.0f,0.5f,1.0f), "ACTIVE");
                else ImGui::TextColored(ImVec4(1.0f,0.6f,0.3f,1.0f), "DISABLED");
                ImGui::Separator();
                ImGui::Text("Pivot Offset (Quest IMU -> Knuckles center)");
                ImGui::DragFloat3("Left  (X,Y,Z)", g_cfg.pivot_offset[0], 0.001f, -0.1f, 0.1f, "%.3f m");
                ImGui::DragFloat3("Right (X,Y,Z)", g_cfg.pivot_offset[1], 0.001f, -0.1f, 0.1f, "%.3f m");
                ImGui::EndTabItem();
            }

            // --- Finger Tracking Tab ---
            if (ImGui::BeginTabItem("Finger Tracking")) {
                ImGui::Text("SLP1 Protocol (port %d)", g_udp_port);
                ImGui::SameLine();
                if (!g_udp_running && ImGui::Button("Start")) {
                    g_udp_running = true;
                    std::thread(UdpThreadFunc).detach();
                }
                if (g_udp_running && ImGui::Button("Stop")) g_udp_running = false;
                ImGui::SameLine();
                ImGui::Text("%s", g_udp_running ? "Listening" : "Stopped");
                ImGui::Separator();

                // Device status + hand reassignment
                {
                    std::lock_guard<std::mutex> lock(g_hands_mutex);
                    for (int h = 0; h < 2; h++) {
                        HandState& hs = g_hands[h];
                        ImGui::Text("%s:", hs.name.c_str());
                        ImGui::SameLine(60);
                        if (hs.isAlive()) {
                            auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - hs.last_seen).count();
                            ImGui::Text("MAC=%s IP=%s rssi=%d %dms ago  pkt=%u",
                                hs.mac.c_str(), hs.ip.c_str(), hs.rssi, (int)(age*1000), hs.last_packet_id);
                        } else {
                            ImGui::TextDisabled("not connected");
                        }
                    }
                    // Swap hands button
                    if (ImGui::Button("Swap L <-> R")) {
                        LogI("Swapping ESP32 left/right assignments");
                        std::swap(g_hands[0], g_hands[1]);
                        g_hands[0].name = "left";
                        g_hands[1].name = "right";
                        std::swap(g_cfg.saved_mac[0], g_cfg.saved_mac[1]);
                        // Reset smoothers and keepalives so WELCOME gets re-sent
                        for (int s = 0; s < 2; s++) {
                            for (int j = 0; j < NUM_JOINTS; j++) g_smoothers[s][j].reset();
                            g_hands[s].last_keepalive = {};
                        }
                    }
                }
                ImGui::Separator();

                // Electrode / source mapping
                if (ImGui::CollapsingHeader("Source Map (per joint)")) {
                    ImGui::TextWrapped("Map each joint to an electrode (0-11), a SteamVR input, or None (fallback).");
                    const char* hand_labels[2] = {"Left hand", "Right hand"};
                    for (int h = 0; h < 2; h++) {
                        ImGui::PushID(h);
                        ImGui::Text("%s", hand_labels[h]);
                        for (int j = 0; j < NUM_JOINTS; j++) {
                            ImGui::PushID(j);
                            ImGui::Text("%s", JOINT_NAMES[j]);
                            ImGui::SameLine(80);
                            int cur = g_cfg.elec_map[h][j];
                            // Find preview label
                            const char* preview = "None";
                            for (int s = 0; s < NUM_SOURCES; s++) {
                                if (SOURCE_OPTIONS[s].id == cur) { preview = SOURCE_OPTIONS[s].label; break; }
                            }
                            if (ImGui::BeginCombo("##src", preview, 0)) {
                                for (int s = 0; s < NUM_SOURCES; s++) {
                                    bool sel = (cur == SOURCE_OPTIONS[s].id);
                                    if (ImGui::Selectable(SOURCE_OPTIONS[s].label, sel))
                                        g_cfg.elec_map[h][j] = SOURCE_OPTIONS[s].id;
                                    if (sel) ImGui::SetItemDefaultFocus();
                                }
                                ImGui::EndCombo();
                            }
                            ImGui::PopID();
                        }
                        ImGui::Separator();
                        ImGui::PopID();
                    }
                    if (ImGui::Button("Save Config")) SaveConfig();
                    ImGui::SameLine();
                    if (ImGui::Button("Load Config")) LoadConfig();
                }
                ImGui::Separator();

                // Config sliders
                ImGui::SliderFloat("Baseline", &g_cfg.baseline, 0, 400, "%.0f");
                ImGui::SliderFloat("Max Delta", &g_cfg.max_delta, 1, 400, "%.0f");
                ImGui::SliderFloat("Distal Coupling", &g_cfg.coupling, 0, 1, "%.2f");
                ImGui::Separator();
                ImGui::Checkbox("Smoothing", &g_cfg.smooth_enabled);
                ImGui::SliderFloat("EMA Alpha", &g_cfg.smooth_alpha, 0.01f, 1.0f, "%.2f");
                ImGui::SliderInt("Median Window", &g_cfg.smooth_median, 1, 9);
                if (g_cfg.smooth_median % 2 == 0) g_cfg.smooth_median++; // force odd
                ImGui::SliderFloat("Deadband", &g_cfg.smooth_deadband, 0, 0.2f, "%.3f");
                ImGui::Separator();

                // Manual override
                ImGui::Checkbox("Manual Override", &g_manual_override);
                if (g_manual_override) {
                    ImGui::TextColored(ImVec4(1,1,0,1), "BYPASSING ESP32 — using manual curl values");
                    const char* fnames[5] = {"Thumb","Index","Middle","Ring","Pinky"};
                    for (int h = 0; h < 2; h++) {
                        ImGui::Text("%s:", h==0?"Left":"Right");
                        for (int f = 0; f < 5; f++) {
                            ImGui::PushID(1000 + h*10 + f);
                            ImGui::SliderFloat(fnames[f], &g_manual_curls[h][f], 0.0f, 1.0f, "%.2f");
                            ImGui::PopID();
                        }
                    }
                }
                ImGui::Separator();
                ImGui::Text("Fused Curls (sent to driver):");
                const char* fnames[5] = {"Thumb","Index","Middle","Ring","Pinky"};
                for (int h = 0; h < 2; h++) {
                    ImGui::Text("  %s:", h==0?"Left":"Right");
                    for (int f = 0; f < 5; f++) {
                        ImGui::PushID(h*10+f);
                        int p_joint = CURL_TO_JOINT[f];
                        int d_joint = p_joint + 1;
                        int src_p = g_cfg.elec_map[h][p_joint];
                        int src_d = g_cfg.elec_map[h][d_joint];
                        auto src_name = [](int id) -> const char* {
                            for (int s = 0; s < NUM_SOURCES; s++)
                                if (SOURCE_OPTIONS[s].id == id) return SOURCE_OPTIONS[s].label;
                            return "?";
                        };
                        ImGui::Text("    %s: %.2f  [p=%s, d=%s]",
                            fnames[f], g_fused_curls[h][f],
                            src_name(src_p), src_name(src_d));
                        ImGui::PopID();
                    }
                }

                // Raw electrode bars (ESP32 data)
                ImGui::Separator();
                ImGui::Text("Raw Electrode Values:");
                {
                    std::lock_guard<std::mutex> lock(g_hands_mutex);
                    for (int h = 0; h < 2; h++) {
                        HandState& hs = g_hands[h];
                        bool alive = hs.isAlive();
                        ImGui::Text("%s: %s", h==0?"Left":"Right", alive?"connected":"offline");
                        if (alive) {
                            for (int e = 0; e < 12; e++) {
                                ImGui::PushID(h*20 + e);
                                if (e > 0) ImGui::SameLine();
                                float pct = fmaxf(0, g_cfg.baseline - hs.filtered[e]) / fmaxf(1, g_cfg.max_delta);
                                if (pct > 1) pct = 1;
                                ImGui::ProgressBar(pct, ImVec2(22, 8), "");
                                if (ImGui::IsItemHovered())
                                    ImGui::SetTooltip("E%d: raw=%u delta=%.0f", e, hs.filtered[e],
                                        fmaxf(0, g_cfg.baseline - hs.filtered[e]));
                                ImGui::PopID();
                            }
                        }
                    }
                }
                ImGui::EndTabItem();
            }

            // --- Log Tab ---
            if (ImGui::BeginTabItem("Log")) {
                if (ImGui::Button("Clear")) { std::lock_guard<std::mutex> l(g_log_mutex); g_log_entries.clear(); }
                ImGui::Separator();
                ImGui::BeginChild("logscroll", ImVec2(0,0), false, ImGuiWindowFlags_HorizontalScrollbar);
                {
                    std::lock_guard<std::mutex> l(g_log_mutex);
                    for (auto& e : g_log_entries)
                        ImGui::TextColored(e.color, "[%s] %s", e.ts.c_str(), e.text.c_str());
                }
                if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) ImGui::SetScrollHereY(1.0f);
                ImGui::EndChild();
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
        ImGui::End();

        // Render
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
        SDL_Delay(16);
    }

    // Cleanup
    SaveConfig();
    g_udp_running = false;
    SDL_Delay(100);
    ShutdownOpenVRClient();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl_ctx);
    SDL_DestroyWindow(window);
    SDL_Quit();
    DisconnectDriver();
    return 0;
}