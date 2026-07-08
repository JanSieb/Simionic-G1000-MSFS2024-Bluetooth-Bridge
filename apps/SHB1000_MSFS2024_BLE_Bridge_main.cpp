#include <array>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <fstream>
#include <cctype>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <optional>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>
#include <conio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <map>
#include <Windows.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <future>
#include <cmath>

#include "simpleble/SimpleBLE.h"
#include "simpleble/Config.h"
#include "WASMIF.h"

#include <set>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Devices.Bluetooth.Advertisement.h>

#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>

using json = nlohmann::json;

#ifndef NOMINMAX
#define NOMINMAX
#endif

// Application version
static const std::string APP_VERSION = "1.08.1";

// --- ORBIT LOGIC ---
static std::atomic<bool> g_orbit_active{false};
static std::atomic<bool> g_orbit_evaluation_pending{false};
static auto g_last_orbit_time = std::chrono::steady_clock::now();
// -------------------

// --- GGGD ALARM STATE ---
static std::atomic<bool> g_air_alarm_triggered{true};
static std::atomic<bool> g_ground_alarm_triggered{true};
// ------------------------

// Configuration of filenames (default)
constexpr const char* CONFIG_FILENAME = "SHB1000.config";             // HARD CODE filename for general settings
std::string KNOWN_DEVICES_FILENAME = "SHB1000_known_devices.config";  // Variable for the configuration of known devices, pre-assigned with DEFAULT
std::string COMMANDS_FILENAME = "SHB1000_commands.map";               // Variable for the configuration of commands, pre-assigned with DEFAULT

bool g_discoveryFinished = false; // Is only set to true when the setup dialog is finished
bool initialBrightnessSet = false;  // If the brightness of the backlight has been initially set
bool g_print_commands_to_console = false; // If true, ble commands and mappings are printed to std::cout

std::atomic<bool> g_trigger_auto_reconnect{false}; // Trigger flag for auto-reconnect

struct HardwareDevice {
    std::string instrument; // PFD, MFD, RADIO
    std::string type;       // SHB1000, SH100AP
    std::string position;   // left, right, center
    std::string ble_name;   // Bluetooth name (e.g. "SHB1000")
    std::string address;    // AA:BB:CC:DD:EE:FF
    std::string last_connected; // Timestamp of the last connection
    bool auto_reconnect;     // Should the device automatically be reconnected in this role?
};

// List of known devices
static std::vector<HardwareDevice> g_knownDevices;

// Stores the history (the last 5 timestamps) of the calls per role and dial/button
static std::unordered_map<std::string, std::vector<std::chrono::steady_clock::time_point>> last_input_times;

// This vector holds the snapshot of the last successful connection
std::map<std::string, std::string> lastSessionDevices; // Key: Role, Value: MAC

// New structure for the global hardware settings from [Settings]
struct AircraftHistoryEntry {
    std::string map_id;
    std::string last_used;
};

struct GlobalSettings {
    std::string device_map_file;        // Specifies the file of the last know devices (with their roles etc.)
    std::string default_map_id = "std_g1000";
    std::map<std::string, std::string> map_definitions;
    std::map<std::string, std::string> map_names; // Maps map_id to user friendly name
    std::map<std::string, AircraftHistoryEntry> aircraft_history;
    int scan_timeout_sec;               // Timeout for Bluetooth scanning in seconds - not used
    std::string characteristicUUID;                 // Bluetooth data characteristic UUID for button data
    std::string lightBrightnessCharacteristicUUID;  // Bluetooth data characteristic UUID for backlight control
    uint8_t defaultBrightness = 0x00;   // Default backlight intensity: Fully on (level 4) - update from config
    // For rotary encoder acceleration:
    int fastTurnSensitivity1 = 250; // Threshold for brisk turning (ms)
    int fastTurnSensitivity2 = 130; // Threshold for fast turning (ms)
    int fastTurnIncrements1 = 2;    // Steps for brisk turning
    int fastTurnIncrements2 = 5;    // Steps for fast turning
    int newTurnPause = 500;         // Pause in ms, after which a turn is considered new
    
    // For orbit feature
    double orbitRadius = 5.0;            // NM distance to trigger orbit
    double orbitHeadingCorrectionFar = 60.0; // Degrees to add to bearing when dist > orbitRadius
    double orbitHeadingCorrectionNear = 85.0; // Degrees to add to bearing when dist <= orbitRadius

    bool refuel_reminder = false;            // Feature flag for the GGGD audio reminder

    int longPressDelayMs = 2000;            // Wait time in milliseconds to trigger a configure long-press command
};
static GlobalSettings g_settings;
static std::mutex g_consoleMutex;

// Globals for interactive aircraft map prompt
static std::mutex g_pending_aircraft_mutex;
static std::string g_pending_new_aircraft;
static std::string g_suggested_map_id;
static std::atomic<bool> g_interactive_prompt_active{false};
static std::string g_current_aircraft_title;

// Structure for the commands: Key (e.g. "PFD_C0") -> Command
using CommandMap = std::unordered_map<std::string, std::string>;
CommandMap g_commandMaps;

// Globals to track dynamic long presses
static std::atomic<bool> generic_long_press_active{false};
static std::atomic<int> generic_long_press_token{0};
static std::atomic<uint8_t> generic_long_press_byte{0}; // Track which byte is being held
static std::string generic_short_press_cmd; // To execute if released early
static std::string generic_role; // To use for logging/executing

// Stores the active Bluetooth connections to send commands (such as lights)
static std::map<std::string, SimpleBLE::Peripheral> g_activePeripherals;

static WASMIF* wasmPtr = nullptr;

// Buffer for WASM commands for the Sim
static std::array<char, MAX_CALC_CODE_SIZE> ccode{};

nlohmann::json g_master_config_json; // Global holder so we can rewrite aircraft updates without wiping comments
void save_master_config(); // Forward declaration
void save_known_devices(); // Forward declaration
bool load_command_map(const std::string& filename); // Forward declaration
void update_last_session_snapshot(); // Forward declaration
static void play_toggle_beep(bool is_swapped, int count = 1, bool is_long = false); // Forward declaration
static void play_gggd_sequence(); // Forward declaration

// --- NATIVE SIMCONNECT FOR A-VARS ---
#include <SimConnect.h>
HANDLE g_hSimConnectNative = NULL;
enum DATA_DEFINE_ID { DEFINITION_AIRCRAFT_TITLE = 1, DEFINITION_ORBIT_DATA = 2 };
enum DATA_REQUEST_ID { REQUEST_AIRCRAFT_TITLE = 1, REQUEST_ORBIT_DATA = 2 };
enum EVENT_ID { EVENT_SIM_RATE_INCR = 1, EVENT_SIM_RATE_DECR = 2 };

struct AircraftData {
    char title[256];
};

#pragma pack(push, 1) // Ensure no padding is added so SimConnect bytes match exactly
struct OrbitData {
    int32_t autopilot_master;
    int32_t autopilot_hdg_lock;
    double gps_wp_distance;
    double gps_wp_bearing;
    int32_t sim_on_ground;
    double flaps_left_percent;
    double radio_altitude;
    double velocity_body_z;
    double vertical_speed;
    int32_t simulation_rate;
};
#pragma pack(pop)

// Global storage for the latest fetched orbit data
static std::mutex g_orbitDataMutex;
static OrbitData g_latestOrbitData = {0, 0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0};
static std::atomic<bool> g_orbitDataReady{false};

// --- SIM RATE CONTROLLER LOGIC ---
struct SimRateControlConfig {
    bool enabled = true;
    char activation_hotkey = 'x';
    int sampling_interval_ms = 100;
    size_t window_size = 15;
    double deadband_threshold = 1.5;
    double trend_damping_ratio = 0.7;
    int cooldown_downshift_ms = 2000;
    int cooldown_rearm_upshift_ms = 20000;
    int cooldown_evaluation_delay_ms = 3500;
    bool console_telemetry_logging = true;
};

class SimRateController {
private:
    SimRateControlConfig config;
    std::vector<double> rolling_array;
    size_t array_index = 0;
    std::atomic<bool> system_active{false}; 
    
    enum State { STABLE, UNSTABLE };
    State current_state = STABLE;
    
    double last_peak_amplitude = 0.0;
    int current_sim_rate = 1; 
    
    using time_point = std::chrono::steady_clock::time_point;
    time_point last_downshift_time;
    time_point last_unstable_time;
    time_point last_upshift_time;
    bool in_downshift_cooldown = false;
    
    // Latency locks
    bool waiting_for_upshift_ack = false;
    int expected_sim_rate_after_upshift = 1;

    double getPeakAmplitude() const {
        double peak = 0.0;
        for (double val : rolling_array) {
            peak = std::max(peak, std::abs(val));
        }
        return peak;
    }

    bool isBouncing() const {
        bool has_positive = false;
        bool has_negative = false;
        for (double val : rolling_array) {
            if (val > 0.0) has_positive = true;
            if (val < 0.0) has_negative = true;
        }
        return has_positive && has_negative;
    }

    void downshiftSimRate() {
        if (current_sim_rate > 1) { // Block downshift if rate is already 1x or below (0x)
            last_peak_amplitude = getPeakAmplitude();
            last_downshift_time = std::chrono::steady_clock::now();
            in_downshift_cooldown = true;
            
            if (g_hSimConnectNative) {
                SimConnect_TransmitClientEvent(g_hSimConnectNative, SIMCONNECT_OBJECT_ID_USER, EVENT_SIM_RATE_DECR, 0, SIMCONNECT_GROUP_PRIORITY_HIGHEST, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
            }
        }
    }

    void upshiftSimRate() {
        if (waiting_for_upshift_ack) {
            return; // Hard lock: block entirely until previous request resolves
        }
        
        if (current_sim_rate < 16) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed_upshift_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_upshift_time).count();
            
            // Double fail-safe gate to absolutely guarantee no multi-fires
            if (elapsed_upshift_ms >= config.cooldown_rearm_upshift_ms) {
                last_upshift_time = now; // Instantly lock out the next 20 seconds
                waiting_for_upshift_ack = true; // Lock out further attempts until the sim responds
                expected_sim_rate_after_upshift = current_sim_rate * 2;
                
                if (g_hSimConnectNative) {
                    SimConnect_TransmitClientEvent(g_hSimConnectNative, SIMCONNECT_OBJECT_ID_USER, EVENT_SIM_RATE_INCR, 0, SIMCONNECT_GROUP_PRIORITY_HIGHEST, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
                }
            }
        }
    }

public:
    SimRateController(const SimRateControlConfig& cfg) : config(cfg) {
        if (config.window_size % 2 == 0) {
            config.window_size += 1; 
        }
        rolling_array.resize(config.window_size, 0.0);
    }

    void deactivate() {
        if (!system_active) return;
        system_active = false;
        std::fill(rolling_array.begin(), rolling_array.end(), 0.0);
        current_state = STABLE;
        in_downshift_cooldown = false;

        std::thread([this]() {
            int attempt = 0;
            while (!system_active && attempt < 20) {
                if (current_sim_rate == 1) {
                    break;
                }
                if (g_hSimConnectNative) {
                    if (current_sim_rate > 1) {
                        SimConnect_TransmitClientEvent(g_hSimConnectNative, SIMCONNECT_OBJECT_ID_USER, EVENT_SIM_RATE_DECR, 0, SIMCONNECT_GROUP_PRIORITY_HIGHEST, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
                    } else if (current_sim_rate < 1) {
                        SimConnect_TransmitClientEvent(g_hSimConnectNative, SIMCONNECT_OBJECT_ID_USER, EVENT_SIM_RATE_INCR, 0, SIMCONNECT_GROUP_PRIORITY_HIGHEST, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                attempt++;
            }
        }).detach();
    }

    void toggleActive() {
        if (system_active) {
             deactivate();
        } else {
             system_active = true;
             // Reset timers to strictly enforce initial waits when (re)activated
             auto now = std::chrono::steady_clock::now();
             last_unstable_time = now;
             last_upshift_time = now;
        }
    }

    bool isActive() const { return system_active; }
    SimRateControlConfig& getConfig() { return config; }

    void processTelemetry(double v_speed_fps, int sim_rate) {
        current_sim_rate = sim_rate;
        auto now = std::chrono::steady_clock::now();
        
        // Release the network latency lock if the SimRate has caught up
        if (waiting_for_upshift_ack && current_sim_rate >= expected_sim_rate_after_upshift) {
            waiting_for_upshift_ack = false;
        }

        double filtered_y = 0.0;
        if (std::abs(v_speed_fps) >= config.deadband_threshold) {
            filtered_y = v_speed_fps;
        }

        rolling_array[array_index] = filtered_y;
        array_index = (array_index + 1) % config.window_size;

        if (!system_active) return; // Keep array primed, but skip actions
        if (config.enabled == false) return; // Feature globally disabled

        bool bouncing = isBouncing();
        double current_peak = getPeakAmplitude();
        bool is_quiet = (current_peak == 0.0);
        
        // Strict stability: Must have no sign changes AND be within deadband (all 0.0)
        State new_state = is_quiet ? STABLE : UNSTABLE;
        current_state = new_state;

        // CRITICAL: Always keep tracking instability even during bypasses so the 
        // 20-second cooldown timer starts perfectly from the last unquiet moment!
        if (current_state == UNSTABLE) {
            last_unstable_time = now;
        }

        // CRITICAL: Always keep tracking instability even during bypasses so the 
        // 20-second cooldown timer starts perfectly from the last unquiet moment!
        if (new_state == UNSTABLE) {
            last_unstable_time = now;
        }

        std::string action = "NONE";
        
        auto elapsed_since_last_upshift = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_upshift_time).count();
        if (waiting_for_upshift_ack) {
            action = "AWAITING SIM ACK";
        }
        else if (elapsed_since_last_upshift < config.cooldown_evaluation_delay_ms) {
            // Suspend telemetry evaluations completely to let physics settle
            // Bypasses all downshift and upshift evaluations strictly
            action = "SETTLING EVAL DELAY";
        }
        else if (in_downshift_cooldown) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_downshift_time).count();
            if (elapsed_ms >= config.cooldown_downshift_ms) {
                in_downshift_cooldown = false;
                
                double new_peak = getPeakAmplitude();
                if (new_peak >= (last_peak_amplitude * config.trend_damping_ratio)) {
                    if (current_sim_rate > 1) { // Apply floor safety here too
                        downshiftSimRate();
                        action = "DOWNSHIFT (SUSTAINED)";
                    } else {
                        action = "FLOOR (AT 1X)";
                    }
                } else {
                    action = "HOLD (DECAYING)";
                }
            } else {
                action = "COOLDOWN";
            }
        } 
        else {
            if (new_state == UNSTABLE && current_sim_rate > 1) {
                // If the instability is purely sustained/constant extreme vertical speed but not bouncing,
                // we might want to still step down or treat it as unstable
                downshiftSimRate();
                action = "INITIAL DOWNSHIFT";
            } 
            else if (new_state == UNSTABLE && current_sim_rate <= 1) {
                // Instability detected but we are already at the floor
                action = "FLOOR (AT 1X)";
            }
            else if (new_state == STABLE) {
                auto stable_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_unstable_time).count();
                // Instead of relying purely on current_state == STABLE which can flip in 100ms, use the strict time-differential since last upshift.
                if (stable_duration_ms >= config.cooldown_rearm_upshift_ms &&
                    elapsed_since_last_upshift >= config.cooldown_rearm_upshift_ms) {
                    
                    if (current_sim_rate < 16) {
                        upshiftSimRate();
                        action = "UPSHIFT";
                    }
                }
            }
        }
        
        current_state = new_state;

        if (config.console_telemetry_logging) {
            std::lock_guard<std::mutex> lock(g_consoleMutex);
            std::string state_str_display = (current_state == UNSTABLE) ? (bouncing ? "BOUNCING " : "UNSTABLE ") : "STABLE   ";
            std::cout << "[" << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() << "] "
                      << "| SimRate: " << current_sim_rate << "x "
                      << "| Raw_Y: " << std::fixed << std::setprecision(2) << std::showpos << v_speed_fps << std::noshowpos
                      << " | Filtered_Y: " << std::showpos << filtered_y << std::noshowpos
                      << " | Array_State: " << state_str_display
                      << " | Peak_Amp: " << current_peak
                      << " | Action: " << action 
                      << "\r" << std::flush;
        }
    }
};

static SimRateControlConfig g_simRateConfig;
static SimRateController* g_simRateController = nullptr;
// ---------------------------------

static std::vector<std::string> tokenize_string(const std::string& str) {
    std::vector<std::string> tokens;
    std::string current_token;
    for (char c : str) {
        if (std::isalnum(static_cast<unsigned char>(c))) {
            current_token += std::tolower(static_cast<unsigned char>(c));
        } else if (!current_token.empty()) {
            tokens.push_back(current_token);
            current_token.clear();
        }
    }
    if (!current_token.empty()) {
        tokens.push_back(current_token);
    }
    return tokens;
}

static int calculate_similarity_score(const std::string& str1, const std::string& str2) {
    auto tokens1 = tokenize_string(str1);
    auto tokens2 = tokenize_string(str2);
    
    std::unordered_set<std::string> set1(tokens1.begin(), tokens1.end());
    int score = 0;
    for (const auto& t : tokens2) {
        if (set1.count(t)) {
            score++;
        }
    }
    return score;
}

static std::string suggest_map_for_title(const std::string& newTitle, std::string& outSimilarAircraft) {
    std::string best_map_id = g_settings.default_map_id;
    int highest_score = 0;
    std::string best_last_used = "";

    outSimilarAircraft = "";

    for (const auto& [title, entry] : g_settings.aircraft_history) {
        int score = calculate_similarity_score(newTitle, title);
        if (score > highest_score || (score == highest_score && score > 0 && entry.last_used > best_last_used)) {
            highest_score = score;
            best_map_id = entry.map_id;
            best_last_used = entry.last_used;
            outSimilarAircraft = title;
        }
    }

    // fallback to default if no useful match
    if (highest_score == 0) {
        outSimilarAircraft = "";
        best_map_id = g_settings.default_map_id;
    }
    return best_map_id;
}

void CALLBACK MyNativeDispatchProc(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext) {
    switch (pData->dwID) {
        case SIMCONNECT_RECV_ID_SIMOBJECT_DATA: {
            SIMCONNECT_RECV_SIMOBJECT_DATA* pObjData = (SIMCONNECT_RECV_SIMOBJECT_DATA*)pData;
            if (pObjData->dwRequestID == REQUEST_AIRCRAFT_TITLE) {
                AircraftData* pAircraftData = (AircraftData*)&pObjData->dwData;
                std::string title = pAircraftData->title;
                g_current_aircraft_title = title;
                std::string map_id = g_settings.default_map_id;

                bool is_new_aircraft = false;
                auto it = g_settings.aircraft_history.find(title);
                if (it != g_settings.aircraft_history.end()) {
                    map_id = it->second.map_id;
                } else {
                    is_new_aircraft = true;
                }

                if (is_new_aircraft) {
                    // Start interactive suggestion process
                    std::string similar_title;
                    std::string suggested_map = suggest_map_for_title(title, similar_title);
                    
                    {
                        std::lock_guard<std::mutex> lock(g_pending_aircraft_mutex);
                        g_pending_new_aircraft = title;
                        g_suggested_map_id = suggested_map;
                    }
                    
                    {
                        std::lock_guard<std::mutex> lock(g_consoleMutex);
                        std::cout << "\n[SIMCONNECT] Newly discovered Aircraft: " << title;
                        std::cout << "\n[SIMCONNECT] Waiting for user input via console...\n";
                    }
                    
                    // Temporarily load suggested map or default so it's not entirely empty
                    auto map_it = g_settings.map_definitions.find(suggested_map);
                    if (map_it != g_settings.map_definitions.end()) {
                        COMMANDS_FILENAME = map_it->second;
                        load_command_map(COMMANDS_FILENAME);
                    }
                } else {
                    // Normal known aircraft update path
                    // Update timestamp
                    auto now = std::chrono::system_clock::now();
                    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
                    std::tm bt{};
                    localtime_s(&bt, &now_c);
                    std::stringstream ss;
                    ss << std::put_time(&bt, "%Y-%m-%dT%H:%M:%S");
                    
                    g_settings.aircraft_history[title] = {map_id, ss.str()};
                    save_master_config(); // Save right away

                    // Match with map filenames
                    auto map_it = g_settings.map_definitions.find(map_id);
                    if (map_it != g_settings.map_definitions.end()) {
                        COMMANDS_FILENAME = map_it->second;
                    }

                    {
                        std::lock_guard<std::mutex> lock(g_consoleMutex);
                        std::cout << "\n[SIMCONNECT] Currently used Aircraft: " << title;
                        std::cout << "\n[SIMCONNECT] Auto loaded Map ID: " << map_id << " (" << COMMANDS_FILENAME << ")\n";
                        std::cout << "PRESS 'M' TO RELOAD MAP | 'R' TO RECONNECT | 'S' TO SWAP ROLES | 'L' TO ASSIGN ROLES | '?' FOR AIRCRAFT | 'Q' TO QUIT" << std::endl;
                    }
                    
                    load_command_map(COMMANDS_FILENAME);
                }
            } else if (pObjData->dwRequestID == REQUEST_ORBIT_DATA) {
                OrbitData* pOrbitData = (OrbitData*)&pObjData->dwData;
                {
                    std::lock_guard<std::mutex> lock(g_orbitDataMutex);
                    g_latestOrbitData = *pOrbitData;
                    g_orbitDataReady = true;

                    // Output debug logs as requested
                    auto t = std::time(nullptr);
                    auto tm = *std::localtime(&t);
                    std::ostringstream oss;
                    oss << std::put_time(&tm, "%H:%M:%S");
                    std::string timestamp = oss.str();

                    bool dist_above_orbit = (pOrbitData->gps_wp_distance > g_settings.orbitRadius);

                    // --- GGGD Alarm Logic ---
                    if (g_settings.refuel_reminder) {
                        // Rule A: Airborne Armed Reset (Anti-Spam Filter)
                        // Rearm the alarms only when at a safe altitude AND the aircraft is clean (flaps retracted)
                        if (pOrbitData->radio_altitude > 500.0 && pOrbitData->flaps_left_percent < 5.0) {
                            g_air_alarm_triggered = false;
                            g_ground_alarm_triggered = false;
                        }

                        // Rule B: In-Air Approach Trigger (Primary Alarm)
                        if (pOrbitData->flaps_left_percent > 95.0 && pOrbitData->sim_on_ground == 0) {
                            if (!g_air_alarm_triggered) {
                                std::thread(play_gggd_sequence).detach();
                                g_air_alarm_triggered = true;
                            }
                        }

                        // Rule C: On-Ground Failsafe Trigger (Secondary Alarm)
                        if (pOrbitData->sim_on_ground == 1 && std::abs(pOrbitData->velocity_body_z) < 0.1 && pOrbitData->flaps_left_percent < 5.0) {
                            if (!g_ground_alarm_triggered && g_air_alarm_triggered) {
                                std::thread(play_gggd_sequence).detach();
                                g_ground_alarm_triggered = true;
                            }
                        }
                    }
                    // ------------------------

                    if (g_print_commands_to_console) {
                        std::cout << "[" << timestamp << "] [ORBIT] AP_MASTER=" << pOrbitData->autopilot_master 
                                  << " | AP_HDG=" << pOrbitData->autopilot_hdg_lock
                                  << " | DIST=" << pOrbitData->gps_wp_distance 
                                  << " | BRG=" << pOrbitData->gps_wp_bearing 
                                  << " | >" << g_settings.orbitRadius << "NM=" << (dist_above_orbit ? "true" : "false") << std::endl;
                    }

                    if (g_orbit_evaluation_pending) {
                        g_orbit_evaluation_pending = false;
                        if (pOrbitData->autopilot_master == 0) {
                            play_toggle_beep(false, 1, true);
                            std::cout << "\n[SYSTEM] Cannot engage Orbit Function: Autopilot is OFF." << std::endl;
                        } else if (pOrbitData->autopilot_hdg_lock == 0) {
                            play_toggle_beep(false, 1, true);
                            std::cout << "\n[SYSTEM] Cannot engage Orbit Function: Autopilot HDG mode is disabled." << std::endl;
                        } else {
                            g_orbit_active = true;
                            play_toggle_beep(g_orbit_active);
                            std::cout << "\n[SYSTEM] Orbit Function successfully engaged! (ON)" << std::endl;
                        }
                    } else if (g_orbit_active) {
                        if (pOrbitData->autopilot_master == 0) {
                            g_orbit_active = false;
                            play_toggle_beep(g_orbit_active, 3);
                            std::cout << "\n[SYSTEM] AFK Orbit Function disengaged: Autopilot was turned OFF." << std::endl;
                        } else if (pOrbitData->autopilot_hdg_lock == 0) {
                            g_orbit_active = false;
                            play_toggle_beep(g_orbit_active, 3);
                            std::cout << "\n[SYSTEM] AFK Orbit Function disengaged: Autopilot HDG mode was disabled." << std::endl;
                        } else {
                            double correction = dist_above_orbit ? g_settings.orbitHeadingCorrectionFar : g_settings.orbitHeadingCorrectionNear;
                            double target_heading = pOrbitData->gps_wp_bearing + correction;
                            target_heading = std::fmod((target_heading + 360.0), 360.0);
                            if (target_heading < 0.0) target_heading += 360.0;
                            
                            int rounded_heading = static_cast<int>(std::round(target_heading));
                            
                            if (g_print_commands_to_console) {
                                std::cout << "[" << timestamp << "] [ORBIT] -> New HDG: " << rounded_heading << " (Correction: +" << correction << "\370)" << std::endl;
                            }

                            if (wasmPtr && wasmPtr->isRunning()) {
                                char code[128];
                                snprintf(code, sizeof(code), "%d (>K:HEADING_BUG_SET)", rounded_heading);
                                wasmPtr->executeCalclatorCode(code);
                            }
                        }
                    }
                }
            }
            break;
        }
        case SIMCONNECT_RECV_ID_QUIT: {
            g_hSimConnectNative = NULL;
            break;
        }
    }
}

class WASMIFGuard {
public:
    explicit WASMIFGuard(WASMIF* p) : p_(p) {}
    ~WASMIFGuard() { if (p_) p_->end(); }
    WASMIFGuard(const WASMIFGuard&) = delete;
    WASMIFGuard& operator=(const WASMIFGuard&) = delete;
private:
    WASMIF* p_;
};


// Forward declarations for various functions
static void interactive_map_selection(const std::string& aircraft_title);
// for the packet handler
// Shared connected-device record
struct ConnectedDevice {
    SimpleBLE::Peripheral p;              // The Bluetooth device
    SimpleBLE::BluetoothUUID service_uuid; // The service (general)
    SimpleBLE::BluetoothUUID char_uuid;    // The characteristic (for light & buttons)
    bool used_indicate;
};

// Runtime context for shutdown across normal path and console-close handler
struct BleRuntime {
    std::mutex m;
    std::atomic_bool shuttingDown{false};
    std::optional<SimpleBLE::Adapter> adapter;
    std::unordered_map<std::string, ConnectedDevice> connected; // mac -> device
};
static BleRuntime g_ble;

// Configuration for Combo 
const std::chrono::milliseconds COMBO_WINDOW(250);
std::atomic<uint8_t> pending_first_key(0x00); 
std::chrono::steady_clock::time_point first_key_time;
std::atomic<bool> combo_active_block(false);

// Beep routine extracted to allow passing the toggle state
static void play_toggle_beep(bool is_swapped, int count, bool is_long) {
    int duration = is_long ? 900 : 150;
    int frequency = is_swapped ? 800 : 1600;

    for (int i = 0; i < count; ++i) {
        Beep(frequency, duration);
        if (i < count - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

// GGGD Alarm Audio Sequence
static void play_gggd_sequence() {
    auto play_morse_G = [](int freq) {
        Beep(freq, 350); // Dash
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        Beep(freq, 350); // Dash
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        Beep(freq, 120); // Dot
    };

    auto play_morse_D_modified = [](int freq) {
        Beep(freq, 580); // X-Long Dash
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        Beep(freq, 120); // Dot
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        Beep(freq, 120); // Dot
    };

    // Block 1: "Go"
    play_morse_G(784); // G5
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    // Block 2: "Get"
    play_morse_G(1175); // D6
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    // Block 3: "Gas"
    play_morse_G(1047); // C6
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    // Block 4: "Dude"
    play_morse_D_modified(1047); // C6
}

// Role Switch Helper
static void perform_role_switch(bool play_beep = true) {
    static bool g_role_swapped_state = false;
    g_role_swapped_state = !g_role_swapped_state;

    {
        // We briefly lock the BLE object so that on_packet does not read at the same time
        std::lock_guard<std::mutex> lock(g_ble.m);
        
        // Step 1: Mark intention for ALL active devices (even if temporarly disconnected)
        for (auto& dev : g_knownDevices) {
            if (dev.instrument == "PFD") {
                dev.instrument = "TEMP_MFD";
            } else if (dev.instrument == "MFD") {
                dev.instrument = "TEMP_PFD";
            }
        }

        // Step 2: "Washing" - Finalize all temporary labels
        for (auto& dev : g_knownDevices) {
            if (dev.instrument == "TEMP_MFD") {
                dev.instrument = "MFD";
                std::cout << "[SWITCH] " << dev.ble_name << " [" << dev.address << "] is now MFD" << std::endl;
            } else if (dev.instrument == "TEMP_PFD") {
                dev.instrument = "PFD";
                std::cout << "[SWITCH] " << dev.ble_name << " [" << dev.address << "] is now PFD" << std::endl;
            }
        }
    }

    // Save the new roles explicitly so it's not lost on restart
    save_known_devices();


    // Acoustic feedback & log
    {
        std::lock_guard<std::mutex> lock(g_consoleMutex);
        if (play_beep) {
            play_toggle_beep(g_role_swapped_state);
        }
        std::cout << "[SYSTEM] Role switch performed.\n" << std::endl;
        update_last_session_snapshot(); // Synchronize snapshot for reconnect
    }
}

void on_packet(const SimpleBLE::ByteArray& data, const std::string& address);
// for the fast reconnect function
void trigger_fast_reconnect();
// for brightness control
static void send_brightness_to_all(uint8_t level);


// Helper function: Simulates keyboard inputs in the console for pre-assignment
void inject_default_input(const std::string& text) {
    HANDLE hInput = GetStdHandle(STD_INPUT_HANDLE);
    if (hInput == INVALID_HANDLE_VALUE) return;

    for (char c : text) {
        INPUT_RECORD ir[2]; // We send two events: Key down & Key up

        // 1. Press key (Key Down)
        ir[0].EventType = KEY_EVENT;
        ir[0].Event.KeyEvent.bKeyDown = TRUE;
        ir[0].Event.KeyEvent.wRepeatCount = 1;
        ir[0].Event.KeyEvent.wVirtualKeyCode = 0;
        ir[0].Event.KeyEvent.wVirtualScanCode = 0;
        ir[0].Event.KeyEvent.uChar.AsciiChar = c;
        ir[0].Event.KeyEvent.dwControlKeyState = 0;

        // 2. Release key (Key Up)
        ir[1] = ir[0];
        ir[1].Event.KeyEvent.bKeyDown = FALSE;

        DWORD written;
        // We write both events (2) into the buffer
        WriteConsoleInputA(hInput, ir, 2, &written);
    }
}

// Helper function: Trims whitespace
static std::string trim(std::string s) {
    auto isspace2 = [](unsigned char ch){ return std::isspace(ch) != 0; };
    size_t start = 0;
    while (start < s.size() && isspace2(static_cast<unsigned char>(s[start]))) ++start;
    size_t end = s.size();
    while (end > start && isspace2(static_cast<unsigned char>(s[end - 1]))) --end;
    return s.substr(start, end - start);
}

// Helper function: Converts a string to lowercase
static std::string to_lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
}

// Rewrites aircraft histories while preserving the rest of the generic JSON config
void save_master_config() {
    if (g_master_config_json.is_null()) return;

    for (const auto& [title, entry] : g_settings.aircraft_history) {
        g_master_config_json["aircraft_history"][title]["map_id"] = entry.map_id;
        g_master_config_json["aircraft_history"][title]["last_used"] = entry.last_used;
    }

    // Ensure SimRateControl block is populated in the JSON output, particularly helpful on first run
    nlohmann::json s_json;
    s_json["enabled"] = g_simRateConfig.enabled;
    s_json["activation_hotkey"] = std::string(1, g_simRateConfig.activation_hotkey);
    s_json["sampling_interval_ms"] = g_simRateConfig.sampling_interval_ms;
    s_json["window_size"] = g_simRateConfig.window_size;
    s_json["deadband_threshold"] = g_simRateConfig.deadband_threshold;
    s_json["trend_damping_ratio"] = g_simRateConfig.trend_damping_ratio;
    s_json["cooldown_downshift_ms"] = g_simRateConfig.cooldown_downshift_ms;
    s_json["cooldown_rearm_upshift_ms"] = g_simRateConfig.cooldown_rearm_upshift_ms;
    s_json["cooldown_evaluation_delay_ms"] = g_simRateConfig.cooldown_evaluation_delay_ms;
    s_json["console_telemetry_logging"] = g_simRateConfig.console_telemetry_logging;
    g_master_config_json["SimRateControl"] = s_json;

    std::ofstream file(CONFIG_FILENAME);
    if (file.is_open()) {
        file << g_master_config_json.dump(2);
    }
}

// Loads the Master Config with general settings (e.g. file paths etc.)
bool load_master_config(const std::string& filename) {
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "[ERROR] Master config not found: " << filename << std::endl;
            return false;
        }

        nlohmann::json j;
        file >> j;
        g_master_config_json = j;

        // Apply file paths from 'general' section
        if (j.contains("general")) {
            std::string temp_device = j["general"].value("device_map_file", "");
            if (!temp_device.empty()) {
                g_settings.device_map_file = temp_device;
                KNOWN_DEVICES_FILENAME = temp_device; // Update global default
            }

            g_settings.default_map_id = j["general"].value("default_map_id", "std_g1000");
            g_settings.refuel_reminder = j["general"].value("refuel_reminder", false);
        }

        // Apply SimRateControl from config
        if (j.contains("SimRateControl")) {
            auto& s_json = j["SimRateControl"];
            g_simRateConfig.enabled = s_json.value("enabled", true);
            
            std::string hotkey_str = s_json.value("activation_hotkey", "x");
            if (!hotkey_str.empty()) g_simRateConfig.activation_hotkey = std::tolower(hotkey_str[0]);
            
            g_simRateConfig.sampling_interval_ms = s_json.value("sampling_interval_ms", 100);
            g_simRateConfig.window_size = s_json.value("window_size", 15);
            g_simRateConfig.deadband_threshold = s_json.value("deadband_threshold", 1.5);
            g_simRateConfig.trend_damping_ratio = s_json.value("trend_damping_ratio", 0.7);
            g_simRateConfig.cooldown_downshift_ms = s_json.value("cooldown_downshift_ms", 2000);
            g_simRateConfig.cooldown_rearm_upshift_ms = s_json.value("cooldown_rearm_upshift_ms", 20000);
            g_simRateConfig.cooldown_evaluation_delay_ms = s_json.value("cooldown_evaluation_delay_ms", 2000);
            g_simRateConfig.console_telemetry_logging = s_json.value("console_telemetry_logging", true);
        } else {
            // Write defaults to global JSON so it gets persisted next time save_master_config is called
            nlohmann::json s_json;
            s_json["enabled"] = g_simRateConfig.enabled;
            s_json["activation_hotkey"] = std::string(1, g_simRateConfig.activation_hotkey);
            s_json["sampling_interval_ms"] = g_simRateConfig.sampling_interval_ms;
            s_json["window_size"] = g_simRateConfig.window_size;
            s_json["deadband_threshold"] = g_simRateConfig.deadband_threshold;
            s_json["trend_damping_ratio"] = g_simRateConfig.trend_damping_ratio;
            s_json["cooldown_downshift_ms"] = g_simRateConfig.cooldown_downshift_ms;
            s_json["cooldown_rearm_upshift_ms"] = g_simRateConfig.cooldown_rearm_upshift_ms;
            s_json["cooldown_evaluation_delay_ms"] = g_simRateConfig.cooldown_evaluation_delay_ms;
            s_json["console_telemetry_logging"] = g_simRateConfig.console_telemetry_logging;
            
            g_master_config_json["SimRateControl"] = s_json;
            // Optionally force an immediate save here: save_master_config(); 
        }
        
        // Apply maps dictionary
        g_settings.map_definitions.clear();
        g_settings.map_names.clear();
        if (j.contains("maps") && j["maps"].is_array()) {
            for (const auto& item : j["maps"]) {
                std::string id = item.value("id", "");
                std::string filepath = item.value("file", "");
                std::string mapName = item.value("name", id);
                if (!id.empty() && !filepath.empty()) {
                    g_settings.map_definitions[id] = filepath;
                    g_settings.map_names[id] = mapName;
                }
            }
        }
        
        // Apply aircraft history
        g_settings.aircraft_history.clear();
        if (j.contains("aircraft_history")) {
            for (auto& el : j["aircraft_history"].items()) {
                g_settings.aircraft_history[el.key()] = {
                    el.value().value("map_id", g_settings.default_map_id),
                    el.value().value("last_used", "")
                };
            }
        }

        // Apply UUIDs from the 'SHB1000' section
        if (j.contains("devices") && j["devices"].contains("SHB1000")) {
            auto& shb = j["devices"]["SHB1000"];
            g_settings.characteristicUUID = shb.value("characteristicUUID", "");
            g_settings.lightBrightnessCharacteristicUUID = shb.value("lightBrightnessCharacteristicUUID", "");
            g_settings.defaultBrightness = shb.value("defaultBrightness", 4);
            
            // Load values for brisk / fast turning (if present in config, otherwise fallback)
            g_settings.fastTurnSensitivity1 = shb.value("fastTurnSensitivity1", g_settings.fastTurnSensitivity1);
            g_settings.fastTurnSensitivity2 = shb.value("fastTurnSensitivity2", g_settings.fastTurnSensitivity2);
            g_settings.fastTurnIncrements1 = shb.value("fastTurnIncrements1", g_settings.fastTurnIncrements1);
            g_settings.fastTurnIncrements2 = shb.value("fastTurnIncrements2", g_settings.fastTurnIncrements2);
            g_settings.newTurnPause = shb.value("newTurnPause", g_settings.newTurnPause);
            
            g_settings.orbitRadius = shb.value("orbitRadius", g_settings.orbitRadius);
            g_settings.orbitHeadingCorrectionFar = shb.value("orbitHeadingCorrectionFar", g_settings.orbitHeadingCorrectionFar);
            g_settings.orbitHeadingCorrectionNear = shb.value("orbitHeadingCorrectionNear", g_settings.orbitHeadingCorrectionNear);
            g_settings.longPressDelayMs = shb.value("longPressDelayMs", g_settings.longPressDelayMs);
        }

        std::cout << "[OK] Master config loaded."<< std::endl;
        std::cout << "[INFO] Device config: " << g_settings.device_map_file << std::endl;
        return true;

    } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "[CRITICAL] JSON parse error in " << filename << ": " << e.what() << std::endl;
        return false;
    }
}

// Convert input to canonical MAC "aa:bb:cc:dd:ee:ff" (lowercase).
// Accepts formats like "AA:BB:CC:DD:EE:FF", "aa-bb-cc-dd-ee-ff", or "AABBCCDDEEFF".
static void canonicalize_mac(const std::string& raw, std::string& out) {
    out = to_lower(raw);
    out.erase(std::remove(out.begin(), out.end(), ':'), out.end());
    out.erase(std::remove(out.begin(), out.end(), '-'), out.end());
    out = trim(out);
}

// Loads the known devices from the JSON file to facilitate connection and role assignment
bool load_known_devices(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;

    try {
        json j;
        file >> j;

        g_knownDevices.clear();
        for (const auto& item : j["devices"]) {
            HardwareDevice dev;

            // Function for safe reading of strings
            auto safeString = [&](const std::string& key, const std::string& fallback) {
                return (item.contains(key) && !item[key].is_null()) ? item[key].get<std::string>() : fallback;
            };

            dev.instrument     = safeString("instrument", "UNKNOWN");
            dev.type           = safeString("type", "UNKNOWN");
            dev.position       = safeString("position", "none");
            dev.ble_name       = safeString("ble_name", "Unknown");
            dev.address        = safeString("address", "");
            dev.last_connected = safeString("last_connected", "never");
            dev.auto_reconnect = item.value("auto_reconnect", false);

            g_knownDevices.push_back(dev);
        }
    } catch (json::parse_error& e) {
        std::cerr << "JSON Error: " << e.what() << std::endl;
        return false;
    }
    return true;
}

// Loads the command mapping from the config file to translate button/dial codes to sim commands
bool load_command_map(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Error: Could not open " << filename << "!" << std::endl;
        return false;
    }

    g_commandMaps.clear();

    while (std::getline(file, line)) {
        // Ignore comments and empty lines
        if (line.empty() || line[0] == '#' || line[0] == ';' || line[0] == '[') continue; // Also ignore header lines in the new format if any remain

        // Parse command lines (Format: PFD_4A = "Command")
        size_t delimiter = line.find('=');
        if (delimiter != std::string::npos) {
            std::string keyStr = line.substr(0, delimiter);
            std::string command = line.substr(delimiter + 1);

            // 1. Helper function for trimming (remove spaces front/back)
            auto trim = [](std::string& s) {
                size_t first = s.find_first_not_of(" \t\r\n");
                if (first == std::string::npos) { s = ""; return; }
                size_t last = s.find_last_not_of(" \t\r\n");
                s = s.substr(first, (last - first + 1));
            };

            trim(keyStr);
            trim(command);

            // Convert key to upper case for case-insensitive lookup
            std::transform(keyStr.begin(), keyStr.end(), keyStr.begin(),
                [](unsigned char c){ return std::toupper(c); });

            // 2. Remove quotes, if present
            if (command.size() >= 2 && command.front() == '"' && command.back() == '"') {
                command = command.substr(1, command.size() - 2);
                trim(command); // In case there were still spaces inside the quotes
            }
            
            // 3. Save into the map
            // We use the full string as key, e.g. "PFD_C0" or "PFD_C0_L"
            if (!keyStr.empty() && !command.empty()) {
                g_commandMaps[keyStr] = command;
            }
        }
    }
    std::cout << "[OK] Command mapping successfully loaded. Entries: " << g_commandMaps.size() << std::endl;
    return true;
}

// Saves the current known devices (with their roles etc.) into the JSON file
void save_known_devices() {
    // 1. Get and format current time (YYYY-MM-DDTHH:MM:SS)
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm bt{};
    localtime_s(&bt, &now_c); // Uses the local time of your PC

    std::stringstream ss;
    ss << std::put_time(&bt, "%Y-%m-%dT%H:%M:%S");
    std::string timestamp = ss.str();

    // 2. Build JSON structure
    nlohmann::json j;
    j["devices"] = nlohmann::json::array();

    for (const auto& dev : g_knownDevices) {
        j["devices"].push_back({
            {"instrument", dev.instrument},
            {"type", dev.type},
            {"position", dev.position},
            {"name", dev.ble_name},
            {"address", dev.address},
            {"last_connected", dev.last_connected},
            {"auto_reconnect", dev.auto_reconnect}
        });
    }

    std::ofstream file(g_settings.device_map_file);
    if (file.is_open()) {
        file << j.dump(2); // Save with 2 spaces indentation
        std::lock_guard<std::mutex> lock(g_consoleMutex);
        std::cout << "[OK] Current device configuration file (" << g_settings.device_map_file << ") updated." << std::endl;
    }
}

// Function for a snapshot of the connected devices at the end of the discovery phase
void update_last_session_snapshot() {
    lastSessionDevices.clear();
    std::cout << "[INFO] Creating a snapshot for fast reconnect:" << std::endl;

    for (auto const& [key, peripheral] : g_activePeripherals) {
        auto& p = const_cast<SimpleBLE::Peripheral&>(peripheral); 
        std::string currentMac = p.address();
        
        // We search for the actual role in the g_knownDevices list,
        std::string realRole = "Unknown";
        std::string cleanMac;
        canonicalize_mac(currentMac, cleanMac);

        for (const auto& dev : g_knownDevices) {
            if (dev.address == cleanMac) {
                realRole = dev.instrument; // Here we get "PFD", "MFD" etc.
                break;
            }
        }

        lastSessionDevices[realRole] = cleanMac;
        std::cout << "  -> Role: " << realRole << " | MAC: " << cleanMac << std::endl;
    }

}

// Helper function: Sends commands to the Sim safely with a mutex lock
// in case Bluetooth data arrives faster than SimConnect can process it
static void send_calc_code_safely(const std::string& code) {
    if (!wasmPtr || code.empty() || !wasmPtr->isRunning()) return;
    
    // Use a mutex lock in case Bluetooth data arrives faster than SimConnect processes it
    static std::mutex sendMutex;
    std::lock_guard<std::mutex> lock(sendMutex);

    strncpy_s(ccode.data(), ccode.size(), code.c_str(), _TRUNCATE);
    wasmPtr->executeCalclatorCode(ccode.data());
}

// Handler for incoming Bluetooth packets from the devices
void on_packet(const SimpleBLE::ByteArray& bytes, const std::string& devTag) {
    // 1. Normalize MAC
    std::string cleanMac;
    canonicalize_mac(devTag, cleanMac);

    // Deactivate eXcellerate on any input
    if (g_simRateController && g_simRateController->isActive()) {
        g_simRateController->deactivate();
        std::cout << "\n[SYSTEM] SimRate Auto-Correction deactivated by device input.\n";
    }

    // 1. Find out which ROLE this MAC currently has
    std::string role = "";
    for (const auto& dev : g_knownDevices) {
        if (dev.address == cleanMac) {
            role = dev.instrument; // e.g., "PFD"
            break;
        }
    }
    if (role.empty()) return;

    uint8_t rawByte = (uint8_t)bytes[0];

    // --- Generic Long Press Evaluation (Release check) ---
    // Evaluated unconditionally before mapping checks since release bytes might not be mapped
    if (generic_long_press_active.load() && rawByte == (uint8_t)(generic_long_press_byte.load() + 1)) {
        generic_long_press_active.store(false);
        generic_long_press_token++; // Invalidate timer thread
        if (!generic_short_press_cmd.empty()) {
            if (g_print_commands_to_console) {
                std::cout << "[COMMAND] " << generic_role << " mapped byte 0x" << std::hex << std::uppercase << (int)generic_long_press_byte.load() << std::nouppercase << std::dec << " (short press) to: " << generic_short_press_cmd << "\n";
            }
            send_calc_code_safely(generic_short_press_cmd);
        }
    }

    std::stringstream key_stream;
    key_stream << role << "_" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)rawByte;
    std::string shortKey = key_stream.str();
    std::string longKey = shortKey + "_L";
    
    if (g_print_commands_to_console) {
        std::cout << "[DEBUG] Looking for key: '" << shortKey << "'" << std::endl;
    }

    std::string command = "";
    bool has_short_command = false;
    auto itShort = g_commandMaps.find(shortKey);
    if (itShort != g_commandMaps.end()) {
        command = itShort->second;
        has_short_command = true;
    }

    std::string longCommand = "";
    bool has_long_command = false;
    auto itLong = g_commandMaps.find(longKey);
    if (itLong != g_commandMaps.end()) {
        longCommand = itLong->second;
        has_long_command = true;
    }

    if (has_short_command || has_long_command) {
            
            // Speedy dial turn functionality ---
            // Generate key to identify exactly this button of this device
            std::string inputKey = role + "_" + std::to_string(rawByte);
            auto now = std::chrono::steady_clock::now();
            
            auto& history = last_input_times[inputKey];
            
            // If the last input is older than config value, we consider it a new turn movement and clear the history (smoothing reset).
            if (!history.empty()) {
                auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(now - history.back()).count();
                if (time_since_last > g_settings.newTurnPause) {
                    history.clear();
                }
            }
            
            history.push_back(now);
            
            // Keep a maximum of the last 5 timestamps in the buffer
            if (history.size() > 5) {
                history.erase(history.begin());
            }
            
            int repetitions = 1; // Count as 1 step by default
            
            // Only with 2 or more stored timestamps can we calculate an average difference
            if (history.size() >= 2) {
                auto total_diff = std::chrono::duration_cast<std::chrono::milliseconds>(history.back() - history.front()).count();
                auto avg_diff = total_diff / (history.size() - 1);
                
                // Two-stage threshold for turning
                if (avg_diff > 0 && avg_diff <= g_settings.fastTurnSensitivity2) {
                    repetitions = g_settings.fastTurnIncrements2; // Steps for fast turning
                } else if (avg_diff > 0 && avg_diff <= g_settings.fastTurnSensitivity1) {
                    repetitions = g_settings.fastTurnIncrements1; // Steps for brisk turning
                }

                // DEBUG: Output of the average time difference
//                std::cout << "[DEBUG] " << inputKey << " Average time difference (" << history.size() << " points): " << avg_diff << " ms  -> " << repetitions << " repetitions" << std::endl;

            }

            // COMBO LOGIC for 0x5E (FLC) and 0x5A (VS)
            if (rawByte == 0x5E || rawByte == 0x5A) {
                uint8_t pending = pending_first_key.load();
                if (pending == 0) {
                    // No key waiting
                    pending_first_key.store(rawByte);
                    first_key_time = std::chrono::steady_clock::now();
                    
                    // Launch timeout worker
                    std::thread([rawByte, command, repetitions, role]() {
                        std::this_thread::sleep_for(COMBO_WINDOW);
                        if (pending_first_key.load() == rawByte) {
                            // Timeout expired
                            pending_first_key.store(0x00);
                            
                            // Send delayed command
                            if (g_print_commands_to_console) {
                                auto now_sys = std::chrono::system_clock::now();
                                auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_sys.time_since_epoch()) % 1000;
                                auto now_time_t = std::chrono::system_clock::to_time_t(now_sys);
                                std::tm now_tm;
                                localtime_s(&now_tm, &now_time_t);

                                std::cout << "[" << std::put_time(&now_tm, "%H:%M:%S") << "." << std::setfill('0') << std::setw(3) << now_ms.count() << "] "
                                          << "[COMMAND] " << role << " mapped byte 0x" << std::hex << std::uppercase << (int)rawByte << std::nouppercase << std::dec << " to: " << command;
                                if (repetitions > 1) {
                                    std::cout << " (x" << repetitions << ")";
                                }
                                std::cout << "\n";
                            }
                            for(int i = 0; i < repetitions; ++i) {
                                send_calc_code_safely(command);
                            }
                        }
                    }).detach();
                    return; // Halt execution
                } else {
                    // A key is already waiting
                    if ((pending == 0x5E && rawByte == 0x5A) || (pending == 0x5A && rawByte == 0x5E)) {
                        auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - first_key_time);
                        if (delta <= COMBO_WINDOW) {
                            // Combo Detected!
                            pending_first_key.store(0x00);
                            combo_active_block.store(true);
                            
                            // Safety cooldown to reset combo block after release bytes arrive
                            std::thread([]() {
                                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                combo_active_block.store(false);
                            }).detach();

                            perform_role_switch(true);
                            return; // Halt execution
                        }
                    }
                }
            }

            // Ignoring Release Bytes if Combo activated
            if ((rawByte == 0x5F || rawByte == 0x5B) && combo_active_block.load()) {
                return;
            }

            // --- Generic Long Press Evaluation (Press check) ---
            if (has_long_command) {
                generic_long_press_byte.store(rawByte);
                generic_short_press_cmd = command; // Can be empty if only _l is configured
                generic_role = role;
                generic_long_press_active.store(true);
                int token = ++generic_long_press_token;

                std::thread([token, rawByte, longCommand, role]() {
                    std::this_thread::sleep_for(std::chrono::milliseconds(g_settings.longPressDelayMs));
                    if (generic_long_press_active.load() && generic_long_press_token.load() == token) {
                        generic_long_press_active.store(false); // Timer expired, consumed long press
                        send_calc_code_safely(longCommand);
                        if (g_print_commands_to_console) {
                            std::cout << "\n[COMMAND] " << role << " Long press 0x" << std::hex << std::uppercase << (int)rawByte << std::nouppercase << std::dec << " detected! Executing: " << longCommand << "\n";
                        }
                    }
                }).detach();
                return; // Halt execution for the short command! Handled on release or timeout.
            }

            if (has_short_command) {
                if (g_print_commands_to_console) {
                    // Formatting rawByte as hex
                    std::cout << "[COMMAND] " << role << " mapped byte 0x" << std::hex << std::uppercase << (int)rawByte << std::nouppercase << std::dec << " to: " << command;
                    if (repetitions > 1) {
                        std::cout << " (x" << repetitions << ")";
                    }
                    std::cout << "\n";
                }
                // Send the command now 1x for slow or 5x for fast turning
                for(int i = 0; i < repetitions; ++i) {
                    send_calc_code_safely(command);
                }
            }
        }
}

// Helper function: Finds the Bluetooth service UUID a desired characteristic UUID runs on for a particular peripheral
// Important in case we know the characteristic UUID (e.g. from the config) but not the service UUID (because it might have changed with a firmware update etc.)
static bool find_characteristic(SimpleBLE::Peripheral& p,
                         const std::string& desired_lower,
                         SimpleBLE::BluetoothUUID& out_service,
                         SimpleBLE::BluetoothUUID& out_char) {
    try {
        for (auto& service : p.services()) {
            for (auto& chr : service.characteristics()) {
                if (to_lower(chr.uuid()) == desired_lower) {
                    out_service = service.uuid();
                    out_char    = chr.uuid();
                    return true;
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Service discovery failed on " << p.identifier()
                  << ": " << e.what() << std::endl;
    }
    return false;
}

// Extra teardown helpers to ensure clean disconnects on WinRT
static void clear_adapter_callbacks(SimpleBLE::Adapter& adapter) {
    adapter.set_callback_on_scan_found({});
    adapter.set_callback_on_scan_start({});
    adapter.set_callback_on_scan_stop({});
}

// Function to cleanly tear down the Bluetooth connection without crashing the application
static void safe_unsubscribe_and_disconnect(SimpleBLE::Peripheral& p,
                                            const SimpleBLE::BluetoothUUID& service_uuid,
                                            const SimpleBLE::BluetoothUUID& characteristic_uuid) {
    try { p.unsubscribe(service_uuid, characteristic_uuid); } catch (...) {}
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // let CCCD write flush
    try { p.disconnect(); } catch (...) {}
    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // settle time
}

// As the name implies: Gracefully shutdown of Bluetooth connections and WASMIF session with the Sim
static void graceful_shutdown() {
    bool expected = false;
    if (!g_ble.shuttingDown.compare_exchange_strong(expected, true)) {
        return; // already running
    }

    std::optional<SimpleBLE::Adapter> localAdapter;
    std::unordered_map<std::string, ConnectedDevice> localConnected;
    std::map<std::string, SimpleBLE::Peripheral> localActivePeripherals;

    {
        std::lock_guard<std::mutex> lk(g_ble.m);
        localAdapter = g_ble.adapter;
        localConnected.swap(g_ble.connected); // take ownership of current set
        localActivePeripherals.swap(g_activePeripherals); // take ownership of active peripherals
        g_ble.adapter.reset();
    }

    for (auto& kv : localConnected) {
        auto& cd = kv.second;
        safe_unsubscribe_and_disconnect(cd.p, cd.service_uuid, cd.char_uuid);
    }

    localActivePeripherals.clear(); // clear any remaining references so destructors run safely before process exit
    localConnected.clear(); // perform actual cleanup locally

    if (localAdapter) {
        clear_adapter_callbacks(localAdapter.value());
    }

    // Attempt to end WASMIF session as well
    if (wasmPtr) {
        try { wasmPtr->end(); } catch (...) {}
    }
}

// Console control handler: handle close, Ctrl+C, shutdown
static BOOL WINAPI ConsoleCtrlHandler(DWORD type) {
    switch (type) {
        case CTRL_C_EVENT:
        case CTRL_BREAK_EVENT:
        case CTRL_CLOSE_EVENT:
        case CTRL_LOGOFF_EVENT:
        case CTRL_SHUTDOWN_EVENT:
            graceful_shutdown();
            Sleep(200); // brief pause to help OS tear down links
            return TRUE;
        default:
            return FALSE;
    }
}

// Helper function: Blinks the device Blink_count times 
static void blink_device(SimpleBLE::Peripheral p, 
                         const SimpleBLE::BluetoothUUID s_uuid, 
                         const SimpleBLE::BluetoothUUID c_uuid, 
                         int blink_count, 
                         bool fire_and_forget) {
    
    // The actual blink logic wrapped in a lambda expression
    auto blink_task = [p, s_uuid, c_uuid, blink_count]() mutable {
        try {
            for (int i = 0; i < blink_count; ++i) {
                p.write_request(s_uuid, c_uuid, SimpleBLE::ByteArray(1, 0x00)); // ON
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                p.write_request(s_uuid, c_uuid, SimpleBLE::ByteArray(1, 0x40)); // OFF
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        } catch (const std::exception& e) {
            std::cerr << "[BLINK] Error while blinking: " << e.what() << std::endl;
        }
    };

    // If "Fire & Forget", we offload the logic to a background thread
    if (fire_and_forget) {
        std::thread(blink_task).detach();
    } else {
        // Otherwise we execute the task synchronously and block the code here
        blink_task();
    }
}

// Scan, identify and connect to all target devices
int ble_run_session_scan_until_all_addresses(
    const std::unordered_set<std::string>& target_macs,
    const std::string& desired_char_lower,
    std::function<void(const SimpleBLE::ByteArray&, const std::string&)> on_packet,
    bool is_reconnect = false) {
    
    auto adapters = SimpleBLE::Adapter::get_adapters();
    if (adapters.empty()) {
        std::cerr << "[ERROR] No Bluetooth adapter found." << std::endl;
        return -1;
    }

    // We take the first adapter directly
    SimpleBLE::Adapter adapter = adapters[0];

    // IMPORTANT: No & before adapter! We assign the object directly.
    g_ble.adapter = adapter; 
    g_ble.shuttingDown = false;


    // 1. Container for found, relevant devices
    std::vector<SimpleBLE::Peripheral> found_candidates;

    // Extract unique device types from known devices for dynamic matching
    std::vector<std::string> valid_types;
    for (const auto& dev : g_knownDevices) {
        if (!dev.type.empty() && dev.type != "UNKNOWN" && dev.type != "unknown") {
            if (std::find(valid_types.begin(), valid_types.end(), dev.type) == valid_types.end()) {
                valid_types.push_back(dev.type);
            }
        }
    }
    // Fallback if config has no valid types listed
    if (valid_types.empty()) {
        valid_types.push_back("SHB");
        valid_types.push_back("SH100");
    }

    // 2. Scan callback: Only check MACs, don't query names
    adapter.set_callback_on_scan_found([&, valid_types](SimpleBLE::Peripheral p) {
        std::string raw_addr = p.address();
        std::string clean_addr;
        canonicalize_mac(raw_addr, clean_addr); // Compress MAC to "aabbccddeeff" for matching

        // Debug: Show EVERYTHING Windows finds
        {
            std::lock_guard<std::mutex> lock(g_consoleMutex);
            std::cout << "[INFO] Found: " << (p.identifier().empty() ? "BLE Device" : p.identifier()) 
                      << " | MAC: " << raw_addr << std::endl;
        }

        if (target_macs.count(clean_addr)) {
            // Check if we already have the MAC in the array to avoid duplicates
            auto it = std::find_if(found_candidates.begin(), found_candidates.end(),
                [&clean_addr](SimpleBLE::Peripheral& existing) {
                    std::string existing_clean;
                    canonicalize_mac(existing.address(), existing_clean);
                    return existing_clean == clean_addr;
                });

            if (it == found_candidates.end()) {
                found_candidates.push_back(p);
                std::cout << "  >>> MATCH! Device found in list and added." << std::endl;
            }
        } 
        // Always admit devices showing matching identifiers, even if unknown to config yet
        else {
            bool matched_type = false;
            for (const auto& vt : valid_types) {
                if (p.identifier().find(vt) == 0) {
                    matched_type = true;
                    break;
                }
            }

            if (matched_type) {
                auto it = std::find_if(found_candidates.begin(), found_candidates.end(),
                    [&clean_addr](SimpleBLE::Peripheral& existing) {
                        std::string existing_clean;
                        canonicalize_mac(existing.address(), existing_clean);
                        return existing_clean == clean_addr;
                    });

                if (it == found_candidates.end()) {
                    found_candidates.push_back(p);
                    std::cout << "  >>> [*NEW*] Discovered UNKNOWN but matching device: " << p.identifier() << " [" << raw_addr << "]" << std::endl;
                }
            }
        }
    });

    // Default 10 seconds, in reconnect only 1 second
    int scan_duration_ms = is_reconnect ? 1000 : 10000;
    int attempts = 0;
    int max_attempts = is_reconnect ? 10 : 2; // 10 attempts on reconnect, 2 on startup

    // If we have no known devices, we want to force scanning to find unseen/new devices
    size_t required_targets = target_macs.size();
    if (required_targets == 0 && !is_reconnect) {
        required_targets = 1; // Force at least one scan iteration to find any new device
    }

    std::cout << "\n[INFO] Start scan for " << target_macs.size() << " known devices..." << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;

    while (found_candidates.size() < required_targets && attempts < max_attempts) {
        attempts++;
        if (attempts > 1) std::cout << "[RETRY] Starting attempt " << attempts << "..." << std::endl;
        // Only the actual scan process is repeated
        adapter.scan_for(scan_duration_ms); 
        
        // Short pause between attempts so the hardware can "breathe"
        if (found_candidates.size() < required_targets && attempts < max_attempts) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    }

    // If there are still no candidates, back out completely and repeat with the long scan
    if (found_candidates.empty()) {
        std::cout << "[WARNING] Nothing found even after " << attempts << " attempts." << std::endl;
        return 42; 
    }

    int connectedCount = 0;
    std::string mac;
    SimpleBLE::BluetoothUUID service_uuid, char_uuid;


    if (is_reconnect) {     // In case of a fast reconnect
        for (auto& p : found_candidates) {
            std::string clean_addr;
            canonicalize_mac(p.address(), clean_addr);

            // --- THE SHORTCUT (FAST-LANE) ---
            // 1. Get role from the snapshot (lastSessionDevices)
            std::string role = "";
            for (auto const& [r, m] : lastSessionDevices) {
                if (m == clean_addr) { role = r; break; }
            }

            if (!role.empty()) {
                std::cout << "[FAST] Auto-Match: " << role << " (" << clean_addr << ")" << std::endl;
                
                // 2. Direct setup (Connect + Callbacks + Map-entry)
                // Here we use the code that normally only comes after 'cin'
                try {
                    if (p.is_connected()) 
                    {std::cout << " [OK] " << p.identifier() << " [" << clean_addr << "] IS already connected as " << role << "!" << std::endl;}
                    else {std::cout << " [NO] P was not yet connected." << std::endl;};
                    if (!p.is_connected()) p.connect();

                    std::cout << " [OK] " << p.identifier() << " [" << clean_addr << "] should be connected as " << role << " again." << std::endl;

                    if (!find_characteristic(p, desired_char_lower, service_uuid, char_uuid)) {
                        std::cout << "   [!] No matching characteristic found. Skipping." << std::endl;
                        p.disconnect();
                        continue;
                    }

                 // --- SUBSCRIBE & REGISTER (PFD or MFD) ---
                bool subscribed = false;
                bool used_indicate = false;
                auto devTag = p.address();

                // Check capabilities
                bool can_indicate = false, can_notify = false;
                for (auto& s : p.services()) {
                    if (s.uuid() != service_uuid) continue;
                    for (auto& chr : s.characteristics()) {
                        if (chr.uuid() == char_uuid) { can_indicate = chr.can_indicate(); can_notify = chr.can_notify(); break; }
                    }
                }

                if (can_indicate) {
                    p.indicate(service_uuid, char_uuid, [on_packet, devTag](SimpleBLE::ByteArray b) { on_packet(b, devTag); });
                    subscribed = true; used_indicate = true;
                } else {
                    p.notify(service_uuid, char_uuid, [on_packet, devTag](SimpleBLE::ByteArray b) { on_packet(b, devTag); });
                    subscribed = true;
                }

                if (subscribed) {
                    std::lock_guard<std::mutex> lk(g_ble.m);
                    // 1. Data logic (Map for button presses)
                    g_ble.connected.emplace(clean_addr, ConnectedDevice{p, service_uuid, char_uuid, used_indicate});
                    // 2. Light control (Map for write access)
                    g_activePeripherals.insert_or_assign(clean_addr, p);
                    
                    std::cout << "   [OK] " << (used_indicate ? "Indication" : "Notification") << " active." << std::endl;
                    connectedCount++;
                }


                } catch (const std::exception& e) {
                    std::cerr << "   [ERROR] Communication with " << p.address() << " failed: " << e.what() << std::endl;
                }                    

            }
        }
    if (!connectedCount == 0) {
        std::cout << "[SYSTEM] Reconnect finished." << std::endl;
        
        // After a reconnect, reset the brightness of the backlighting.
        // Give a short pause so that the callbacks and internal BLE states are finished.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        initialBrightnessSet = true;
        send_brightness_to_all(g_settings.defaultBrightness);
    }
    else {
        std::cout << "[SYSTEM] Reconnect attempt was unfortunately unsuccessful."<< std::endl;
    }
    } 
    else {   // --- THE NORMAL WAY (Program start) ---
        // 4. AFTER the scan: User verification (in the main thread!)
        for (auto& p : found_candidates) {
            std::string addr = p.address();
            
            // Find the associated instrument in g_knownDevices
            auto it = std::find_if(g_knownDevices.begin(), g_knownDevices.end(),
                                [&](const HardwareDevice& d) { return d.address == addr; });
            
            if (it != g_knownDevices.end()) {
                std::cout << "\nFound: " << it->instrument << " (" << addr << ")" << std::endl;
                std::cout << "Should this device be connected as '" << it->instrument << "'? (y/n): ";
                
                char choice;
                std::cin >> choice;
                
                if (choice == 'y' || choice == 'Y' || choice == 'j' || choice == 'J') {
                    std::cout << "[INFO] Connecting to " << it->instrument << "..." << std::endl;
                }
            }
        }

        // --- PHASE 2: INTERACTIVE IDENTIFICATION & ASSIGNMENT ---
        std::cout << "----------------------------------------------------" << std::endl;
        std::cout << "\n[INFO] Starting identification of " << found_candidates.size() << " candidates...\n";

        for (auto& p : found_candidates) {
            std::string mac;
            canonicalize_mac(p.address(), mac);

            std::cout << "Checking device: " << p.identifier() << " [" << p.address() << "]" << std::endl;

            try {
                if (!p.is_connected()) p.connect();

//                SimpleBLE::BluetoothUUID service_uuid, char_uuid;
                if (!find_characteristic(p, desired_char_lower, service_uuid, char_uuid)) {
                    std::cout << "   [!] No matching characteristic found. Skipping." << std::endl;
                    p.disconnect();
                    continue;
                }

                // --- IDENTIFICATION: 3x BLINK (Background) ---
                std::cout << "   -> Bezel is blinking to identify..." << std::endl;
                blink_device(p, service_uuid, char_uuid, 3, true);

                // --- Dynamic Role Assignment ---
                // 1. Determine standard role (proposal)
                char defChar = 'K'; // Fallback: Skip
                std::string defLabel = "K"; 
                bool foundInConfig = false;

                for (const auto& dev : g_knownDevices) {
                    std::string cleanDevAddr;
                    canonicalize_mac(dev.address, cleanDevAddr);
                    
                    if (cleanDevAddr == mac) { // 'mac' is the already cleaned MAC of the current device
                        if (dev.instrument == "PFD") { defChar = 'P'; defLabel = "P"; }
                        else if (dev.instrument == "MFD") { defChar = 'M'; defLabel = "M"; }
                        else if (dev.instrument == "RADIO") { defChar = 'A'; defLabel = "A"; }
                        foundInConfig = true;
                        break;
                    }
                }

                // 2. Display prompt
                // (without the value at the end - that comes in a moment)
                std::cout << "\n>>> Choose role for " << p.identifier() << " [" << mac << "]:" << std::endl;
                std::cout << "(P)FD, (M)FD, R(A)dio, S(K)IP? : ";

                // 2. "Inject" the proposal into the buffer
                if (foundInConfig) {
                    inject_default_input(defLabel); 
                }

                std::string input;
                std::getline(std::cin, input);

                // 3. Process input
                char choice;
                if (input.empty()) {
                    choice = defChar; // Take the proposal
                } else {
                    choice = std::toupper(input[0]);
                }


                if (choice == 'P' || choice == 'M' || choice == 'A') {
                    std::string role = (choice == 'P') ? "PFD" : (choice == 'M') ? "MFD" : "RADIO";
                    
                    // Look for existing device in g_knownDevices or add as new
                    bool found = false;
        
                    // Helper function to provide current timestamp
                    auto getCurrentTimestamp = []() {
                        auto now = std::chrono::system_clock::now();
                        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
                        std::tm bt{};
                        localtime_s(&bt, &now_c);
                        std::stringstream ss;
                        ss << std::put_time(&bt, "%Y-%m-%dT%H:%M:%S");
                        return ss.str();
                    };

                    for (auto& dev : g_knownDevices) {
                        std::string cleanDevAddr;
                        canonicalize_mac(dev.address, cleanDevAddr);

                        if (cleanDevAddr == mac) {
                            dev.instrument = role;
                            dev.address = mac;
                            // ONLY update if p.identifier() is NOT empty
                            std::string scannedName = p.identifier();
                            if (!scannedName.empty()) {
                                dev.ble_name = scannedName;
                            } 
                            // If dev.ble_name is STILL empty (even after the Load function), 
                            // THEN set to "Unknown"
                            if (dev.ble_name.empty()) {
                                dev.ble_name = "Unknown";
                            }
                            // Update timestamp for this device
                            dev.last_connected = getCurrentTimestamp(); 
                            found = true;
                            break;
                        }
                    }

                    if (!found) {
                        // If a completely new device is found, it also gets the current time
                        g_knownDevices.push_back({role, "unknown", p.identifier(), "none", mac, getCurrentTimestamp(), true});
                    }

                    std::cout << "[OK] " << p.identifier() << " [" << mac << "] was registered as " << role << "." << std::endl;
                } else {
                    std::cout << " [SKIP] " << p.identifier() << " [" << mac << "] is ignored." << std::endl;
                    p.disconnect();
                    continue; // Skip to next loop iteration
                }
                // -----------------------------------------------------

                // --- SUBSCRIBE & REGISTER (PFD or MFD) ---
                bool subscribed = false;
                bool used_indicate = false;
                auto devTag = p.address();

                // Check Capabilities
                bool can_indicate = false, can_notify = false;
                for (auto& s : p.services()) {
                    if (s.uuid() != service_uuid) continue;
                    for (auto& chr : s.characteristics()) {
                        if (chr.uuid() == char_uuid) { can_indicate = chr.can_indicate(); can_notify = chr.can_notify(); break; }
                    }
                }

                if (can_indicate) {
                    p.indicate(service_uuid, char_uuid, [on_packet, devTag](SimpleBLE::ByteArray b) { on_packet(b, devTag); });
                    subscribed = true; used_indicate = true;
                } else {
                    p.notify(service_uuid, char_uuid, [on_packet, devTag](SimpleBLE::ByteArray b) { on_packet(b, devTag); });
                    subscribed = true;
                }

                if (subscribed) {
                    std::lock_guard<std::mutex> lk(g_ble.m);
                    // 1. Data logic (Map for button presses)
                    g_ble.connected.emplace(mac, ConnectedDevice{p, service_uuid, char_uuid, used_indicate});
                    // 2. Light control (Map for write access)
                    g_activePeripherals.insert_or_assign(mac, p);
                    
                    std::cout << "[OK] Button monitoring (via " << (used_indicate ? "Indication" : "Notification") << ") activated." << std::endl;
                    connectedCount++;
                }

            } catch (const std::exception& e) {
                std::cerr << "   [ERROR] Communication with " << p.address() << " failed: " << e.what() << std::endl;
            }
        }
        if (connectedCount == 0) {
            std::cout << "\n[INFO] No devices assigned. Ending session." << std::endl;
            return 0;
        }
        else {
            std::cout << "\n[INFO] " << connectedCount << " devices successfully connected and assigned.\n" << std::endl;
        }
    }   // End of Normal Way (Program start)

    // Set flag that discovery phase is over (important for logic in on_packet)
    g_discoveryFinished = true; 

    // Save Known Devices to file only if it is NOT an in-flight fast reconnect
    // The snapshot of connected devices is also not updated during reconnect
    if(!is_reconnect) {
        // --- Save configuration permanently now ---
        save_known_devices(); 

        // Create a snapshot of connected devices at end of discovery phase (for a possible fast reconnect)
        update_last_session_snapshot();
    }

    // Initial backlight setting
    if (!initialBrightnessSet && g_discoveryFinished && !g_ble.connected.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
       
        send_brightness_to_all(g_settings.defaultBrightness);
        initialBrightnessSet = true; 
        {
            std::lock_guard<std::mutex> lock(g_consoleMutex);
            std::cout << "\n[INFO] Brightness set from config." << std::endl;
            std::cout << "Press [1] Low ... [4] High for Backlight, [0]/[^] OFF.\n" << std::endl;
        }
    }

    std::cout << "----------------------------------------------------" << std::endl;
    std::cout << "[OK] SETUP COMPLETED!" << std::endl;
    if (wasmPtr && !wasmPtr->isRunning()) {
        std::cout << "BUT: [INFO] Simulator connection still pending. Retrying in the background..." << std::endl;
    }
    std::cout << "----------------------------------------------------\n" << std::endl;

    // --- PHASE 3: WAIT LOOP FOR RECONNECT OR QUIT ---
    std::cout << "PRESS 'H' FOR HELP | 'M' TO RELOAD MAP | 'A' TO ASSIGN MAP | 'R' TO RECONNECT | 'S' TO SWAP ROLES | 'L' TO ASSIGN ROLES | 'C' TO TOGGLE CMD PRINT | '?' FOR AIRCRAFT | 'Q' TO QUIT\n" << std::endl;

    while (!g_ble.shuttingDown) {
        // Check for pending aircraft to prompt
        std::string current_pending_aircraft;
        std::string current_suggested_map;
        {
            std::lock_guard<std::mutex> lock(g_pending_aircraft_mutex);
            if (!g_pending_new_aircraft.empty()) {
                current_pending_aircraft = g_pending_new_aircraft;
                current_suggested_map = g_suggested_map_id;
            }
        }

        if (!current_pending_aircraft.empty()) {
            std::string display_name = current_suggested_map;
            auto name_it = g_settings.map_names.find(current_suggested_map);
            if (name_it != g_settings.map_names.end() && !name_it->second.empty()) {
                display_name = name_it->second;
            }
            
            bool valid_input = false;
            g_interactive_prompt_active = true;
            while (!valid_input && !g_ble.shuttingDown) {
                {
                    std::lock_guard<std::mutex> cout_lock(g_consoleMutex);
                    std::cout << "\n==========================================================\n";
                    std::cout << "NEW AIRCRAFT DETECTED: \"" << current_pending_aircraft << "\"\n";
                    std::cout << "There is no mapping assigned to this variant yet.\n";
                    std::cout << "Suggestion: Apply the '" << display_name << "' command map.\n";
                    std::cout << "Press [Y] (and Enter) to ACCEPT, or [N] for other options.\n";
                    std::cout << "==========================================================\n> ";
                }

                // Pre-fill 'Y' using the Windows API buffer injection
                inject_default_input("Y");

                std::string input;
                std::getline(std::cin, input);

                char ch = input.empty() ? 'Y' : std::toupper(input[0]);

                if (ch == 'Y') {
                    valid_input = true;
                    // ACCEPT
                    {
                        std::lock_guard<std::mutex> lock(g_consoleMutex);
                        std::cout << "[SYSTEM] Assigned '" << current_suggested_map << "' to '" << current_pending_aircraft << "'.\n";
                    }

                    // Save 
                    auto now = std::chrono::system_clock::now();
                    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
                    std::tm bt{};
                    localtime_s(&bt, &now_c);
                    std::stringstream ss;
                    ss << std::put_time(&bt, "%Y-%m-%dT%H:%M:%S");

                    g_settings.aircraft_history[current_pending_aircraft] = {current_suggested_map, ss.str()};
                    save_master_config(); 

                } else if (ch == 'N') {
                    valid_input = true;
                    interactive_map_selection(current_pending_aircraft);
                } else {
                    std::lock_guard<std::mutex> lock(g_consoleMutex);
                    std::cout << "[ERROR] Invalid input. Please enter 'Y' or 'N'.\n";
                }
            }
            g_interactive_prompt_active = false;

            // Clear prompt state
            {
                std::lock_guard<std::mutex> lock(g_pending_aircraft_mutex);
                g_pending_new_aircraft.clear();
                g_suggested_map_id.clear();
            }

            continue; // Re-evaluate loop
        }

        if (_kbhit()) {
            char ch = _getch();

            if (std::tolower(ch) == g_simRateConfig.activation_hotkey) {
                if (g_simRateController) {
                    g_simRateController->toggleActive();
                    std::cout << "\n[SYSTEM] SimRate Auto-Correction is now " 
                              << (g_simRateController->isActive() ? "ACTIVE" : "INACTIVE") << "\n";
                }
                continue;
            }

            if (ch == '-') {
                if (g_simRateController && g_simRateController->isActive()) {
                    g_simRateController->deactivate();
                    std::cout << "\n[SYSTEM] SimRate Auto-Correction is now INACTIVE\n";
                }
                continue;
            }

            if (ch == 'a' || ch == 'A') {
                if (g_current_aircraft_title.empty()) {
                    std::cout << "\n[SYSTEM] No aircraft loaded yet. Cannot assign map.\n";
                    continue;
                }
                
                g_interactive_prompt_active = true;
                std::cout << "\n[SYSTEM] Re-assign command map for '" << g_current_aircraft_title << "'? [Y/N] (timeouts in 5s)...";
                
                int wait_ms = 5000;
                bool confirmed = false;
                bool answered = false;
                
                while(wait_ms > 0 && !g_ble.shuttingDown) {
                    if (_kbhit()) {
                        char ans = std::toupper(_getch());
                        if (ans == 'Y') { confirmed = true; answered = true; break; }
                        if (ans == 'N' || ans == 27) { confirmed = false; answered = true; break; } // 27 = ESC
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    wait_ms -= 100;
                }
                
                if (!answered) {
                    std::cout << " Timeout.\n[SYSTEM] Current map assignment kept.\n";
                } else if (!confirmed) {
                    std::cout << " Cancelled.\n[SYSTEM] Current map assignment kept.\n";
                } else {
                    std::cout << " Y\n";
                    interactive_map_selection(g_current_aircraft_title);
                }
                
                g_interactive_prompt_active = false;
                continue;
            }

            if (ch == 'o' || ch == 'O') {
                if (g_orbit_active) {
                    g_orbit_active = false;
                    play_toggle_beep(g_orbit_active);
                    std::cout << "\n[SYSTEM] Orbit Function is now OFF." << std::endl;
                } else {
                    if (!g_hSimConnectNative) {
                        play_toggle_beep(false, 1, true);
                        std::cout << "\n[SYSTEM] Cannot engage Orbit Function: No active SIM connection." << std::endl;
                    } else {
                        std::cout << "\n[SYSTEM] Checking conditions to engage Orbit Function..." << std::endl;
                        g_orbit_evaluation_pending = true;
                        SimConnect_RequestDataOnSimObject(g_hSimConnectNative, REQUEST_ORBIT_DATA, DEFINITION_ORBIT_DATA, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_ONCE);
                    }
                }
            }

            if (ch == 'l' || ch == 'L') {
                g_interactive_prompt_active = true;
                std::cout << "\n--- Manual Role Assignment ---\n";
                bool changed = false;
                for (auto& dev : g_knownDevices) {
                    if (dev.address.empty()) continue;
                    std::cout << "Device: " << dev.ble_name << " [" << dev.address << "] is currently: " << dev.instrument << "\n";
                    std::cout << "Enter new role (P=PFD, M=MFD, R=RADIO), or any other key to keep: ";
                    
                    int wait_ms = 10000;
                    char choice = 0;
                    while (wait_ms > 0 && !g_ble.shuttingDown) {
                        if (_kbhit()) {
                            choice = std::toupper(_getch());
                            std::cout << choice << "\n";
                            break;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        wait_ms -= 100;
                    }
                    
                    if (choice == 0) {
                         std::cout << " Timeout.\n";
                    } else if (choice == 'P' || choice == 'M' || choice == 'R') {
                         if (choice == 'P') dev.instrument = "PFD";
                         else if (choice == 'M') dev.instrument = "MFD";
                         else if (choice == 'R') dev.instrument = "RADIO";
                         changed = true;
                    }
                    std::cout << " -> Final role: " << dev.instrument << "\n\n";
                }
                if (changed) {
                    save_known_devices();
                    update_last_session_snapshot();
                    std::cout << "[OK] Roles updated and saved.\n";
                } else {
                    std::cout << "[OK] No changes made.\n";
                }
                g_interactive_prompt_active = false;
            }

            if (ch == 'h' || ch == 'H') {
                std::lock_guard<std::mutex> lock(g_consoleMutex);
                std::cout << "\n--- Console Shortcuts Quick Reference ---\n";
                std::cout << "  [O]     : Toggle Orbit Function on/off.\n";
                std::cout << "  [H]     : Show this Help menu.\n";
                std::cout << "  [Y]/[N] : Respond to prompts (e.g. Map Assignment).\n";
                std::cout << "  [1]-[4] : Adjust Backlight Brightness ([0] for OFF).\n";
                std::cout << "  [A]     : Manually Assign a new Command Map for the current aircraft.\n";
                std::cout << "  [M]     : Reload Command Map and recheck current aircraft.\n";
                std::cout << "  [R]     : Force Reconnect to Simionic Bezels and MSFS.\n";
                std::cout << "  [S]     : Swap the roles of your devices (PFD <-> MFD).\n";
                std::cout << "  [L]     : Assign Roles to your known devices.\n";
                std::cout << "  [C]     : Toggle Command output printing in the console.\n";
                std::cout << "  [?]     : Determine currently connected aircraft name.\n";
                std::cout << "  [Q]     : Quit the application gracefully.\n";
                std::cout << "-----------------------------------------\n";
            }

            if (ch == 'c' || ch == 'C') {
                g_print_commands_to_console = !g_print_commands_to_console;
                if (g_print_commands_to_console) {
                    std::cout << "\n[CONSOLE] Command printing enabled." << std::endl;
                } else {
                    std::cout << "\n[CONSOLE] Command printing disabled." << std::endl;
                }
            }

            if (ch == 'r' || ch == 'R') {
                std::cout << "\n[RECONNECT] Starting fast reconnect attempt..." << std::endl;
                // We stay in the main loop and only call the fast logic
                trigger_fast_reconnect(); 
            }

            if (ch == '?') {
                if (g_hSimConnectNative) {
                    std::cout << "\n[SIMCONNECT] Requesting current aircraft..." << std::endl;
                    SimConnect_RequestDataOnSimObject(g_hSimConnectNative, REQUEST_AIRCRAFT_TITLE, DEFINITION_AIRCRAFT_TITLE, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_ONCE);
                } else {
                    std::cout << "\n[SIMCONNECT] Not connected to simulator natively yet." << std::endl;
                }
            }

            if (ch == 'q' || ch == 'Q') {
                std::cout << "\n[QUIT] Exiting program..." << std::endl;
                graceful_shutdown();
                break;
            }
        }
        
        if (g_hSimConnectNative) {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last_orbit_time).count() >= 500) {
                // Request this data fast (every 500ms instead of 5s) because we want GGGD telemetry to be responsive.
                SimConnect_RequestDataOnSimObject(g_hSimConnectNative, REQUEST_ORBIT_DATA, DEFINITION_ORBIT_DATA, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_ONCE);
                g_last_orbit_time = now;
            }
        }

        if (g_trigger_auto_reconnect) {
            g_trigger_auto_reconnect = false;
            std::cout << "\n[AUTO-RECONNECT] Initiating fast reconnect after connection loss..." << std::endl;
            trigger_fast_reconnect();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}

// Function to trigger a fast reconnect to lost Bluetooth devices based on the session snapshot
void trigger_fast_reconnect() {
    if (lastSessionDevices.empty()) return;
    std::cout << "[RECONNECT] Disconnecting old devices asynchronously" << std::endl;

    for (auto& [role, peripheral] : g_activePeripherals) {
        std::thread([p = peripheral]() mutable {
                try {
                    if (p.is_connected()) {
                        p.disconnect();
                    }
                } catch (...) {}
            }).detach(); // Run in the background
    }

    // 2. IMPORTANT: Clear maps immediately so we have room for the new objects
    g_activePeripherals.clear(); 
    {
        std::lock_guard<std::mutex> lk(g_ble.m);
        g_ble.connected.clear();
    }

    // 2. Prepare structure (unordered_set)
    std::unordered_set<std::string> macsToFind;
    for (auto const& [role, mac] : lastSessionDevices) {
        macsToFind.insert(mac); // Pulls the MACs from the map for ble_run... function
    }
    
    // A short pause (0.5s) for the Bluetooth stack
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[RECONNECT] Starting targeted search for reconnect..." << std::endl;
    int result = ble_run_session_scan_until_all_addresses(
        macsToFind, 
        g_settings.characteristicUUID, // UUID from global config
        on_packet,                      // Callback handler
        true                            // Is Fast Reconnect
    );

}

// Helper function: Send backlight brightness level to all connected devices
static void send_brightness_to_all(uint8_t level) {
    SimpleBLE::ByteArray data;
    data.push_back(level); 

    // We use g_ble.connected as all info (p, service, char) is stored there
    for (auto& [mac, device] : g_ble.connected) {
        if (!device.p.is_connected()) continue;
        
        try {
            device.p.write_request(
                device.service_uuid, 
                SimpleBLE::BluetoothUUID(g_settings.lightBrightnessCharacteristicUUID), // from the settings of the config
                data
            );
        } catch (const std::exception& e) {
            std::cerr << "[LIGHT] Error on " << mac << ": " << e.what() << std::endl;
        }
    }
}

// ----------------------------------------------------
// -------------- MAIN FUNCTION -----------------------
static void interactive_map_selection(const std::string& aircraft_title) {
    std::vector<std::pair<std::string, std::string>> available_maps;
    for (const auto& [id, file] : g_settings.map_definitions) {
        std::string nice_name = id;
        auto nm_it = g_settings.map_names.find(id);
        if (nm_it != g_settings.map_names.end() && !nm_it->second.empty()) {
            nice_name = nm_it->second;
        }
        available_maps.push_back({id, nice_name});
    }

    bool valid_map_selected = false;
    while (!valid_map_selected && !g_ble.shuttingDown) {
        {
            std::lock_guard<std::mutex> lock(g_consoleMutex);
            std::cout << "\n--- Available Command Maps ---\n";
            for (size_t i = 0; i < available_maps.size(); ++i) {
                std::cout << "[" << (i + 1) << "] " << available_maps[i].second << " (" << available_maps[i].first << ")\n";
            }
            std::cout << "Enter the number of the map to assign to '" << aircraft_title << "': ";
        }

        std::string num_input;
        std::getline(std::cin, num_input);

        try {
            size_t choice = std::stoull(num_input);
            if (choice > 0 && choice <= available_maps.size()) {
                valid_map_selected = true;
                std::string selected_map_id = available_maps[choice - 1].first;
                
                {
                    std::lock_guard<std::mutex> lock(g_consoleMutex);
                    std::cout << "[SYSTEM] Assigned '" << selected_map_id << "' to '" << aircraft_title << "'.\n";
                }

                auto now = std::chrono::system_clock::now();
                std::time_t now_c = std::chrono::system_clock::to_time_t(now);
                std::tm bt{};
                localtime_s(&bt, &now_c);
                std::stringstream ss;
                ss << std::put_time(&bt, "%Y-%m-%dT%H:%M:%S");

                g_settings.aircraft_history[aircraft_title] = {selected_map_id, ss.str()};
                save_master_config(); 

                auto map_it = g_settings.map_definitions.find(selected_map_id);
                if (map_it != g_settings.map_definitions.end()) {
                    COMMANDS_FILENAME = map_it->second;
                    load_command_map(COMMANDS_FILENAME);
                }
            } else {
                std::lock_guard<std::mutex> lock(g_consoleMutex);
                std::cout << "[ERROR] Invalid selection. Please choose a number between 1 and " << available_maps.size() << ".\n";
            }
        } catch (...) {
            std::lock_guard<std::mutex> lock(g_consoleMutex);
            std::cout << "[ERROR] Invalid input. Please enter a valid number.\n";
        }
    }
}

int main(int argc, char* argv[]) {

    std::unordered_set<std::string> knownDevices_macs;    // List of MACs we loaded from the new JSON config


    // Sets the console to UTF-8 encoding keyboard encoding
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    // Set process priority to elevated
    SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);

    std::cout << "[INFO]: Program start! [Vers. " << APP_VERSION << "]" << std::endl; 

    // Ensure teardown runs on console close / Ctrl+C
    SetConsoleCtrlHandler(ConsoleCtrlHandler, TRUE);

    // Load hardware (MACs, roles) first
   try {
    // 0. Load general settings (e.g. names of other config files, Default Brightness, Characteristic UUID etc.)
    if (!load_master_config(CONFIG_FILENAME)) {
        std::cerr << "[FATAL]: The main configuration could not be loaded." << std::endl;
        // --- INSERT CODE HERE ---
        std::cout << "Press [Enter] to exit the program..." << std::endl;
        std::cin.ignore(); // Clears the buffer
        std::cin.get();    // Waits for the enter key to be pressed
        // ------------------------------
        return -1;
    }

    // 1. Load hardware (MACs, roles)
    if (!load_known_devices(KNOWN_DEVICES_FILENAME)) {
        std::cerr << "[FATAL]: Hardware configuration could not be loaded." << std::endl;
        return -1;
    } else {
        // Create MAC list from the loaded hardware devices (for the scan)
        std::cout << "[OK] Device config loaded. Number of device types: " << count_if(g_knownDevices.begin(), g_knownDevices.end(), [](const auto& dev) { return !dev.address.empty(); })  << std::endl;
        for (const auto& dev : g_knownDevices) {
            if (!dev.address.empty()) {
                // We add the MAC. If it is capitalized/lowercase in the config,
                // we should normalize it here (e.g. all to lowercase), 
                // so that the comparison in the scanner works safely.
                std::string mac = dev.address;
                canonicalize_mac(dev.address, mac); // Ensure again that it is in the correct format
                knownDevices_macs.insert(mac);
                std::cout << "[INFO] Last known device: " << dev.instrument 
                        << " | MAC: " << dev.address 
                        << " | last seen: " << dev.last_connected << std::endl;
            }
        }
    }

    // 2. Load commands (Hex-to-SimConnect)
    if (!load_command_map(COMMANDS_FILENAME)) {
        std::cerr << "[WARNING] Commands could not be loaded. Functionality limited." << std::endl;
    }

    } catch (const std::exception& e) {
        std::cerr << "[CRITICAL ERROR] Exception caught: " << e.what() << std::endl;
        std::system("pause");
        return -1;
    }


    // WinRT teardown robustness between runs
    SimpleBLE::Config::WinRT::experimental_use_own_mta_apartment = true;
    SimpleBLE::Config::WinRT::experimental_reinitialize_winrt_apartment_on_main_thread = true;

    // SimConnect / WASM init (Non-blocking)
    wasmPtr = WASMIF::GetInstance();
    WASMIFGuard guard(wasmPtr);
    wasmPtr->setSimConfigConnection(SIMCONNECT_OPEN_CONFIGINDEX_LOCAL);
    wasmPtr->start();

    std::cout << "[INFO] Starting WASM connection to simulator in background..." << std::endl;

    // >>> NATIVE SIMCONNECT THREAD <<<
    std::thread nativeSimConnectThread([&]() {
        bool initial_aircraft_check_done = false;

        while (!g_ble.shuttingDown) {
            if (g_hSimConnectNative == NULL) {
                if (SUCCEEDED(SimConnect_Open(&g_hSimConnectNative, "BLE Bridge Native", NULL, 0, 0, 0))) {
                    SimConnect_AddToDataDefinition(g_hSimConnectNative, DEFINITION_AIRCRAFT_TITLE, "TITLE", NULL, SIMCONNECT_DATATYPE_STRING256);
                    SimConnect_AddToDataDefinition(g_hSimConnectNative, DEFINITION_ORBIT_DATA, "AUTOPILOT MASTER", "Bool", SIMCONNECT_DATATYPE_INT32);
                    SimConnect_AddToDataDefinition(g_hSimConnectNative, DEFINITION_ORBIT_DATA, "AUTOPILOT HEADING LOCK", "Bool", SIMCONNECT_DATATYPE_INT32);
                    SimConnect_AddToDataDefinition(g_hSimConnectNative, DEFINITION_ORBIT_DATA, "GPS WP DISTANCE", "Nautical miles", SIMCONNECT_DATATYPE_FLOAT64);
                    SimConnect_AddToDataDefinition(g_hSimConnectNative, DEFINITION_ORBIT_DATA, "GPS WP BEARING", "Degrees", SIMCONNECT_DATATYPE_FLOAT64);
                    SimConnect_AddToDataDefinition(g_hSimConnectNative, DEFINITION_ORBIT_DATA, "SIM ON GROUND", "Bool", SIMCONNECT_DATATYPE_INT32);
                    SimConnect_AddToDataDefinition(g_hSimConnectNative, DEFINITION_ORBIT_DATA, "TRAILING EDGE FLAPS LEFT PERCENT", "Percent", SIMCONNECT_DATATYPE_FLOAT64);
                    SimConnect_AddToDataDefinition(g_hSimConnectNative, DEFINITION_ORBIT_DATA, "PLANE ALT ABOVE GROUND", "Feet", SIMCONNECT_DATATYPE_FLOAT64);
                    SimConnect_AddToDataDefinition(g_hSimConnectNative, DEFINITION_ORBIT_DATA, "VELOCITY BODY Z", "Feet per second", SIMCONNECT_DATATYPE_FLOAT64);
                    SimConnect_AddToDataDefinition(g_hSimConnectNative, DEFINITION_ORBIT_DATA, "VERTICAL SPEED", "Feet per minute", SIMCONNECT_DATATYPE_FLOAT64);
                    SimConnect_AddToDataDefinition(g_hSimConnectNative, DEFINITION_ORBIT_DATA, "SIMULATION RATE", "Number", SIMCONNECT_DATATYPE_INT32);
                    
                    SimConnect_MapClientEventToSimEvent(g_hSimConnectNative, EVENT_SIM_RATE_INCR, "SIM_RATE_INCR");
                    SimConnect_MapClientEventToSimEvent(g_hSimConnectNative, EVENT_SIM_RATE_DECR, "SIM_RATE_DECR");
                    
                    std::cout << "[INFO] Native SimConnect connection established for A-Vars and Events." << std::endl;
                    initial_aircraft_check_done = false;
                }
            }
            if (g_hSimConnectNative) {
                HRESULT hr = SimConnect_CallDispatch(g_hSimConnectNative, MyNativeDispatchProc, NULL);
                
                if (FAILED(hr)) {
                    std::cout << "\n[SIMCONNECT] Native connection lost! Will attempt to reconnect..." << std::endl;
                    SimConnect_Close(g_hSimConnectNative);
                    g_hSimConnectNative = NULL;
                } else {
                    // Periodic refresh of Orbit Data to drive SimRate tracking
                    SimConnect_RequestDataOnSimObject(g_hSimConnectNative, REQUEST_ORBIT_DATA, DEFINITION_ORBIT_DATA, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_ONCE);
                    
                    // Trigger auto load the very first time the plane fully connects
                    if (g_discoveryFinished && !initial_aircraft_check_done) {
                        initial_aircraft_check_done = true;
                        std::cout << "\n[SIMCONNECT] Auto-requesting current aircraft to load mapping..." << std::endl;
                        SimConnect_RequestDataOnSimObject(g_hSimConnectNative, REQUEST_AIRCRAFT_TITLE, DEFINITION_AIRCRAFT_TITLE, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_ONCE);
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Base SimConnect loop rate
        }
        if (g_hSimConnectNative) {
            SimConnect_Close(g_hSimConnectNative);
            g_hSimConnectNative = NULL;
        }
    });
    nativeSimConnectThread.detach();

    // >>> SIM RATE MONITORING THREAD <<<
    g_simRateController = new SimRateController(g_simRateConfig);
    std::thread simRateTelemetryThread([&]() {
        while (!g_ble.shuttingDown) {
            if (g_simRateController && g_hSimConnectNative && g_orbitDataReady) {
                // Safely extract the latest values
                double v_speed_fpm;
                int s_rate;
                {
                    std::lock_guard<std::mutex> lock(g_orbitDataMutex);
                    v_speed_fpm = g_latestOrbitData.vertical_speed;
                    s_rate = g_latestOrbitData.simulation_rate;
                }
                
                // Convert Feet Per Minute to Feet Per Second to match array sensitivity expectations
                double v_speed_fps = v_speed_fpm / 60.0;
                
                // Run the rolling array logic (which handles its own deadband & state)
                g_simRateController->processTelemetry(v_speed_fps, s_rate);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(g_simRateConfig.sampling_interval_ms)); // Polling rate
        }
    });
    simRateTelemetryThread.detach();

    // >>> HEARTBEAT THREAD <<<
    std::thread heartbeatThread([&]() {
        bool was_connected = false;
        int check_counter = 0;

        while (!g_ble.shuttingDown) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            if (wasmPtr) {
                if (wasmPtr->isRunning()) {
                    if (!was_connected) {
                        std::cout << "\n[OK] Connection to simulator (WASM module) established!" << std::endl;
                        was_connected = true;
                    }
                    // Keep the line active by querying a common variable every 5 seconds
                    if (check_counter % 5 == 0) {
                        wasmPtr->getLvar("VerticalSpeed");
                    }
                } else {
                    if (was_connected) {
                        std::cout << "\n[SimPing] Lost Connection. Trying to reconnect..." << std::endl;
                        was_connected = false;
                    }
                    
                    // Periodically try to restart / notify user we are still looking
                    if (check_counter % 10 == 0 && !was_connected) {
                        if (g_discoveryFinished) {
                            std::cout << "[SimPing] Searching for WASM module. Is the simulator in cockpit mode?" << std::endl;
                        }
                        wasmPtr->start();
                    }
                }
            }
            check_counter++;
        }
    });
    heartbeatThread.detach(); 
    // >>> END HEARTBEAT <<<

    // >>> BLE DEVICE HEALTH CHECK THREAD <<<
    std::thread bleHealthCheckThread([&]() {
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            if (g_ble.shuttingDown) break;

            // Only check connected devices if setup is complete
            if (!g_discoveryFinished || g_ble.connected.empty()) {
                continue;
            }

            std::vector<std::pair<std::string, SimpleBLE::Peripheral>> peripherals_to_check;
            {
                std::lock_guard<std::mutex> lock(g_ble.m);
                for (auto& [mac, device] : g_ble.connected) {
                    peripherals_to_check.push_back({mac, device.p});
                }
            }

            bool connection_lost = false;
            for (auto& [mac, p] : peripherals_to_check) {
                if (p.is_connected()) {
                    try {
                        // Attempt to read the Generic Device Name (Service 1800, Characteristic 2A00)
                        p.read("00001800-0000-1000-8000-00805f9b34fb", "00002a00-0000-1000-8000-00805f9b34fb");
                    } catch (const std::exception&) {
                        // Throws if device disconnected abruptly or response times out
                        {
                            std::lock_guard<std::mutex> lock(g_consoleMutex);
                            std::cout << "\n[BLE-PING] Device " << p.identifier() << " [" << mac << "] stopped responding!" << std::endl;
                        }
                        connection_lost = true;
                        break; 
                    }
                } else {
                    // It already knows it's disconnected
                    {
                        std::lock_guard<std::mutex> lock(g_consoleMutex);
                        std::cout << "\n[BLE-PING] Device " << p.identifier() << " [" << mac << "] is no longer connected!" << std::endl;
                    }
                    connection_lost = true;
                    break;
                }
            }

            if (connection_lost && !g_trigger_auto_reconnect) {
                g_trigger_auto_reconnect = true;
            }
        }
    });
    bleHealthCheckThread.detach();
    // >>> END BLE HEALTH CHECK <<<

    // >>> COMBINED THREAD: ROLE SWITCH & LIGHT CONTROL <<<
    std::thread lightControlThread([&]() {
        bool s_was_pressed = false;
        bool m_was_pressed = false;

        DWORD myPid = GetCurrentProcessId();
        HWND hConsole = GetConsoleWindow();

        while (true) {
            HWND hForeground = GetForegroundWindow();
            DWORD foregroundProcessId = 0;
            GetWindowThreadProcessId(hForeground, &foregroundProcessId);

            // Focus check
            if ((foregroundProcessId == myPid || hForeground == hConsole || (hConsole && hForeground == GetParent(hConsole))) && !g_interactive_prompt_active) {
                
                // 1. ROLE SWITCH (Key 'S')
                bool s_is_down = (GetAsyncKeyState('S') & 0x8000) != 0;
                if (s_is_down && !s_was_pressed) {
                    perform_role_switch(true);
                }
                s_was_pressed = s_is_down;

                // 2. RELOAD COMMAND MAP (Key 'M')
                bool m_is_down = (GetAsyncKeyState('M') & 0x8000) != 0;
                if (m_is_down && !m_was_pressed) {
                    {
                        std::lock_guard<std::mutex> lock(g_consoleMutex);
                        if (g_hSimConnectNative) {
                            std::cout << "\n[SYSTEM] Re-querying current aircraft and reloading command map..." << std::endl;
                            SimConnect_RequestDataOnSimObject(g_hSimConnectNative, REQUEST_AIRCRAFT_TITLE, DEFINITION_AIRCRAFT_TITLE, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_ONCE);
                        } else {
                            // Find user friendly name for the current file
                            std::string map_friendly_name = COMMANDS_FILENAME;
                            for (const auto& [id, file] : g_settings.map_definitions) {
                                if (file == COMMANDS_FILENAME) {
                                    auto name_it = g_settings.map_names.find(id);
                                    if (name_it != g_settings.map_names.end()) {
                                        map_friendly_name = name_it->second;
                                    }
                                    break;
                                }
                            }

                            if (load_command_map(COMMANDS_FILENAME)) {
                                std::cout << "[SYSTEM] Command map (" << map_friendly_name << ") reloaded successfully." << std::endl;
                            } else {
                                std::cout << "[SYSTEM] Error reloading command map (" << map_friendly_name << ")." << std::endl;
                            }
                        }
                    }
                }
                m_was_pressed = m_is_down;

                // 3. BRIGHTNESS (0-4)
                uint8_t targetVal = 0xFF; 
                if ((GetAsyncKeyState(VK_OEM_5) & 0x8000) || (GetAsyncKeyState('0') & 0x8000)) targetVal = 0x40;
                else if (GetAsyncKeyState('1') & 0x8000) targetVal = 0x30;
                else if (GetAsyncKeyState('2') & 0x8000) targetVal = 0x20;
                else if (GetAsyncKeyState('3') & 0x8000) targetVal = 0x10;
                else if (GetAsyncKeyState('4') & 0x8000) targetVal = 0x00;

                if (targetVal != 0xFF) {
                    send_brightness_to_all(targetVal);
                    {
                        std::lock_guard<std::mutex> lock(g_consoleMutex);
                        std::cout << "[LIGHT] Brightness set to 0x" << std::hex << (int)targetVal << std::dec << "." << std::endl;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(400));
                }
            } else {
                s_was_pressed = false;
                m_was_pressed = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    });
    lightControlThread.detach();

    int result = 42; // Our "Magic Code" for the reconnect
    while (result == 42) {
        result = ble_run_session_scan_until_all_addresses(
            knownDevices_macs,
            g_settings.characteristicUUID,
            on_packet,
            false // is_reconnect = false for the first run            
        );

        if (result == 42) {
            std::cout << "\n[RECONNECT] Restarting BLE system...\n" << std::endl;
            
            // Important: Reset global flag
            g_ble.shuttingDown = false; 
            g_discoveryFinished = false;

            // Short pause for the Windows Bluetooth stack
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    return result; // Only here does the program actually end
}
