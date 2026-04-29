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

// Configuration of filenames (default)
constexpr const char* CONFIG_FILENAME = "SHB1000.config";             // HARD CODE filename for general settings
std::string KNOWN_DEVICES_FILENAME = "SHB1000_known_devices.config";  // Variable for the configuration of known devices, pre-assigned with DEFAULT
std::string COMMANDS_FILENAME = "SHB1000_commands.map";               // Variable for the configuration of commands, pre-assigned with DEFAULT

bool g_discoveryFinished = false; // Is only set to true when the setup dialog is finished
bool initialBrightnessSet = false;  // If the brightness of the backlight has been initially set
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
    std::string command_map_file;       // Specifies the file of the command mapping (button/dial HEX codes -> MSFS2024 sim commands)
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
};
static GlobalSettings g_settings;
static std::mutex g_consoleMutex;

// Structure for the commands: Section name -> (Hex-Code -> Command)
using CommandMap = std::unordered_map<std::string, std::map<uint8_t, std::string>>;
CommandMap g_commandMaps;

// Stores the active Bluetooth connections to send commands (such as lights)
static std::map<std::string, SimpleBLE::Peripheral> g_activePeripherals;

static WASMIF* wasmPtr = nullptr;

// Buffer for WASM commands for the Sim
static std::array<char, MAX_CALC_CODE_SIZE> ccode{};

nlohmann::json g_master_config_json; // Global holder so we can rewrite aircraft updates without wiping comments
void save_master_config(); // Forward declaration
bool load_command_map(const std::string& filename); // Forward declaration

// --- NATIVE SIMCONNECT FOR A-VARS ---
#include <SimConnect.h>
HANDLE g_hSimConnectNative = NULL;
enum DATA_DEFINE_ID { DEFINITION_AIRCRAFT_TITLE = 1 };
enum DATA_REQUEST_ID { REQUEST_AIRCRAFT_TITLE = 1 };

struct AircraftData {
    char title[256];
};

void CALLBACK MyNativeDispatchProc(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext) {
    switch (pData->dwID) {
        case SIMCONNECT_RECV_ID_SIMOBJECT_DATA: {
            SIMCONNECT_RECV_SIMOBJECT_DATA* pObjData = (SIMCONNECT_RECV_SIMOBJECT_DATA*)pData;
            if (pObjData->dwRequestID == REQUEST_AIRCRAFT_TITLE) {
                AircraftData* pAircraftData = (AircraftData*)&pObjData->dwData;
                std::string title = pAircraftData->title;
                std::string map_id = g_settings.default_map_id;

                auto it = g_settings.aircraft_history.find(title);
                if (it != g_settings.aircraft_history.end()) {
                    map_id = it->second.map_id;
                }

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
                }
                
                load_command_map(COMMANDS_FILENAME);
                
                {
                    std::lock_guard<std::mutex> lock(g_consoleMutex);
                    std::cout << "PRESS 'M' TO RELOAD MAP | 'R' TO RECONNECT | 'S' TO SWAP ROLES | '?' FOR AIRCRAFT | 'Q' TO QUIT" << std::endl;
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
// for the packet handler
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

            std::string temp_command = j["general"].value("command_map_file", "");
            if (!temp_command.empty()) {
                g_settings.command_map_file = temp_command;
                COMMANDS_FILENAME = temp_command; // Update global default
            }
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
        }

        std::cout << "[OK] Master config loaded."<< std::endl;
        std::cout << "[INFO] Device config: " << g_settings.device_map_file << std::endl;
        std::cout << "[INFO] Command config: " << g_settings.command_map_file << std::endl;
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
    std::string line, currentSection;

    if (!file.is_open()) {
        std::cerr << "Error: Could not open " << filename << "!" << std::endl;
        return false;
    }

    while (std::getline(file, line)) {
        // Ignore comments and empty lines
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;

        // Detect section: [Name]
        if (line[0] == '[') {
            // Ignore the "_Commands" part in the section so we are more flexible later (e.g. "PFD_Commands" -> "PFD", "MFD_Commands" -> "MFD", etc.)
            size_t underscorePos = line.find('_');
            if (underscorePos != std::string::npos) {
                currentSection = line.substr(1, underscorePos - 1);
            } else {
                // Fallback: If there is no underscore, take everything up to ']'
                currentSection = line.substr(1, line.find(']') - 1);
            }
            continue;
        }

        // Parse command lines (Format: 4A = "Command")
        size_t delimiter = line.find('=');
        if (delimiter != std::string::npos && !currentSection.empty()) {
            std::string hexStr = line.substr(0, delimiter);
            std::string command = line.substr(delimiter + 1);

            // 1. Helper function for trimming (remove spaces front/back)
            auto trim = [](std::string& s) {
                size_t first = s.find_first_not_of(" \t\r\n");
                if (first == std::string::npos) return;
                size_t last = s.find_last_not_of(" \t\r\n");
                s = s.substr(first, (last - first + 1));
            };

            trim(hexStr);
            trim(command);

            // 2. Remove quotes, if present
            if (command.size() >= 2 && command.front() == '"' && command.back() == '"') {
                command = command.substr(1, command.size() - 2);
                trim(command); // In case there were still spaces inside the quotes
            }

            // 3. Convert Hex-Key to uint8_t
            try {
                uint8_t keyCode = (uint8_t)std::stoul(hexStr, nullptr, 16);
                
                // 4. Save into the map (Key is now a clean MSFS code)
                g_commandMaps[currentSection][keyCode] = command;
            } catch (...) {
                // Simply ignore invalid lines
            }
        }

    }
    std::cout << "[OK] Command mapping successfully loaded." << std::endl;
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

    // 1. Find out which ROLE this MAC currently has
    std::string role = "";
    for (const auto& dev : g_knownDevices) {
        if (dev.address == cleanMac) {
            role = dev.instrument; // e.g., "PFD"
            break;
        }
    }
    if (role.empty()) return;

    // 2. Search the map for the set of this ROLE
    auto itMap = g_commandMaps.find(role); 
    if (itMap != g_commandMaps.end()) {
        uint8_t rawByte = (uint8_t)bytes[0];
        
        auto itCmd = itMap->second.find(rawByte);
        if (itCmd != itMap->second.end()) {
            std::string command = itCmd->second;
            
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
            // Send the command now 1x for slow or 5x for fast turning
            for(int i = 0; i < repetitions; ++i) {
                send_calc_code_safely(command);
            }
        }
    }
}

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
                    g_ble.connected.emplace(mac, ConnectedDevice{p, service_uuid, char_uuid, used_indicate});
                    // 2. Light control (Map for write access)
                    g_activePeripherals.insert_or_assign(mac, p);
                    
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
    std::cout << "PRESS 'M' TO RELOAD MAP | 'R' TO RECONNECT | 'S' TO SWAP ROLES | '?' FOR AIRCRAFT | 'Q' TO QUIT\n" << std::endl;

    while (!g_ble.shuttingDown) {
        if (_kbhit()) {
            char ch = _getch();
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
int main(int argc, char* argv[]) {

    std::unordered_set<std::string> knownDevices_macs;    // List of MACs we loaded from the new JSON config


    // Sets the console to UTF-8 encoding keyboard encoding
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    // Set process priority to elevated
    SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);

    std::cout << "[INFO]: Program start!" << std::endl; 

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
                    std::cout << "[INFO] Native SimConnect connection established for A-Vars." << std::endl;
                    initial_aircraft_check_done = false;
                }
            }
            if (g_hSimConnectNative) {
                SimConnect_CallDispatch(g_hSimConnectNative, MyNativeDispatchProc, NULL);
                
                // Trigger auto load the very first time the plane fully connects
                if (g_discoveryFinished && !initial_aircraft_check_done) {
                    initial_aircraft_check_done = true;
                    std::cout << "\n[SIMCONNECT] Auto-requesting current aircraft to load mapping..." << std::endl;
                    SimConnect_RequestDataOnSimObject(g_hSimConnectNative, REQUEST_AIRCRAFT_TITLE, DEFINITION_AIRCRAFT_TITLE, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_ONCE);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (g_hSimConnectNative) {
            SimConnect_Close(g_hSimConnectNative);
            g_hSimConnectNative = NULL;
        }
    });
    nativeSimConnectThread.detach();

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
            if (foregroundProcessId == myPid || hForeground == hConsole || (hConsole && hForeground == GetParent(hConsole))) {
                
                // 1. ROLE SWITCH (Key 'S')
                bool s_is_down = (GetAsyncKeyState('S') & 0x8000) != 0;
                if (s_is_down && !s_was_pressed) {
                    {
                        // We briefly lock the BLE object so that on_packet does not read at the same time
                        std::lock_guard<std::mutex> lock(g_ble.m);
                        
                        // Step 1: Mark intention for ALL active devices
                        for (auto& dev : g_knownDevices) {
                            if (g_ble.connected.count(dev.address)) {
                                if (dev.instrument == "PFD") {
                                    dev.instrument = "TEMP_MFD";
                                } else if (dev.instrument == "MFD") {
                                    dev.instrument = "TEMP_PFD";
                                }
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

                    // Acoustic feedback & log
                    {
                        std::lock_guard<std::mutex> lock(g_consoleMutex);
                        Beep(800, 150); 
                        std::cout << "[SYSTEM] Role switch performed.\n" << std::endl;
                        update_last_session_snapshot(); // Synchronize snapshot for reconnect
                    }
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