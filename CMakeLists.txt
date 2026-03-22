cmake_minimum_required(VERSION 3.21)

project(msfs_ble_g1000 LANGUAGES CXX)

if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(FATAL_ERROR "x86 build not supported. Reconfigure with x64 toolchain.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable IPO/LTO for Release when available
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    include(CheckIPOSupported)
    check_ipo_supported(RESULT ipo_supported OUTPUT ipo_msg)
    if (ipo_supported)
        message(STATUS "IPO/LTO enabled")
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
    endif()
endif()

# Dependencies
add_subdirectory(libs/simpleble/simpleble)
add_subdirectory(cmake/FSUIPC_WAPI_wrapper)

function(apply_common_settings target)
    if (MSVC AND CMAKE_BUILD_TYPE STREQUAL "Release")
        target_compile_options(${target} PRIVATE /O2 /GL)
        target_link_options(${target} PRIVATE /LTCG)
    endif()
    if (MSVC)
        target_compile_definitions(${target} PRIVATE UNICODE _UNICODE)
    endif()
endfunction()

set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(MSFS_BLE_BRIDGE_SRC "apps/msfs_ble_bridge_main.cpp")
set(BLE_SCANNER_SRC "apps/ble_scanner_main.cpp")
set(BLE_COMMON_SRC "${SRC_DIR}/ble_session.cpp")

include(FetchContent)

FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
)

FetchContent_MakeAvailable(nlohmann_json)

# msfs_ble_bridge target
add_executable(msfs_ble_bridge
    "${MSFS_BLE_BRIDGE_SRC}"
    "${BLE_COMMON_SRC}"
)

# Damit dein Hauptprogramm die Bibliothek findet:
target_link_libraries(msfs_ble_bridge PRIVATE nlohmann_json::nlohmann_json)

target_include_directories(msfs_ble_bridge PRIVATE ${SRC_DIR})
target_link_libraries(msfs_ble_bridge PRIVATE
    simpleble::simpleble
    FSUIPC_WAPI::FSUIPC_WAPI
)
apply_common_settings(msfs_ble_bridge)
install(TARGETS msfs_ble_bridge RUNTIME DESTINATION .)

add_custom_command(TARGET msfs_ble_bridge POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/SHB1000.config"
    "$<TARGET_FILE_DIR:msfs_ble_bridge>/SHB1000.config"
    COMMENT "Kopiere default.bleconfig in das Build-Verzeichnis..."
)
# Kopiere die Config-Dateien in den Build-Ordner
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/SHB1000_known_devices.config"
    "${CMAKE_CURRENT_BINARY_DIR}/SHB1000_known_devices.config"
    COPYONLY
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/SHB1000_commands.map"
    "${CMAKE_CURRENT_BINARY_DIR}/SHB1000_commands.map"
    COPYONLY
)

# ble_scanner target
add_executable(ble_scanner
    "${BLE_SCANNER_SRC}"
    "${BLE_COMMON_SRC}"
)
target_include_directories(ble_scanner      PRIVATE ${SRC_DIR})
target_link_libraries(ble_scanner PRIVATE
    simpleble::simpleble
)
apply_common_settings(ble_scanner)
install(TARGETS ble_scanner RUNTIME DESTINATION .)


