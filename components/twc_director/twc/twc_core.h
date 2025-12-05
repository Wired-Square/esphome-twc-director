// twc_core.h
// Core TWC controller: device registry and master mode coordination.
//
// OBSERVER MODE (master_mode = false):
//   - Passively listens to TWC bus traffic
//   - Updates device registry as frames arrive
//   - Does not transmit
//
// MASTER MODE (master_mode = true):
//   - Actively controls peripherals
//   - Sends heartbeats (E0) to allocate current
//   - Requests device info (VIN, serial, meter, version)
//   - Manages session negotiation (E1/E2)
//   - Handles multi-device current allocation with global limit

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "twc_protocol.h"
#include "twc_device.h"

#ifndef TWC_CORE_MAX_DEVICES
#define TWC_CORE_MAX_DEVICES 8
#endif

// =============================================================================
// TYPES
// =============================================================================

typedef struct twc_core_device twc_core_device_t;
typedef struct twc_core twc_core_t;

// TX callback: called when core wants to transmit a raw TWC frame
// (post-checksum, ready for SLIP encoding and wire transmission).
typedef void (*twc_core_tx_callback_t)(const uint8_t *frame,
                                       size_t len,
                                       void *user_data);

// Negotiation callback: called when core updates applied current allocation
// for a peripheral (after reconciling desired vs global limit).
typedef void (*twc_core_negotiation_cb_t)(uint16_t address,
                                          float applied_initial_current_a,
                                          uint8_t session_id,
                                          void *user_data);

// Auto-bind callback: called when core discovers a new peripheral device.
// Application can use this to bind discovered devices to UI slots.
typedef void (*twc_core_autobind_cb_t)(uint16_t address,
                                       twc_mode_t mode,
                                       void *user_data);

// Log levels for diagnostic callback
typedef enum {
  TWC_LOG_ERROR   = 0,
  TWC_LOG_WARNING = 1,
  TWC_LOG_INFO    = 2,
  TWC_LOG_DEBUG   = 3,
  TWC_LOG_VERBOSE = 4
} twc_log_level_t;

// Diagnostic log callback: called when core needs to log diagnostic information
// (e.g., checksum failures, protocol errors, etc.)
typedef void (*twc_core_log_cb_t)(twc_log_level_t level,
                                  const char *message,
                                  void *user_data);

// Per-device state tracked by core
struct twc_core_device {
  // Identity
  uint16_t address;
  bool present;
  
  // Activity timestamps
  uint32_t last_frame_ms;         // Any frame from this device
  uint32_t last_status_ms;        // Last E0/E1/E2 (strong presence)
  uint32_t last_serial_request_ms;

  // Current negotiation
  uint8_t peripheral_session;      // Session ID from peripheral
  float current_available_a;       // What peripheral reports it can supply
  float max_current_a;             // Authoritative upper bound (never changes unless set)
  float desired_initial_current_a; // What operator wants for initial (0x05 frame)
  float desired_session_current_a; // What operator wants for session (0x09 frame)
  float applied_initial_current_a; // What we actually send (after scaling)

  // Pending commands (master mode only)
  bool  pending_initial_current_cmd;
  float last_initial_current_cmd_a;
  bool  pending_session_current_cmd;
  float last_session_current_cmd_a;

  // Edge detection
  bool last_vehicle_connected;

  // Diagnostics
  uint32_t restart_counter;

  // Info probe state machine (master mode)
  uint8_t  info_probe_stage;       // 0-5: VIN_HI/MID/LO, SERIAL, METER, VERSION
  uint32_t last_info_probe_ms;
  uint8_t  e0_since_last_probe;    // Heartbeats since last probe

  // Full device state
  twc_device_t device;
};

// Core controller state
struct twc_core {
  // Device registry
  twc_core_device_t devices[TWC_CORE_MAX_DEVICES];

  // Configuration
  uint16_t master_address;
  float global_max_current_a;
  uint32_t online_timeout_ms;
  bool master_mode;

  // Callbacks
  twc_core_tx_callback_t tx_cb;
  void *tx_user;
  twc_core_negotiation_cb_t negotiation_cb;
  void *negotiation_user;
  twc_core_autobind_cb_t autobind_cb;
  void *autobind_user;
  twc_core_log_cb_t log_cb;
  void *log_user;

  // Master mode TX scheduling
  uint32_t last_e0_heartbeat_ms;
  uint8_t next_e0_device_index;

  // Session management
  bool master_just_enabled;
  uint8_t master_session_id;

  // Startup burst (5x E1 + 4x E2)
  bool startup_burst_active;
  uint8_t startup_burst_e1_sent;
  uint8_t startup_burst_e2_sent;
  uint32_t startup_burst_last_ms;
};

// =============================================================================
// LIFECYCLE & CONFIGURATION
// =============================================================================

// Initialize core. Must be called first.
void twc_core_init(twc_core_t *core);

// Configure online timeout (default: 15000ms)
void twc_core_set_online_timeout(twc_core_t *core, uint32_t timeout_ms);

// Enable/disable master mode
void twc_core_set_master_mode(twc_core_t *core, bool enabled);

// Set master address (default: 0xF00D)
void twc_core_set_master_address(twc_core_t *core, uint16_t address);
uint16_t twc_core_get_master_address(const twc_core_t *core);

// Set global current budget (0 = no limit)
void twc_core_set_global_max_current(twc_core_t *core, float max_current_a);
float twc_core_get_global_max_current(const twc_core_t *core);

// =============================================================================
// CALLBACKS
// =============================================================================

void twc_core_set_tx_callback(twc_core_t *core,
                              twc_core_tx_callback_t cb,
                              void *user_data);

void twc_core_set_negotiation_callback(twc_core_t *core,
                                       twc_core_negotiation_cb_t cb,
                                       void *user_data);

void twc_core_set_autobind_callback(twc_core_t *core,
                                    twc_core_autobind_cb_t cb,
                                    void *user_data);

void twc_core_set_log_callback(twc_core_t *core,
                               twc_core_log_cb_t cb,
                               void *user_data);

// =============================================================================
// RX PATH (observer and master modes)
// =============================================================================

// Process incoming TWC frame
// Returns true if frame was valid (passed checksum), false otherwise
bool twc_core_handle_frame(twc_core_t *core,
                           const uint8_t *frame,
                           size_t len,
                           uint32_t now_ms);

// =============================================================================
// TX PATH (master mode only)
// =============================================================================

// Master mode periodic tick (call regularly, e.g. every 100ms)
void twc_core_master_tick(twc_core_t *core, uint32_t now_ms);

// =============================================================================
// DEVICE QUERY API
// =============================================================================

twc_core_device_t *twc_core_get_device_by_address(twc_core_t *core,
                                                  uint16_t address);

const twc_device_t *twc_core_get_device_const(const twc_core_t *core,
                                              uint16_t address);

bool twc_core_device_present(const twc_core_device_t *dev);

bool twc_core_device_online(const twc_core_t *core,
                            const twc_core_device_t *dev,
                            uint32_t now_ms);

// =============================================================================
// CURRENT ALLOCATION API
// =============================================================================

uint8_t twc_core_get_peripheral_session(const twc_core_device_t *dev);
float twc_core_get_current_available_a(const twc_core_device_t *dev);

void twc_core_set_max_current(twc_core_t *core,
                               uint16_t address,
                               float current_a);

float twc_core_get_max_current(const twc_core_t *core,
                                const twc_core_device_t *dev);

void twc_core_set_desired_initial_current(twc_core_t *core,
                                          uint16_t address,
                                          float current_a);

float twc_core_get_desired_initial_current(const twc_core_t *core,
                                           const twc_core_device_t *dev);

void twc_core_set_desired_session_current(twc_core_t *core,
                                          uint16_t address,
                                          float current_a);

float twc_core_get_desired_session_current(const twc_core_t *core,
                                           const twc_core_device_t *dev);

float twc_core_get_applied_initial_current(const twc_core_t *core,
                                           const twc_core_device_t *dev);

uint32_t twc_core_get_restart_counter(const twc_core_device_t *dev);
