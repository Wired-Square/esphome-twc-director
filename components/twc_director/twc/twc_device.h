// twc_device.h
// Per-device TWC state and metric representation.
//
// Responsibilities:
//   - Store per-device state (mode, VIN, meter, identification)
//   - Update state from decoded frame payloads
//   - Provide accessors for higher layers

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "twc_protocol.h"

// Logical role of the EVSE on the TWC bus, derived from protocol frames.
// This does NOT encode whether *we* are the active master.
typedef enum {
  TWC_MODE_UNKNOWN = 0,
  TWC_MODE_MASTER,
  TWC_MODE_FOREIGN_MASTER,
  TWC_MODE_UNCONF_PERIPHERAL,     // Seen E2 but not yet claimed
  TWC_MODE_PERIPHERAL,            // Claimed and responding
  TWC_MODE_FOREIGN_PERIPHERAL,    // Responding to another master
} twc_mode_t;

// Per-device state container
typedef struct twc_device {
  uint16_t id;        // Logical device identifier (TWC address)
  twc_mode_t mode;    // Current role on bus

  // Binary state
  bool contactor_closed;
  bool vehicle_connected;   // Derived from complete VIN presence

  // Electrical measurements (from EB meter frames)
  float phase_a_current_a;
  float phase_a_voltage_v;
  float phase_b_current_a;
  float phase_b_voltage_v;
  float phase_c_current_a;
  float phase_c_voltage_v;
  float total_energy_kwh;
  float session_energy_kwh;

  // Session energy tracking
  float session_energy_baseline_kwh;
  bool session_active;

  // Current limits (optional, set by controller)
  float initial_current_limit_a;
  float session_current_limit_a;

  // Negotiation state (from E0/E2 frames)
  uint8_t peripheral_session;
  float   current_available_a;

  int status_code;

  // Identification (from EC/ED frames)
  char serial_number[32];
  char software_version[16];

  // VIN assembly (from EE/EF/F1 frames)
  char vehicle_vin[32];
  char vin_chunks[3][8];      // [0]=high, [1]=mid, [2]=low
  uint8_t vin_chunks_present; // Bitmask: bit 0=high, 1=mid, 2=low
} twc_device_t;

// =============================================================================
// LIFECYCLE
// =============================================================================

// Initialize device struct. All metrics start at zero/false.
void twc_device_init(twc_device_t *dev, uint16_t id);

// =============================================================================
// FRAME UPDATES (called by twc_core)
// =============================================================================

// Update device state from a decoded frame.
// Called by twc_core_handle_frame() after decoding.
// Handles:
//   - Mode detection (E0/E1/E2)
//   - Negotiation state (E0/E2)
//   - Meter updates (EB)
//   - Identification (EC/ED)
//   - VIN assembly (EE/EF/F1)
void twc_device_update_from_frame(twc_device_t *dev,
                                   const uint8_t *header,
                                   const uint8_t *payload,
                                   size_t payload_len,
                                   twc_cmd_t cmd,
                                   uint32_t now_ms);

// =============================================================================
// STATE ACCESSORS
// =============================================================================

bool       twc_device_get_contactor_closed(const twc_device_t *dev);
bool       twc_device_get_vehicle_connected(const twc_device_t *dev);
twc_mode_t twc_device_get_mode(const twc_device_t *dev);
void       twc_device_set_mode(twc_device_t *dev, twc_mode_t mode);

// =============================================================================
// ELECTRICAL METRICS
// =============================================================================

float twc_device_get_phase_a_current_a(const twc_device_t *dev);
float twc_device_get_phase_a_voltage_v(const twc_device_t *dev);
float twc_device_get_phase_b_current_a(const twc_device_t *dev);
float twc_device_get_phase_b_voltage_v(const twc_device_t *dev);
float twc_device_get_phase_c_current_a(const twc_device_t *dev);
float twc_device_get_phase_c_voltage_v(const twc_device_t *dev);
float twc_device_get_total_energy_kwh(const twc_device_t *dev);
float twc_device_get_session_energy_kwh(const twc_device_t *dev);

// =============================================================================
// METER UPDATES (called internally by update_from_frame)
// =============================================================================

void twc_device_set_meter_values(twc_device_t *dev,
                                 float total_energy_kwh,
                                 float phase_l1_v, float phase_l2_v, float phase_l3_v,
                                 float phase_l1_a, float phase_l2_a, float phase_l3_a);

// =============================================================================
// CURRENT LIMITS (optional, controller-side only)
// =============================================================================

void twc_device_set_initial_current_a(twc_device_t *dev, uint8_t amps);
void twc_device_set_session_current_a(twc_device_t *dev, uint8_t amps);

// =============================================================================
// NEGOTIATION STATE
// =============================================================================

uint8_t twc_device_get_peripheral_session(const twc_device_t *dev);
void    twc_device_set_peripheral_session(twc_device_t *dev, uint8_t session);
float   twc_device_get_current_available_a(const twc_device_t *dev);
void    twc_device_set_current_available_a(twc_device_t *dev, float amps);

// =============================================================================
// IDENTIFICATION
// =============================================================================

const char *twc_device_get_serial_number(const twc_device_t *dev);
void        twc_device_set_serial_number(twc_device_t *dev, const char *serial);
const char *twc_device_get_software_version(const twc_device_t *dev);
void        twc_device_set_software_version(twc_device_t *dev, const char *version);

// =============================================================================
// VIN ASSEMBLY
// =============================================================================

// VIN arrives in three chunks (EE/EF/F1). Each call updates one chunk
// and rebuilds the full VIN. When all three chunks are present and non-empty,
// vehicle_connected is set true.
void twc_device_set_vehicle_vin_chunk(twc_device_t *dev,
                                      uint8_t chunk_index,
                                      const char *chunk);

// Clear all VIN state and mark vehicle as disconnected.
void twc_device_clear_vehicle_vin(twc_device_t *dev);

// Get assembled VIN string. Returns NULL if no complete VIN present.
const char *twc_device_get_vehicle_vin(const twc_device_t *dev);

// =============================================================================
// STATUS CODE
// =============================================================================

int twc_device_get_status_code(const twc_device_t *dev);
