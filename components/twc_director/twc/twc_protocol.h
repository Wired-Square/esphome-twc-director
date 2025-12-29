// twc_protocol.h//
// Gen2 TWC on-wire protocol helpers (post-SLIP).
//
// This module is responsible for:
//   - Defining the on-wire marker/cmd enums and charge_state values
//   - Building and parsing the standard 4-byte TWC frame header
//   - Assembling full frames (header + payload + checksum) after SLIP
//   - Validating and decoding frames back into header + payload views
//   - Encoding/decoding the payload formats for common commands (E0/E1/E2,
//     METER, VERSION, SERIAL, VIN)
//
// It does not know about higher-level controller or device state. Callers
// construct payloads using the payload helpers (or directly), then use
// twc_build_frame() to get a complete on-wire frame ready for SLIP and
// transmission. On the receive path, callers use twc_decode_frame() to
// validate the checksum and split header/payload, then pass the payload
// into the relevant decode helpers.

/* Building a heartbeat frame the new way:
uint8_t frame[16];
size_t len = twc_build_heartbeat_frame(0xF00D, 0x1234, 
                                       TWC_HB_CHARGING, 
                                       3200, 3000, 
                                       frame, sizeof(frame));

// Building a custom frame:
uint8_t header[4], payload[11], frame[16];
twc_build_frame_header(TWC_MARKER_REQUEST, TWC_CMD_VERSION, 0xF00D, header, 4);
twc_build_simple_request_payload(0x1234, payload, 11);
size_t len = twc_build_frame(header, 4, payload, 11, frame, 16);

// Decoding:
const uint8_t *header, *payload;
size_t payload_len;
if (twc_decode_frame(rx_frame, rx_len, &header, &payload, &payload_len)) {
  twc_marker_t marker;
  twc_cmd_t cmd;
  twc_parse_header(header, &marker, &cmd, NULL);
  
  if (cmd == TWC_CMD_METER) {
    twc_meter_data_t meter;
    twc_decode_meter_payload(payload, payload_len, &meter);
    // Use meter data...
  }
}
*/

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// On-wire TWC markers (direction) and commands.
// Keeping these here ensures all protocol users share a single source of truth.
//
// Marker semantics:
//   0xFB (REQUEST)  - Query from master to peripheral, expects 0xFD response
//   0xFC (ANNOUNCE) - Command/broadcast from master, no response expected
//                     WARNING: Some 0xFC commands WRITE TO FLASH on the TWC!
//   0xFD (RESPONSE) - Response from peripheral to master
typedef enum {
  TWC_MARKER_REQUEST  = 0xFB,  // Request from controller/master to a peripheral
  TWC_MARKER_RESPONSE = 0xFD,  // Response from peripheral to controller/master
  TWC_MARKER_ANNOUNCE = 0xFC,  // Controller/master broadcast/announce (e.g. E1 session)
                               // WARNING: Some commands with this marker write to TWC flash!
} twc_marker_t;

typedef enum {
  // ==========================================================================
  // DANGEROUS COMMANDS - These write to TWC flash memory!
  // DO NOT USE unless you know exactly what you're doing.
  // These are defined here so they can be explicitly blocked.
  // ==========================================================================
  TWC_CMD_WRITE_ID_DATE           = 0x19,  // DANGEROUS: Writes ID/date to flash
  TWC_CMD_WRITE_MODEL_NUMBER      = 0x1A,  // DANGEROUS: Writes model number to flash

  // ==========================================================================
  // Idle/keepalive
  // ==========================================================================
  TWC_CMD_IDLE                    = 0x1D,  // Idle message (no-op keepalive)

  // ==========================================================================
  // Contactor control (used with 0xFC marker)
  // ==========================================================================
  TWC_CMD_CLOSE_CONTACTORS        = 0xB1,  // Start charging (close contactors)
  TWC_CMD_OPEN_CONTACTORS         = 0xB2,  // Stop charging (open contactors)
  TWC_CMD_PLUG_STATE              = 0xB4,  // Query plug state

  // ==========================================================================
  // Heartbeat and negotiation
  // ==========================================================================
  TWC_CMD_HEARTBEAT               = 0xE0,  // Primary/secondary heartbeat
  TWC_CMD_CONTROLLER_NEGOTIATION  = 0xE1,  // Controller presence announcement
  TWC_CMD_PERIPHERAL_NEGOTIATION  = 0xE2,  // Peripheral presence/negotiation

  // ==========================================================================
  // Information queries (used with 0xFB marker, responses on 0xFD)
  // ==========================================================================
  TWC_CMD_METER                   = 0xEB,  // Power/energy meter data
  TWC_CMD_VERSION                 = 0xEC,  // Firmware version (extended)
  TWC_CMD_SERIAL                  = 0xED,  // Serial number
  TWC_CMD_VIN_HI                  = 0xEE,  // Vehicle VIN (first segment)
  TWC_CMD_VIN_MID                 = 0xEF,  // Vehicle VIN (middle segment)
  TWC_CMD_VIN_LO                  = 0xF1,  // Vehicle VIN (last segment)
} twc_cmd_t;

// =============================================================================
// COMMAND SAFETY HELPERS
// =============================================================================

// Check if a command is dangerous (writes to TWC flash)
// Only dangerous when used with TWC_MARKER_ANNOUNCE (0xFC) - that marker
// triggers flash writes on the TWC. With other markers these are safe queries.
static inline bool twc_cmd_is_dangerous(twc_marker_t marker, twc_cmd_t cmd) {
  if (marker != TWC_MARKER_ANNOUNCE) {
    return false;  // Only 0xFC marker triggers flash writes
  }
  return (cmd == TWC_CMD_WRITE_ID_DATE || cmd == TWC_CMD_WRITE_MODEL_NUMBER);
}

// E0 frame charge_state
typedef enum {
  TWC_HB_READY                 = 0x00,
  TWC_HB_CHARGING              = 0x01,
  TWC_HB_ERROR                 = 0x02,
  TWC_HB_WAITING               = 0x03,
  TWC_HB_NEGOTIATING           = 0x04,
  TWC_HB_MAX_CHARGE            = 0x05,
  TWC_HB_ACK_INCREASE_CURRENT  = 0x06,
  TWC_HB_ACK_DECREASE_CURRENT  = 0x07,
  TWC_HB_CHARGE_STARTED        = 0x08,
  TWC_HB_SETTING_LIMIT         = 0x09,
  TWC_HB_ADJUSTMENT_COMPLETE   = 0x0A,
  TWC_HB_UNKNOWN               = 0xFF,
} twc_charge_state_t;

// =============================================================================
// CORE FRAME BUILDING BLOCKS
// =============================================================================

// Standard TWC frame header (4 bytes)
typedef struct {
  uint8_t  marker;   // TWC_MARKER_*
  uint8_t  cmd;      // TWC_CMD_*
  uint16_t address;  // Source address (for master) or destination (varies by context)
} twc_frame_header_t;

// Build a standard 4-byte TWC frame header.
// Since masters send most frames, 'address' is typically the master address.
// Returns number of bytes written (always 4 on success).
size_t twc_build_frame_header(twc_marker_t marker,
                               twc_cmd_t cmd,
                               uint16_t address,
                               uint8_t *out_header,
                               size_t capacity);

// Assemble a complete TWC frame from header + payload + compute checksum.
//
// Frame layout:
//   [0..3]              : header (4 bytes)
//   [4..4+payload_len-1]: payload
//   [4+payload_len]     : checksum (sum of bytes [1..4+payload_len-1] & 0xFF)
//
// Returns total frame length on success, 0 on failure.
size_t twc_build_frame(const uint8_t *header,
                       size_t header_len,
                       const uint8_t *payload,
                       size_t payload_len,
                       uint8_t *out_frame,
                       size_t out_capacity);

// Decode a complete TWC frame into header + payload + validate checksum.
//
// On success:
//   - out_header points to the 4-byte header within frame
//   - out_payload points to the payload bytes within frame
//   - out_payload_len is set to the payload length (excludes checksum)
//   - checksum is validated
//
// Returns true if frame is valid and checksum matches.
bool twc_decode_frame(const uint8_t *frame,
                      size_t frame_len,
                      const uint8_t **out_header,
                      const uint8_t **out_payload,
                      size_t *out_payload_len);

// Extract fields from a 4-byte header
void twc_parse_header(const uint8_t *header,
                      twc_marker_t *out_marker,
                      twc_cmd_t *out_cmd,
                      uint16_t *out_address);

// =============================================================================
// PAYLOAD BUILDERS (Master/Controller -> Peripheral)
// =============================================================================
// These functions construct ONLY the payload portion.
// Callers use twc_build_frame() to assemble the complete message.

// Build HEARTBEAT (E0) payload for current allocation.
//
// Payload layout (11 bytes):
//   [0..1] : destination address (peripheral)
//   [2]    : charge_state (0x00=ready, 0x05=set initial current, 0x09=set session current)
//   [3..4] : current_available_centiamps (big-endian)
//   [5..6] : current_delivered_centiamps (big-endian)
//   [7..10]: padding (0x00)
//
// Returns payload length on success (11), 0 on failure.
size_t twc_build_heartbeat_payload(uint16_t dest_address,
                                    uint8_t charge_state,
                                    uint16_t current_available_centiamps,
                                    uint16_t current_delivered_centiamps,
                                    uint8_t *out_payload,
                                    size_t capacity);

// Build CONTROLLER_NEGOTIATION (E1) payload for session announce.
//
// Payload layout (11 bytes):
//   [0]    : session_id
//   [1..10]: padding (0x00)
//
// Returns payload length on success (11), 0 on failure.
size_t twc_build_controller_negotiation_payload(uint8_t session_id,
                                                 uint8_t *out_payload,
                                                 size_t capacity);

// Build PERIPHERAL_NEGOTIATION (E2) payload for session pause.
//
// Payload layout (11 bytes):
//   [0]    : session_id
//   [1..10]: padding (0x00)
//
// Returns payload length on success (11), 0 on failure.
size_t twc_build_peripheral_pause_payload(uint8_t session_id,
                                           uint8_t *out_payload,
                                           size_t capacity);

// Build request payload for VERSION/SERIAL/VIN/METER commands.
//
// Payload layout (11 bytes):
//   [0..1] : destination address (peripheral)
//   [2..10]: padding (0x00)
//
// Returns payload length on success (11), 0 on failure.
size_t twc_build_simple_request_payload(uint16_t dest_address,
                                         uint8_t *out_payload,
                                         size_t capacity);

// =============================================================================
// PAYLOAD DECODERS (Peripheral -> Master/Controller)
// =============================================================================
// These functions parse ONLY the payload portion.
// Callers use twc_decode_frame() to extract header and payload first.

// Decode HEARTBEAT (E0) payload from peripheral.
//
// Payload layout (7+ bytes):
//   [0..1] : destination address
//   [2]    : charge_state
//   [3..4] : current_available_centiamps
//   [5..6] : current_delivered_centiamps
//   [7+]   : optional additional data
//
// Returns true on success.
typedef struct {
  uint16_t dest_address;
  uint8_t  charge_state;
  uint16_t current_available_centiamps;
  uint16_t current_delivered_centiamps;
} twc_heartbeat_data_t;

bool twc_decode_heartbeat_payload(const uint8_t *payload,
                                   size_t payload_len,
                                   twc_heartbeat_data_t *out);

// Decode PERIPHERAL_NEGOTIATION (E2) payload.
//
// Payload layout (3+ bytes):
//   [0]    : session_id
//   [1..2] : current_available_centiamps
//   [3+]   : padding
//
// Returns true on success.
typedef struct {
  uint8_t  session_id;
  uint16_t current_available_centiamps;
} twc_peripheral_negotiation_data_t;

bool twc_decode_peripheral_negotiation_payload(const uint8_t *payload,
                                                size_t payload_len,
                                                twc_peripheral_negotiation_data_t *out);

// Decode METER (EB) payload.
//
// Payload layout (15 bytes):
//   [0..3]  : total_energy_kwh (big-endian uint32)
//   [4]     : phase_l2_v
//   [5]     : phase_l1_v
//   [6]     : phase_l3_v
//   [7..8]  : separator (ignored)
//   [9]     : phase_l2_i (units: 0.5A)
//   [10]    : phase_l1_i (units: 0.5A)
//   [11]    : phase_l3_i (units: 0.5A)
//   [12..14]: padding
//
// Returns true on success.
typedef struct {
  float total_energy_kwh;
  float phase_l1_v;
  float phase_l2_v;
  float phase_l3_v;
  float phase_l1_a;
  float phase_l2_a;
  float phase_l3_a;
} twc_meter_data_t;

bool twc_decode_meter_payload(const uint8_t *payload,
                               size_t payload_len,
                               twc_meter_data_t *out);

// Decode VERSION (EC) payload.
//
// Payload layout (4+ bytes):
//   [0] : major
//   [1] : minor
//   [2] : patch
//   [3] : build
//   [4+]: padding
//
// Returns true on success.
typedef struct {
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
  uint8_t build;
} twc_version_data_t;

bool twc_decode_version_payload(const uint8_t *payload,
                                 size_t payload_len,
                                 twc_version_data_t *out);

// Decode SERIAL (ED) payload.
//
// Payload: variable-length printable ASCII, NUL or 0x00 terminated.
// Returns true if valid serial found, false otherwise.
typedef struct {
  char value[32];  // NUL-terminated
} twc_serial_data_t;

bool twc_decode_serial_payload(const uint8_t *payload,
                                size_t payload_len,
                                twc_serial_data_t *out);

// Decode VIN (EE/EF/F1) payload.
//
// Payload: variable-length printable ASCII, NUL or 0x00 terminated.
// Returns true on success. has_text=false indicates VIN reset (all zeros).
typedef struct {
  uint8_t chunk_index;  // 0=high, 1=mid, 2=low (derived from cmd)
  bool    has_text;
  char    value[8];     // NUL-terminated
} twc_vin_data_t;

bool twc_decode_vin_payload(const uint8_t *payload,
                             size_t payload_len,
                             twc_cmd_t cmd,
                             twc_vin_data_t *out);

// =============================================================================
// CONVENIENCE WRAPPERS (Full Frame Builders)
// =============================================================================
// These combine header + payload building for common messages.

// Build complete HEARTBEAT (E0) frame from master to peripheral.
size_t twc_build_heartbeat_frame(uint16_t master_address,
                                  uint16_t dest_address,
                                  uint8_t charge_state,
                                  uint16_t current_available_centiamps,
                                  uint16_t current_delivered_centiamps,
                                  uint8_t *out_frame,
                                  size_t capacity);

// Build complete CONTROLLER_NEGOTIATION (E1) broadcast frame.
size_t twc_build_controller_negotiation_frame(uint16_t master_address,
                                               uint8_t session_id,
                                               uint8_t *out_frame,
                                               size_t capacity);

// Build complete PERIPHERAL_NEGOTIATION (E2) pause frame.
size_t twc_build_peripheral_pause_frame(uint16_t master_address,
                                         uint8_t session_id,
                                         uint8_t *out_frame,
                                         size_t capacity);

// Build complete request frames (VERSION/SERIAL/VIN_*/METER).
size_t twc_build_request_frame(uint16_t master_address,
                                uint16_t dest_address,
                                twc_cmd_t cmd,
                                uint8_t *out_frame,
                                size_t capacity);

// Build complete contactor control frame (CLOSE_CONTACTORS/OPEN_CONTACTORS).
// Uses TWC_MARKER_ANNOUNCE (0xFC) as per protocol spec.
size_t twc_build_contactor_frame(uint16_t master_address,
                                  uint16_t dest_address,
                                  twc_cmd_t cmd,
                                  uint8_t *out_frame,
                                  size_t capacity);

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

// Convert charge state enum to human-readable string
const char *twc_charge_state_to_string(twc_charge_state_t state);
