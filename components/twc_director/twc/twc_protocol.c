// twc_protocol.c
//
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

#include "twc_protocol.h"
#include <string.h>  // memcpy, memset

// =============================================================================
// CORE FRAME BUILDING BLOCKS
// =============================================================================

size_t twc_build_frame_header(twc_marker_t marker,
                               twc_cmd_t cmd,
                               uint16_t address,
                               uint8_t *out_header,
                               size_t capacity) {
  if (out_header == NULL || capacity < 4) {
    return 0;
  }

  // SAFETY: Block dangerous commands that write to TWC flash memory.
  // Only block when used with ANNOUNCE marker (0xFC) which triggers flash writes.
  if (twc_cmd_is_dangerous(marker, cmd)) {
    return 0;
  }

  out_header[0] = (uint8_t)marker;
  out_header[1] = (uint8_t)cmd;
  out_header[2] = (uint8_t)((address >> 8) & 0xFF);
  out_header[3] = (uint8_t)(address & 0xFF);

  return 4;
}

size_t twc_build_frame(const uint8_t *header,
                       size_t header_len,
                       const uint8_t *payload,
                       size_t payload_len,
                       uint8_t *out_frame,
                       size_t out_capacity) {
  if (out_frame == NULL || header == NULL) {
    return 0;
  }

  // Validate header length (must be 4 bytes)
  if (header_len != 4) {
    return 0;
  }

  // Calculate total frame size: header + payload + 1 checksum byte
  const size_t total_len = header_len + payload_len + 1;
  if (out_capacity < total_len) {
    return 0;
  }

  // Copy header
  memcpy(out_frame, header, header_len);

  // Copy payload if present
  if (payload != NULL && payload_len > 0) {
    memcpy(out_frame + header_len, payload, payload_len);
  }

  // Compute checksum: sum of all bytes from cmd through end of payload
  // (bytes [1] through [header_len + payload_len - 1])
  uint8_t checksum = 0;
  for (size_t i = 1; i < header_len + payload_len; ++i) {
    checksum = (uint8_t)((checksum + out_frame[i]) & 0xFF);
  }

  // Append checksum
  out_frame[total_len - 1] = checksum;

  return total_len;
}

bool twc_decode_frame(const uint8_t *frame,
                      size_t frame_len,
                      const uint8_t **out_header,
                      const uint8_t **out_payload,
                      size_t *out_payload_len) {
  if (frame == NULL || out_header == NULL || 
      out_payload == NULL || out_payload_len == NULL) {
    return false;
  }

  // Minimum frame: 4-byte header + 1-byte checksum
  if (frame_len < 5) {
    return false;
  }

  // Validate checksum
  uint8_t checksum = 0;
  for (size_t i = 1; i < frame_len - 1; ++i) {
    checksum = (uint8_t)((checksum + frame[i]) & 0xFF);
  }

  if (checksum != frame[frame_len - 1]) {
    return false;
  }

  // Extract header and payload
  *out_header = frame;
  
  const size_t header_len = 4;
  const size_t payload_len = frame_len - header_len - 1;  // -1 for checksum
  
  if (payload_len > 0) {
    *out_payload = frame + header_len;
  } else {
    *out_payload = NULL;
  }
  
  *out_payload_len = payload_len;

  return true;
}

void twc_parse_header(const uint8_t *header,
                      twc_marker_t *out_marker,
                      twc_cmd_t *out_cmd,
                      uint16_t *out_address) {
  if (header == NULL) {
    return;
  }

  if (out_marker != NULL) {
    *out_marker = (twc_marker_t)header[0];
  }

  if (out_cmd != NULL) {
    *out_cmd = (twc_cmd_t)header[1];
  }

  if (out_address != NULL) {
    *out_address = ((uint16_t)header[2] << 8) | (uint16_t)header[3];
  }
}

// =============================================================================
// PAYLOAD BUILDERS
// =============================================================================

size_t twc_build_heartbeat_payload(uint16_t dest_address,
                                    uint8_t charge_state,
                                    uint16_t current_available_centiamps,
                                    uint16_t current_delivered_centiamps,
                                    uint8_t *out_payload,
                                    size_t capacity) {
  const size_t required = 11;
  if (out_payload == NULL || capacity < required) {
    return 0;
  }

  out_payload[0] = (uint8_t)((dest_address >> 8) & 0xFF);
  out_payload[1] = (uint8_t)(dest_address & 0xFF);
  out_payload[2] = charge_state;
  out_payload[3] = (uint8_t)((current_available_centiamps >> 8) & 0xFF);
  out_payload[4] = (uint8_t)(current_available_centiamps & 0xFF);
  out_payload[5] = (uint8_t)((current_delivered_centiamps >> 8) & 0xFF);
  out_payload[6] = (uint8_t)(current_delivered_centiamps & 0xFF);
  
  // Padding
  memset(out_payload + 7, 0x00, 4);

  return required;
}

size_t twc_build_controller_negotiation_payload(uint8_t session_id,
                                                 uint8_t *out_payload,
                                                 size_t capacity) {
  const size_t required = 11;
  if (out_payload == NULL || capacity < required) {
    return 0;
  }

  out_payload[0] = session_id;
  memset(out_payload + 1, 0x00, 10);

  return required;
}

size_t twc_build_peripheral_pause_payload(uint8_t session_id,
                                           uint8_t *out_payload,
                                           size_t capacity) {
  // Same layout as controller negotiation
  return twc_build_controller_negotiation_payload(session_id, out_payload, capacity);
}

size_t twc_build_simple_request_payload(uint16_t dest_address,
                                         uint8_t *out_payload,
                                         size_t capacity) {
  const size_t required = 11;
  if (out_payload == NULL || capacity < required) {
    return 0;
  }

  out_payload[0] = (uint8_t)((dest_address >> 8) & 0xFF);
  out_payload[1] = (uint8_t)(dest_address & 0xFF);
  memset(out_payload + 2, 0x00, 9);

  return required;
}

// =============================================================================
// PAYLOAD DECODERS
// =============================================================================

bool twc_decode_heartbeat_payload(const uint8_t *payload,
                                   size_t payload_len,
                                   twc_heartbeat_data_t *out) {
  if (payload == NULL || out == NULL || payload_len < 7) {
    return false;
  }

  out->dest_address = ((uint16_t)payload[0] << 8) | (uint16_t)payload[1];
  out->charge_state = payload[2];
  out->current_available_centiamps = ((uint16_t)payload[3] << 8) | (uint16_t)payload[4];
  out->current_delivered_centiamps = ((uint16_t)payload[5] << 8) | (uint16_t)payload[6];

  return true;
}

bool twc_decode_peripheral_negotiation_payload(const uint8_t *payload,
                                                size_t payload_len,
                                                twc_peripheral_negotiation_data_t *out) {
  if (payload == NULL || out == NULL || payload_len < 3) {
    return false;
  }

  out->session_id = payload[0];
  out->current_available_centiamps = ((uint16_t)payload[1] << 8) | (uint16_t)payload[2];

  return true;
}

bool twc_decode_meter_payload(const uint8_t *payload,
                               size_t payload_len,
                               twc_meter_data_t *out) {
  if (payload == NULL || out == NULL || payload_len < 15) {
    return false;
  }

  // Big-endian 32-bit total energy in kWh
  uint32_t total_kwh = ((uint32_t)payload[0] << 24) |
                       ((uint32_t)payload[1] << 16) |
                       ((uint32_t)payload[2] << 8)  |
                       ((uint32_t)payload[3]);

  out->total_energy_kwh = (float)total_kwh;

  // Voltages (direct values)
  out->phase_l2_v = (float)payload[4];
  out->phase_l1_v = (float)payload[5];
  out->phase_l3_v = (float)payload[6];

  // Currents (0.5A units)
  out->phase_l2_a = (float)payload[9] / 2.0f;
  out->phase_l1_a = (float)payload[10] / 2.0f;
  out->phase_l3_a = (float)payload[11] / 2.0f;

  return true;
}

bool twc_decode_version_payload(const uint8_t *payload,
                                 size_t payload_len,
                                 twc_version_data_t *out) {
  if (payload == NULL || out == NULL || payload_len < 4) {
    return false;
  }

  out->major = payload[0];
  out->minor = payload[1];
  out->patch = payload[2];
  out->build = payload[3];

  return true;
}

bool twc_decode_serial_payload(const uint8_t *payload,
                                size_t payload_len,
                                twc_serial_data_t *out) {
  if (payload == NULL || out == NULL || payload_len == 0) {
    return false;
  }

  size_t out_idx = 0;
  for (size_t i = 0; i < payload_len && out_idx + 1 < sizeof(out->value); ++i) {
    uint8_t b = payload[i];

    // Stop at NUL/padding
    if (b == 0x00) {
      break;
    }

    // Require printable ASCII
    if (b < 0x20 || b > 0x7E) {
      break;
    }

    out->value[out_idx++] = (char)b;
  }

  out->value[out_idx] = '\0';

  return (out_idx > 0);
}

bool twc_decode_vin_payload(const uint8_t *payload,
                             size_t payload_len,
                             twc_cmd_t cmd,
                             twc_vin_data_t *out) {
  if (payload == NULL || out == NULL || payload_len == 0) {
    return false;
  }

  // Determine chunk index from command
  switch (cmd) {
    case TWC_CMD_VIN_HI:
      out->chunk_index = 0;
      break;
    case TWC_CMD_VIN_MID:
      out->chunk_index = 1;
      break;
    case TWC_CMD_VIN_LO:
      out->chunk_index = 2;
      break;
    default:
      return false;
  }

  out->has_text = false;
  size_t out_idx = 0;

  for (size_t i = 0; i < payload_len && out_idx + 1 < sizeof(out->value); ++i) {
    uint8_t b = payload[i];

    if (b == 0x00) {
      break;
    }

    if (b < 0x20 || b > 0x7E) {
      break;
    }

    out->value[out_idx++] = (char)b;
  }

  out->value[out_idx] = '\0';

  if (out_idx > 0) {
    out->has_text = true;
  }

  return true;
}

// =============================================================================
// CONVENIENCE WRAPPERS
// =============================================================================

size_t twc_build_heartbeat_frame(uint16_t master_address,
                                  uint16_t dest_address,
                                  uint8_t charge_state,
                                  uint16_t current_available_centiamps,
                                  uint16_t current_delivered_centiamps,
                                  uint8_t *out_frame,
                                  size_t capacity) {
  uint8_t header[4];
  uint8_t payload[11];

  if (twc_build_frame_header(TWC_MARKER_REQUEST, TWC_CMD_HEARTBEAT,
                              master_address, header, sizeof(header)) != 4) {
    return 0;
  }

  size_t payload_len = twc_build_heartbeat_payload(dest_address, charge_state,
                                                    current_available_centiamps,
                                                    current_delivered_centiamps,
                                                    payload, sizeof(payload));
  if (payload_len == 0) {
    return 0;
  }

  return twc_build_frame(header, 4, payload, payload_len, out_frame, capacity);
}

size_t twc_build_controller_negotiation_frame(uint16_t master_address,
                                               uint8_t session_id,
                                               uint8_t *out_frame,
                                               size_t capacity) {
  uint8_t header[4];
  uint8_t payload[11];

  if (twc_build_frame_header(TWC_MARKER_ANNOUNCE, TWC_CMD_CONTROLLER_NEGOTIATION,
                              master_address, header, sizeof(header)) != 4) {
    return 0;
  }

  size_t payload_len = twc_build_controller_negotiation_payload(session_id,
                                                                 payload,
                                                                 sizeof(payload));
  if (payload_len == 0) {
    return 0;
  }

  return twc_build_frame(header, 4, payload, payload_len, out_frame, capacity);
}

size_t twc_build_peripheral_pause_frame(uint16_t master_address,
                                         uint8_t session_id,
                                         uint8_t *out_frame,
                                         size_t capacity) {
  uint8_t header[4];
  uint8_t payload[11];

  if (twc_build_frame_header(TWC_MARKER_REQUEST, TWC_CMD_PERIPHERAL_NEGOTIATION,
                              master_address, header, sizeof(header)) != 4) {
    return 0;
  }

  size_t payload_len = twc_build_peripheral_pause_payload(session_id,
                                                           payload,
                                                           sizeof(payload));
  if (payload_len == 0) {
    return 0;
  }

  return twc_build_frame(header, 4, payload, payload_len, out_frame, capacity);
}

size_t twc_build_request_frame(uint16_t master_address,
                                uint16_t dest_address,
                                twc_cmd_t cmd,
                                uint8_t *out_frame,
                                size_t capacity) {
  uint8_t header[4];
  uint8_t payload[11];

  if (twc_build_frame_header(TWC_MARKER_REQUEST, cmd,
                              master_address, header, sizeof(header)) != 4) {
    return 0;
  }

  size_t payload_len = twc_build_simple_request_payload(dest_address,
                                                         payload,
                                                         sizeof(payload));
  if (payload_len == 0) {
    return 0;
  }

  return twc_build_frame(header, 4, payload, payload_len, out_frame, capacity);
}

size_t twc_build_contactor_frame(uint16_t master_address,
                                  uint16_t dest_address,
                                  twc_cmd_t cmd,
                                  uint8_t *out_frame,
                                  size_t capacity) {
  uint8_t header[4];
  uint8_t payload[11];

  // Contactor commands use TWC_MARKER_ANNOUNCE (0xFC) per protocol spec
  if (twc_build_frame_header(TWC_MARKER_ANNOUNCE, cmd,
                              master_address, header, sizeof(header)) != 4) {
    return 0;
  }

  size_t payload_len = twc_build_simple_request_payload(dest_address,
                                                         payload,
                                                         sizeof(payload));
  if (payload_len == 0) {
    return 0;
  }

  return twc_build_frame(header, 4, payload, payload_len, out_frame, capacity);
}

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

const char *twc_charge_state_to_string(twc_charge_state_t state) {
  switch (state) {
    case TWC_HB_READY:                return "Ready";
    case TWC_HB_CHARGING:             return "Charging";
    case TWC_HB_ERROR:                return "Error";
    case TWC_HB_WAITING:              return "Waiting";
    case TWC_HB_NEGOTIATING:          return "Negotiating";
    case TWC_HB_MAX_CHARGE:           return "Max Charge";
    case TWC_HB_ACK_INCREASE_CURRENT: return "Increase Current OK";
    case TWC_HB_ACK_DECREASE_CURRENT: return "Decrease Current OK";
    case TWC_HB_CHARGE_STARTED:       return "Charge Started";
    case TWC_HB_SETTING_LIMIT:        return "Setting Limit";
    case TWC_HB_ADJUSTMENT_COMPLETE:  return "Adjustment Complete";
    case TWC_HB_UNKNOWN:
    default:                          return "Unknown";
  }
}
