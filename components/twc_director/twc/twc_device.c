// twc_device.c
// Per-device TWC state and metric handling.

#include "twc_device.h"
#include "twc_protocol.h"

#include <string.h>
#include <stdio.h>

// External: get configured master address for role detection
extern uint16_t twc_controller_get_master_address(void);

// Forward declarations
static void rebuild_vin_(twc_device_t *dev);
static void update_mode_from_frame_(twc_device_t *dev,
                                    const uint8_t *header,
                                    const uint8_t *payload,
                                    size_t payload_len,
                                    twc_cmd_t cmd);

// =============================================================================
// LIFECYCLE
// =============================================================================

void twc_device_init(twc_device_t *dev, uint16_t id) {
  if (!dev) return;
  
  memset(dev, 0, sizeof(*dev));
  dev->id = id;
  dev->mode = TWC_MODE_UNKNOWN;
}

// =============================================================================
// FRAME UPDATES
// =============================================================================

void twc_device_update_from_frame(twc_device_t *dev,
                                   const uint8_t *header,
                                   const uint8_t *payload,
                                   size_t payload_len,
                                   twc_cmd_t cmd,
                                   uint32_t now_ms) {
  if (!dev || !header) return;
  
  // Update mode first (may affect how we interpret other fields)
  update_mode_from_frame_(dev, header, payload, payload_len, cmd);
  
  // Route to specific handlers based on command
  switch (cmd) {
    case TWC_CMD_HEARTBEAT: {
      // E0: Extract current_available and charge_state from FD E0 responses
      twc_marker_t marker;
      twc_parse_header(header, &marker, NULL, NULL);

      if (marker == TWC_MARKER_RESPONSE && payload && payload_len >= 5) {
        twc_heartbeat_data_t hb;
        if (twc_decode_heartbeat_payload(payload, payload_len, &hb)) {
          twc_device_set_current_available_a(dev,
              (float)hb.current_available_centiamps / 100.0f);
          // Store charge_state (status byte) from the EVSE
          dev->status_code = hb.charge_state;
        }
      }
      break;
    }
    
    case TWC_CMD_PERIPHERAL_NEGOTIATION: {
      // E2: Peripheral negotiation
      twc_peripheral_negotiation_data_t pn;
      if (twc_decode_peripheral_negotiation_payload(payload, payload_len, &pn)) {
        twc_device_set_peripheral_session(dev, pn.session_id);
        twc_device_set_current_available_a(dev,
            (float)pn.current_available_centiamps / 100.0f);
      }
      break;
    }
    
    case TWC_CMD_METER: {
      // EB: Meter data
      twc_meter_data_t meter;
      if (twc_decode_meter_payload(payload, payload_len, &meter)) {
        twc_device_set_meter_values(dev,
                                    meter.total_energy_kwh,
                                    meter.phase_l1_v, meter.phase_l2_v, meter.phase_l3_v,
                                    meter.phase_l1_a, meter.phase_l2_a, meter.phase_l3_a);
      }
      break;
    }
    
    case TWC_CMD_VERSION: {
      // EC: Firmware version
      twc_version_data_t ver;
      if (twc_decode_version_payload(payload, payload_len, &ver)) {
        char ver_str[16];
        snprintf(ver_str, sizeof(ver_str), "%u.%u.%u.%u",
                 (unsigned)ver.major, (unsigned)ver.minor,
                 (unsigned)ver.patch, (unsigned)ver.build);
        twc_device_set_software_version(dev, ver_str);
      }
      break;
    }
    
    case TWC_CMD_SERIAL: {
      // ED: Serial number
      twc_serial_data_t serial;
      if (twc_decode_serial_payload(payload, payload_len, &serial)) {
        if (serial.value[0] != '\0') {
          twc_device_set_serial_number(dev, serial.value);
        }
      }
      break;
    }
    
    case TWC_CMD_VIN_HI:
    case TWC_CMD_VIN_MID:
    case TWC_CMD_VIN_LO: {
      // EE/EF/F1: VIN chunks
      twc_vin_data_t vin;
      if (twc_decode_vin_payload(payload, payload_len, cmd, &vin)) {
        if (!vin.has_text) {
          // All-zero VIN = vehicle disconnected
          twc_device_clear_vehicle_vin(dev);
        } else {
          // Valid VIN chunk
          twc_device_set_vehicle_vin_chunk(dev, vin.chunk_index, vin.value);
        }
      }
      break;
    }
    
    default:
      // Unhandled command, ignore
      break;
  }
  
  (void)now_ms;  // Available for future timestamping needs
}

// =============================================================================
// MODE DETECTION (internal helper)
// =============================================================================

static void update_mode_from_frame_(twc_device_t *dev,
                                    const uint8_t *header,
                                    const uint8_t *payload,
                                    size_t payload_len,
                                    twc_cmd_t cmd) {
  if (!dev || !header) return;
  
  twc_marker_t marker;
  uint16_t src_address;
  twc_parse_header(header, &marker, NULL, &src_address);
  
  uint16_t our_master = twc_controller_get_master_address();
  twc_mode_t current_mode = dev->mode;
  twc_mode_t new_mode = current_mode;
  
  // Mode detection based on command and marker
  if (cmd == TWC_CMD_HEARTBEAT) {
    // E0 heartbeat frames
    uint16_t dest_address = 0;
    if (payload && payload_len >= 2) {
      dest_address = ((uint16_t)payload[0] << 8) | (uint16_t)payload[1];
    }
    
    if (marker == TWC_MARKER_REQUEST) {
      // FB E0: Controller heartbeat
      if (src_address != our_master) {
        new_mode = TWC_MODE_FOREIGN_MASTER;
      }
      // src == our_master: seeing our own traffic, leave mode unchanged
      
    } else if (marker == TWC_MARKER_RESPONSE) {
      // FD E0: Peripheral heartbeat
      if (dest_address == our_master) {
        new_mode = TWC_MODE_PERIPHERAL;
      } else if (dest_address != 0 && dest_address != our_master) {
        new_mode = TWC_MODE_FOREIGN_PERIPHERAL;
      }
      // dest == 0: broadcast, leave mode unchanged
    }
    
  } else if (cmd == TWC_CMD_PERIPHERAL_NEGOTIATION &&
             marker == TWC_MARKER_RESPONSE) {
    // FD E2: Peripheral negotiation (unconfigured device announcing itself)
    if (current_mode == TWC_MODE_UNKNOWN) {
      new_mode = TWC_MODE_UNCONF_PERIPHERAL;
    }
  }
  
  // Update mode if changed
  if (new_mode != current_mode) {
    dev->mode = new_mode;
  }
}

// =============================================================================
// STATE ACCESSORS
// =============================================================================

bool twc_device_get_contactor_closed(const twc_device_t *dev) {
  return dev ? dev->contactor_closed : false;
}

bool twc_device_get_vehicle_connected(const twc_device_t *dev) {
  return dev ? dev->vehicle_connected : false;
}

twc_mode_t twc_device_get_mode(const twc_device_t *dev) {
  return dev ? dev->mode : TWC_MODE_UNKNOWN;
}

void twc_device_set_mode(twc_device_t *dev, twc_mode_t mode) {
  if (dev) {
    dev->mode = mode;
  }
}

// =============================================================================
// ELECTRICAL METRICS
// =============================================================================

float twc_device_get_phase_a_current_a(const twc_device_t *dev) {
  return dev ? dev->phase_a_current_a : 0.0f;
}

float twc_device_get_phase_a_voltage_v(const twc_device_t *dev) {
  return dev ? dev->phase_a_voltage_v : 0.0f;
}

float twc_device_get_phase_b_current_a(const twc_device_t *dev) {
  return dev ? dev->phase_b_current_a : 0.0f;
}

float twc_device_get_phase_b_voltage_v(const twc_device_t *dev) {
  return dev ? dev->phase_b_voltage_v : 0.0f;
}

float twc_device_get_phase_c_current_a(const twc_device_t *dev) {
  return dev ? dev->phase_c_current_a : 0.0f;
}

float twc_device_get_phase_c_voltage_v(const twc_device_t *dev) {
  return dev ? dev->phase_c_voltage_v : 0.0f;
}

float twc_device_get_total_energy_kwh(const twc_device_t *dev) {
  return dev ? dev->total_energy_kwh : 0.0f;
}

float twc_device_get_session_energy_kwh(const twc_device_t *dev) {
  return dev ? dev->session_energy_kwh : 0.0f;
}

// =============================================================================
// METER UPDATES
// =============================================================================

void twc_device_set_meter_values(twc_device_t *dev,
                                 float total_energy_kwh,
                                 float phase_l1_v, float phase_l2_v, float phase_l3_v,
                                 float phase_l1_a, float phase_l2_a, float phase_l3_a) {
  if (!dev) return;

  // Map L1/L2/L3 to phase A/B/C
  dev->phase_a_voltage_v = phase_l1_v;
  dev->phase_b_voltage_v = phase_l2_v;
  dev->phase_c_voltage_v = phase_l3_v;
  dev->phase_a_current_a = phase_l1_a;
  dev->phase_b_current_a = phase_l2_a;
  dev->phase_c_current_a = phase_l3_a;
  dev->total_energy_kwh = total_energy_kwh;

  // Session energy tracking
  if (!dev->vehicle_connected) {
    dev->session_active = false;
    return;
  }

  if (!dev->session_active) {
    // Start new session
    dev->session_active = true;
    dev->session_energy_baseline_kwh = total_energy_kwh;
    dev->session_energy_kwh = 0.0f;
    return;
  }

  // Update session energy
  float delta = total_energy_kwh - dev->session_energy_baseline_kwh;
  if (delta < 0.0f) {
    // Counter reset, re-baseline
    dev->session_energy_baseline_kwh = total_energy_kwh;
    dev->session_energy_kwh = 0.0f;
  } else {
    dev->session_energy_kwh = delta;
  }
}

// =============================================================================
// CURRENT LIMITS
// =============================================================================

void twc_device_set_initial_current_a(twc_device_t *dev, uint8_t amps) {
  if (dev) {
    dev->initial_current_limit_a = (float)amps;
  }
}

void twc_device_set_session_current_a(twc_device_t *dev, uint8_t amps) {
  if (dev) {
    dev->session_current_limit_a = (float)amps;
  }
}

// =============================================================================
// NEGOTIATION STATE
// =============================================================================

uint8_t twc_device_get_peripheral_session(const twc_device_t *dev) {
  return dev ? dev->peripheral_session : 0;
}

float twc_device_get_current_available_a(const twc_device_t *dev) {
  return dev ? dev->current_available_a : 0.0f;
}

void twc_device_set_peripheral_session(twc_device_t *dev, uint8_t session) {
  if (dev) {
    dev->peripheral_session = session;
  }
}

void twc_device_set_current_available_a(twc_device_t *dev, float amps) {
  if (dev) {
    dev->current_available_a = (amps < 0.0f) ? 0.0f : amps;
  }
}

// =============================================================================
// IDENTIFICATION
// =============================================================================

const char *twc_device_get_serial_number(const twc_device_t *dev) {
  return dev ? dev->serial_number : "";
}

void twc_device_set_serial_number(twc_device_t *dev, const char *serial) {
  if (dev && serial) {
    snprintf(dev->serial_number, sizeof(dev->serial_number), "%s", serial);
  }
}

const char *twc_device_get_software_version(const twc_device_t *dev) {
  return dev ? dev->software_version : "";
}

void twc_device_set_software_version(twc_device_t *dev, const char *version) {
  if (dev && version) {
    snprintf(dev->software_version, sizeof(dev->software_version), "%s", version);
  }
}

// =============================================================================
// VIN ASSEMBLY
// =============================================================================

const char *twc_device_get_vehicle_vin(const twc_device_t *dev) {
  if (!dev || !dev->vehicle_connected || dev->vehicle_vin[0] == '\0') {
    return NULL;
  }
  return dev->vehicle_vin;
}

void twc_device_set_vehicle_vin_chunk(twc_device_t *dev,
                                      uint8_t chunk_index,
                                      const char *chunk) {
  if (!dev || !chunk || chunk_index >= 3) return;

  // Store chunk
  snprintf(dev->vin_chunks[chunk_index],
           sizeof(dev->vin_chunks[chunk_index]),
           "%s", chunk);

  // Mark chunk as present
  dev->vin_chunks_present |= (1u << chunk_index);

  // Rebuild full VIN and update vehicle_connected flag
  rebuild_vin_(dev);
}

void twc_device_clear_vehicle_vin(twc_device_t *dev) {
  if (!dev) return;

  dev->vin_chunks_present = 0;
  memset(dev->vin_chunks, 0, sizeof(dev->vin_chunks));
  dev->vehicle_vin[0] = '\0';
  dev->vehicle_connected = false;
}

static void rebuild_vin_(twc_device_t *dev) {
  if (!dev) return;

  dev->vehicle_vin[0] = '\0';

  // Concatenate all non-empty chunks
  char *out = dev->vehicle_vin;
  size_t remaining = sizeof(dev->vehicle_vin) - 1;

  for (uint8_t i = 0; i < 3 && remaining > 0; ++i) {
    const char *part = dev->vin_chunks[i];
    if (part[0] == '\0') continue;

    size_t part_len = strlen(part);
    if (part_len > remaining) {
      part_len = remaining;
    }

    memcpy(out, part, part_len);
    out += part_len;
    remaining -= part_len;
  }
  *out = '\0';

  // Vehicle connected when all three chunks present and VIN non-empty
  dev->vehicle_connected =
      ((dev->vin_chunks_present & 0x07) == 0x07) &&
      (dev->vehicle_vin[0] != '\0');
}

// =============================================================================
// STATUS CODE
// =============================================================================

int twc_device_get_status_code(const twc_device_t *dev) {
  return dev ? dev->status_code : 0;
}
