// twc_core.c
// Core TWC controller logic and device registry.

#include "twc_core.h"
#include "twc_device.h"
#include "twc_protocol.h"

#include <stdio.h>
#include <string.h>

// =============================================================================
// GLOBAL STATE
// =============================================================================

// Exported to device layer for mode detection
static uint16_t g_twcc_master_address = 0u;

uint16_t twc_controller_get_master_address(void) {
  return g_twcc_master_address;
}

// =============================================================================
// DEVICE REGISTRY
// =============================================================================

static int find_device_index(const twc_core_t *core, uint16_t address) {
  for (int i = 0; i < TWC_CORE_MAX_DEVICES; ++i) {
    if (core->devices[i].present && core->devices[i].address == address) {
      return i;
    }
  }
  return -1;
}

static twc_core_device_t *ensure_device(twc_core_t *core, uint16_t address) {
  int idx = find_device_index(core, address);
  if (idx >= 0) {
    return &core->devices[idx];
  }

  // Find free slot
  for (int i = 0; i < TWC_CORE_MAX_DEVICES; ++i) {
    twc_core_device_t *dev = &core->devices[i];
    if (!dev->present) {
      memset(dev, 0, sizeof(*dev));
      dev->present = true;
      dev->address = address;
      twc_device_init(&dev->device, address);
      return dev;
    }
  }

  return NULL;  // Registry full
}

// =============================================================================
// LIFECYCLE
// =============================================================================

void twc_core_init(twc_core_t *core) {
  if (!core) return;
  memset(core, 0, sizeof(*core));

  core->master_address = 0xF00D;
  g_twcc_master_address = core->master_address;
  core->global_max_current_a = 0.0f;
  core->online_timeout_ms = 15000u;
  core->master_mode = false;
}

void twc_core_set_online_timeout(twc_core_t *core, uint32_t timeout_ms) {
  if (core) {
    core->online_timeout_ms = timeout_ms;
  }
}

void twc_core_set_master_address(twc_core_t *core, uint16_t address) {
  if (core) {
    core->master_address = address;
    g_twcc_master_address = address;
  }
}

uint16_t twc_core_get_master_address(const twc_core_t *core) {
  return core ? core->master_address : 0u;
}

void twc_core_set_global_max_current(twc_core_t *core, float max_current_a) {
  if (core) {
    core->global_max_current_a = (max_current_a > 0.0f) ? max_current_a : 0.0f;
  }
}

float twc_core_get_global_max_current(const twc_core_t *core) {
  return core ? core->global_max_current_a : 0.0f;
}

void twc_core_set_tx_callback(twc_core_t *core,
                              twc_core_tx_callback_t cb,
                              void *user_data) {
  if (core) {
    core->tx_cb = cb;
    core->tx_user = user_data;
  }
}

void twc_core_set_negotiation_callback(twc_core_t *core,
                                       twc_core_negotiation_cb_t cb,
                                       void *user_data) {
  if (core) {
    core->negotiation_cb = cb;
    core->negotiation_user = user_data;
  }
}

void twc_core_set_autobind_callback(twc_core_t *core,
                                    twc_core_autobind_cb_t cb,
                                    void *user_data) {
  if (core) {
    core->autobind_cb = cb;
    core->autobind_user = user_data;
  }
}

void twc_core_set_log_callback(twc_core_t *core,
                               twc_core_log_cb_t cb,
                               void *user_data) {
  if (core) {
    core->log_cb = cb;
    core->log_user = user_data;
  }
}

// =============================================================================
// SESSION MANAGEMENT
// =============================================================================

static uint8_t generate_session_id(void) {
  uint8_t sid = (uint8_t)((rand() % 254u) + 1u);
  return (sid == 0u) ? 1u : sid;
}

void twc_core_set_master_mode(twc_core_t *core, bool enabled) {
  if (!core) return;

  if (enabled && !core->master_mode) {
    // Transition to master mode
    core->master_mode = true;
    core->master_just_enabled = true;
    core->master_session_id = generate_session_id();
  } else if (!enabled) {
    // Transition to observer mode
    core->master_mode = false;
    core->master_just_enabled = false;
  }
}

// =============================================================================
// RX PATH
// =============================================================================

bool twc_core_handle_frame(twc_core_t *core,
                           const uint8_t *frame,
                           size_t len,
                           uint32_t now_ms) {
  if (!core || !frame || len == 0) return false;

  // Decode frame
  const uint8_t *header, *payload;
  size_t payload_len;
  if (!twc_decode_frame(frame, len, &header, &payload, &payload_len)) {
    // Log checksum validation failure
    if (core->log_cb) {
      char msg[256];
      char hex_buf[200];
      int pos = 0;

      // Build hex dump of failed frame (limit to 60 bytes)
      size_t dump_len = (len > 60) ? 60 : len;
      for (size_t i = 0; i < dump_len && pos < (int)sizeof(hex_buf) - 4; i++) {
        pos += snprintf(hex_buf + pos, sizeof(hex_buf) - pos, "%02X ", frame[i]);
      }
      if (len > dump_len) {
        pos += snprintf(hex_buf + pos, sizeof(hex_buf) - pos, "...");
      }

      // Calculate expected vs actual checksum
      uint8_t calculated = 0;
      if (len >= 2) {
        for (size_t i = 1; i < len - 1; ++i) {
          calculated = (uint8_t)((calculated + frame[i]) & 0xFF);
        }
      }
      uint8_t received = (len > 0) ? frame[len - 1] : 0;

      snprintf(msg, sizeof(msg),
               "Checksum FAILED: len=%u calculated=0x%02X received=0x%02X frame: %s",
               (unsigned)len, calculated, received, hex_buf);
      core->log_cb(TWC_LOG_WARNING, msg, core->log_user);
    }
    return false;  // Invalid checksum or malformed
  }

  // Parse header
  twc_marker_t marker;
  twc_cmd_t cmd;
  uint16_t src_address;
  twc_parse_header(header, &marker, &cmd, &src_address);

  // Ensure device entry exists
  twc_core_device_t *dev = ensure_device(core, src_address);
  if (!dev) {
    // Registry full - log warning
    if (core->log_cb) {
      char msg[64];
      snprintf(msg, sizeof(msg),
               "Device registry full, cannot track 0x%04X", src_address);
      core->log_cb(TWC_LOG_WARNING, msg, core->log_user);
    }
    return false;
  }

  // Update timestamps
  dev->last_frame_ms = now_ms;
  if (cmd == TWC_CMD_HEARTBEAT ||
      cmd == TWC_CMD_CONTROLLER_NEGOTIATION ||
      cmd == TWC_CMD_PERIPHERAL_NEGOTIATION) {
    dev->last_status_ms = now_ms;
  }

  // Capture old mode before update
  twc_mode_t old_mode = twc_device_get_mode(&dev->device);

  // Update device state from frame
  twc_device_update_from_frame(&dev->device, header, payload, payload_len, cmd, now_ms);

  // Detect new peripheral for auto-binding
  twc_mode_t new_mode = twc_device_get_mode(&dev->device);
  bool is_new_peripheral = (old_mode == TWC_MODE_UNKNOWN) &&
                           (new_mode == TWC_MODE_PERIPHERAL ||
                            new_mode == TWC_MODE_UNCONF_PERIPHERAL);

  if (is_new_peripheral && core->autobind_cb) {
    core->autobind_cb(src_address, new_mode, core->autobind_user);
  }

  // Sync negotiation metrics
  dev->peripheral_session = twc_device_get_peripheral_session(&dev->device);
  dev->current_available_a = twc_device_get_current_available_a(&dev->device);

  // MASTER MODE: Auto-respond to unconfigured peripherals
  if (core->master_mode && core->tx_cb &&
      marker == TWC_MARKER_RESPONSE &&
      cmd == TWC_CMD_PERIPHERAL_NEGOTIATION) {

    bool needs_claim = (new_mode == TWC_MODE_UNCONF_PERIPHERAL ||
                        new_mode == TWC_MODE_UNKNOWN);

    if (needs_claim) {
      uint8_t claim_frame[16];
      size_t claim_len = twc_build_heartbeat_frame(
          core->master_address, dev->address,
          TWC_HB_READY, 0, 0,
          claim_frame, sizeof(claim_frame)
      );

      if (claim_len > 0) {
        core->tx_cb(claim_frame, claim_len, core->tx_user);
      }
    }
  }

  return true;  // Frame was valid
}

// =============================================================================
// TX PATH: MASTER MODE
// =============================================================================

static bool handle_startup_burst(twc_core_t *core, uint32_t now_ms) {
  const uint32_t BURST_INTERVAL_MS = 200u;

  if (!core->startup_burst_active) {
    return false;
  }

  uint32_t elapsed = (now_ms >= core->startup_burst_last_ms)
                         ? (now_ms - core->startup_burst_last_ms)
                         : 0u;

  if (elapsed < BURST_INTERVAL_MS) {
    return false;
  }

  uint8_t frame[16];
  size_t frame_len = 0;

  // Send 5x E1, then 4x E2
  if (core->startup_burst_e1_sent < 5u) {
    frame_len = twc_build_controller_negotiation_frame(
        core->master_address,
        core->master_session_id,
        frame, sizeof(frame)
    );
    if (frame_len > 0) {
      core->tx_cb(frame, frame_len, core->tx_user);
      core->startup_burst_e1_sent++;
      core->startup_burst_last_ms = now_ms;
      return true;
    }
  } else if (core->startup_burst_e2_sent < 4u) {
    frame_len = twc_build_peripheral_pause_frame(
        core->master_address,
        core->master_session_id,
        frame, sizeof(frame)
    );
    if (frame_len > 0) {
      core->tx_cb(frame, frame_len, core->tx_user);
      core->startup_burst_e2_sent++;
      core->startup_burst_last_ms = now_ms;
      return true;
    }
  }

  // Burst complete
  core->startup_burst_active = false;
  return false;
}

static void reconcile_current_allocation(twc_core_t *core) {
  float per_device_desired[TWC_CORE_MAX_DEVICES];
  int eligible_indices[TWC_CORE_MAX_DEVICES];
  int eligible_count = 0;
  float total_desired = 0.0f;

  // Gather eligible peripherals
  for (int i = 0; i < TWC_CORE_MAX_DEVICES; ++i) {
    twc_core_device_t *dev = &core->devices[i];
    if (!dev->present) continue;

    twc_mode_t mode = twc_device_get_mode(&dev->device);
    if (mode != TWC_MODE_PERIPHERAL && mode != TWC_MODE_UNCONF_PERIPHERAL) {
      continue;
    }

    // Clamp to [0, 32A]
    float desired = dev->desired_initial_current_a;
    if (desired < 0.0f) desired = 0.0f;
    if (desired > 32.0f) desired = 32.0f;

    per_device_desired[eligible_count] = desired;
    eligible_indices[eligible_count] = i;
    total_desired += desired;
    eligible_count++;
  }

  // Compute scaling factor
  float scale = 1.0f;
  if (core->global_max_current_a > 0.0f &&
      total_desired > core->global_max_current_a) {
    scale = core->global_max_current_a / total_desired;
    if (scale < 0.0f) scale = 0.0f;
    if (scale > 1.0f) scale = 1.0f;
  }

  // Apply scaled currents
  for (int n = 0; n < eligible_count; ++n) {
    int idx = eligible_indices[n];
    twc_core_device_t *dev = &core->devices[idx];

    float applied = per_device_desired[n] * scale;
    if (applied < 0.0f) applied = 0.0f;

    bool changed = (dev->applied_initial_current_a != applied);
    dev->applied_initial_current_a = applied;

    // Detect vehicle connection edge
    bool vehicle_connected = twc_device_get_vehicle_connected(&dev->device);
    bool car_just_connected = (vehicle_connected && !dev->last_vehicle_connected);
    dev->last_vehicle_connected = vehicle_connected;

    // Schedule 0x05 command when needed
    // Always send when car connects (even if current unchanged) to ensure TWC has the right value
    if (changed || car_just_connected) {
      dev->pending_initial_current_cmd = true;
      dev->last_initial_current_cmd_a = applied;
    }

    // Notify application
    if (changed && core->negotiation_cb) {
      core->negotiation_cb(dev->address, applied,
                          dev->peripheral_session,
                          core->negotiation_user);
    }
  }
}

static bool send_info_probe(twc_core_t *core, uint32_t now_ms) {
  const uint32_t INFO_PROBE_INTERVAL_MS = 2000u;
  const uint32_t POST_HEARTBEAT_DELAY_MS = 500u;

  for (int i = 0; i < TWC_CORE_MAX_DEVICES; ++i) {
    twc_core_device_t *dev = &core->devices[i];
    if (!dev->present) continue;

    twc_mode_t mode = twc_device_get_mode(&dev->device);
    if (mode != TWC_MODE_PERIPHERAL) continue;

    if (dev->e0_since_last_probe < 2u) continue;

    if (core->last_e0_heartbeat_ms != 0u) {
      uint32_t since_e0 = (now_ms >= core->last_e0_heartbeat_ms)
                              ? (now_ms - core->last_e0_heartbeat_ms)
                              : 0u;
      if (since_e0 < POST_HEARTBEAT_DELAY_MS) continue;
    }

    uint32_t elapsed = (now_ms >= dev->last_info_probe_ms)
                           ? (now_ms - dev->last_info_probe_ms)
                           : 0u;
    if (elapsed < INFO_PROBE_INTERVAL_MS) continue;

    // Determine probe command
    twc_cmd_t probe_cmd;
    uint8_t next_stage;

    switch (dev->info_probe_stage) {
      case 0: probe_cmd = TWC_CMD_VIN_HI;  next_stage = 1; break;
      case 1: probe_cmd = TWC_CMD_VIN_MID; next_stage = 2; break;
      case 2: probe_cmd = TWC_CMD_VIN_LO;  next_stage = 3; break;
      case 3: probe_cmd = TWC_CMD_SERIAL;  next_stage = 4; break;
      case 4: probe_cmd = TWC_CMD_METER;   next_stage = 5; break;
      case 5: probe_cmd = TWC_CMD_VERSION; next_stage = 0; break;
      default:
        dev->info_probe_stage = 0;
        dev->last_info_probe_ms = now_ms;
        continue;
    }

    // Build and send
    uint8_t frame[16];
    size_t frame_len = twc_build_request_frame(
        core->master_address, dev->address,
        probe_cmd, frame, sizeof(frame)
    );

    if (frame_len > 0) {
      core->tx_cb(frame, frame_len, core->tx_user);
      dev->info_probe_stage = next_stage;
      dev->last_info_probe_ms = now_ms;
      dev->e0_since_last_probe = 0u;
      return true;
    }
  }

  return false;
}

static bool send_heartbeat(twc_core_t *core, uint32_t now_ms) {
  const uint32_t HEARTBEAT_INTERVAL_MS = 1000u;

  if (core->last_e0_heartbeat_ms != 0u) {
    uint32_t elapsed = (now_ms >= core->last_e0_heartbeat_ms)
                           ? (now_ms - core->last_e0_heartbeat_ms)
                           : 0u;
    if (elapsed < HEARTBEAT_INTERVAL_MS) {
      return false;
    }
  }

  // Round-robin
  for (int scanned = 0; scanned < TWC_CORE_MAX_DEVICES; ++scanned) {
    int idx = core->next_e0_device_index % TWC_CORE_MAX_DEVICES;
    core->next_e0_device_index = (core->next_e0_device_index + 1) % TWC_CORE_MAX_DEVICES;

    twc_core_device_t *dev = &core->devices[idx];
    if (!dev->present) continue;

    twc_mode_t mode = twc_device_get_mode(&dev->device);
    if (mode != TWC_MODE_PERIPHERAL && mode != TWC_MODE_UNCONF_PERIPHERAL) {
      continue;
    }

    // Determine charge state and currents
    uint8_t charge_state = 0u;
    uint16_t available_centiamps = 0u;
    uint16_t delivered_centiamps = 0u;

    if (mode == TWC_MODE_UNCONF_PERIPHERAL) {
      // All zeros for unconfigured
      charge_state = 0u;
      available_centiamps = 0u;
      delivered_centiamps = 0u;
    } else {
      // Prioritize 0x09 session current over 0x05 initial current
      if (dev->pending_session_current_cmd) {
        float current_a = dev->last_session_current_cmd_a;
        if (current_a < 0.0f) current_a = 0.0f;
        if (current_a > 32.0f) current_a = 32.0f;

        charge_state = 0x09u;
        available_centiamps = (uint16_t)(current_a * 100.0f + 0.5f);
        delivered_centiamps = 0u;
      } else if (dev->pending_initial_current_cmd) {
        float current_a = dev->last_initial_current_cmd_a;
        if (current_a < 0.0f) current_a = 0.0f;
        if (current_a > 32.0f) current_a = 32.0f;

        charge_state = 0x05u;
        available_centiamps = (uint16_t)(current_a * 100.0f + 0.5f);
        delivered_centiamps = 0u;
      } else {
        // Normal heartbeat: all zeros
        charge_state = 0u;
        available_centiamps = 0u;
        delivered_centiamps = 0u;
      }
    }

    // Build frame
    uint8_t frame[16];
    size_t frame_len = twc_build_heartbeat_frame(
        core->master_address, dev->address,
        charge_state, available_centiamps, delivered_centiamps,
        frame, sizeof(frame)
    );

    if (frame_len == 0) continue;

    // Log heartbeat details
    if (core->log_cb) {
      char msg[128];
      snprintf(msg, sizeof(msg),
               "Sending E0 to 0x%04X: state=0x%02X avail=%u deliv=%u",
               dev->address, charge_state, available_centiamps, delivered_centiamps);
      core->log_cb(TWC_LOG_DEBUG, msg, core->log_user);
    }

    // Send
    core->tx_cb(frame, frame_len, core->tx_user);
    core->last_e0_heartbeat_ms = now_ms;

    // Clear pending flags after successful send
    if (charge_state == 0x05u) {
      dev->pending_initial_current_cmd = false;
    } else if (charge_state == 0x09u) {
      dev->pending_session_current_cmd = false;
    }

    if (dev->e0_since_last_probe < 255u) {
      dev->e0_since_last_probe++;
    }

    return true;
  }

  return false;
}

void twc_core_master_tick(twc_core_t *core, uint32_t now_ms) {
  if (!core || !core->master_mode || !core->tx_cb) {
    return;
  }

  // Initialize burst on mode entry
  if (core->master_just_enabled) {
    core->master_just_enabled = false;
    core->startup_burst_active = true;
    core->startup_burst_e1_sent = 0u;
    core->startup_burst_e2_sent = 0u;
    core->startup_burst_last_ms = now_ms;
    if (core->master_session_id == 0u) {
      core->master_session_id = 1u;
    }
  }

  // Priority 1: Startup burst
  if (handle_startup_burst(core, now_ms)) {
    return;
  }

  // Priority 2: Current allocation
  reconcile_current_allocation(core);

  // Priority 3: Info probes
  if (send_info_probe(core, now_ms)) {
    return;
  }

  // Priority 4: Heartbeat
  send_heartbeat(core, now_ms);
}

// =============================================================================
// DEVICE QUERY API
// =============================================================================

twc_core_device_t *twc_core_get_device_by_address(twc_core_t *core,
                                                  uint16_t address) {
  if (!core) return NULL;
  int idx = find_device_index(core, address);
  return (idx >= 0) ? &core->devices[idx] : NULL;
}

const twc_device_t *twc_core_get_device_const(const twc_core_t *core,
                                              uint16_t address) {
  if (!core) return NULL;
  int idx = find_device_index(core, address);
  return (idx >= 0) ? &core->devices[idx].device : NULL;
}

bool twc_core_device_present(const twc_core_device_t *dev) {
  return dev && dev->present;
}

bool twc_core_device_online(const twc_core_t *core,
                            const twc_core_device_t *dev,
                            uint32_t now_ms) {
  if (!core || !dev || !dev->present) return false;

  twc_mode_t mode = twc_device_get_mode(&dev->device);
  if (mode == TWC_MODE_UNKNOWN || mode == TWC_MODE_UNCONF_PERIPHERAL) {
    return false;
  }

  uint32_t ref_ms = (dev->last_status_ms != 0u)
                        ? dev->last_status_ms
                        : dev->last_frame_ms;
  if (ref_ms == 0u) return false;

  uint32_t age_ms = (now_ms >= ref_ms) ? (now_ms - ref_ms) : 0u;
  return age_ms <= core->online_timeout_ms;
}

// =============================================================================
// CURRENT ALLOCATION API
// =============================================================================

uint8_t twc_core_get_peripheral_session(const twc_core_device_t *dev) {
  return dev ? dev->peripheral_session : 0u;
}

float twc_core_get_current_available_a(const twc_core_device_t *dev) {
  return dev ? dev->current_available_a : 0.0f;
}

void twc_core_set_max_current(twc_core_t *core,
                               uint16_t address,
                               float current_a) {
  if (!core) return;
  twc_core_device_t *dev = ensure_device(core, address);
  if (dev) {
    dev->max_current_a = (current_a > 0.0f) ? current_a : 0.0f;
  }
}

float twc_core_get_max_current(const twc_core_t *core,
                                const twc_core_device_t *dev) {
  (void)core;
  return dev ? dev->max_current_a : 0.0f;
}

void twc_core_set_desired_initial_current(twc_core_t *core,
                                          uint16_t address,
                                          float current_a) {
  if (!core) return;
  twc_core_device_t *dev = ensure_device(core, address);
  if (dev) {
    dev->desired_initial_current_a = (current_a > 0.0f) ? current_a : 0.0f;
  }
}

float twc_core_get_desired_initial_current(const twc_core_t *core,
                                           const twc_core_device_t *dev) {
  (void)core;
  return dev ? dev->desired_initial_current_a : 0.0f;
}

void twc_core_set_desired_session_current(twc_core_t *core,
                                          uint16_t address,
                                          float current_a) {
  if (!core) return;
  twc_core_device_t *dev = ensure_device(core, address);
  if (dev) {
    dev->desired_session_current_a = (current_a > 0.0f) ? current_a : 0.0f;
  }
}

float twc_core_get_desired_session_current(const twc_core_t *core,
                                           const twc_core_device_t *dev) {
  (void)core;
  return dev ? dev->desired_session_current_a : 0.0f;
}

float twc_core_get_applied_initial_current(const twc_core_t *core,
                                           const twc_core_device_t *dev) {
  (void)core;
  return dev ? dev->applied_initial_current_a : 0.0f;
}

uint32_t twc_core_get_restart_counter(const twc_core_device_t *dev) {
  return dev ? dev->restart_counter : 0u;
}