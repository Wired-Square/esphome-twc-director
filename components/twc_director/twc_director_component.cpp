// twc_director_component.cpp
//
// ESPHome glue layer for TWC control via twc_core.
//
// Responsibilities:
//   RX: UART → SLIP decode → twc_core_handle_frame() → update ESPHome sensors
//   TX: twc_core_master_tick() → callback → SLIP encode → UART
//   Binding: Auto-bind discovered devices to EVSE slots

#include "twc_director_component.h"
#include "esphome/core/log.h"
#include <math.h>

extern "C" {
#include "twc/twc_frame.h"
#include "twc/twc_protocol.h"
#include "twc/twc_core.h"
}

namespace esphome {
namespace twc_director {

static const char *const TAG = "twc";

// =============================================================================
// CALLBACK SHIMS (C → C++)
// =============================================================================

// TX callback: core wants to send a frame
void tx_callback_shim(const uint8_t *frame, size_t len, void *user_data) {
  auto *self = static_cast<TWCDirectorComponent *>(user_data);
  if (self) {
    self->handle_tx_frame_(frame, len);
  }
}

// Negotiation callback: core updated current allocation
void negotiation_callback_shim(uint16_t address,
                                float applied_initial_current_a,
                                uint8_t session_id,
                                void *user_data) {
  auto *self = static_cast<TWCDirectorComponent *>(user_data);
  if (self) {
    ESP_LOGI(TAG,
             "Negotiation update TWC 0x%04X: session=%u applied=%.1fA",
             address, session_id, applied_initial_current_a);
  }
}

// Auto-bind callback: core discovered a new peripheral
void autobind_callback_shim(uint16_t address,
                             twc_mode_t mode,
                             void *user_data) {
  auto *self = static_cast<TWCDirectorComponent *>(user_data);
  if (self) {
    self->handle_auto_bind_(address, mode);
  }
}

// Log callback: core wants to log diagnostic information
void log_callback_shim(twc_log_level_t level,
                       const char *message,
                       void *user_data) {
  (void)user_data;  // Not needed for logging

  if (!message) return;

  // Map TWC log levels to ESPHome log levels
  switch (level) {
    case TWC_LOG_ERROR:
      ESP_LOGE(TAG, "%s", message);
      break;
    case TWC_LOG_WARNING:
      ESP_LOGW(TAG, "%s", message);
      break;
    case TWC_LOG_INFO:
      ESP_LOGI(TAG, "%s", message);
      break;
    case TWC_LOG_DEBUG:
      ESP_LOGD(TAG, "%s", message);
      break;
    case TWC_LOG_VERBOSE:
      ESP_LOGV(TAG, "%s", message);
      break;
    default:
      ESP_LOGD(TAG, "%s", message);
      break;
  }
}

// =============================================================================
// COMPONENT LIFECYCLE
// =============================================================================

void TWCDirectorComponent::setup() {
  ESP_LOGI(TAG, "TWC Director setup (master=0x%04X)", this->master_address_);

  // Initialize core and frame decoder
  twc_core_init(&this->core_);
  twc_frame_decoder_init(&this->decoder_);

  // Configure core
  twc_core_set_master_address(&this->core_, this->master_address_);
  twc_core_set_online_timeout(&this->core_, 15000);
  
  if (this->global_max_current_a_ > 0.0f) {
    twc_core_set_global_max_current(&this->core_, this->global_max_current_a_);
    ESP_LOGI(TAG, "Global max current (safety limit): %.1fA", this->global_max_current_a_);

    // Initialize global max control entity to the compile-time value if provided
    if (this->global_max_current_control_ && !this->global_max_current_control_->has_state()) {
      this->global_max_current_control_->publish_state(this->global_max_current_a_);
    }
  }

  ESP_LOGI(TAG, "EVSE max current limit: %.1fA", this->evse_max_current_limit_a_);

  // Wire callbacks
  twc_core_set_tx_callback(&this->core_, &tx_callback_shim, this);
  twc_core_set_negotiation_callback(&this->core_, &negotiation_callback_shim, this);
  twc_core_set_autobind_callback(&this->core_, &autobind_callback_shim, this);
  twc_core_set_log_callback(&this->core_, &log_callback_shim, this);

  // Initialize all EVSE slots as offline
  for (auto &evse : evse_entries_) {
    this->publish_binary_sensor_if_changed_(evse.online, false);
  }
}

void TWCDirectorComponent::loop() {
  uint32_t now = millis();

  this->process_rx_(now);           // UART → SLIP → core
  this->update_master_mode_(now);   // Monitor master mode switch
  this->run_master_tick_(now);      // Core master logic
  this->update_evse_metrics_(now);  // Core → ESPHome sensors
  this->drain_tx_queue_(now);       // Queued frames → UART

  // Update link health sensor
  if (this->link_ok_sensor_) {
    bool link_ok = false;
    if (this->last_valid_frame_ms_ != 0) {
      uint32_t age_ms = (now >= this->last_valid_frame_ms_)
                          ? (now - this->last_valid_frame_ms_)
                          : 0;
      link_ok = (age_ms < LINK_TIMEOUT_MS);
    }

    // Log decoder stats when link goes down
    bool prev_link_ok = this->link_ok_sensor_->state;
    if (prev_link_ok && !link_ok) {
      ESP_LOGW(TAG, "Link went down! SLIP decoder stats: decoded=%u dropped_overflow=%u dropped_invalid_esc=%u",
               this->decoder_.frames_decoded,
               this->decoder_.frames_dropped_overflow,
               this->decoder_.frames_dropped_invalid_esc);
    }

    this->publish_binary_sensor_if_changed_(this->link_ok_sensor_, link_ok);
  }

  // Periodic SLIP decoder and TX diagnostics
  static uint32_t last_stats_ms = 0;
  if (now - last_stats_ms > DECODER_STATS_INTERVAL_MS) {
    last_stats_ms = now;
    ESP_LOGD(TAG, "RX stats: decoded=%u dropped_overflow=%u dropped_invalid_esc=%u",
             this->decoder_.frames_decoded,
             this->decoder_.frames_dropped_overflow,
             this->decoder_.frames_dropped_invalid_esc);
    ESP_LOGD(TAG, "TX stats: queued=%u dropped=%u encode_failures=%u",
             (unsigned)this->tx_frames_queued_,
             (unsigned)this->tx_frames_dropped_,
             (unsigned)this->tx_encode_failures_);
  }
}

// =============================================================================
// RX PATH: UART → SLIP → CORE
// =============================================================================

void TWCDirectorComponent::process_rx_(uint32_t now) {
  // Feed UART bytes into SLIP decoder
  // Limit bytes per iteration to avoid starving other tasks (OTA, network, etc.)
  constexpr size_t MAX_BYTES_PER_LOOP = 64;
  size_t bytes_processed = 0;

  while (this->available() && bytes_processed < MAX_BYTES_PER_LOOP) {
    uint8_t b;
    if (!this->read_byte(&b)) break;
    bytes_processed++;

    uint32_t delta_ms = (this->last_rx_ms_ == 0) ? 0 : (now - this->last_rx_ms_);
    ESP_LOGV(TAG, "RX byte: 0x%02X t=%ums dt=%ums", b, (unsigned) now, (unsigned) delta_ms);

    // Track RX activity for bus-idle detection
    this->last_rx_ms_ = now;

    // Diagnostic: track SLIP decoder state before pushing byte
    bool was_in_frame = this->decoder_.in_frame;
    size_t prev_len = this->decoder_.len;

    // Push byte through decoder
    const uint8_t *frame = nullptr;
    size_t len = 0;

    if (twc_frame_decoder_push(&this->decoder_, b, &frame, &len)) {
      // Complete frame decoded
      ESP_LOGV(TAG, "SLIP: Frame complete, %u bytes", (unsigned)len);
      this->handle_rx_frame_(frame, len, now);
    } else if (b == 0xC0) {
      // END byte received but no frame returned - log why
      if (!was_in_frame && prev_len == 0) {
        ESP_LOGV(TAG, "SLIP: Frame START");
      } else if (was_in_frame && prev_len == 0) {
        ESP_LOGV(TAG, "SLIP: Frame START (empty frame discarded)");
      } else {
        ESP_LOGW(TAG, "SLIP: END received but no frame returned (was_in_frame=%d prev_len=%u)",
                 was_in_frame ? 1 : 0, (unsigned)prev_len);
      }
    }
  }
}

void TWCDirectorComponent::handle_rx_frame_(const uint8_t *frame,
                                            size_t len,
                                            uint32_t now) {
  if (!frame || len == 0) return;

  // Filter out 1-byte frames - these are typically SLIP end-type markers (0xFC)
  // that appear between consecutive 0xC0 (END) bytes
  if (len == 1) {
    ESP_LOGV(TAG, "Ignoring 1-byte SLIP frame: 0x%02X (likely end-type marker)", frame[0]);
    return;
  }

  char hex[3 * 256 + 1];
  this->format_hex_(frame, len, hex, sizeof(hex));
  ESP_LOGD(TAG, "RX: %s (len=%u)", hex, (unsigned)len);

  // Check for 2-byte broadcast messages (ride on SLIP but aren't TWC frames)
  uint8_t bc_hi = 0, bc_lo = 0;
  twc_broadcast_kind_t bcast =
      twc_frame_classify_broadcast(frame, len, &bc_hi, &bc_lo);

  if (bcast != TWC_BCAST_NONE) {
    ESP_LOGV(TAG, "Broadcast: 0x%02X 0x%02X (kind=%d)", bc_hi, bc_lo, bcast);
    return;  // Don't process as TWC frame
  }

  // Ignore obvious noise (too short to be valid)
  if (len < 4) {
    ESP_LOGV(TAG, "Ignoring short frame (%u bytes)", len);
    return;
  }

  // Let core handle the frame (it validates checksum internally)
  // Core will update device state and trigger auto-bind callback if needed
  // If the frame passes validation, update link health timestamp
  bool valid = twc_core_handle_frame(&this->core_, frame, len, now);
  if (valid) {
    this->last_valid_frame_ms_ = now;
  }
}

// =============================================================================
// TX PATH: CORE → SLIP → UART
// =============================================================================

void TWCDirectorComponent::run_master_tick_(uint32_t now) {
  // Core master logic: heartbeats, probes, current allocation
  twc_core_master_tick(&this->core_, now);
}

void TWCDirectorComponent::handle_tx_frame_(const uint8_t *frame, size_t len) {
  if (!frame || len == 0 || len > 256) {
    ESP_LOGW(TAG, "TX frame rejected: invalid parameters (frame=%p len=%u)",
             (const void *)frame, (unsigned)len);
    return;
  }

  // Log unescaped frame
  char hex[3 * 256 + 1];
  this->format_hex_(frame, len, hex, sizeof(hex));
  ESP_LOGD(TAG, "TX: %s", hex);

  // SLIP encode
  PendingTx pending;
  size_t encoded_len = 0;

  if (!twc_frame_encode_slip(frame, len, pending.buf,
                              sizeof(pending.buf), &encoded_len)) {
    this->tx_encode_failures_++;
    ESP_LOGW(TAG, "SLIP encode failed (len=%u, total_failures=%u)",
             (unsigned)len, (unsigned)this->tx_encode_failures_);
    return;
  }

  pending.len = encoded_len;

  // Queue for transmission (with flow control)
  if (this->tx_queue_.size() >= MAX_TX_QUEUE_SIZE) {
    this->tx_frames_dropped_++;
    ESP_LOGW(TAG, "TX queue full, dropping oldest (total_dropped=%u)",
             (unsigned)this->tx_frames_dropped_);
    this->tx_queue_.erase(this->tx_queue_.begin());
  }

  this->tx_queue_.push_back(pending);
  this->tx_frames_queued_++;
}

void TWCDirectorComponent::drain_tx_queue_(uint32_t now) {
  if (this->tx_queue_.empty()) return;
  if (now < this->next_tx_at_ms_) return;

  // Bus-idle guard: wait for quiet bus before transmitting
  uint32_t idle_ms = (this->last_rx_ms_ == 0) ? 1000 : (now - this->last_rx_ms_);
  if (idle_ms < BUS_IDLE_GUARD_MS) return;

  // Send one frame
  PendingTx pending = this->tx_queue_.front();
  this->tx_queue_.erase(this->tx_queue_.begin());

  for (size_t i = 0; i < pending.len; ++i) {
    this->write(pending.buf[i]);
  }

  this->next_tx_at_ms_ = now + TX_INTERFRAME_GAP_MS;
}

// =============================================================================
// AUTO-BINDING
// =============================================================================

void TWCDirectorComponent::handle_auto_bind_(uint16_t address, twc_mode_t mode) {
  // Don't auto-bind our own address
  if (address == this->master_address_) return;

  // Don't rebind already-bound EVSEs
  for (auto &evse : this->evse_entries_) {
    if (evse.bound && evse.address == address) {
      return;  // Already bound
    }
  }

  // Find first unbound slot
  for (auto &evse : this->evse_entries_) {
    if (!evse.bound) {
      evse.bound = true;
      evse.address = address;

      // Update number entity addresses
      if (evse.max_current) {
        auto *n = static_cast<TWCDirectorCurrentNumber *>(evse.max_current);
        n->set_parent(this, address, TWCDirectorCurrentNumber::TYPE_MAX);
      }
      if (evse.initial_current) {
        auto *n = static_cast<TWCDirectorCurrentNumber *>(evse.initial_current);
        n->set_parent(this, address, TWCDirectorCurrentNumber::TYPE_INITIAL);
      }
      if (evse.session_current) {
        auto *n = static_cast<TWCDirectorCurrentNumber *>(evse.session_current);
        n->set_parent(this, address, TWCDirectorCurrentNumber::TYPE_SESSION);
      }
      if (evse.contactor) {
        evse.contactor->set_parent(this, address);
      }
      if (evse.increase_current_button) {
        evse.increase_current_button->set_parent(this, address, TWCDirectorCurrentButton::INCREASE);
      }
      if (evse.decrease_current_button) {
        evse.decrease_current_button->set_parent(this, address, TWCDirectorCurrentButton::DECREASE);
      }
      if (evse.enable_switch) {
        evse.enable_switch->set_parent(this, address);
      }

      int slot = &evse - this->evse_entries_.data();
      ESP_LOGI(TAG, "Auto-bound slot %d → TWC 0x%04X (mode=%d)",
               slot, address, (int)mode);
      return;
    }
  }

  ESP_LOGW(TAG, "Cannot auto-bind TWC 0x%04X: all slots full", address);
}

// =============================================================================
// MASTER MODE CONTROL
// =============================================================================

void TWCDirectorComponent::update_master_mode_(uint32_t now) {
  if (!this->master_mode_switch_) return;

  bool enabled = this->master_mode_switch_->state;
  
  if (enabled != this->master_mode_last_state_) {
    twc_core_set_master_mode(&this->core_, enabled);
    
    if (enabled) {
      ESP_LOGI(TAG, "Master mode ENABLED (addr=0x%04X)", this->master_address_);
    } else {
      ESP_LOGI(TAG, "Master mode DISABLED");
    }
    
    this->master_mode_last_state_ = enabled;
  }
}

// =============================================================================
// EVSE METRICS: CORE → ESPHOME SENSORS
// =============================================================================

void TWCDirectorComponent::update_evse_metrics_(uint32_t now) {
  for (auto &evse : this->evse_entries_) {
    if (!evse.bound) continue;  // Skip unbound slots
    
    // Sync desired current from number entity to core
    this->sync_desired_current_(evse);
    
    // Update all sensors from core state
    this->update_evse_sensors_(evse, now);
  }
}

void TWCDirectorComponent::sync_desired_current_(EvseEntry &evse) {
  if (!evse.initial_current || !evse.initial_current->has_state()) return;

  float desired = evse.initial_current->state;
  
  // Only update if changed (avoid spam)
  if (evse.last_initial_current_setpoint_a < 0.0f ||
      fabsf(desired - evse.last_initial_current_setpoint_a) > 0.1f) {
    twc_core_set_desired_initial_current(&this->core_, evse.address, desired);
    evse.last_initial_current_setpoint_a = desired;
  }
}

void TWCDirectorComponent::update_evse_sensors_(EvseEntry &evse, uint32_t now) {
  twc_core_device_t *core_dev = 
      twc_core_get_device_by_address(&this->core_, evse.address);
  
  if (!core_dev) return;
  
  const twc_device_t *dev = &core_dev->device;
  
  // Online status
  bool online = twc_core_device_online(&this->core_, core_dev, now);
  this->publish_binary_sensor_if_changed_(evse.online, online);

  // Mode (always publish, even if offline)
  const char *mode_str = this->mode_to_string_(twc_device_get_mode(dev));
  this->publish_text_sensor_if_changed_(evse.mode, mode_str);

  // Status text from E0 status byte
  int status_code = twc_device_get_status_code(dev);
  const char *status_str = twc_charge_state_to_string(static_cast<twc_charge_state_t>(status_code));

  // Log status transitions with current values for debugging
  if (evse.last_status_code != -1 && status_code != evse.last_status_code) {
    const char *old_status_str = twc_charge_state_to_string(
        static_cast<twc_charge_state_t>(evse.last_status_code));

    float available_a = twc_core_get_current_available_a(core_dev);
    float applied_initial_a = core_dev->applied_initial_current_a;
    float desired_initial_a = core_dev->desired_initial_current_a;

    ESP_LOGI(TAG, "TWC 0x%04X: %s -> %s (available=%.1fA, applied=%.1fA, desired=%.1fA)",
             evse.address, old_status_str, status_str,
             available_a, applied_initial_a, desired_initial_a);
  }

  this->publish_text_sensor_if_changed_(evse.status_text, status_str);
  evse.last_status_code = status_code;

  // Contactor status (always publish, even if offline)
  // NOTE: We infer contactor state from delivered current since there's no
  // reliable way to read the actual contactor status from the TWC protocol.
  // If delivered current > 0, contactors are closed. If = 0, contactors are open.
  float session_amps = this->compute_session_amps_(dev);
  bool contactor_closed = (session_amps > 0.1f);  // 0.1A threshold to avoid noise
  this->publish_binary_sensor_if_changed_(evse.contactor_status, contactor_closed);

  // Sync session current number on charging state transitions
  bool was_zero = (evse.last_session_amps < 0.1f);
  bool is_nonzero = (session_amps > 0.1f);
  if (evse.session_current) {
    if (was_zero && is_nonzero) {
      // 0 → >0: charging started, sync to TWC's available current
      float available_a = twc_device_get_current_available_a(dev);
      if (available_a > 0.0f) {
        ESP_LOGI(TAG, "TWC 0x%04X: charging started (%.1fA), syncing session current number to %.1fA",
                 evse.address, session_amps, available_a);
        evse.session_current->publish_state(available_a);
      }
    } else if (!was_zero && !is_nonzero) {
      // >0 → 0: charging stopped, reset session current number to 0
      if (evse.session_current->state > 0.1f) {
        ESP_LOGI(TAG, "TWC 0x%04X: charging stopped, resetting session current number to 0",
                 evse.address);
        evse.session_current->publish_state(0.0f);
      }
    }

    // When in "Setting Limit" status (0x09), sync session current number to the
    // applied value so it "springs back" if the requested value was scaled down
    // due to global max current constraints
    if (status_code == TWC_HB_SETTING_LIMIT) {
      twc_core_device_t *core_dev = twc_core_get_device_by_address(&this->core_, evse.address);
      if (core_dev && core_dev->applied_session_current_a > 0.0f) {
        float applied = core_dev->applied_session_current_a;
        float current_state = evse.session_current->state;
        // Only update if there's a meaningful difference (avoid float noise)
        if (fabsf(current_state - applied) > 0.1f) {
          ESP_LOGD(TAG, "TWC 0x%04X: Setting Limit - syncing session current %.1fA → %.1fA (scaled)",
                   evse.address, current_state, applied);
          evse.session_current->publish_state(applied);
        }
      }
    }
  }
  evse.last_session_amps = session_amps;

  // Sync contactor switch state from session current:
  // - When session current > 0, switch should show ON (charging active)
  // - When session current = 0, switch should show OFF (not charging)
  if (evse.contactor) {
    bool should_be_on = contactor_closed;
    if (evse.last_contactor_switch_state != should_be_on) {
      ESP_LOGD(TAG, "TWC 0x%04X: syncing contactor switch to %s (session=%.1fA)",
               evse.address, should_be_on ? "ON" : "OFF", session_amps);
      evse.contactor->publish_state(should_be_on);
      evse.last_contactor_switch_state = should_be_on;
    }
  }

  if (!online) return;  // Don't spam metrics when offline
  
  // Throttle updates
  if (evse.last_metrics_publish_ms != 0 &&
      (now - evse.last_metrics_publish_ms) < METRICS_UPDATE_INTERVAL_MS) {
    return;
  }
  evse.last_metrics_publish_ms = now;
  
  // Identity
  this->publish_nonempty_(evse.firmware_version, 
                         twc_device_get_software_version(dev));
  this->publish_nonempty_(evse.serial_number,
                         twc_device_get_serial_number(dev));
  
  // Vehicle
  bool vehicle_connected = twc_device_get_vehicle_connected(dev);
  this->publish_binary_sensor_if_changed_(evse.vehicle_connected, vehicle_connected);
  
  const char *vin = twc_device_get_vehicle_vin(dev);
  std::string vin_str = vin ? std::string(vin) : std::string();
  this->publish_text_sensor_if_changed_(evse.vehicle_vin, vin_str);
  
  // Meter (currents, voltages, energy)
  this->publish_sensor_(evse.meter_current_phase_a, 
                       twc_device_get_phase_a_current_a(dev));
  this->publish_sensor_(evse.meter_current_phase_b,
                       twc_device_get_phase_b_current_a(dev));
  this->publish_sensor_(evse.meter_current_phase_c,
                       twc_device_get_phase_c_current_a(dev));
  
  this->publish_sensor_(evse.meter_voltage_phase_a,
                       twc_device_get_phase_a_voltage_v(dev));
  this->publish_sensor_(evse.meter_voltage_phase_b,
                       twc_device_get_phase_b_voltage_v(dev));
  this->publish_sensor_(evse.meter_voltage_phase_c,
                       twc_device_get_phase_c_voltage_v(dev));
  
  this->publish_sensor_(evse.meter_energy_total,
                       twc_device_get_total_energy_kwh(dev), 0.001f);
  this->publish_sensor_(evse.meter_energy_session,
                       twc_device_get_session_energy_kwh(dev), 0.001f);
  
  // Current allocation
  float initial_current_a = twc_core_get_current_available_a(core_dev);
  this->publish_sensor_(evse.available_current_sensor, initial_current_a);

  this->publish_sensor_(evse.session_current_sensor, session_amps);
}

float TWCDirectorComponent::compute_session_amps_(const twc_device_t *dev) const {
  if (!dev) return 0.0f;
  
  float a = twc_device_get_phase_a_current_a(dev);
  float b = twc_device_get_phase_b_current_a(dev);
  float c = twc_device_get_phase_c_current_a(dev);
  
  return (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
}

// =============================================================================
// CURRENT CONTROL (ESPHome Number → Core)
// =============================================================================

void TWCDirectorComponent::handle_current_number_control(
    uint16_t address, float value, TWCDirectorCurrentNumber::CurrentType type) {
  // Special handling for global max current control
  if (type == TWCDirectorCurrentNumber::TYPE_GLOBAL_MAX) {
    // Clamp to safety maximum
    if (value < 0.0f) value = 0.0f;
    if (this->global_max_current_a_ > 0.0f && value > this->global_max_current_a_) {
      value = this->global_max_current_a_;
    }

    ESP_LOGI(TAG, "Global max current control: %.1fA (safety max: %.1fA)",
             value, this->global_max_current_a_);

    // The value is stored in the number entity itself, we just validate it here
    // The get_effective_global_max_current_() helper will read it when needed
    return;
  }

  if (address == 0) {
    ESP_LOGW(TAG, "Current control rejected: address is 0");
    return;
  }

  // Clamp to reasonable range
  if (value < 0.0f) value = 0.0f;
  float max_val = (type == TWCDirectorCurrentNumber::TYPE_MAX) ? 80.0f : this->evse_max_current_limit_a_;
  if (value > max_val) value = max_val;

  const char *type_str = (type == TWCDirectorCurrentNumber::TYPE_MAX) ? "max" :
                         (type == TWCDirectorCurrentNumber::TYPE_SESSION) ? "session" : "initial";

  ESP_LOGI(TAG, "Current control: TWC 0x%04X %s current %.1fA", address, type_str, value);

  EvseEntry *evse = this->find_evse_(address);
  if (!evse) {
    ESP_LOGW(TAG, "Current control: EVSE 0x%04X not found in slots", address);
    return;
  }

  switch (type) {
    case TWCDirectorCurrentNumber::TYPE_INITIAL:
      // Initial current: update initial setpoint and redistribute if needed
      this->handle_initial_current_change_(evse, address, value);
      break;

    case TWCDirectorCurrentNumber::TYPE_MAX:
      // Max current: authoritative upper bound
      twc_core_set_max_current(&this->core_, address, value);
      evse->last_max_current_setpoint_a = value;
      break;

    case TWCDirectorCurrentNumber::TYPE_SESSION:
      // Session current: update session setpoint
      // The C library will handle reconciliation and scheduling the 0x09 command
      // with the applied (scaled) value that respects global max current
      twc_core_set_desired_session_current(&this->core_, address, value);
      ESP_LOGI(TAG, "Set desired session current for TWC 0x%04X to %.1fA", address, value);
      break;
  }
}

void TWCDirectorComponent::handle_initial_current_change_(EvseEntry *changed_evse,
                                                          uint16_t address,
                                                          float new_value) {
  // Get the effective global max (runtime or compile-time)
  float effective_max = this->get_effective_global_max_current_();

  // If no global max, just set the value directly
  if (effective_max <= 0.0f) {
    twc_core_set_desired_initial_current(&this->core_, address, new_value);
    changed_evse->last_initial_current_setpoint_a = new_value;
    return;
  }

  // Calculate total desired current with the new value
  float total_desired = new_value;
  for (auto &evse : this->evse_entries_) {
    if (!evse.bound || evse.address == address) continue;
    if (!evse.initial_current || !evse.initial_current->has_state()) continue;
    total_desired += evse.initial_current->state;
  }

  // If we're within limit, just set it
  if (total_desired <= effective_max) {
    twc_core_set_desired_initial_current(&this->core_, address, new_value);
    changed_evse->last_initial_current_setpoint_a = new_value;
    ESP_LOGD(TAG, "Initial current %.1fA for 0x%04X OK (total %.1fA / %.1fA)",
             new_value, address, total_desired, effective_max);
    return;
  }

  // Exceeds limit - redistribute proportionally
  ESP_LOGW(TAG, "Initial current %.1fA for 0x%04X would exceed global max (total %.1fA > %.1fA), redistributing",
           new_value, address, total_desired, effective_max);

  // Calculate how much we need to reduce
  float scale = effective_max / total_desired;

  // Apply scaled values to all EVSEs (including the one being changed)
  for (auto &evse : this->evse_entries_) {
    if (!evse.bound) continue;
    if (!evse.initial_current) continue;

    float desired = (evse.address == address) ? new_value : evse.initial_current->state;
    float scaled = desired * scale;

    // Round to 0.1A for cleaner UI
    scaled = roundf(scaled * 10.0f) / 10.0f;

    // Update the number entity - this will be visible to the user
    if (evse.initial_current->state != scaled) {
      evse.initial_current->publish_state(scaled);
      ESP_LOGI(TAG, "Redistributed: TWC 0x%04X initial current %.1fA → %.1fA",
               evse.address, desired, scaled);
    }

    // Update core
    twc_core_set_desired_initial_current(&this->core_, evse.address, scaled);
    evse.last_initial_current_setpoint_a = scaled;
  }
}

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

float TWCDirectorComponent::get_effective_global_max_current_() const {
  // If runtime control is available and has a value, use it (already clamped to safety max)
  if (this->global_max_current_control_ && this->global_max_current_control_->has_state()) {
    return this->global_max_current_control_->state;
  }
  // Otherwise use compile-time value
  return this->global_max_current_a_;
}

void TWCDirectorComponent::publish_binary_sensor_if_changed_(
    binary_sensor::BinarySensor *sensor, bool value) {
  if (sensor && (!sensor->has_state() || sensor->state != value)) {
    sensor->publish_state(value);
  }
}

void TWCDirectorComponent::publish_sensor_(sensor::Sensor *sensor,
                                           float value,
                                           float epsilon) {
  if (sensor && (!sensor->has_state() || fabsf(sensor->state - value) > epsilon)) {
    sensor->publish_state(value);
  }
}

void TWCDirectorComponent::publish_text_sensor_if_changed_(
    text_sensor::TextSensor *sensor, const std::string &value) {
  if (sensor && (!sensor->has_state() || sensor->state != value)) {
    sensor->publish_state(value);
  }
}

void TWCDirectorComponent::publish_nonempty_(text_sensor::TextSensor *sensor,
                                             const char *value) {
  if (sensor && value && value[0] != '\0') {
    this->publish_text_sensor_if_changed_(sensor, value);
  }
}

void TWCDirectorComponent::format_hex_(const uint8_t *data, size_t len,
                                      char *out, size_t out_size) {
  if (!out || out_size == 0) return;
  if (!data || len == 0) {
    out[0] = '\0';
    return;
  }
  
  size_t pos = 0;
  for (size_t i = 0; i < len && pos + 4 < out_size; i++) {
    int n = snprintf(&out[pos], out_size - pos,
                     (i == 0) ? "%02X" : " %02X", data[i]);
    if (n <= 0) break;
    pos += n;
  }
  out[pos] = '\0';
}

const char *TWCDirectorComponent::mode_to_string_(twc_mode_t mode) const {
  switch (mode) {
    case TWC_MODE_MASTER: return "Master";
    case TWC_MODE_FOREIGN_MASTER: return "Foreign Master";
    case TWC_MODE_UNCONF_PERIPHERAL: return "Unconfigured Peripheral";
    case TWC_MODE_PERIPHERAL: return "Peripheral";
    case TWC_MODE_FOREIGN_PERIPHERAL: return "Foreign Peripheral";
    case TWC_MODE_UNKNOWN:
    default: return "Unknown";
  }
}

TWCDirectorComponent::EvseEntry *TWCDirectorComponent::find_evse_(uint16_t address) {
  for (auto &evse : this->evse_entries_) {
    if (evse.bound && evse.address == address) {
      return &evse;
    }
  }
  return nullptr;
}

// =============================================================================
// ENTITY IMPLEMENTATIONS
// =============================================================================

void TWCDirectorContactorSwitch::write_state(bool state) {
  if (this->parent_) {
    this->parent_->request_contactor_state(this->address_, state);
  }
  this->publish_state(state);
}

void TWCDirectorMasterModeSwitch::write_state(bool state) {
  this->publish_state(state);
}

void TWCDirectorEnableSwitch::write_state(bool state) {
  ESP_LOGI(TAG, "EnableSwitch::write_state: addr=0x%04X enabled=%s",
           this->address_, state ? "true" : "false");
  if (this->parent_) {
    this->parent_->set_evse_enabled(this->address_, state);
  }
  this->publish_state(state);
}

void TWCDirectorCurrentNumber::control(float value) {
  const char *type_str = (this->type_ == TYPE_MAX) ? "max" :
                         (this->type_ == TYPE_SESSION) ? "session" :
                         (this->type_ == TYPE_GLOBAL_MAX) ? "global_max" : "initial";

  ESP_LOGI(TAG, "CurrentNumber::control called: addr=0x%04X value=%.1fA type=%s parent=%p",
           this->address_, value, type_str, (void*)this->parent_);

  if (this->parent_) {
    // Global max doesn't need a valid address
    if (this->type_ == TYPE_GLOBAL_MAX || this->address_ != 0) {
      this->parent_->handle_current_number_control(this->address_, value, this->type_);
    } else {
      ESP_LOGW(TAG, "CurrentNumber::control: address is 0 for non-global type!");
    }
  } else {
    ESP_LOGW(TAG, "CurrentNumber::control: parent is null!");
  }
  this->publish_state(value);
}

void TWCDirectorCurrentButton::press_action() {
  const char *type_str = (this->type_ == INCREASE) ? "increase" : "decrease";
  ESP_LOGI(TAG, "CurrentButton::press_action: addr=0x%04X type=%s", this->address_, type_str);

  if (this->parent_) {
    this->parent_->request_current_change(this->address_, this->type_);
  } else {
    ESP_LOGW(TAG, "CurrentButton::press_action: parent is null!");
  }
}

void TWCDirectorComponent::request_current_change(uint16_t address,
                                                   TWCDirectorCurrentButton::ButtonType type) {
  if (!this->master_mode_enabled()) {
    ESP_LOGW(TAG, "Current change request ignored: master mode not enabled");
    return;
  }

  if (type == TWCDirectorCurrentButton::INCREASE) {
    ESP_LOGI(TAG, "Queuing increase current command for TWC 0x%04X", address);
    twc_core_send_increase_current(&this->core_, address);
  } else {
    ESP_LOGI(TAG, "Queuing decrease current command for TWC 0x%04X", address);
    twc_core_send_decrease_current(&this->core_, address);
  }
}

// =============================================================================
// CONFIGURATION
// =============================================================================

void TWCDirectorComponent::set_master_address(uint16_t addr) {
  this->master_address_ = addr;
}

void TWCDirectorComponent::set_evse_enabled(uint16_t address, bool enabled) {
  EvseEntry *evse = this->find_evse_(address);
  if (!evse) {
    // For auto-bind slots that haven't been bound yet, find by slot index
    for (auto &e : this->evse_entries_) {
      if (!e.bound && e.enable_switch && e.enable_switch->state != enabled) {
        // This is likely the unbound slot being toggled
        e.enabled = enabled;
        ESP_LOGI(TAG, "EVSE (unbound slot) enabled=%s", enabled ? "true" : "false");
        return;
      }
    }
    ESP_LOGW(TAG, "set_evse_enabled: EVSE 0x%04X not found", address);
    return;
  }

  evse->enabled = enabled;
  ESP_LOGI(TAG, "EVSE 0x%04X enabled=%s", address, enabled ? "true" : "false");

  // If disabling, also tell the core to stop sending heartbeats to this device
  twc_core_device_t *core_dev = twc_core_get_device_by_address(&this->core_, address);
  if (core_dev) {
    twc_core_set_device_enabled(&this->core_, address, enabled);
  }
}

void TWCDirectorComponent::request_contactor_state(uint16_t address, bool closed) {
  ESP_LOGI(TAG, "Contactor %s request for TWC 0x%04X",
           closed ? "CLOSE" : "OPEN", address);

  // Send contactor command frame (0xB1 = close, 0xB2 = open)
  // Uses TWC_MARKER_ANNOUNCE (0xFC) per protocol spec
  twc_cmd_t cmd = closed ? TWC_CMD_CLOSE_CONTACTORS : TWC_CMD_OPEN_CONTACTORS;

  uint8_t frame[16];
  size_t len = twc_build_contactor_frame(this->master_address_, address, cmd,
                                         frame, sizeof(frame));

  if (len > 0) {
    this->handle_tx_frame_(frame, len);
  } else {
    ESP_LOGW(TAG, "Failed to build contactor %s frame for TWC 0x%04X",
             closed ? "CLOSE" : "OPEN", address);
  }
}

void TWCDirectorComponent::add_evse(
    uint16_t address,
    binary_sensor::BinarySensor *online,
    text_sensor::TextSensor *firmware_version,
    text_sensor::TextSensor *serial_number,
    binary_sensor::BinarySensor *vehicle_connected,
    text_sensor::TextSensor *vehicle_vin,
    sensor::Sensor *meter_current_phase_a,
    sensor::Sensor *meter_current_phase_b,
    sensor::Sensor *meter_current_phase_c,
    sensor::Sensor *meter_voltage_phase_a,
    sensor::Sensor *meter_voltage_phase_b,
    sensor::Sensor *meter_voltage_phase_c,
    sensor::Sensor *meter_energy_total,
    sensor::Sensor *meter_energy_session,
    TWCDirectorContactorSwitch *contactor,
    binary_sensor::BinarySensor *contactor_status,
    sensor::Sensor *available_current_sensor,
    number::Number *initial_current,
    number::Number *max_current,
    sensor::Sensor *max_current_sensor,
    number::Number *session_current,
    sensor::Sensor *session_current_sensor,
    text_sensor::TextSensor *mode,
    text_sensor::TextSensor *status_text,
    TWCDirectorCurrentButton *increase_current_button,
    TWCDirectorCurrentButton *decrease_current_button,
    TWCDirectorEnableSwitch *enable_switch) {
  EvseEntry entry{};
  entry.address = address;
  entry.auto_bound = (address == 0);
  entry.bound = (address != 0);

  // Wire up all the sensors/entities
  entry.online = online;
  entry.mode = mode;
  entry.status_text = status_text;
  entry.firmware_version = firmware_version;
  entry.serial_number = serial_number;
  entry.vehicle_connected = vehicle_connected;
  entry.vehicle_vin = vehicle_vin;
  entry.meter_current_phase_a = meter_current_phase_a;
  entry.meter_current_phase_b = meter_current_phase_b;
  entry.meter_current_phase_c = meter_current_phase_c;
  entry.meter_voltage_phase_a = meter_voltage_phase_a;
  entry.meter_voltage_phase_b = meter_voltage_phase_b;
  entry.meter_voltage_phase_c = meter_voltage_phase_c;
  entry.meter_energy_total = meter_energy_total;
  entry.meter_energy_session = meter_energy_session;
  entry.contactor = contactor;
  entry.contactor_status = contactor_status;
  entry.available_current_sensor = available_current_sensor;
  entry.initial_current = initial_current;
  entry.max_current = max_current;
  entry.max_current_sensor = max_current_sensor;
  entry.session_current = session_current;
  entry.session_current_sensor = session_current_sensor;

  if (contactor) contactor->set_parent(this, address);

  if (max_current) {
    auto *n = static_cast<TWCDirectorCurrentNumber *>(max_current);
    n->set_parent(this, address, TWCDirectorCurrentNumber::TYPE_MAX);
  }

  if (initial_current) {
    auto *n = static_cast<TWCDirectorCurrentNumber *>(initial_current);
    n->set_parent(this, address, TWCDirectorCurrentNumber::TYPE_INITIAL);
  }

  if (session_current) {
    auto *n = static_cast<TWCDirectorCurrentNumber *>(session_current);
    n->set_parent(this, address, TWCDirectorCurrentNumber::TYPE_SESSION);
  }

  entry.increase_current_button = increase_current_button;
  entry.decrease_current_button = decrease_current_button;
  entry.enable_switch = enable_switch;

  if (increase_current_button) {
    increase_current_button->set_parent(this, address, TWCDirectorCurrentButton::INCREASE);
  }

  if (decrease_current_button) {
    decrease_current_button->set_parent(this, address, TWCDirectorCurrentButton::DECREASE);
  }

  if (enable_switch) {
    enable_switch->set_parent(this, address);
    // Default to enabled (switch ON)
    entry.enabled = true;
  }

  evse_entries_.push_back(entry);
}

}  // namespace twc_director
}  // namespace esphome