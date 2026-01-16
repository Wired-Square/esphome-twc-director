#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/button/button.h"

extern "C" {
#include "twc/twc_frame.h"
#include "twc/twc_core.h"
#include "twc/twc_protocol.h"
}

namespace esphome {
namespace twc_director {

class TWCDirectorComponent;

// Concrete switch entity used for per-EVSE contactor control. The actual
// bus-level behaviour is delegated back to TWCDirectorComponent via
// request_contactor_state().
class TWCDirectorContactorSwitch : public switch_::Switch {
 public:
  void set_parent(TWCDirectorComponent *parent, uint16_t address) {
    this->parent_ = parent;
    this->address_ = address;
  }

 protected:
  void write_state(bool state) override;

  TWCDirectorComponent *parent_{nullptr};
  uint16_t address_{0};
};

class TWCDirectorMasterModeSwitch : public switch_::Switch {
 public:
  void set_parent(TWCDirectorComponent *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;

  TWCDirectorComponent *parent_{nullptr};
};

// Per-EVSE enable switch. When OFF, the TWC Director will not communicate with
// this EVSE (no heartbeats, no current allocation). This effectively stops charging.
class TWCDirectorEnableSwitch : public switch_::Switch {
 public:
  void set_parent(TWCDirectorComponent *parent, uint16_t address) {
    this->parent_ = parent;
    this->address_ = address;
  }

 protected:
  void write_state(bool state) override;

  TWCDirectorComponent *parent_{nullptr};
  uint16_t address_{0};
};

class TWCDirectorCurrentNumber : public number::Number {
 public:
  enum CurrentType {
    TYPE_MAX,        // Max current (authoritative upper bound)
    TYPE_INITIAL,    // Initial current (0x05 frame)
    TYPE_SESSION,    // Session current (0x09 frame)
    TYPE_GLOBAL_MAX  // Global max current control (runtime adjustable)
  };

  TWCDirectorCurrentNumber() = default;

  void set_parent(TWCDirectorComponent *parent, uint16_t address, CurrentType type) {
    this->parent_ = parent;
    this->address_ = address;
    this->type_ = type;
  }

 protected:
  void control(float value) override;

  TWCDirectorComponent *parent_{nullptr};
  uint16_t address_{0};
  CurrentType type_{TYPE_INITIAL};
};

// Button entity for relative current adjustment commands (0x06 increase, 0x07 decrease).
// The actual bus-level behaviour is delegated back to TWCDirectorComponent.
class TWCDirectorCurrentButton : public button::Button {
 public:
  enum ButtonType { INCREASE, DECREASE };

  void set_parent(TWCDirectorComponent *parent, uint16_t address, ButtonType type) {
    this->parent_ = parent;
    this->address_ = address;
    this->type_ = type;
  }

 protected:
  void press_action() override;

  TWCDirectorComponent *parent_{nullptr};
  uint16_t address_{0};
  ButtonType type_{INCREASE};
};

class TWCDirectorComponent : public Component, public uart::UARTDevice {
 public:
  explicit TWCDirectorComponent(uart::UARTComponent *parent)
      : uart::UARTDevice(parent) {}

  static constexpr std::uint16_t MASTER_ID = 0xF00D;

  // Configure the TWC Director's own address on the RS-485 bus. This value
  // will be used as the sender ID for controller (E1) frames when acting as
  // master.
  void set_master_address(std::uint16_t addr);

  std::uint16_t master_address() const { return this->master_address_; }

  // Component lifecycle
  void setup() override;
  void loop() override;

  // Called by TWCDirectorContactorSwitch when the contactor switch is toggled.
  // This is where bus-level contactor control will eventually be implemented.
  void request_contactor_state(uint16_t address, bool closed);

  // Called by TWCDirectorCurrentButton when the increase/decrease button is pressed.
  void request_current_change(uint16_t address, TWCDirectorCurrentButton::ButtonType type);

  // Called by TWCDirectorEnableSwitch when the enable switch is toggled.
  // When disabled, the TWC Director will not communicate with this EVSE.
  void set_evse_enabled(uint16_t address, bool enabled);

  void add_evse(uint16_t address,
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
                text_sensor::TextSensor *status_log,
                TWCDirectorCurrentButton *increase_current_button,
                TWCDirectorCurrentButton *decrease_current_button,
                TWCDirectorEnableSwitch *enable_switch);

  void set_link_ok_sensor(binary_sensor::BinarySensor *sensor) {
    this->link_ok_sensor_ = sensor;
  }

  // Optional master mode switch. We only store this pointer and expose a
  // simple helper for reading its current state.
  void set_master_mode_switch(TWCDirectorMasterModeSwitch *sw) {
    this->master_mode_switch_ = sw;
    if (sw != nullptr) {
      sw->set_parent(this);
    }
  }

  bool master_mode_enabled() const {
    return this->master_mode_switch_ != nullptr &&
           this->master_mode_switch_->state;
  }

  // Number of frames that failed checksum validation.
  std::uint32_t checksum_error_count() const { return this->checksum_error_count_; }

  void set_global_max_current(float max_current_a) {
    this->global_max_current_a_ = max_current_a;
  }

  void set_evse_max_current_limit(float max_current_a) {
    this->evse_max_current_limit_a_ = max_current_a;
  }

  // Optional global max current control number entity for runtime adjustment
  void set_global_max_current_control(number::Number *num) {
    this->global_max_current_control_ = num;
    if (num != nullptr) {
      auto *n = static_cast<TWCDirectorCurrentNumber *>(num);
      n->set_parent(this, 0xFFFF, TWCDirectorCurrentNumber::TYPE_GLOBAL_MAX);
    }
  }

  // Called by TWCDirectorCurrentNumber::control when the operator adjusts
  // max, initial, or session current from the host.
  void handle_current_number_control(uint16_t address, float value,
                                      TWCDirectorCurrentNumber::CurrentType type);

  // =========================================================================
  // Timing Constants
  // =========================================================================

  // How often to push metric updates while an EVSE is online.
  static constexpr std::uint32_t METRICS_UPDATE_INTERVAL_MS = 1000U;

  // Maximum number of frames to buffer in the TX queue before dropping oldest.
  static constexpr std::size_t MAX_TX_QUEUE_SIZE = 8;

  // Interval between SLIP decoder diagnostic log messages.
  static constexpr std::uint32_t DECODER_STATS_INTERVAL_MS = 60000U;

  // =========================================================================
  // TX Statistics
  // =========================================================================

  // Get number of frames successfully queued for transmission.
  std::uint32_t tx_frames_queued() const { return this->tx_frames_queued_; }

  // Get number of frames dropped due to queue overflow.
  std::uint32_t tx_frames_dropped() const { return this->tx_frames_dropped_; }

  // Get number of SLIP encode failures.
  std::uint32_t tx_encode_failures() const { return this->tx_encode_failures_; }

  struct EvseEntry {
    uint16_t address;
    bool auto_bound{false};  // true if this slot should be auto-bound to a discovered device
    bool bound{false};       // true once we've assigned a concrete address

    binary_sensor::BinarySensor *online;
    text_sensor::TextSensor *firmware_version{nullptr};
    text_sensor::TextSensor *serial_number{nullptr};
    binary_sensor::BinarySensor *vehicle_connected{nullptr};
    text_sensor::TextSensor *vehicle_vin{nullptr};
    // Optional meter sensors for this EVSE
    sensor::Sensor *meter_current_phase_a{nullptr};
    sensor::Sensor *meter_current_phase_b{nullptr};
    sensor::Sensor *meter_current_phase_c{nullptr};
    sensor::Sensor *meter_voltage_phase_a{nullptr};
    sensor::Sensor *meter_voltage_phase_b{nullptr};
    sensor::Sensor *meter_voltage_phase_c{nullptr};
    sensor::Sensor *meter_energy_total{nullptr};
    sensor::Sensor *meter_energy_session{nullptr};
    // Per-EVSE entities.
    TWCDirectorContactorSwitch *contactor{nullptr};
    binary_sensor::BinarySensor *contactor_status{nullptr};
    sensor::Sensor *available_current_sensor{nullptr};
    number::Number *initial_current{nullptr};
    number::Number *max_current{nullptr};
    sensor::Sensor *max_current_sensor{nullptr};
    number::Number *session_current{nullptr};
    sensor::Sensor *session_current_sensor{nullptr};
    text_sensor::TextSensor *mode{nullptr};
    text_sensor::TextSensor *status_text{nullptr};
    text_sensor::TextSensor *status_log{nullptr};
    TWCDirectorCurrentButton *increase_current_button{nullptr};
    TWCDirectorCurrentButton *decrease_current_button{nullptr};
    TWCDirectorEnableSwitch *enable_switch{nullptr};

    // Whether this EVSE is enabled for communication. When false, the TWC Director
    // will not send heartbeats or current allocations to this EVSE.
    bool enabled{true};

    // Last operator-set currents (amps) for this EVSE, used to avoid
    // re-applying the same setpoint on every loop.
    float last_max_current_setpoint_a{-1.0f};
    float last_initial_current_setpoint_a{-1.0f};

    // Last time we pushed a metrics update for this EVSE while online.
    std::uint32_t last_metrics_publish_ms{0};

    // Previous status code for detecting state transitions (e.g., to "Setting limit")
    int last_status_code{-1};

    // Track last contactor switch state we published to avoid redundant updates
    bool last_contactor_switch_state{false};

    // Track previous session amps to detect 0 → >0 transition for syncing session current number
    float last_session_amps{0.0f};
  };

  // Helper methods that avoid redundant entity updates.
  void publish_binary_sensor_if_changed_(binary_sensor::BinarySensor *sensor, bool value);
  void publish_sensor_if_changed_(sensor::Sensor *sensor, float value, float epsilon = 0.01f);
  void publish_text_sensor_if_changed_(text_sensor::TextSensor *sensor, const std::string &value);
  const char *mode_to_string_(twc_mode_t mode) const;
  EvseEntry *find_evse_(uint16_t address);

  // Current control helper: handles initial current changes with redistribution
  void handle_initial_current_change_(EvseEntry *changed_evse, uint16_t address, float new_value);

  // Helpers used by complete_frame_ to keep the main flow small and readable.
  bool handle_broadcast_or_noise_(const uint8_t *frame, size_t len, const char *raw_hex);
  bool update_checksum_and_log_(const uint8_t *frame, size_t len);
  void classify_and_autobind_(const uint8_t *frame, size_t len, const char *raw_hex);
  void publish_last_frame_(const char *raw_hex);

  // Get the effective global max current (runtime control if available, otherwise compile-time)
  float get_effective_global_max_current_() const;

 protected:
  TWCDirectorMasterModeSwitch *master_mode_switch_{nullptr};
  binary_sensor::BinarySensor *link_ok_sensor_{nullptr};

  bool master_mode_last_state_{false};

  // Link health tracking
  uint32_t last_valid_frame_ms_{0};
  static constexpr uint32_t LINK_TIMEOUT_MS = 15000;  // 15 seconds without valid frame = link down

  std::uint32_t checksum_error_count_{0};

  bool last_checksum_ok_{false};

  // The TWC Director's own address on the RS-485 bus. Defaults to MASTER_ID
  // but can be overridden via configuration.
  std::uint16_t master_address_{MASTER_ID};

  static constexpr size_t MAX_FRAME_LEN = 256;

  std::vector<EvseEntry> evse_entries_;

  // TX scheduling: simple queue of encoded SLIP frames and timing for
  // inter-frame spacing to avoid spamming the wire.
  struct PendingTx {
    std::size_t len{0};
    uint8_t buf[2 + 2 * MAX_FRAME_LEN]{};
  };

  std::vector<PendingTx> tx_queue_;
  std::uint32_t next_tx_at_ms_{0};
  // Last time (in ms) we observed any byte on the RX line. Used as a simple
  // bus-idle heuristic so we avoid starting a transmission in the middle of
  // someone else's frame.
  std::uint32_t last_rx_ms_{0};

  static constexpr std::uint32_t TX_INTERFRAME_GAP_MS = 5;
  static constexpr std::uint32_t BUS_IDLE_GUARD_MS = 10;

  // Global maximum current (in amps) for all EVSEs, set at core/component level.
  // This is the compile-time safety maximum.
  float global_max_current_a_{0.0f};

  // Per-EVSE maximum current limit (defaults to 32A for international markets).
  float evse_max_current_limit_a_{32.0f};

  // Optional runtime-adjustable global max current control (limited to global_max_current_a_)
  number::Number *global_max_current_control_{nullptr};

  // TX statistics for diagnostics
  std::uint32_t tx_frames_queued_{0};
  std::uint32_t tx_frames_dropped_{0};
  std::uint32_t tx_encode_failures_{0};

  // Core TWC state (C99 library) used for presence/online tracking,
  // automatic EVSE binding, and real-world metrics/orchestration.
  twc_core_t core_{};

  // High-level loop helpers.
  void process_rx_(std::uint32_t now);
  void update_master_mode_(std::uint32_t now);
  void update_evse_metrics_(std::uint32_t now);
  void run_master_tick_(std::uint32_t now);
  void drain_tx_queue_(std::uint32_t now);

  // RX frame handling
  void handle_rx_frame_(const uint8_t *frame, size_t len, uint32_t now);

  // Per-EVSE update helpers used by update_evse_metrics_.
  void sync_desired_current_(EvseEntry &evse);
  void update_evse_sensors_(EvseEntry &evse, uint32_t now);

  // Helper methods for publishing sensor values
  void publish_sensor_(sensor::Sensor *sensor, float value, float epsilon = 0.01f);
  void publish_nonempty_(text_sensor::TextSensor *sensor, const char *value);
  void format_hex_(const uint8_t *data, size_t len, char *out, size_t out_size);

  // Compute a single "session amps" value from the per-phase currents.
  float compute_session_amps_(const twc_device_t *dev) const;

  // Auto-binding helper for discovered peripherals.
  void handle_auto_bind_(uint16_t address, twc_mode_t mode);

  // Feed a single UART byte into the SLIP decoder.
  void handle_rx_byte_(uint8_t byte);

  // SLIP frame decoder state (implemented in C library).
  twc_frame_decoder_t decoder_{};


  // Called when we reach END and have a non-empty decoded frame.
  void complete_frame_(const uint8_t *frame, size_t len);

  /// SLIP-encode and send a raw TWC frame over the UART. The input buffer
  /// should contain the unescaped TWC frame bytes (including type/command,
  /// payload and checksum). This helper will:
  ///   - log the raw frame at INFO,
  ///   - use the C library's twc_frame_encode_slip() to wrap it in SLIP, and
  ///   - write the encoded bytes to the UART.
  void handle_tx_frame_(const uint8_t *frame, size_t len);

  friend void tx_callback_shim(const uint8_t *frame,
                                size_t len,
                                void *user_data);
  friend void negotiation_callback_shim(uint16_t address,
                                        float applied_initial_current_a,
                                        std::uint8_t session_id,
                                        void *user_data);
  friend void autobind_callback_shim(uint16_t address,
                                     twc_mode_t mode,
                                     void *user_data);

};

}  // namespace twc_director
}  // namespace esphome