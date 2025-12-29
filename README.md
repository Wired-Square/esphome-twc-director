# ESPHome TWC Director

An ESPHome implementation of the Gen2 Tesla Wall Connector (TWC) RS-485 protocol, allowing you to control and monitor Tesla Wall Chargers directly from Home Assistant.

## Overview

This component implements a TWC Director (master controller) that communicates with Tesla Gen2 Wall Connectors over RS-485. It enables:

- **Load sharing** across multiple Wall Connectors (up to 4 units: 1 master + 3 peripherals)
- **Real-time monitoring** of charging status, power delivery, and vehicle information
- **Dynamic current allocation** to optimize charging across multiple vehicles
- **Home Assistant integration** with full sensor and control entities

## Features

- Monitor charging current, voltage, and energy consumption per phase
- Control contactor state (start/stop charging) remotely
- View vehicle VIN, connection status, and charging state
- Adjust current limits dynamically through Home Assistant
- Support for automatic device discovery and binding
- Built-in web interface for standalone operation

## Hardware Requirements

### Required Components

1. **ESP32 Development Board** (ESP32-WROOM or similar)
2. **RS-485 Transceiver Module** (MAX13487E, MAX485, or compatible)
3. **Tesla Gen2 Wall Connector(s)** with RS-485 enabled

### Wiring

Connect your RS-485 transceiver to the ESP32:

| ESP32 Pin | RS-485 Module | Function | Description |
|-----------|---------------|----------|-------------|
| GPIO22 | TX/DI | UART Transmit | Data sent from ESP32 to TWC |
| GPIO21 | RX/RO | UART Receive | Data received from TWC |
| GPIO16 | EN (5V Boost) | Power control | Optional: enables 5V boost converter for transceiver power |
| GPIO19 | ~SHDN | Shutdown control | Active LOW shutdown; drive HIGH to enable transceiver |
| GPIO17 | RE | Receiver Enable | Active HIGH enables receiver; drive HIGH for normal operation |
| 3.3V | VCC | Power | Transceiver logic power (or 5V depending on module) |
| GND | GND | Ground | Common ground |

Connect the RS-485 module A/B lines to your Tesla Wall Connector's RS-485 terminals.

#### Pin Function Details

- **TX/DI (Driver Input)**: UART transmit data from ESP32 goes to the RS-485 driver input
- **RX/RO (Receiver Output)**: RS-485 receiver output connects to ESP32 UART receive
- **~SHDN (Shutdown)**: Some transceivers (e.g., MAX13487E) have an active-low shutdown pin. Drive this HIGH for normal operation
- **RE (Receiver Enable)**: Controls whether the transceiver listens to the bus. For half-duplex operation, keep HIGH to always listen. Some designs tie this to DE (Driver Enable) inverted for automatic direction control

#### Compatible RS-485 Transceivers

| Module | Notes |
|--------|-------|
| MAX13487E | Auto-direction, 3.3V logic, recommended |
| MAX485 | Manual direction control via DE/RE pins |
| SP3485 | 3.3V compatible, low power |
| SN65HVD72 | 3.3V, ESD protected |

> **Note**: The pin configuration above is based on the example YAML. Adjust according to your specific RS-485 module requirements. Ensure your transceiver supports 3.3V logic levels or use level shifters.

## Installation

### Method 1: ESPHome Dashboard (Recommended)

1. **Add External Component** to your ESPHome configuration:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Wired-Square/esphome-twc-director.git
```

2. **Copy the example configuration** from [tesla-director.yaml](tesla-director.yaml) and customize for your setup.

3. **Compile and upload** through the ESPHome dashboard.

### Method 2: Command Line

```bash
# Clone the repository
git clone https://github.com/Wired-Square/esphome-twc-director.git
cd esphome-twc-director

# Edit tesla-director.yaml with your configuration
# Then compile and upload
esphome run tesla-director.yaml
```

## Configuration

### Basic Setup

Here's a minimal configuration example:

```yaml
substitutions:
  global_max_current: "48"      # Total circuit maximum for all TWCs (amps)
  global_twc_max_current: "32"  # Maximum per TWC (amps)
  master_addr: "0xF00D"         # TWC Director address (2-byte hex)
  twc_0_name: "Garage Charger"  # Friendly name
  twc_0_initial_current: "15"   # Initial current allocation (amps)
  twc_0_max_current: "32"       # Max current for this TWC (amps)

esphome:
  name: tesla-director
  friendly_name: Tesla Director
  project:
    name: "Wired Square.Tesla Wall Connector Director"
    version: "v0.0.1"

esp32:
  board: esp32dev
  framework:
    type: esp-idf

external_components:
  - source:
      type: git
      url: https://github.com/Wired-Square/esphome-twc-director.git

# UART configuration for RS-485
uart:
  id: twc_uart
  tx_pin: GPIO22
  rx_pin: GPIO21
  baud_rate: 9600
  stop_bits: 1
  parity: NONE

# TWC Director component
twc_director:
  id: twc_component
  uart_id: twc_uart
  addr: ${master_addr}
  global_max_current: ${global_max_current}
  global_twc_max_current: ${global_twc_max_current}

  master_mode:
    id: twc_master_mode
    # Automatically enabled on boot via on_boot automation

  link_ok:
    name: "RS-485 Link Status"

  evse:
    - name: ${twc_0_name}
      twc_online:
        name: "${twc_0_name} Online"

      vehicle_connected:
        name: "${twc_0_name} Vehicle Connected"

      vehicle_vin:
        name: "${twc_0_name} Vehicle VIN"

      firmware_version:
        name: "${twc_0_name} Firmware"

      serial_number:
        name: "${twc_0_name} Serial"

      # Metering sensors
      meter_current_phase_a:
        name: "${twc_0_name} Current"

      meter_voltage_phase_a:
        name: "${twc_0_name} Voltage"

      meter_energy_session:
        name: "${twc_0_name} Session Energy"

      meter_energy_total:
        name: "${twc_0_name} Total Energy"

      # Control entities
      contactor:
        name: "${twc_0_name} Charging"

      contactor_status:
        name: "${twc_0_name} Contactor Closed"

      max_current:
        id: twc_0_max_current_num
        name: "${twc_0_name} Max Current"

      initial_current:
        id: twc_0_initial_current_num
        name: "${twc_0_name} Initial Current"

      session_current:
        name: "${twc_0_name} Session Current"

      # Current adjustment buttons
      increase_current:
        name: "${twc_0_name} Increase Current"

      decrease_current:
        name: "${twc_0_name} Decrease Current"

      # Enable/disable switch
      enable:
        name: "${twc_0_name} Enable"

      mode:
        name: "${twc_0_name} Mode"

      status_text:
        name: "${twc_0_name} Status"

# WiFi configuration
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

  ap:
    ssid: "Tesla-Director Fallback"
    password: !secret wifi_hotspot_password

# Home Assistant API
api:
  encryption:
    key: !secret tesla_director_api_key

# OTA updates
ota:
  - platform: esphome
    password: !secret tesla_director_ota_password

# Safe mode for recovery if firmware causes boot loops
safe_mode:
  boot_is_good_after: 60s
  num_attempts: 5

# Web server for standalone access
web_server:
  port: 80
  auth:
    username: !secret web_username
    password: !secret web_password

# Logger configuration
logger:
  level: INFO
  baud_rate: 0  # Disable serial logging to avoid UART conflicts
```

### Secrets Configuration

Create a `secrets.yaml` file in your ESPHome directory:

```yaml
wifi_ssid: "YourWiFiSSID"
wifi_password: "YourWiFiPassword"
wifi_hotspot_password: "FallbackHotspotPassword"
tesla_director_api_key: "your-32-character-encryption-key"
tesla_director_ota_password: "your-ota-password"
web_username: "admin"
web_password: "your-web-password"
```

### Multiple Wall Connectors

To configure multiple TWCs, add additional entries to the `evse` list:

```yaml
substitutions:
  global_max_current: "80"
  twc_0_name: "Garage Left"
  twc_1_name: "Garage Right"

twc_director:
  # ... other config ...
  evse:
    - name: ${twc_0_name}
      # ... sensors and controls ...

    - name: ${twc_1_name}
      # ... sensors and controls ...
```

## Home Assistant Integration

Once configured and running, the TWC Director will automatically appear in Home Assistant through the ESPHome integration.

### Available Entities

For each configured Wall Connector, you'll get:

#### Binary Sensors
- **TWC Online** - Connection status
- **Vehicle Connected** - Whether a vehicle is plugged in
- **Contactor Closed** - Actual contactor state
- **RS-485 Link Status** - Communication health

#### Sensors
- **Current (A)** - Real-time charging current per phase
- **Voltage (V)** - Line voltage per phase
- **Session Energy (kWh)** - Energy delivered in current session
- **Total Energy (kWh)** - Lifetime energy delivered
- **Available Current** - Current allocated by director
- **Max Current** - Upper bound for this TWC
- **Session Current** - Active session current limit

#### Text Sensors
- **Vehicle VIN** - Connected vehicle identification
- **Firmware Version** - TWC firmware version
- **Serial Number** - TWC serial number
- **Mode** - Current operational mode
- **Status** - Human-readable status

#### Controls
- **Master Mode Switch** - Enable/disable director mode
- **Enable Switch** - Enable/disable communication with individual TWC (stops charging when OFF)
- **Charging Switch** - Start/stop charging (contactor control)
- **Max Current Number** - Adjust maximum current limit
- **Initial Current Number** - Set startup current allocation
- **Session Current Number** - Adjust active session current
- **Increase Current Button** - Request TWC to increase current (sends 0x06 command)
- **Decrease Current Button** - Request TWC to decrease current (sends 0x07 command)

### Automation Examples

#### Start charging when solar production is high

```yaml
automation:
  - alias: "Start TWC charging with excess solar"
    trigger:
      - platform: numeric_state
        entity_id: sensor.solar_excess_power
        above: 3000  # 3kW excess
    action:
      - service: switch.turn_on
        target:
          entity_id: switch.garage_charger_charging
      - service: number.set_value
        target:
          entity_id: number.garage_charger_max_current
        data:
          value: 16  # ~3.8kW at 240V
```

#### Stop charging when battery is full

```yaml
automation:
  - alias: "Stop TWC charging when car battery full"
    trigger:
      - platform: numeric_state
        entity_id: sensor.garage_charger_current
        below: 0.5
        for:
          minutes: 5
    condition:
      - condition: state
        entity_id: binary_sensor.garage_charger_vehicle_connected
        state: 'on'
    action:
      - service: switch.turn_off
        target:
          entity_id: switch.garage_charger_charging
```

#### Dynamic load balancing

```yaml
automation:
  - alias: "Balance TWC current with home load"
    trigger:
      - platform: state
        entity_id: sensor.home_power_usage
    action:
      - service: number.set_value
        target:
          entity_id: number.garage_charger_max_current
        data:
          value: >
            {% set max_circuit = 80 %}
            {% set home_current = (states('sensor.home_power_usage') | float / 240) %}
            {% set available = max_circuit - home_current %}
            {{ [6, available, 32] | sort | list | 1 }}  # Clamp between 6-32A
```

#### Disable TWC during peak hours

```yaml
automation:
  - alias: "Disable TWC charging during peak hours"
    trigger:
      - platform: time
        at: "16:00:00"
    action:
      - service: switch.turn_off
        target:
          entity_id: switch.garage_charger_enable

  - alias: "Re-enable TWC charging after peak hours"
    trigger:
      - platform: time
        at: "21:00:00"
    action:
      - service: switch.turn_on
        target:
          entity_id: switch.garage_charger_enable
```

## Current Allocation Behavior

The TWC Director manages three types of current limits that work together:

### Current Types

| Current Type | Purpose | When Applied |
|--------------|---------|--------------|
| **Max Current** | Hard upper limit per TWC | Always enforced - session/initial current cannot exceed this |
| **Initial Current** | Current allocated when TWC first starts charging | Sent via 0x05 heartbeat command |
| **Session Current** | Dynamic current limit during active charging | Sent via 0x09 heartbeat command |

### Global Limits

- **`global_max_current`** - Total current budget across ALL TWCs combined
- **`global_twc_max_current`** - Maximum any single TWC can receive (safety clamp)

### Automatic Reconciliation

When multiple TWCs are charging simultaneously, the director automatically scales session currents to respect the global limit:

1. Each TWC's desired session current is clamped to its individual max current
2. Total desired current is calculated across all actively charging TWCs
3. If total exceeds `global_max_current`, all session currents are scaled proportionally
4. The scaled "applied" values are sent to TWCs

**Example**: Two TWCs each want 32A, but `global_max_current` is 48A:
- Each TWC receives 24A (48A ÷ 2)

Reconciliation is triggered when:
- A TWC starts or stops drawing current
- `max_current` is changed for any TWC
- `global_max_current` is changed
- A new TWC comes online in a charging state

### Charging States

The **Status** text sensor shows the current TWC state:

| Status | Description |
|--------|-------------|
| Ready | Idle, waiting for vehicle |
| Charging | Actively delivering power |
| Charge Started | Vehicle just connected, ramping up |
| Setting Limit | Applying new current limit |
| Adjustment Complete | Current limit change confirmed |
| Increase Current OK | TWC acknowledged increase request |
| Decrease Current OK | TWC acknowledged decrease request |
| Negotiating | Protocol negotiation in progress |
| Waiting | Waiting for master commands |
| Error | Fault condition |

## Protocol Details

This component implements the Tesla Gen2 Wall Connector RS-485 protocol. For detailed protocol documentation, see [PROTOCOL.md](components/twc_director/twc/PROTOCOL.md).

Key protocol features:
- **SLIP framing** for reliable RS-485 communication
- **Master/peripheral architecture** with automatic negotiation
- **Load sharing** with dynamic current allocation
- **Comprehensive metering** including per-phase measurements
- **Vehicle identification** via VIN reporting

## Troubleshooting

### No communication with TWC

1. **Check wiring**: Verify A/B lines are connected correctly
2. **Check RS-485 enable pins**: Ensure shutdown and RE pins are high
3. **Verify UART config**: Should be 9600 baud, 8N1
4. **Check master address**: Must not conflict with TWC addresses

### TWC shows offline

1. **Increase log level**: Set `logger: level: VERBOSE` to see detailed protocol traffic
2. **Check RS-485 Link Status** sensor in Home Assistant
3. **Verify physical connections**: Look for loose wiring
4. **Check power**: Ensure TWC is powered and LED is lit

### Current limits not applying

1. **Verify master mode**: The master mode switch must be ON
2. **Check global limits**: Individual TWC limits cannot exceed global max
3. **Review logs**: Look for negotiation frames in verbose logs
4. **Restart device**: Some changes require a protocol renegotiation

### Enable debug logging

```yaml
logger:
  level: VERBOSE
  logs:
    twc_director: VERBOSE
```

## Development

### Project Structure

```
esphome-twc-director/
├── components/
│   └── twc_director/
│       ├── __init__.py              # ESPHome component registration
│       ├── twc_director_component.cpp  # Main component implementation
│       ├── twc_director_component.h    # Component header
│       ├── twc_lib_shim.cpp         # C/C++ library bridge
│       └── twc/                     # Core TWC protocol library (C)
│           ├── PROTOCOL.md          # Protocol documentation
│           ├── twc_core.c/h         # Core protocol state machine
│           ├── twc_device.c/h       # Device management
│           ├── twc_frame.c/h        # SLIP framing
│           └── twc_protocol.c/h     # Protocol definitions
├── tesla-director.yaml              # Example configuration
└── README.md                        # This file
```

### Building from Source

```bash
git clone https://github.com/Wired-Square/esphome-twc-director.git
cd esphome-twc-director
esphome compile tesla-director.yaml
```

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly with real hardware
5. Submit a pull request

## License

This project is provided as-is for use with Tesla Gen2 Wall Connectors. Please ensure compliance with local electrical codes and safety regulations.

## Acknowledgments

- Protocol reverse engineering by the Tesla Motors Club community
- SLIP framing specification from RFC 1055
- ESPHome framework by the ESPHome team

## Support

For issues and questions:
- GitHub Issues: https://github.com/Wired-Square/esphome-twc-director/issues
- Tesla Motors Club Forum: https://teslamotorsclub.com/tmc/threads/new-wall-connector-load-sharing-protocol.72830/

## Threading Model

This component is designed for ESPHome's single-threaded execution model:

- **Main loop execution**: All component logic (`setup()`, `loop()`) runs on ESPHome's main task
- **No internal threading**: The component does not create threads or use FreeRTOS tasks
- **Callback safety**: All callbacks (TX, negotiation, auto-bind, logging) are invoked synchronously from the main loop context
- **Shared state**: The `twc_core_t` structure and EVSE entries are accessed only from the main loop

### ESP-IDF Considerations

When using the ESP-IDF framework (recommended for this component):

- ESPHome runs on a single FreeRTOS task by default
- UART operations are non-blocking and integrated with the main loop
- No mutex protection is needed for internal state

### Important Notes

- **Do not call component methods from ISRs** or other FreeRTOS tasks
- If integrating with custom components that use threading, ensure all TWC Director API calls are made from the main loop context
- The component's timing (heartbeats, probes, bus idle detection) assumes consistent `loop()` execution

## Safety Warning

This component interfaces with high-voltage charging equipment. Improper installation or configuration could result in:
- Equipment damage
- Fire hazard
- Electric shock
- Voided warranties

**Always**:
- Follow local electrical codes
- Use properly rated circuit breakers
- Verify current limits match your installation
- Test thoroughly before leaving unattended
- Consult a licensed electrician

**Never exceed**:
- Your circuit breaker rating
- Wall Connector hardware limits (typically 32A max)
- Wire gauge ampacity ratings
