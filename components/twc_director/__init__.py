# components/twc_director/__init__.py

import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import uart, switch, binary_sensor, text_sensor, sensor, number
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
    CONF_NAME,
    UNIT_AMPERE,
    UNIT_VOLT,
    UNIT_KILOWATT_HOURS,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_ENERGY,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    ICON_CURRENT_AC,
    ICON_FLASH,
    ICON_MEMORY,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "number", "sensor", "switch", "text_sensor"]

twc_director_ns = cg.esphome_ns.namespace("twc_director")
TWCDirectorComponent = twc_director_ns.class_(
    "TWCDirectorComponent", cg.Component
)
TWCDirectorMasterModeSwitch = twc_director_ns.class_(
    "TWCDirectorMasterModeSwitch", switch.Switch
)
TWCDirectorContactorSwitch = twc_director_ns.class_(
    "TWCDirectorContactorSwitch", switch.Switch
)
TWCDirectorCurrentNumber = twc_director_ns.class_(
    "TWCDirectorCurrentNumber", number.Number
)

CONF_MASTER_MODE = "master_mode"
CONF_MASTER_ADDR = "addr"
CONF_GLOBAL_MAX_CURRENT = "global_max_current"
CONF_GLOBAL_MAX_CURRENT_CONTROL = "global_max_current_control"
CONF_EVSE_MAX_CURRENT_LIMIT = "global_twc_max_current"
CONF_LINK_OK = "link_ok"

CONF_EVSE = "evse"
CONF_ADDRESS = "address"
CONF_TWC_ONLINE = "twc_online"
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_SERIAL_NUMBER = "serial_number"
CONF_VEHICLE_CONNECTED = "vehicle_connected"
CONF_VEHICLE_VIN = "vehicle_vin"

CONF_METER_CURRENT_PHASE_A = "meter_current_phase_a"
CONF_METER_CURRENT_PHASE_B = "meter_current_phase_b"
CONF_METER_CURRENT_PHASE_C = "meter_current_phase_c"
CONF_METER_VOLTAGE_PHASE_A = "meter_voltage_phase_a"
CONF_METER_VOLTAGE_PHASE_B = "meter_voltage_phase_b"
CONF_METER_VOLTAGE_PHASE_C = "meter_voltage_phase_c"
CONF_METER_ENERGY_TOTAL = "meter_energy_total"
CONF_METER_ENERGY_SESSION = "meter_energy_session"

CONF_CONTACTOR = "contactor"
CONF_CONTACTOR_STATUS = "contactor_status"
CONF_AVAILABLE_CURRENT_SENSOR = "available_current_sensor"
CONF_INITIAL_CURRENT = "initial_current"
CONF_MAX_CURRENT = "max_current"
CONF_MAX_CURRENT_SENSOR = "max_current_sensor"
CONF_SESSION_CURRENT = "session_current"
CONF_SESSION_CURRENT_SENSOR = "session_current_sensor"
CONF_MODE = "mode"
CONF_STATUS_TEXT = "status_text"


# --- Helper: Ensure named child for EVSE nested entities ---
def _ensure_named_child(evse_conf, key, suffix):
    if key not in evse_conf:
        return
    val = evse_conf[key]
    if val is None:
        val = {}
    # If not a dict, just assign and return (leave as is)
    if not isinstance(val, dict):
        evse_conf[key] = val
        return
    # If already has a name, leave as is
    if "name" in val:
        evse_conf[key] = val
        return
    evse_name = evse_conf.get(CONF_NAME, "TWC")
    val["name"] = f"{evse_name} {suffix}"
    evse_conf[key] = val

# --- Helper: Preprocess EVSE config dict for default names ---
# Pre-process a single EVSE config:
# - auto-inject default names for nested children
# - leaves explicit name/id configs untouched
def _evse_preprocess(config):
    _ensure_named_child(config, CONF_TWC_ONLINE, "Online")
    _ensure_named_child(config, CONF_FIRMWARE_VERSION, "Firmware Version")
    _ensure_named_child(config, CONF_SERIAL_NUMBER, "Serial Number")
    _ensure_named_child(config, CONF_VEHICLE_CONNECTED, "Vehicle Connected")
    _ensure_named_child(config, CONF_VEHICLE_VIN, "Vehicle VIN")
    _ensure_named_child(config, CONF_CONTACTOR, "Contactor")
    _ensure_named_child(config, CONF_CONTACTOR_STATUS, "Contactor Closed")
    _ensure_named_child(config, CONF_AVAILABLE_CURRENT_SENSOR, "Available Current")
    _ensure_named_child(config, CONF_INITIAL_CURRENT, "Initial Current Target")
    _ensure_named_child(config, CONF_MAX_CURRENT, "Max Current")
    _ensure_named_child(config, CONF_MAX_CURRENT_SENSOR, "Max Current Sensor")
    _ensure_named_child(config, CONF_SESSION_CURRENT, "Session Current Target")
    _ensure_named_child(config, CONF_SESSION_CURRENT_SENSOR, "Session Current")
    _ensure_named_child(config, CONF_MODE, "Mode")
    _ensure_named_child(config, CONF_STATUS_TEXT, "Status")
    _ensure_named_child(config, CONF_METER_CURRENT_PHASE_A, "Current Phase A")
    _ensure_named_child(config, CONF_METER_CURRENT_PHASE_B, "Current Phase B")
    _ensure_named_child(config, CONF_METER_CURRENT_PHASE_C, "Current Phase C")
    _ensure_named_child(config, CONF_METER_VOLTAGE_PHASE_A, "Voltage Phase A")
    _ensure_named_child(config, CONF_METER_VOLTAGE_PHASE_B, "Voltage Phase B")
    _ensure_named_child(config, CONF_METER_VOLTAGE_PHASE_C, "Voltage Phase C")
    _ensure_named_child(config, CONF_METER_ENERGY_TOTAL, "Energy Meter Total")
    _ensure_named_child(config, CONF_METER_ENERGY_SESSION, "Energy Meter Session")
    return config

# --- Helper: Default name factory for top-level entities ---
def _with_default_name(default_name):
    def inner(conf):
        if conf is None:
            conf = {}
        else:
            conf = dict(conf)
        if "name" not in conf:
            conf["name"] = default_name
        return conf
    return inner


CONFIG_SCHEMA = (
    cv.Schema(
        {
            # The main TWCDirector component ID
            cv.GenerateID(CONF_ID): cv.declare_id(TWCDirectorComponent),

            # UART this director will use to talk to the TWC bus
            cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),

            # Optional switch that controls "master mode" vs passive/observer mode
            cv.Optional(CONF_MASTER_MODE): cv.All(
                _with_default_name("Master Mode"),
                switch.switch_schema(TWCDirectorMasterModeSwitch),
            ),
            cv.Required(CONF_MASTER_ADDR): cv.hex_int,
            cv.Optional(CONF_GLOBAL_MAX_CURRENT): cv.positive_float,
            cv.Optional(CONF_EVSE_MAX_CURRENT_LIMIT): cv.positive_float,

            # Optional number entity for runtime global max current control
            # (limited to the compile-time CONF_GLOBAL_MAX_CURRENT safety maximum)
            cv.Optional(CONF_GLOBAL_MAX_CURRENT_CONTROL): cv.All(
                _with_default_name("Global Max Current"),
                number.number_schema(
                    TWCDirectorCurrentNumber,
                    unit_of_measurement=UNIT_AMPERE,
                    icon=ICON_CURRENT_AC,
                    device_class=DEVICE_CLASS_CURRENT,
                ),
            ),

            # Optional binary sensor for link health (true = receiving valid frames)
            cv.Optional(CONF_LINK_OK): cv.All(
                _with_default_name("Link OK"),
                binary_sensor.binary_sensor_schema(
                    device_class="connectivity",
                ),
            ),

            # Optional per-EVSE configuration, including an online binary sensor and optional sensors
            cv.Optional(CONF_EVSE): cv.ensure_list(
                cv.All(
                    _evse_preprocess,
                    cv.Schema(
                        {
                            cv.Optional(CONF_ADDRESS): cv.hex_int,
                            cv.Required(CONF_NAME): cv.string,
                            cv.Required(CONF_TWC_ONLINE): binary_sensor.binary_sensor_schema(),
                            cv.Optional(CONF_FIRMWARE_VERSION): text_sensor.text_sensor_schema(
                                icon="mdi:chip",
                            ),
                            cv.Optional(CONF_SERIAL_NUMBER): text_sensor.text_sensor_schema(
                                icon=ICON_MEMORY,
                            ),
                            cv.Optional(CONF_VEHICLE_CONNECTED): binary_sensor.binary_sensor_schema(
                                device_class="plug",
                            ),
                            cv.Optional(CONF_VEHICLE_VIN): text_sensor.text_sensor_schema(
                                icon="mdi:car-electric",
                            ),
                            # Per-charger contactor and current/session controls
                            cv.Optional(CONF_CONTACTOR): switch.switch_schema(
                                TWCDirectorContactorSwitch
                            ),
                            cv.Optional(CONF_CONTACTOR_STATUS): binary_sensor.binary_sensor_schema(),
                            cv.Optional(CONF_MAX_CURRENT): number.number_schema(
                                TWCDirectorCurrentNumber,
                                unit_of_measurement=UNIT_AMPERE,
                                icon=ICON_CURRENT_AC,
                                device_class=DEVICE_CLASS_CURRENT,
                            ),
                            cv.Optional(CONF_MAX_CURRENT_SENSOR): sensor.sensor_schema(
                                unit_of_measurement=UNIT_AMPERE,
                                icon=ICON_CURRENT_AC,
                                device_class=DEVICE_CLASS_CURRENT,
                                state_class=STATE_CLASS_MEASUREMENT,
                            ),
                            cv.Optional(CONF_INITIAL_CURRENT): number.number_schema(
                                TWCDirectorCurrentNumber,
                                unit_of_measurement=UNIT_AMPERE,
                                icon=ICON_CURRENT_AC,
                                device_class=DEVICE_CLASS_CURRENT,
                            ),
                            cv.Optional(CONF_AVAILABLE_CURRENT_SENSOR): sensor.sensor_schema(
                                unit_of_measurement=UNIT_AMPERE,
                                icon=ICON_CURRENT_AC,
                                device_class=DEVICE_CLASS_CURRENT,
                                state_class=STATE_CLASS_MEASUREMENT,
                            ),
                            cv.Optional(CONF_SESSION_CURRENT): number.number_schema(
                                TWCDirectorCurrentNumber,
                                unit_of_measurement=UNIT_AMPERE,
                                icon=ICON_CURRENT_AC,
                                device_class=DEVICE_CLASS_CURRENT,
                            ),
                            cv.Optional(CONF_SESSION_CURRENT_SENSOR): sensor.sensor_schema(
                                unit_of_measurement=UNIT_AMPERE,
                                icon=ICON_CURRENT_AC,
                                device_class=DEVICE_CLASS_CURRENT,
                                state_class=STATE_CLASS_MEASUREMENT,
                            ),
                            cv.Optional(CONF_MODE): text_sensor.text_sensor_schema(),
                            cv.Optional(CONF_STATUS_TEXT): text_sensor.text_sensor_schema(
                                icon="mdi:information-outline",
                            ),
                            # Per-phase current sensors (A)
                            cv.Optional(CONF_METER_CURRENT_PHASE_A): sensor.sensor_schema(
                                unit_of_measurement=UNIT_AMPERE,
                                icon=ICON_CURRENT_AC,
                                device_class=DEVICE_CLASS_CURRENT,
                                state_class=STATE_CLASS_MEASUREMENT,
                            ),
                            cv.Optional(CONF_METER_CURRENT_PHASE_B): sensor.sensor_schema(
                                unit_of_measurement=UNIT_AMPERE,
                                icon=ICON_CURRENT_AC,
                                device_class=DEVICE_CLASS_CURRENT,
                                state_class=STATE_CLASS_MEASUREMENT,
                            ),
                            cv.Optional(CONF_METER_CURRENT_PHASE_C): sensor.sensor_schema(
                                unit_of_measurement=UNIT_AMPERE,
                                icon=ICON_CURRENT_AC,
                                device_class=DEVICE_CLASS_CURRENT,
                                state_class=STATE_CLASS_MEASUREMENT,
                            ),
                            # Per-phase voltage sensors (V)
                            cv.Optional(CONF_METER_VOLTAGE_PHASE_A): sensor.sensor_schema(
                                unit_of_measurement=UNIT_VOLT,
                                icon=ICON_FLASH,
                                device_class=DEVICE_CLASS_VOLTAGE,
                                state_class=STATE_CLASS_MEASUREMENT,
                            ),
                            cv.Optional(CONF_METER_VOLTAGE_PHASE_B): sensor.sensor_schema(
                                unit_of_measurement=UNIT_VOLT,
                                icon=ICON_FLASH,
                                device_class=DEVICE_CLASS_VOLTAGE,
                                state_class=STATE_CLASS_MEASUREMENT,
                            ),
                            cv.Optional(CONF_METER_VOLTAGE_PHASE_C): sensor.sensor_schema(
                                unit_of_measurement=UNIT_VOLT,
                                icon=ICON_FLASH,
                                device_class=DEVICE_CLASS_VOLTAGE,
                                state_class=STATE_CLASS_MEASUREMENT,
                            ),
                            # Energy sensors (kWh)
                            cv.Optional(CONF_METER_ENERGY_TOTAL): sensor.sensor_schema(
                                unit_of_measurement=UNIT_KILOWATT_HOURS,
                                icon=ICON_FLASH,
                                device_class=DEVICE_CLASS_ENERGY,
                                state_class=STATE_CLASS_TOTAL_INCREASING,
                            ),
                            cv.Optional(CONF_METER_ENERGY_SESSION): sensor.sensor_schema(
                                unit_of_measurement=UNIT_KILOWATT_HOURS,
                                icon=ICON_FLASH,
                                device_class=DEVICE_CLASS_ENERGY,
                                state_class=STATE_CLASS_MEASUREMENT,
                            ),
                        }
                    ),
                )
            ),

        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    # Get the UART parent (the RS-485 UART you configured in YAML)
    uart_comp = await cg.get_variable(config[CONF_UART_ID])

    # Add the component directory to the include path so C++ can find twc/*.h files
    import os
    component_dir = os.path.dirname(__file__)
    cg.add_build_flag(f"-I{component_dir}")

    # Create the C++ TWCDirectorComponent instance, passing the UART
    var = cg.new_Pvariable(config[CONF_ID], uart_comp)
    # Set the master address (the TWC Director's own ID on the RS-485 bus)
    master_addr = config[CONF_MASTER_ADDR]
    cg.add(var.set_master_address(master_addr))

    if CONF_GLOBAL_MAX_CURRENT in config:
        global_max = config[CONF_GLOBAL_MAX_CURRENT]
        cg.add(var.set_global_max_current(global_max))

    if CONF_EVSE_MAX_CURRENT_LIMIT in config:
        evse_max = config[CONF_EVSE_MAX_CURRENT_LIMIT]
        cg.add(var.set_evse_max_current_limit(evse_max))

    # Register as a regular ESPHome component so loop()/setup() are called
    await cg.register_component(var, config)

    # Optional master-mode switch entity so C++ can check/set it.
    # If omitted, the director will default to passive/observer mode.
    master_mode_switch = cg.nullptr
    if CONF_MASTER_MODE in config:
        master_mode_conf = config[CONF_MASTER_MODE]
        master_mode_switch = await switch.new_switch(master_mode_conf)
    cg.add(var.set_master_mode_switch(master_mode_switch))

    # Optional global max current control number entity
    global_max_control = cg.nullptr
    if CONF_GLOBAL_MAX_CURRENT_CONTROL in config:
        global_max_conf = config[CONF_GLOBAL_MAX_CURRENT_CONTROL]
        # Get the safety maximum from CONF_GLOBAL_MAX_CURRENT (defaults to 80A if not set)
        safety_max = config.get(CONF_GLOBAL_MAX_CURRENT, 80.0)
        global_max_control = await number.new_number(
            global_max_conf,
            min_value=0.0,
            max_value=safety_max,
            step=1.0,
        )
    cg.add(var.set_global_max_current_control(global_max_control))

    if CONF_LINK_OK in config:
        link_ok_conf = config[CONF_LINK_OK]
        link_ok_sensor = await binary_sensor.new_binary_sensor(link_ok_conf)
        cg.add(var.set_link_ok_sensor(link_ok_sensor))

    if CONF_EVSE in config:
        for evse_conf in config[CONF_EVSE]:
            addr = evse_conf.get(CONF_ADDRESS, 0)
            evse_name = evse_conf.get(CONF_NAME, f"TWC 0x{addr:04X}")

            # Required online binary sensor (use schema-processed config directly)
            online_sensor = await binary_sensor.new_binary_sensor(
                evse_conf[CONF_TWC_ONLINE]
            )

            # Optional firmware version text sensor
            fw_sensor = cg.nullptr
            if CONF_FIRMWARE_VERSION in evse_conf:
                fw_sensor = await text_sensor.new_text_sensor(
                    evse_conf[CONF_FIRMWARE_VERSION]
                )

            # Optional serial number text sensor
            serial_sensor = cg.nullptr
            if CONF_SERIAL_NUMBER in evse_conf:
                serial_sensor = await text_sensor.new_text_sensor(
                    evse_conf[CONF_SERIAL_NUMBER]
                )

            # Optional vehicle-connected binary sensor
            vehicle_sensor = cg.nullptr
            if CONF_VEHICLE_CONNECTED in evse_conf:
                vehicle_sensor = await binary_sensor.new_binary_sensor(
                    evse_conf[CONF_VEHICLE_CONNECTED]
                )

            # Optional vehicle VIN text sensor
            vin_sensor = cg.nullptr
            if CONF_VEHICLE_VIN in evse_conf:
                vin_sensor = await text_sensor.new_text_sensor(
                    evse_conf[CONF_VEHICLE_VIN]
                )

            # Optional per-charger contactor and current/session entities
            contactor_sw = cg.nullptr
            if CONF_CONTACTOR in evse_conf:
                contactor_sw = await switch.new_switch(evse_conf[CONF_CONTACTOR])

            contactor_status_bin = cg.nullptr
            if CONF_CONTACTOR_STATUS in evse_conf:
                contactor_status_bin = await binary_sensor.new_binary_sensor(
                    evse_conf[CONF_CONTACTOR_STATUS]
                )

            # Use configurable EVSE max current limit (defaults to 32A)
            evse_max_limit = config.get(CONF_EVSE_MAX_CURRENT_LIMIT, 32.0)

            max_current_num = cg.nullptr
            if CONF_MAX_CURRENT in evse_conf:
                max_current_num = await number.new_number(
                    evse_conf[CONF_MAX_CURRENT],
                    min_value=0.0,
                    max_value=evse_max_limit,
                    step=1.0,
                )

            max_current_sensor_ent = cg.nullptr
            if CONF_MAX_CURRENT_SENSOR in evse_conf:
                max_current_sensor_ent = await sensor.new_sensor(
                    evse_conf[CONF_MAX_CURRENT_SENSOR]
                )

            initial_current_num = cg.nullptr
            if CONF_INITIAL_CURRENT in evse_conf:
                initial_current_num = await number.new_number(
                    evse_conf[CONF_INITIAL_CURRENT],
                    min_value=0.0,
                    max_value=evse_max_limit,
                    step=1.0,
                )

            available_current_sensor = cg.nullptr
            if CONF_AVAILABLE_CURRENT_SENSOR in evse_conf:
                available_current_sensor = await sensor.new_sensor(
                    evse_conf[CONF_AVAILABLE_CURRENT_SENSOR]
                )

            session_current_num = cg.nullptr
            if CONF_SESSION_CURRENT in evse_conf:
                session_current_num = await number.new_number(
                    evse_conf[CONF_SESSION_CURRENT],
                    min_value=0.0,
                    max_value=evse_max_limit,
                    step=1.0,
                )

            session_current_sensor_ent = cg.nullptr
            if CONF_SESSION_CURRENT_SENSOR in evse_conf:
                session_current_sensor_ent = await sensor.new_sensor(
                    evse_conf[CONF_SESSION_CURRENT_SENSOR]
                )

            mode_text = cg.nullptr
            if CONF_MODE in evse_conf:
                mode_text = await text_sensor.new_text_sensor(
                    evse_conf[CONF_MODE]
                )

            status_text = cg.nullptr
            if CONF_STATUS_TEXT in evse_conf:
                status_text = await text_sensor.new_text_sensor(
                    evse_conf[CONF_STATUS_TEXT]
                )

            # Optional per-phase current sensors
            current_phase_a_sensor = cg.nullptr
            if CONF_METER_CURRENT_PHASE_A in evse_conf:
                current_phase_a_sensor = await sensor.new_sensor(
                    evse_conf[CONF_METER_CURRENT_PHASE_A]
                )

            current_phase_b_sensor = cg.nullptr
            if CONF_METER_CURRENT_PHASE_B in evse_conf:
                current_phase_b_sensor = await sensor.new_sensor(
                    evse_conf[CONF_METER_CURRENT_PHASE_B]
                )

            current_phase_c_sensor = cg.nullptr
            if CONF_METER_CURRENT_PHASE_C in evse_conf:
                current_phase_c_sensor = await sensor.new_sensor(
                    evse_conf[CONF_METER_CURRENT_PHASE_C]
                )

            # Optional per-phase voltage sensors
            voltage_phase_a_sensor = cg.nullptr
            if CONF_METER_VOLTAGE_PHASE_A in evse_conf:
                voltage_phase_a_sensor = await sensor.new_sensor(
                    evse_conf[CONF_METER_VOLTAGE_PHASE_A]
                )

            voltage_phase_b_sensor = cg.nullptr
            if CONF_METER_VOLTAGE_PHASE_B in evse_conf:
                voltage_phase_b_sensor = await sensor.new_sensor(
                    evse_conf[CONF_METER_VOLTAGE_PHASE_B]
                )

            voltage_phase_c_sensor = cg.nullptr
            if CONF_METER_VOLTAGE_PHASE_C in evse_conf:
                voltage_phase_c_sensor = await sensor.new_sensor(
                    evse_conf[CONF_METER_VOLTAGE_PHASE_C]
                )

            # Optional energy sensors
            energy_total_sensor = cg.nullptr
            if CONF_METER_ENERGY_TOTAL in evse_conf:
                energy_total_sensor = await sensor.new_sensor(
                    evse_conf[CONF_METER_ENERGY_TOTAL]
                )

            energy_session_sensor = cg.nullptr
            if CONF_METER_ENERGY_SESSION in evse_conf:
                energy_session_sensor = await sensor.new_sensor(
                    evse_conf[CONF_METER_ENERGY_SESSION]
                )

            # Register the EVSE with the core component
            cg.add(
                var.add_evse(
                    addr,
                    online_sensor,
                    fw_sensor,
                    serial_sensor,
                    vehicle_sensor,
                    vin_sensor,
                    current_phase_a_sensor,
                    current_phase_b_sensor,
                    current_phase_c_sensor,
                    voltage_phase_a_sensor,
                    voltage_phase_b_sensor,
                    voltage_phase_c_sensor,
                    energy_total_sensor,
                    energy_session_sensor,
                    contactor_sw,
                    contactor_status_bin,
                    available_current_sensor,
                    initial_current_num,
                    max_current_num,
                    max_current_sensor_ent,
                    session_current_num,
                    session_current_sensor_ent,
                    mode_text,
                    status_text,
                )
            )
