# Tesla Wall Connector RS-485 / SLIP framing (Gen 2 Load Sharing Protocol)

The Tesla Gen 2 Wall Connector uses a master-driven load sharing protocol designed to distribute a total available current across up to four HPWCs (1 master + 3 peripherals). Communication occurs over a 2-wire RS-485 bus.

This document describes how the Gen 2 Tesla Wall Connector (TWC) implements its proprietary RS-485 load-sharing protocol using SLIP-style framing, as captured via on-wire analysis.

Sources:
- Commands B1, B2 and frame details from:
  https://teslamotorsclub.com/tmc/threads/new-wall-connector-load-sharing-protocol.72830/page-25
- SLIP escape process from:
  https://en.wikipedia.org/wiki/Serial_Line_Internet_Protocol

<!--toc
-->

# On the wire

## RS-485 / SLIP frame layout (TWC protocol)

Example:

```text
C0  FD  EB  60 61  00 A3 20 EC 00 F1 00 00 00 00 00 00 00 00 00  4C  C0  FC
│   │   │    │ │    └────────────────────────────────────────┘   │   │   │
│   │   │    │ │                   Payload                       │   │   └─ "END_TYPE"
│   │   │    │ │                                                 │   └───── SLIP END
│   │   │    │ │                                                 └───────── Checksum
│   │   │    │ └───────── Sender ID (low)
│   │   │    └─────────── Sender ID (high)
│   │   └──────────────── Command = 0xEB (TWC_METER)
│   └──────────────────── Frame Type = 0xFD (TWC_DATA)
└──────────────────────── SLIP END
```

## SLIP Encoding

Payload bytes are escaped before transmission and unescaped by the decoder.

TWC uses standard SLIP markers:

```text
C0  FD EB 60 61 00 A3 20 EC 00 F1 00 00 00 00 00 00 00 00 00 4C  C0  FC
│   └─────────────────────────────────────────────────────────┘  │   │
│                             Payload                            │   │
└────────────────────────────────────────────────────────────────┘   │
                   SLIP END (frame boundary)                         │
                                                       "END_TYPE" ───┘
```


### Markers
| Byte | Description               |
|------|---------------------------|
| 0xC0 | SLIP_END (frame boundary) |
| 0xDB | SLIP_ESC                  |


### Escaped sequences:
| Literal Value | | Escaped Bytes |
|---------------|-|---------------|
| 0xC0          |→| 0xDB 0xDC     |
| 0xDB          |→| 0xDB 0xDD     |



### End


All SLIP encoded frames are suffixed with an additional protocol marker `0xFC` immediately after the terminating `0xC0`. This trailing byte is referred to in this document as **END_TYPE**. It is important to note that `0xFC` can appear inside the payload and is **not** SLIP-escaped; only `0xC0` and `0xDB` participate in the SLIP escaping rules.

> **Note**
>
> In real-world captures, additional **END_TYPE** values have been observed on some installations. While `0xFC` appears to be the canonical value, certain frames have been seen terminating with `0xF8` or `0xFE` after the trailing `0xC0`. In one site’s traffic, these alternate tails were only seen on data-style replies such as `FD EB` / `FD ED` / `FD EE` / `FD EF` / `FD F1`, while status traffic (for example `E0`) continued to use `0xFC`.
>
> At the time of writing it is not clear whether these alternate END_TYPE values encode link state, firmware variation, vehicle type (e.g. SWCAN/J1772 vs Tesla), or are simply implementation quirks. Decoders should therefore treat the byte immediately following the final `0xC0` as an END_TYPE marker that is **not** part of the TWC frame body and **not** included in the checksum, without assuming it will always be `0xFC`.

### Conventions

Throughout this document:

- Multi‑byte integers are **big‑endian** (most significant byte first).
- The `Sender ID` and optional `Destination ID` are 16‑bit identifiers derived from the Wall Connector’s serial number.
- Payload layouts and sizes are defined per command; fields described here match the structures used in the reference decoder implementations.


## Frame Header

Every TWC frame starts with a small header that identifies the message type, command, and sender. Some commands also include an explicit destination ID which can be considered part as of the payload.

General layout:

```text
TT  CC  SS SS  [DD DD]
│   │   │  │     │  │
│   │   │  │     │  │
│   │   │  │     └──┴─ Destination ID (high + low bytes)  (optional)
│   │   └──┴────────── Sender ID (high + low bytes)
│   └───────────────── Command
└───────────────────── Frame Type
```


### Frame Type 0xFB - TWC_DATA_REQUEST
The master uses `0xFB` frames to **poll peripherals for information**, and expects a `0xFD` frame in response.

| Command        | Byte | Destination | Meaning                                        |
|----------------|------|-------------|------------------------------------------------|
| TWC_STATUS     | 0xE0	| Yes         | Request status block or issue a status command |
| TWC_CONTROLLER | 0xE1	| Unknown     | Request controller negotiation (rare)          |
| TWC_PERIPHERAL | 0xE2	| No          | Request peripheral negotiation (rare)          |
| TWC_METER      | 0xEB	| Yes         | Request meter/voltage/current data             |
| TWC_VERSION    | 0xEC	| Yes         | Request firmware version                       |
| TWC_SERIAL     | 0xED	| Yes         | Request serial number                          |
| TWC_VIN_HIGH   | 0xEE	| Yes         | Request VIN (first 7 bytes)                    |
| TWC_VIN_MID    | 0xEF	| Yes         | Request VIN (middle 7 bytes)                   |
| TWC_VIN_LOW    | 0xF1	| Yes         | Request VIN (remaining bytes)                  |


### Frame Type 0xFC - TWC_COMMAND
The master uses `0xFC` frames to **control a peripheral**, and expects a `0xFD` frame in response.

| Command              | Byte | Destination | Meaning                            |
|----------------------|------|-------------|------------------------------------|
| TWC_CLOSE_CONTACTORS | 0xB1 |             | Close the relay and allow charging |
| TWC_OPEN_CONTACTORS  | 0xB2 |             | Open the relay and stop charging   |
| TWC_STATUS           | 0xE0 |             | with a `StatusCommand` payload:    |
| TWC_CONTROLLER       | 0xE1 | ??          | |
| TWC_PERIPHERAL       | 0xE2 | ??          | |


### Frame Type 0xFD - TWC_DATA
The peripheral uses `0xFD` frames to **reply to a master** (for example, status, negotiation, meter, version, serial and VIN data). The master also uses `0xFD` frames when sending certain data messages on the bus (such as broadcast status or controller‑side version/meter frames observed in real captures).


## Source and Destination IDs
Every TWC frame includes a **Sender ID**, which is always present and identifies the originating logical endpoint. The value is derived from the last four digits of the device’s serial number.

Some commands also include a **Destination ID**, which occupies the two bytes immediately after the sender in the header. From the protocol’s point of view this logically belongs to the payload for those commands, even though it is laid out contiguously after the source address.


## Payload
Payload contents and length depend on the command. Multi‑byte integers follow the conventions described above. Common payloads are described below.

| `StatusCommand` Payload | Byte |
|-------------------------|------|
| GET_STATUS              | 0x00 |
| SET_INITIAL_CURRENT     | 0x05 |
| SET_INCREASE_CURRENT    | 0x06 |
| SET_DECREASE_CURRENT    | 0x07 |
| SET_SESSION_CURRENT     | 0x09 |


### Status data (FD E0 payload)

`FD E0` frames carry the unit’s status in a compact payload:

| Field             | Size (bytes) | Type    | Notes                                                  |
|-------------------|--------------|---------|--------------------------------------------------------|
| controller        | 2            | uint16  | Controller/master ID as seen by this unit              |
| charge_state      | 1            | uint8   | Uses the `Status` enum defined below                   |
| current_available | 2            | uint16  | Current the controller has made available to this unit |
| current_delivered | 2            | uint16  | Current the unit is presently delivering               |

`current_available` and `current_delivered` are encoded as unsigned integers in centi‑amps (0.01 A units).

### Controller negotiation (FB/FD E1)

`E1` frames implement the controller side of the load‑sharing negotiation.

- `FB E1` — controller negotiation request (master &gt; peripheral)
- `FD E1` — controller negotiation data in replies, when present

Payload layout:

| Field   | Size (bytes) | Type   | Notes                               |
|---------|--------------|--------|-------------------------------------|
| session | 1            | uint8  | Session identifier, typically incremented by the master |
| padding | 10           | bytes  | Reserved / padding                  |

### Peripheral negotiation (FB/FD E2)

`E2` frames carry the peripheral’s contribution to negotiation.

- `FB E2` — peripheral negotiation request (master &gt; peripheral)
- `FD E2` — peripheral negotiation reply (peripheral &gt; master)

Payload layout:

| Field             | Size (bytes) | Type   | Notes                                        |
|-------------------|--------------|--------|----------------------------------------------|
| session           | 1            | uint8  | Mirrors the controller’s session identifier  |
| current_available | 2            | uint16 | Peripheral’s maximum available current in centi‑amps (0.01 A units) |

The `current_available` field is encoded as centi‑amps; for example, `3200` represents `32.00 A`.

### Meter data (FD EB)

`FD EB` frames contain voltage and current measurements, plus a running energy counter.

Payload layout (as observed from decoded frames):

| Field       | Size (bytes) | Type    | Notes                          |
|-------------|--------------|---------|--------------------------------|
| total_kwh   | 4            | uint32  | Cumulative energy counter      |
| phase_l2_v  | 1            | uint8   | Line‑to‑neutral or phase L2 V  |
| phase_l1_v  | 1            | uint8   | Phase L1 voltage               |
| phase_l3_v  | 1            | uint8   | Phase L3 voltage               |
| separator   | 2            | uint16  | Reserved / separator           |
| phase_l2_i  | 1            | uint8   | Phase L2 current (scaled)      |
| phase_l1_i  | 1            | uint8   | Phase L1 current (scaled)      |
| phase_l3_i  | 1            | uint8   | Phase L3 current (scaled)      |
| padding     | 3            | bytes   | Reserved / padding             |

In practice, `total_kwh` is reported in kWh units by the Wall Connector and does not require additional scaling. Phase currents are encoded as unsigned integers in 0.5 A steps (the reference implementation divides the raw values by 2.0 to obtain amps).

### Version data (FD EC)

`FD EC` frames carry the Wall Connector’s firmware version.

| Field           | Size (bytes) | Type   | Notes                       |
|-----------------|--------------|--------|-----------------------------|
| version_release | 1            | uint8  | Release channel / variant   |
| version_major   | 1            | uint8  | Major version               |
| version_minor   | 1            | uint8  | Minor version               |
| version_patch   | 1            | uint8  | Patch / build               |
| padding         | 7            | bytes  | Reserved / padding          |

### Serial and VIN data (FD ED / EE / EF / F1)

Serial number and VIN are delivered using the same 15‑byte payload shape:

| Field  | Size (bytes) | Type              | Notes                                   |
|--------|--------------|-------------------|-----------------------------------------|
| bytes  | 15           | uint8\[15]        | ASCII characters, not NUL‑terminated   |

The Wall Connector sends:

- `ED` — unit serial number
- `EE` — VIN high (first slice)
- `EF` — VIN middle slice
- `F1` — VIN low (final slice)

The VIN is reconstructed by concatenating the `EE`, `EF` and `F1` payloads in order.


## Frame Footer

### Checksum

Each TWC frame contains a single‑byte checksum immediately before the terminating `0xC0` and trailing `0xFC`.

Let `frame` be the decoded TWC frame body beginning at the **Frame Type** byte and ending at (but including) the checksum:

```text
index:   0    1    2  3   4  5   6 .. N-1
         ─────────────────────────────────
         type cmd  sender  [dest] payload checksum
```

The checksum is computed as the sum of all bytes **after** the type and **before** the checksum, modulo 256:

```python
checksum = sum(frame[1:-1]) & 0xFF
```

In other words, the checksum covers:

- `command`
- `sender` (both bytes)
- optional `destination` (when present)
- the entire payload

but does **not** include the `type` byte, the checksum byte itself, or any of the SLIP/END_TYPE bytes (`0xC0`, `0xFC`) that surround the frame on the wire.
# Master / Peripheral Behaviour

## Status enum (charge_state values)

The `charge_state` field in status payloads uses the following values:

| Value | Name                | Meaning (inferred from captures)            |
|-------|---------------------|---------------------------------------------|
| 0x00  | READY               | Idle and ready to accept a vehicle          |
| 0x01  | CHARGING            | Charging normally                           |
| 0x02  | ERROR               | Fault present                                |
| 0x03  | WAITING             | Waiting for a vehicle / handshake           |
| 0x04  | NEGOTIATING         | In current‑sharing negotiation               |
| 0x05  | MAX_CHARGE          | At maximum allowed current                   |
| 0x06  | ADJUSTING           | Adjusting current allocation                 |
| 0x07  | CHARGING_CAR_LOW    | Charging at vehicle‑limited rate             |
| 0x08  | CHARGE_STARTED      | Recent transition to charging                |
| 0x09  | SETTING_LIMIT       | Updating current/session limits              |
| 0x0A  | ADJUSTMENT_COMPLETE | Completed a current adjustment cycle         |
| 0xFF  | UNKNOWN             | Reserved / unknown                           |

These names follow the reference decoder implementation; exact semantics are based on observed behaviour and may differ slightly between firmware versions.


# Master / Peripheral Behaviour

The master is responsible for:
  - Announcing system state
  - Polling all peripherals
  - Determining total available service current
  - Assigning each peripheral a current limit
  - Detecting faults or missing peripherals

Peripherals are responsible for:
  - Accepting a current limit and enforcing it
  - Reporting their own serial number
  - Reporting charging state and delivered current
  - Reporting whether a vehicle is connected
  - Forwarding vehicle VIN information
  - Falling back if the master disappears


## Negotiation Logic (High-Level)

Each Wall Connector contains multiple logical endpoints (different sender/destination IDs).
But the behaviour can be summarized from the perspective of the unit.

Negotiation occurs in several repeating steps:

### Step 1 — Heartbeat (E0)

Master → Peripheral

TWC_STATUS (0xE0) frame:
  - charge_state
  - current_available
  - current_delivered

Peripheral → Master
The peripheral echoes its own E0 status back.

This confirms both devices are alive and reachable.

### Step 2 — Controller / Peripheral Negotiation (E1 / E2)

These two commands implement the load-sharing negotiation.

Master → Peripheral (E1 = TWC_CONTROLLER)
E1 Controller Negotiation:
  - session ID (increments each cycle)
  - zero padding (reserved)

Peripheral → Master (E2 = TWC_PERIPHERAL)
E2 Peripheral Negotiation:
  - session ID (mirrors master’s)
  - current_available (peripheral’s maximum)

This is how the master learns the maximum current capability of each unit.

If three peripherals reply with available currents:

P1: 32 A
P2: 24 A
P3: 48 A

The master now knows the total pool of deliverable current.

### Step 3 — Master Computes Current Allocation

The master takes:
  * Service breaker size (e.g., 60 A)
  * Number of cars currently charging
  * Per-unit available current
  * Safety derating policies

Then assigns each unit a session current limit using:

E0 (TWC_STATUS)
→ current_available field


Peripherals MUST obey this allocation and will never exceed it.

10.4 Step 4 — METER Data (E0 + EB)

Peripherals periodically send:

E0 → their live charging state

EB → voltage, phase currents, kWh meter

The master may forward or aggregate these internally.

10.5 Step 5 — VIN & Identification (ED, EE, EF, F1)

If a car is plugged in, peripherals extract VIN and serial information from the vehicle-side protocol and forward it in three slices:

EE (VIN high)

EF (VIN mid)

F1 (VIN low)

The master collects these to determine which vehicle is charging.

11. Annotated Exchange Example

Taken from real traffic (simplified for readability):

1) 6061 → 5523   FB E1   Controller Negotiation (session=0x06)
2) 5523 → 6061   FD E2   Peripheral states it can supply 32 A max

3) 6061 → 5523   FB E0   Master heartbeat / status
4) 5523 → 6061   FD E0   Slave heartbeat, charge_state=WAITING

5) 6061 → 5523   FD EC   Firmware version request
6) 5523 → 6061   FD EC   Firmware: 2.5.1

7) 6061 → 5523   FD EB   Ask for metering data
8) 5523 → 6061   FD EB   240 V, 0.0 A delivered

9) 5523 → 4131   FD ED   "Serial Number" frame


## High-level meaning of this sequence:

1–2: Negotiation — master asks how much current the peripheral can handle
3–4: Heartbeat — confirming both ends alive
5–6: Version exchange
7–8: Meter data — peripheral reports voltage/current
9: Serial forwarded from internal endpoint

This cycle repeats forever at ~100–500 ms intervals.

### Logical data flow (protocol level)

```text
   MASTER 6061                                PERIPHERAL 5523
   ───────────                                ────────────────

   ---- Negotiation Phase ----

   [E1 ControllerNeg]  ───────────────►
                        session=N

                        ◄────────────── [E2 PeripheralNeg]
                                         session=N
                                         current_available=80A

   ---- Heartbeat Phase ----

   [E0 Status]          ───────────────►
   charge_state=READY
   current_available=60A

                        ◄────────────── [E0 Status]
                                         charge_state=WAITING
                                         delivered_current=0.0A

   ---- Meter / Version / VIN ----

   [EB Meter Request]   ───────────────►

                        ◄────────────── [EB Meter]
                                         V=240
                                         I=0.0
                                         kWh=0

                        ◄────────────── [ED Serial]
                                         "8L0026061"
```

## Appendix: Python reference structures

The following Python definitions mirror the on‑wire structures and enums used in this document. They are provided as a convenient executable reference implementation.

```python
class Markers(IntEnum):
    START = 0xC0
    END = 0xC0
    END_TYPE = 0xFC


class MarkerEscape(IntEnum):
    ESCAPE = 0xDB
    ESCAPE_END = 0xDC
    ESCAPE_ESCAPE = 0xDD


class Status(IntEnum):
    READY = 0x00
    CHARGING = 0x01
    ERROR = 0x02
    WAITING = 0x03
    NEGOTIATING = 0x04
    MAX_CHARGE = 0x05
    ADJUSTING = 0x06
    CHARGING_CAR_LOW = 0x07
    CHARGE_STARTED = 0x08
    SETTING_LIMIT = 0x09
    ADJUSTMENT_COMPLETE = 0x0A
    UNKNOWN = 0xFF


class StatusCommands(IntEnum):
    GET_STATUS = 0x00
    SET_INCREASE_CURRENT = 0x06
    SET_DECREASE_CURRENT = 0x07
    SET_INITIAL_CURRENT = 0x05
    SET_SESSION_CURRENT = 0x09


class Commands(IntEnum):
    TWC_EMPTY = 0x00
    TWC_CLOSE_CONTACTORS = 0xB1
    TWC_OPEN_CONTACTORS = 0xB2
    TWC_STATUS = 0xE0
    TWC_CONTROLLER = 0xE1
    TWC_PERIPHERAL = 0xE2
    TWC_METER = 0xEB
    TWC_VERSION = 0xEC
    TWC_SERIAL = 0xED
    TWC_VIN_HIGH = 0xEE
    TWC_VIN_MID = 0xEF
    TWC_VIN_LOW = 0xF1


class MessageType(IntEnum):
    TWC_DATA_REQUEST = 0xFB
    TWC_COMMAND = 0xFC
    TWC_DATA = 0xFD


class TWCProtocol:
    class StartFrame(BigEndianStructure):
        _pack_ = 1
        _fields_ = [
            ("start", ctypes.c_uint8),
            ("type", ctypes.c_uint8),
            ("command", ctypes.c_uint8),
            ("sender", ctypes.c_uint16)
        ]

    class EndFrame(BigEndianStructure):
        _pack_ = 1
        _fields_ = [
            ("checksum", ctypes.c_uint8),
            ("end", ctypes.c_uint8),
            ("type", ctypes.c_uint8)
        ]

    class SerialData(BigEndianStructure):
        _pack_ = 1
        _fields_ = [
            ("serial", ctypes.c_uint8 * 15)
        ]

    class MeterData(BigEndianStructure):
        _pack_ = 1
        _fields_ = [
            ("total_kwh", ctypes.c_uint32),
            ("phase_l2_v", ctypes.c_uint8),
            ("phase_l1_v", ctypes.c_uint8),
            ("phase_l3_v", ctypes.c_uint8),
            ("separator", ctypes.c_uint16),
            ("phase_l2_i", ctypes.c_uint8),
            ("phase_l1_i", ctypes.c_uint8),
            ("phase_l3_i", ctypes.c_uint8),
            ("padding", ctypes.c_uint8 * 3)
        ]

    class VersionData(BigEndianStructure):
        _pack_ = 1
        _fields_ = [
            ("version_release", ctypes.c_uint8),
            ("version_major", ctypes.c_uint8),
            ("version_minor", ctypes.c_uint8),
            ("version_patch", ctypes.c_uint8),
            ("padding", ctypes.c_uint8 * 7)
        ]

    class StatusData(BigEndianStructure):
        _pack_ = 1
        _fields_ = [
            ("controller", ctypes.c_uint16),
            ("charge_state", ctypes.c_uint8),
            ("current_available", ctypes.c_uint16),
            ("current_delivered", ctypes.c_uint16)
        ]

    class PeripheralNegotiation(BigEndianStructure):
        _pack_ = 1
        _fields_ = [
            ("session", ctypes.c_uint8),
            ("current_available", ctypes.c_uint16)
        ]

    class ControllerNegotiation(BigEndianStructure):
        _pack_ = 1
        _fields_ = [
            ("session", ctypes.c_uint8),
            ("padding", ctypes.c_uint8 * 10)
        ]

    class RequestData(BigEndianStructure):
        _pack_ = 1
        _fields_ = [
            ("recipient", ctypes.c_uint16),
            ("padding", ctypes.c_uint8 * 9)
        ]

    class StatusCommand(BigEndianStructure):
        _pack_ = 1
        _fields_ = [
            ("recipient", ctypes.c_uint16),
            ("command", ctypes.c_uint8),
            ("command_arg", ctypes.c_uint16),
            ("padding", ctypes.c_uint8 * 6)
        ]

    _commands = {
        Commands.TWC_SERIAL: {
            "name": "Unit Serial Number",
            "decode": SerialData,
            "encoder": SerialData,
            "request_decode": RequestData,
            "request_encoder": RequestData
        },
        Commands.TWC_VIN_HIGH: {
            "name": "Vehicle VIN first section",
            "decode": SerialData,
            "encoder": SerialData,
            "request_decode": RequestData,
            "request_encoder": RequestData
        },
        Commands.TWC_VIN_MID: {
            "name": "Vehicle VIN mid section",
            "decode": SerialData,
            "encoder": SerialData,
            "request_decode": RequestData,
            "request_encoder": RequestData
        },
        Commands.TWC_VIN_LOW: {
            "name": "Vehicle VIN end section",
            "decode": SerialData,
            "encoder": SerialData,
            "request_decode": RequestData,
            "request_encoder": RequestData
        },
        Commands.TWC_STATUS: {
            "name": "Unit status",
            "decode": StatusData,
            "encoder": RequestData,
            "request_decode": StatusCommand,
            "request_encoder": RequestData
        },
        Commands.TWC_METER: {
            "name": "Unit power delivery meter and voltage",
            "decode": MeterData,
            "encoder": MeterData,
            "request_decode": RequestData,
            "request_encoder": RequestData
        },
        Commands.TWC_VERSION: {
            "name": "Unit firmware version",
            "decode": VersionData,
            "encoder": VersionData,
            "request_decode": RequestData,
            "request_encoder": RequestData
        },
        Commands.TWC_PERIPHERAL: {
            "name": "Unit Peripheral Negotiation",
            "decode": PeripheralNegotiation,
            "encoder": ControllerNegotiation,
            "request_decode": RequestData,
            "request_encoder": RequestData
        },
        Commands.TWC_CONTROLLER: {
            "name": "Unit Peripheral Negotiation",
            "decode": ControllerNegotiation,
            "encoder": ControllerNegotiation,
            "request_decode": RequestData,
            "request_encoder": RequestData
        }
    }
```
