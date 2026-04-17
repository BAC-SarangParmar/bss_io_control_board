# EV DC Charging Station Firmware

A production-grade embedded firmware for a **3-dock DC EV charging station**, built on the Microchip PIC32C platform using MPLAB Harmony 3, FreeRTOS, and Microchip's TCP/IP stack.

---

## Overview

This firmware manages the complete lifecycle of a DC fast-charging session across three independent docking bays. It handles real-time CAN bus communication with Battery Management Systems (BMS) and Power Modules (PM), a TCP/IP-based IO aggregator interface, flash-backed configuration persistence, and a multi-state charging state machine — all running concurrently under FreeRTOS.

---

## Key Features

- **Dual-protocol BMS support** — selectable at compile time:
  - `PROTOCOL_17017_25` — LEVDC ISO 17017-25 standard CAN frames
  - `PROTOCOL_TVS_PROP` — TVS proprietary CAN frames
- **3-channel CAN communication** (CAN0, CAN1, CAN2) with independent FreeRTOS RX/TX queues and dedicated handler tasks per channel
- **TONHE Power Module control** — voltage/current set-point commanding with telemetry feedback (protocol-independent)
- **Per-dock charging state machine** with 10 states: `INIT → AUTH → PARAM_VALIDATE → CONNECTION_CONFIRMED → INITIALIZE → PRECHARGE → CHARGING → SHUTDOWN → SESSION_COMPLETE → ERROR`
- **Comprehensive fault detection** — E-Stop, BMS fault (debounced), PM fault (debounced), CAN timeout, zero-current fault, pre-charge timeout
- **Energy accumulation** — per-session kWh tracking with 1-second resolution
- **IO Aggregator** — TCP server for remote GPIO read/write, analog temperature input, relay/eFuse control, and flash configuration management
- **HTTP web server** — MPFS2-based with dynamic variables for CAN port, baud rate, and RS-485 UART configuration via browser
- **Flash-backed config** — CAN port/baud, RS-485 UART settings, digital/relay output states, and serial number survive power cycles
- **Moving average filter** — 50-sample ADC smoothing across 20 analog channels (PT100 and NTC thermistor support)
- **Heartbeat & watchdog** — TCP heartbeat task + hardware WDT refresh

---

## Architecture

```
main.c
├── FreeRTOS Scheduler
│   ├── CHARGING_TASK           — Per-dock state machine (1 s cycle)
│   ├── CAN0/1/2_RX Tasks       — Poll CAN FIFOs, push to RX queues
│   ├── CAN0/1/2_SRV Tasks      — Dequeue TX frames, dispatch RX to protocol handlers
│   ├── IO_Handler Task         — TCP server: GPIO, analog, charging commands
│   ├── vCommonServerTask       — Debug TCP server (port 9999)
│   └── vHeartbeatTask          — Periodic TCP heartbeat + WDT clear
├── Software Timer (100 ms)
│   └── CAN TX — BMS frames (0x508/509/510 or 0x90/91/92) + PM command
└── HTTP NET Server
    └── Dynamic pages — CAN/RS-485 config, mapping configuration
```

---

## Module Breakdown

| File | Responsibility |
|------|---------------|
| `ChargingHandler.c/.h` | Per-dock state machine, fault detection, energy calculation, LED control |
| `ChargingCommunicationHandler.c/.h` | 100 ms CAN TX timer, BMS/PM RX dispatch, protocol-specific frame building |
| `AppCanHandler.c/.h` | 3-channel CAN RX/TX queue management, FreeRTOS tasks, hardware FIFO abstraction |
| `sessionDBHandler.c/.h` | Per-dock session data store (state, SOC, voltages, fault bitmaps, timestamps) |
| `IOHandler.c/.h` | TCP IO server, GPIO read/write, ADC, flash config, CRC-16 frame parser |
| `io_expander.c/.h` | TCA9539 I2C IO expander driver |
| `Common.c/.h` | Debug TCP server, error reporting, bootloader trigger |
| `app.c/.h` | MPLAB Harmony app state machine, TCP/IP stack init |
| `custom_http_net_app.c` | HTTP GET/POST handlers for web-based configuration |
| `http_net_print.c/.h` | Dynamic variable callbacks for the HTTP server |

---

## Hardware Platform

- **MCU**: Microchip SAMA5D2 / PIC32C (ARM Cortex-A5 / M4)
- **CAN Controllers**: 3× MCAN peripheral (CAN0, CAN1, CAN2) — KSZ8863 Ethernet PHY
- **IO Expander**: TCA9539 (I2C, SERCOM7)
- **UART**: SERCOM8/9 for RS-485 (configurable baud/parity/stop bits at runtime)
- **ADC**: 20-channel, 12-bit (PT100 or NTC thermistor sensor support)
- **Flash**: Internal NVM for persistent configuration (FCW driver)
- **Ethernet**: GMAC with static IP support

---

## Protocol Details

### CAN Frame Timing

| Frame | Direction | Interval |
|-------|-----------|----------|
| LEVDC 0x508/509/510 | EVSE → EV | 100 ms |
| TVS 0x90/0x91 | Charger → BMS | 100 ms |
| TVS 0x92 (FW Version) | Charger → BMS | 1000 ms |
| TONHE PM command | EVSE → PM | 100 ms |
| TONHE PM telemetry | PM → EVSE | asynchronous |

### CAN Bus → Dock Mapping

| CAN Bus | Dock |
|---------|------|
| CAN0 | Dock 1 |
| CAN1 | Dock 2 |
| CAN2 | Dock 3 |

### TCP Frame Format (IO Aggregator)

```
[UniqueID: 4B] [MsgType: 2B] [CompartmentID: 1B] [DockID: 1B]
[CommandID: 1B] [PayloadLen: 1B] [Payload: N bytes] [CRC-16: 2B]
```

Supported commands: `GPIO_OPERATION`, `ANALOG_READ`, `CHARGING_COMMAND`, `BOOT_MODE`, `SOFT_RESET`

---

## Fault Handling

Faults are reported as a 32-bit bitmap in `sessionDB[dock].u32SystemFaultBitmap`:

| Bit | Fault |
|-----|-------|
| 0 | E-Stop triggered |
| 1 | BMS error (debounced 2 s) |
| 2 | PM error (debounced 2 s) |
| 3 | BMS CAN timeout (10 s) |
| 4 | PM CAN timeout (10 s) |
| 5 | Zero-current fault (30 s sustained) |
| 6 | Pre-charge timeout (30 s) |

On any critical fault during `CHARGING` or `PRECHARGE`, the state machine transitions to `SHUTDOWN` immediately and logs the fault bitmap to the console.

---

## Build Configuration

### Protocol Selection

In `ChargingHandler.h`, uncomment exactly one:

```c
/* #define CHARGING_PROTOCOL   PROTOCOL_17017_25 */
#define CHARGING_PROTOCOL      PROTOCOL_TVS_PROP
```

### Board Selection

In `IOHandler.h`:

```c
#define HEV_IO_Aggregator           (1)
#define Two_Wheeler_IO_Aggregator   (0)
```

### CAN Debug Output

In `AppCanHandler.c`:

```c
#define CAN_DEBUG_ENABLE    (1)   /* 0 = off (default), 1 = verbose */
```

---

## Dependencies

- MPLAB Harmony 3 (TCP/IP Stack, SYS_FS, HTTP NET Server, SYS_TMR)
- FreeRTOS (tasks, queues, software timers)
- Microchip CAN / MCAN driver
- cJSON (HTTP POST body parsing)
- FCW (Flash Controller Write driver)

---

## Flash Configuration Layout

Stored at `0x0C0F0000` with magic `0xDEADBEEF`:

```
magic (4B) | digitalOutputs (4B) | relayOutputs (2B) | serialNumber (2B)
canPorts[6] (12B) | canBaudRates[6] (24B)
rs485Ports[2] (4B) | rs485Config[2] (baud + dataBits + parity + stopBits)
```

Defaults are written on first boot if flash is blank or magic is invalid.

---

## Console Output

The firmware logs all significant events to `SYS_CONSOLE`. Key prefixes:

| Prefix | Module |
|--------|--------|
| `[CAN0/1/2 RX]` | CAN receive handler |
| `[SrvTask CAN0/1/2]` | CAN server task |
| `[CanInit]` | CAN initialisation |
| `[Flash]` | Flash read/write |
| `[GPIO]` | GPIO operations |
| `[IO]` | IO handler TCP server |
| `[Net]` | Network (static IP) |
| `G1:/G2:/G3:` | Per-dock charging state machine |

---

## Authors

Developed by **Bacancy Systems Pvt. Ltd.**  
Charging state machine: Sarang Parmar

---

## License

Proprietary — Bacancy Systems Pvt. Ltd. All rights reserved.
