# Aero Hand Lite Firmware

This repository contains the firmware code for the **Aero Hand**, designed for control, homing, calibration, and telemetry of the TetherIA Aero Hand prosthetic.  
The firmware runs on an **ESP32-S3 (Seeed Studio XIAO ESP32S3)** and communicates with a PC host application via USB serial.

---

## 🔌 Communication Protocol

The firmware uses a **fixed 16-byte frame** for all communication between PC and firmware.

### Frame Structure (TX and RX)
| Bytes     | Field            | Description                                                                 |
|-----------|-----------------|-----------------------------------------------------------------------------|
| 0         | **Opcode**      | Command / response code (e.g., `0x01` for HOMING, `0x04` for TRIM).         |
| 1         | **Filler**      | Always `0x00` (reserved for future use).                                    |
| 2..15     | **Payload**     | 14-byte payload. May contain parameters (channels, degrees, IDs, etc.) or be all zeros in acknowledgments. |

- **All commands from PC → firmware are 16 bytes.**  
- **All responses (ACKs) from firmware → PC are also 16 bytes.**

---

## 🚀 Supported Modes

### 1. HOMING (`0x01`)
- **PC → Firmware:**  
  - Opcode `0x01`  
  - Payload: all zeros  
- **Firmware action:**  
  - Runs the full homing sequence for all servos (thumb & fingers).  
  - Blocks other commands until finished.  
- **Firmware → PC (ACK):**  
  - 16 bytes: `[0x01, 0x00, 0x00...0x00]`  
  - Indicates homing complete.

---

### 2. ZERO_ALL (`0x02`)
- **PC → Firmware:**  
  - Opcode `0x02`  
  - Payload: all zeros  
- **Firmware action:**  
  - Sends all servos to their stored **extend** position.  
- **Firmware → PC (ACK):**  
  - 16 bytes: `[0x02, 0x00, 0x00...0x00]`

---

### 3. SET_ID (`0x03`)
- **PC → Firmware:**  
  - Opcode `0x03`  
  - Payload: contains target ID and current limit request.  
  - Word 0 (2 bytes): **new ID**  
  - Word 1 (2 bytes): **current limit (0–1023)**  
- **Firmware action:**  
  - Scans servo bus for first connected servo.  
  - Updates its EEPROM with new ID and current limit.  
- **Firmware → PC (ACK):**  
  - 16 bytes total:  
    - `[0x03, 0x00, oldId(2B), newId(2B), currentLimit(2B), rest zeros]`

---

### 4. TRIM (`0x04`)
- **PC → Firmware:**  
  - Opcode `0x04`  
  - Payload:  
    - Word 0 (2 bytes): Servo channel (0–6)  
    - Word 1 (2 bytes): Degrees offset (+/- 360°)  
- **Firmware action:**  
  - Updates the stored **extend_count** for the given channel.  
  - Persists new value into NVS (survives reboot).  
- **Firmware → PC (ACK):**  
  - 16 bytes total:  
    - `[0x04, 0x00, channel(2B), extendCount(2B), rest zeros]`

---

### 5. CTRL_POS (`0x11`)
- **PC → Firmware:**  
  - Opcode `0x11`  
  - Payload: 7 words (2 bytes each), one for each channel [0..6].  
  - Range: `0–65535` mapped linearly to servo’s calibrated `extend_count` → `grasp_count`.  
- **Firmware action:**  
  - Uses **SyncWritePosEx** to command all 7 servos simultaneously.  
- **Firmware → PC (ACK):**  
  - None (movement commands are not acknowledged).  

---

### 6. Telemetry / GET Modes
The firmware supports batched telemetry reads via **SyncRead**.

- **GET_POS (`0x22`)** → Responds with `[pos0..pos6]` (7×u16).  
- **GET_VEL (`0x23`)** → Responds with `[vel0..vel6]` (7×u16).  
- **GET_CURR (`0x24`)** → Responds with `[cur0..cur6]` (7×u16).  
- **GET_TEMP (`0x25`)** → Responds with `[tmp0..tmp6]` (7×u16).  
- **GET_ALL (`0x21`)** → Reserved (to be implemented).  

Each telemetry response is also a **16-byte frame**.

---

## 📜 Example Sequences

### Homing from PC
1. PC sends: `[0x01, 0x00, 14×0x00]`  
2. Firmware homes all servos.  
3. Firmware replies: `[0x01, 0x00, 14×0x00]`  

### Trim channel 3 by –100°
1. PC sends: `[0x04, 0x00, 0x03,0x00, 0x9C,0xFF, 10×0x00]`  
2. Firmware updates extend count, saves to NVS.  
3. Firmware replies: `[0x04, 0x00, 0x03,0x00, ext_lo,ext_hi, rest zeros]`

---

## 🧩 Implementation Notes
- **Bus locking:**  
  All sync read/write operations use `gBusMux` semaphore to prevent collisions.  
- **Persistence:**  
  Extend counts modified via TRIM are stored in **NVS** (`prefs`) for persistence across reboots.  
- **Timing:**  
  Homing sequence may take up to **25 seconds per channel**. During this time, **all other commands are ignored**.  
- **Frame alignment:**  
  Unknown or invalid opcodes are consumed but ignored, to maintain framing alignment.

---

## ✅ Summary
- Communication is always **16-byte aligned**.  
- Homing (`0x01`), Set ID (`0x03`), and Trim (`0x04`) **send back ACKs**.  
- Position control (`0x11`) and telemetry (`0x22`–`0x25`) are **one-way commands**.  
- Extend calibration survives power cycles via NVS storage.  
- Firmware is **robust to stale/invalid frames** by ignoring them safely.  

---
