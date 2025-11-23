# Gas Leak Detector - Software Specifications

## System Overview

**Project Name:** Gas Leak Detector
**Platform:** Arduino (AVR-based microcontrollers)
**Language:** C++ (Arduino Framework)
**Version:** 1.0
**Author:** Kelvin Mwega

---

## 1. Development Environment

### Required Software
- **Arduino IDE:** 2.x or higher (or PlatformIO)
- **Compiler:** AVR-GCC (bundled with Arduino IDE)
- **Upload Protocol:** AVR ISP or USB bootloader

### Target Hardware
- **Microcontroller:** ATmega328P (Arduino Uno/Nano)
- **Clock Speed:** 16 MHz
- **Flash Memory:** 32 KB (2 KB used by bootloader)
- **SRAM:** 2 KB
- **EEPROM:** 1 KB (not used in this implementation)

---

## 2. Dependencies & Libraries

### Core Libraries (Built-in)
| Library | Version | Purpose |
|---------|---------|---------|
| `Wire.h` | Arduino built-in | I2C communication protocol for LCD |
| `Arduino.h` | Arduino built-in | Core Arduino functions |

### External Libraries
| Library | Version | Repository | Purpose |
|---------|---------|------------|---------|
| `LiquidCrystal_I2C` | 1.1.2+ | [johnrickman/LiquidCrystal_I2C](https://github.com/johnrickman/LiquidCrystal_I2C) | I2C LCD driver with 16×2 display support |

**Installation:**
```bash
# Via Arduino Library Manager
Sketch > Include Library > Manage Libraries
Search: "LiquidCrystal I2C" by Frank de Brabander
```

---

## 3. System Architecture

### Software Layers

```
┌─────────────────────────────────────┐
│   Application Layer                 │
│   - Main control logic              │
│   - State machine                   │
│   - Alarm management                │
└─────────────────────────────────────┘
            ↓
┌─────────────────────────────────────┐
│   Hardware Abstraction Layer        │
│   - LCD driver (LiquidCrystal_I2C)  │
│   - I2C protocol (Wire)             │
│   - GPIO control (digitalWrite)     │
└─────────────────────────────────────┘
            ↓
┌─────────────────────────────────────┐
│   Hardware Layer                    │
│   - ATmega328P registers            │
│   - I2C bus (TWI)                   │
│   - Digital I/O pins                │
└─────────────────────────────────────┘
```

---

## 4. Functional Specifications

### 4.1 Core Functions

#### `setup()`
**Purpose:** System initialization
**Execution:** Once on power-up/reset
**Operations:**
- Initialize serial communication (9600 baud)
- Configure GPIO pins (INPUT/OUTPUT modes)
- Initialize I2C LCD display
- Display startup message
- Set default state (green LED ON)

**Execution Time:** ~2 seconds (includes 2000ms delay)

#### `loop()`
**Purpose:** Main control loop
**Execution:** Continuous (every 200ms)
**Operations:**
- Read MQ-2 sensor digital input
- Log sensor value to Serial
- Apply debounce logic
- Update gas detection counter
- Trigger/clear alarm based on threshold
- Update LCD display
- Delay 200ms

**Cycle Time:** 200ms (5 Hz sampling rate)

---

### 4.2 Alarm Management Functions

#### `activateAlarm()`
**Trigger Condition:** `gasDetectionCount >= 3 && !gasDetected`
**Actions:**
1. Set `gasDetected = true`
2. Turn ON red LED (Pin 3)
3. Turn OFF green LED (Pin 2)
4. Clear LCD and display "GAS DETECTED!"
5. Display "Status: DANGER" on row 2
6. Send alert to Serial: "ALERT: Gas leak detected!"

**Execution Time:** ~10ms

#### `deactivateAlarm()`
**Trigger Condition:** `sensorValue == HIGH && gasDetected == true`
**Actions:**
1. Set `gasDetected = false`
2. Turn ON green LED (Pin 2)
3. Turn OFF red LED (Pin 3)
4. Clear LCD and display "GAS LEVEL"
5. Display "Status: OK" on row 2
6. Send message to Serial: "Gas levels normal"

**Execution Time:** ~10ms

#### `updateDisplay()`
**Purpose:** Non-blocking display refresh
**Trigger:** Every loop cycle
**Actions:**
- If not in alarm state, update indicator at position (12,1)
- Display "CHK" when sensor reads LOW (checking)
- Display "  " (blank) when sensor reads HIGH (safe)
- Prevents full LCD clear to avoid flicker

**Execution Time:** <5ms

---

## 5. Data Specifications

### 5.1 Global Variables

| Variable | Type | Size | Purpose | Default Value |
|----------|------|------|---------|---------------|
| `lcd` | `LiquidCrystal_I2C` | ~40 bytes | LCD object instance | 0x27, 16×2 |
| `sensorValue` | `int` | 2 bytes | Current sensor reading | Uninitialized |
| `gasDetected` | `boolean` | 1 byte | Current alarm state | `false` |
| `lastGasState` | `boolean` | 1 byte | Previous sensor state | `false` |
| `lastDebounceTime` | `unsigned long` | 4 bytes | Timestamp for debounce | 0 |
| `debounceDelay` | `unsigned long` | 4 bytes | Debounce period (ms) | 500 |
| `alarmThreshold` | `int` | 2 bytes | Readings needed for alarm | 3 |
| `gasDetectionCount` | `int` | 2 bytes | Consecutive LOW readings | 0 |

**Total RAM Usage:** ~60 bytes (excluding stack and library overhead)

### 5.2 Constants

| Constant | Value | Type | Purpose |
|----------|-------|------|---------|
| `MQ2pin` | 4 | `#define` | Digital input pin for sensor |
| `greenLedPin` | 2 | `int` | GPIO for safe indicator |
| `redLedPin` | 3 | `int` | GPIO for danger indicator |
| LCD I2C Address | 0x27 | Hex | I2C bus address |

---

## 6. Timing Specifications

### 6.1 Real-Time Constraints

| Parameter | Value | Tolerance | Notes |
|-----------|-------|-----------|-------|
| **Sampling Rate** | 200ms | ±10ms | Loop delay (5 Hz) |
| **Debounce Window** | 500ms | ±5ms | Signal stabilization |
| **Serial Baud Rate** | 9600 bps | ±2% | Standard UART speed |
| **I2C Clock** | 100 kHz | ±10% | Standard I2C speed |
| **Alarm Response Time** | ~1 second | ±200ms | 3 readings × 200ms + debounce |
| **Clear Response Time** | ~700ms | ±100ms | 1 reading + debounce |

### 6.2 Execution Timing

```
Loop Cycle (200ms):
├─ Read Sensor: <1ms
├─ Serial Print: ~1ms
├─ Debounce Logic: <1ms
├─ State Processing: <1ms
├─ LCD Update: 2-5ms
└─ Delay: 200ms
```

---

## 7. Algorithm Specifications

### 7.1 Debounce Algorithm

**Type:** Time-based software debounce
**Method:** Schmitt trigger behavior

```cpp
if (sensorValue != lastGasState) {
    lastDebounceTime = millis();  // Reset timer
}

if ((millis() - lastDebounceTime) > debounceDelay) {
    // Signal is stable; process it
}
```

**Characteristics:**
- Ignores transients < 500ms
- Non-blocking implementation
- Prevents relay chatter

### 7.2 Threshold Detection Algorithm

**Type:** Consecutive reading counter
**Method:** Hysteresis with immediate clear

```cpp
if (sensorValue == LOW) {
    gasDetectionCount++;
    if (gasDetectionCount >= alarmThreshold) {
        activateAlarm();
    }
} else {
    gasDetectionCount = 0;  // Immediate reset
    if (gasDetected) {
        deactivateAlarm();
    }
}
```

**Characteristics:**
- Rising edge: 3 consecutive LOWs required
- Falling edge: Single HIGH immediately clears
- Asymmetric hysteresis for safety

---

## 8. Communication Protocols

### 8.1 Serial Communication (UART)

**Configuration:**
- **Baud Rate:** 9600 bps
- **Data Bits:** 8
- **Parity:** None
- **Stop Bits:** 1
- **Flow Control:** None

**Message Format:**
```
Sensor Reading: "0" or "1" (newline terminated)
Alert: "ALERT: Gas leak detected!"
Normal: "Gas levels normal"
```

### 8.2 I2C Communication

**Configuration:**
- **Clock Speed:** 100 kHz (standard mode)
- **Addressing:** 7-bit
- **LCD Address:** 0x27
- **Pull-ups:** 4.7kΩ (external or internal)

**Data Flow:**
```
Arduino (Master) → LCD (Slave)
- Command bytes (RS=0)
- Data bytes (RS=1)
- Backlight control
```

---

## 9. Pin Configuration

### 9.1 Digital I/O Mapping

| Arduino Pin | Function | Direction | Default State |
|-------------|----------|-----------|---------------|
| D2 | Green LED | OUTPUT | HIGH |
| D3 | Red LED | OUTPUT | LOW |
| D4 | MQ-2 Sensor | INPUT | Floating |
| A4 (SDA) | I2C Data | I/O | Pull-up |
| A5 (SCL) | I2C Clock | OUTPUT | Pull-up |

### 9.2 Pin Electrical Specs

| Pin | Voltage | Current (max) | Notes |
|-----|---------|---------------|-------|
| D2, D3 | 5V | 20 mA | LED current with 220Ω resistor |
| D4 | 5V | <1 mA | High-impedance input |
| SDA/SCL | 5V | 3 mA | I2C bus current |

---

## 10. Performance Specifications

### 10.1 Resource Utilization

| Resource | Used | Available | % Usage |
|----------|------|-----------|---------|
| **Flash Memory** | ~3.5 KB | 30.5 KB | 11% |
| **SRAM** | ~300 bytes | 2048 bytes | 15% |
| **EEPROM** | 0 bytes | 1024 bytes | 0% |

### 10.2 Power Consumption

**Typical Operating Current:**
- Arduino: ~50 mA
- LCD backlight: ~20 mA
- LEDs (one active): ~20 mA
- MQ-2 sensor: ~150 mA (heating element)

**Total: ~240 mA @ 5V** (1.2W)

---

## 11. Error Handling

### 11.1 Implemented Safeguards

| Error Condition | Handling Strategy |
|----------------|-------------------|
| **Sensor disconnection** | No explicit detection; system treats as HIGH (safe) |
| **LCD I2C failure** | Silent failure; LEDs continue operating |
| **Power brownout** | AVR brownout detection resets system |
| **False sensor triggers** | Debounce + threshold filtering |

### 11.2 Limitations

⚠️ **Known Constraints:**
- No watchdog timer implemented
- No EEPROM state persistence
- No battery backup
- No self-diagnostic tests
- No analog PPM measurement

---

## 12. Configuration & Tuning

### 12.1 Adjustable Parameters

```cpp
// Sensitivity tuning
int alarmThreshold = 3;          // 2-10 recommended
unsigned long debounceDelay = 500; // 100-2000ms range

// Display settings
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Change address if needed

// Sampling rate
delay(200);  // 50-1000ms acceptable
```

### 12.2 Calibration Procedure

1. Power on in clean air
2. Wait 5-10 minutes for MQ-2 warmup
3. Adjust onboard potentiometer until LED turns off
4. Expose to target gas
5. Adjust pot until LED turns on at desired threshold
6. Test with known gas concentrations

---

## 13. Testing Specifications

### 13.1 Unit Tests (Manual)

| Test Case | Procedure | Expected Result |
|-----------|-----------|-----------------|
| **Power-On Test** | Apply power | Green LED ON, LCD shows "Gas Leak Detect" |
| **Sensor Test** | Expose to gas | Red LED ON within 1 second |
| **Clear Test** | Remove gas source | Green LED ON within 1 second |
| **Debounce Test** | Quick gas puff (<500ms) | No alarm trigger |
| **Serial Test** | Open Serial Monitor | Continuous "0" or "1" output |

### 13.2 Integration Tests

- I2C bus integrity check
- Multi-sensor interference test
- Power supply ripple test
- Temperature stability test (-10°C to 50°C)

---

## 14. Compliance & Standards

### 14.1 Code Standards
- **Style:** Arduino coding conventions
- **Comments:** Inline documentation
- **Naming:** camelCase for variables, PascalCase for functions

### 14.2 Safety Notes
- ⚠️ **Not UL/CE certified** - Educational use only
- ⚠️ No fail-safe mechanisms for critical safety systems
- ⚠️ Requires regular calibration and maintenance

---

## 15. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024-11-23 | Initial release with debounce and threshold detection |

---

## 16. Build & Deployment

### 16.1 Compilation Settings

```ini
board = arduino:avr:uno
cpu = atmega328p
f_cpu = 16000000L
upload.speed = 115200
```

### 16.2 Upload Instructions

```bash
# Via Arduino CLI
arduino-cli compile --fqbn arduino:avr:uno arduino.ino
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno arduino.ino

# Via IDE
Tools > Board > Arduino Uno
Tools > Port > [Select COM/ttyUSB port]
Sketch > Upload (Ctrl+U)
```

---

## 17. Future Enhancements (Backlog)

- [ ] Implement watchdog timer for crash recovery
- [ ] Add EEPROM alarm history logging
- [ ] Support analog reading mode for PPM measurement
- [ ] Add WiFi/Bluetooth connectivity (ESP32 port)
- [ ] Implement OTA firmware updates
- [ ] Add temperature/humidity compensation
- [ ] Create web dashboard for remote monitoring

---

**Document Maintained By:** Kelvin Mwega
**Last Updated:** 2024-11-23
**License:** MIT
