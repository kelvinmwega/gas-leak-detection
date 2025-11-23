# Gas Leak Detector - Quick Reference

## What It Does

A safety device that monitors the air for combustible gases (LPG, propane, methane) and alerts you immediately when dangerous levels are detected.

**Alert System:**
- 🟢 Green LED = Air is safe
- 🔴 Red LED = Gas detected (DANGER)
- LCD screen shows real-time status
- Serial messages for logging/remote monitoring

---

## Components

| Part | Purpose |
|------|---------|
| **MQ-2 Gas Sensor** | Detects combustible gases in the air |
| **Arduino Uno/Nano** | Brain of the system - runs the detection logic |
| **16×2 LCD Display (I2C)** | Shows status messages |
| **Green LED** | Indicates safe conditions |
| **Red LED** | Warning light for gas detection |
| **220Ω Resistors (×2)** | Protects the LEDs from burning out |

---

## How It Works (Simplified)

```
┌─────────────────────────────────────────────────┐
│ 1. START SYSTEM                                 │
│    - Show "Gas Leak Detect" on screen          │
│    - Turn on GREEN LED (safe mode)             │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│ 2. READ SENSOR (every 200 milliseconds)        │
│    - MQ-2 sensor checks the air                │
│    - Sends signal: HIGH = safe, LOW = gas      │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│ 3. FILTER OUT FALSE ALARMS                     │
│    Step A: Wait 500ms to confirm reading       │
│    Step B: Need 3 consecutive gas readings     │
└─────────────────────────────────────────────────┘
                    ↓
        ┌───────────┴───────────┐
        ↓                       ↓
┌──────────────┐        ┌──────────────┐
│ GAS DETECTED │        │   ALL CLEAR  │
└──────────────┘        └──────────────┘
        ↓                       ↓
┌──────────────┐        ┌──────────────┐
│ Turn on RED  │        │ Turn on GREEN│
│ Turn off GREEN│        │ Turn off RED │
│              │        │              │
│ LCD: DANGER  │        │ LCD: OK      │
│              │        │              │
│ Serial: ALERT│        │ Serial: Normal│
└──────────────┘        └──────────────┘
        └───────────┬───────────┘
                    ↓
            ┌───────────────┐
            │ REPEAT FOREVER│
            └───────────────┘
```

---

## The Logic (Pseudocode)

```plaintext
START:
  Set up sensor, LEDs, and LCD screen
  Display "Gas Leak Detect - Health: OK"
  Turn GREEN LED on

LOOP (every 200ms):
  Read sensor value

  IF sensor changed:
    Start 500ms timer (debounce protection)

  IF reading been stable for 500ms:

    IF sensor says GAS (value = LOW):
      Add 1 to gas counter

      IF counter reaches 3:
        TRIGGER ALARM:
          - Red LED ON, Green LED OFF
          - LCD: "GAS DETECTED! Status: DANGER"
          - Send alert message

    IF sensor says SAFE (value = HIGH):
      Reset gas counter to 0

      IF alarm is currently on:
        CLEAR ALARM:
          - Green LED ON, Red LED OFF
          - LCD: "GAS LEVEL Status: OK"
          - Send "normal" message

  Update display
  Wait 200ms
  Repeat forever
```

---

## Key Safety Features

**Two-Layer Protection Against False Alarms:**

1. **Debounce Timer (500ms)**
   - Ignores electrical noise and quick spikes
   - Only acts on stable readings

2. **3-Reading Threshold**
   - Requires gas to be present for at least 3 consecutive checks
   - One clean reading resets the counter immediately

**Why This Matters:**
- Won't trigger from dust, vibration, or electrical interference
- Responds within ~1 second when real gas is present
- Clears alarm immediately when air is safe again

---

## Quick Setup

1. **Wire the components:**
   - MQ-2 sensor → Pin 4
   - Green LED → Pin 2 (with resistor)
   - Red LED → Pin 3 (with resistor)
   - LCD → I2C pins (SDA/SCL)

2. **Install library:**
   - Open Arduino IDE
   - Install "LiquidCrystal I2C" library

3. **Upload code:**
   - Connect Arduino via USB
   - Upload [arduino.ino](arduino.ino)

4. **Test:**
   - Power on → Green LED should light up
   - Expose sensor to gas (lighter, alcohol) → Red LED triggers

---

## Real-World Behavior

**Normal Operation:**
```
LCD: "GAS LEVEL"
     "Status: OK  "
LED: 🟢 Green ON
```

**Gas Detected:**
```
LCD: "GAS DETECTED!"
     "Status: DANGER"
LED: 🔴 Red ON (Green OFF)
Serial: "ALERT: Gas leak detected!"
```

**Response Time:**
- Sensor reading: **200ms** intervals
- Debounce window: **500ms** for stable signal
- Alarm trigger: **~1 second** after sustained gas presence
- Alarm clear: **Immediate** when air is safe

---

## Tuning for Your Environment

**More Sensitive (faster alerts, more false alarms):**
```cpp
int alarmThreshold = 2;          // 2 readings instead of 3
unsigned long debounceDelay = 300;  // 300ms instead of 500ms
```

**More Stable (slower response, fewer false alarms):**
```cpp
int alarmThreshold = 5;          // 5 readings required
unsigned long debounceDelay = 1000;  // 1 full second delay
```

---

## Safety Warning

⚠️ **This is a learning/prototyping project**

For life-safety applications:
- Use certified commercial gas detectors
- Add loud audible alarm (buzzer)
- Install in proper locations per fire code
- Test regularly with calibration gas
- Consider professional installation

---

**Built with Arduino for safety and education**
