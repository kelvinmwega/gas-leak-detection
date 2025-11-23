#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define MQ2pin 4

// Pin definitions
int greenLedPin = 2;
int redLedPin = 3;

// Sensor variables
int sensorValue;
boolean gasDetected = false;
boolean lastGasState = false;

// Debounce variables
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 500; // 500ms debounce

// Threshold for alarm (consecutive readings)
int alarmThreshold = 3;
int gasDetectionCount = 0;

void setup()
{
   Serial.begin(9600);

   pinMode(redLedPin, OUTPUT);
   pinMode(greenLedPin, OUTPUT);

   lcd.init();
   lcd.backlight();

   lcd.setCursor(0, 0);
   lcd.print("Gas Leak Detect");

   lcd.setCursor(0, 1);
   lcd.print("Health: OK");

   // Initialize with green LED (safe state)
   digitalWrite(greenLedPin, HIGH);
   digitalWrite(redLedPin, LOW);

   delay(2000);
}

void loop()
{
   // Read sensor (MQ-2 outputs LOW when gas detected, HIGH when safe)
   sensorValue = digitalRead(MQ2pin);

   Serial.println(sensorValue);

   // Debounce logic: ensure stable reading
   if (sensorValue != lastGasState)
   {
      lastDebounceTime = millis();
   }

   if ((millis() - lastDebounceTime) > debounceDelay)
   {
      // Reading has been stable for debounce period

      if (sensorValue == LOW)
      {
         // Gas detected (MQ-2 pulls LOW on detection)
         gasDetectionCount++;

         if (gasDetectionCount >= alarmThreshold && !gasDetected)
         {
            gasDetected = true;
            activateAlarm();
         }
      }
      else
      {
         // No gas detected (sensor reads HIGH = safe)
         gasDetectionCount = 0;

         if (gasDetected)
         {
            gasDetected = false;
            deactivateAlarm();
         }
      }
   }

   lastGasState = sensorValue;

   // Update display with live reading
   updateDisplay();

   delay(200); // Sample every 200ms
}

void activateAlarm()
{
   // Turn on red LED, turn off green LED
   digitalWrite(redLedPin, HIGH);
   digitalWrite(greenLedPin, LOW);

   // Update LCD
   lcd.clear();
   lcd.setCursor(0, 0);
   lcd.print("GAS DETECTED!");
   lcd.setCursor(0, 1);
   lcd.print("Status: DANGER");

   // Optional: Sound alarm via Serial
   Serial.println("ALERT: Gas leak detected!");
}

void deactivateAlarm()
{
   // Turn on green LED, turn off red LED
   digitalWrite(greenLedPin, HIGH);
   digitalWrite(redLedPin, LOW);

   // Update LCD
   lcd.clear();
   lcd.setCursor(0, 0);
   lcd.print("GAS LEVEL");
   lcd.setCursor(0, 1);
   lcd.print("Status: OK");

   Serial.println("Gas levels normal");
}

void updateDisplay()
{
   // Periodic display refresh without clearing (to avoid flicker)
   if (!gasDetected)
   {
      lcd.setCursor(12, 1);
      lcd.print(sensorValue == HIGH ? "  " : "CHK ");
   }
}
