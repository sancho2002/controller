#include <HardwareSerial.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define the serial connection to the E32 module
HardwareSerial loraSerial(1); // Using UART1

// E32 module pins
#define M0_PIN 25
#define M1_PIN 26
// AUX pin not connected

// Control pins
#define START_BUTTON_PIN 17
#define STOP_BUTTON_PIN 16
#define PWM_OUTPUT_PIN 21

// LCD configuration
#define LCD_ADDRESS 0x27 // Common I2C address for 20x4 LCD
#define LCD_COLS 20
#define LCD_ROWS 4
#define LCD_SDA 32
#define LCD_SCL 33
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// Configuration parameters
#define LORA_FREQUENCY 433    // Frequency in MHz
#define LORA_ADDRESS_H 0x00   // High byte of address
#define LORA_ADDRESS_L 0x01   // Low byte of address
#define LORA_CHANNEL 0x17     // Channel (0-31)
#define LORA_AIR_DATA_RATE 0x02 // Air data rate

// ADS1115 setup
Adafruit_ADS1115 ads;  // Use ADS1115 (16-bit)
#define ADS1115_ADDRESS 0x4A  // I2C address
#define ADS_SDA 22      // IO22 for SDA
#define ADS_SCL 23      // IO23 for SCL
#define RESISTOR_VALUE 100.0  // 100 ohm resistor

// Sensor ranges
#define PRESSURE_1_RANGE 10.0   // 0-10 bar
#define PRESSURE_2_RANGE 40.0   // 0-25 bar (still read but not transmitted)
#define LEVEL_RANGE 5.0         // 0-5 meters

// Multiplexer configuration
#define MULTIPLEX_RATIO 3.11    // Multiplex ratio

// PWM configuration for voltage control
#define PWM_FREQUENCY 1000      // 1kHz PWM frequency
#define PWM_RESOLUTION 12       // 12-bit resolution (0-4095)
#define PWM_CHANNEL 0           // PWM channel 0
#define MAX_VOLTAGE 3.3         // Maximum output voltage
#define MIN_VOLTAGE 0.0         // Minimum output voltage

// Control thresholds
#define PRESSURE_1_MIN 0.3      // Minimum pressure 1 (bar)
#define PRESSURE_1_START 1.0    // Start pressure 1 (bar)
#define PRESSURE_2_MAX 28.0     // Maximum pressure 2 (bar)
#define LEVEL_MIN 0.7           // Minimum water level (m)
#define LEVEL_START 0.9         // Start water level (m)

// Control parameters
#define VOLTAGE_STEP_UP 0.02    // Voltage increase step (V)
#define VOLTAGE_STEP_P1_DOWN 0.5 // Voltage decrease for pressure 1 (V)
#define VOLTAGE_STEP_P2_DOWN 0.2 // Voltage decrease for pressure 2 (V)
#define VOLTAGE_STEP_LEVEL_DOWN 0.1 // Voltage decrease for level (V)
#define CHECK_DELAY 300         // Check delay (ms)
#define LEVEL_CHECK_DELAY 10000 // Level check delay (ms)

// Timing intervals
unsigned long previousMillis = 0;
const long interval = 1000; // 1 second interval for LoRa transmission
const long packageDelay = 200; // 200ms between packages
const long lcdInterval = 250; // 250ms interval for LCD
unsigned long lastLcdUpdate = 0;

// Message type tracking
bool sendPressureMessage = true; // Start with pressure message

// Control state machine
enum ControlState {
  IDLE,
  STARTING,
  RUNNING,
  ADJUSTING_P1,
  ADJUSTING_P2,
  ADJUSTING_LEVEL,
  STOPPING
};

ControlState currentState = IDLE;
unsigned long stateStartTime = 0;
unsigned long lastCheckTime = 0;
float currentVoltage = 0.0;
float targetVoltage = 0.0;
bool autoControlActive = false;

// Button state tracking
bool startButtonPressed = false;
bool stopButtonPressed = false;
bool lastStartButtonState = HIGH;
bool lastStopButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Initialize LoRa module serial (RX=27, TX=14)
  loraSerial.begin(9600, SERIAL_8N1, 27, 14);
  
  // Set mode pins
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  
  // Set to Normal mode (M0=0, M1=0)
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, LOW);
  
  // Setup control pins
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  
  // Setup PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_OUTPUT_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0); // Start with 0V output
  
  // Initialize LCD
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Initializing");
  delay(1000);
  lcd.clear();

  // Initialize ADS1115
  Wire1.begin(ADS_SDA, ADS_SCL);
  if (!ads.begin(ADS1115_ADDRESS, &Wire1)) {
    Serial.println("Failed to initialize ADS1115!");
    lcd.setCursor(0, 0);
    lcd.print("ADS1115 Error!");
    while (1);
  }
  
  // Set gain to 1x (FS = ±4.096V)
  ads.setGain(GAIN_ONE);
  
  Serial.println("System initialized");
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
  delay(1000);
  lcd.clear();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Handle button inputs
  handleButtons();
  
  // Run automatic control state machine
  runControlStateMachine();
  
  // Update LCD more frequently (every 250ms)
  if (currentMillis - lastLcdUpdate >= lcdInterval) {
    lastLcdUpdate = currentMillis;
    
    // Read sensor values
    float pressure1 = readSensor(0, PRESSURE_1_RANGE);
    float pressure2 = readSensor(1, PRESSURE_2_RANGE);
    float level = readSensor(2, LEVEL_RANGE);
    
    // Read value from ADS1115 fourth pin (A3) and apply multiplex ratio
    float multiplexedValue = readMultiplexedValue(3, MULTIPLEX_RATIO);
    
    // Update LCD display
    updateLCD(pressure1, pressure2, level, multiplexedValue, currentVoltage, currentState);
  }
  
  // Send LoRa messages with alternating types every second
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Read fresh sensor values for transmission
    float pressure1 = readSensor(0, PRESSURE_1_RANGE);
    float pressure2 = readSensor(1, PRESSURE_2_RANGE);
    float level = readSensor(2, LEVEL_RANGE);
    float multiplexedValue = readMultiplexedValue(3, MULTIPLEX_RATIO);
    
    if (sendPressureMessage) {
      // Send pressure message "P0.00,0.00"
      String message = "P" + String(pressure1, 2) + 
                     "," + String(pressure2, 2);
      sendMessage(message);
      Serial.println("Pressure message sent: " + message);
    } else {
      // Send level/mux message "L0.00,0.00"
      String message = "L" + String(level, 2) + 
                     "," + String(multiplexedValue, 2);
      sendMessage(message);
      Serial.println("Level/Mux message sent: " + message);
      
      // Add 200ms delay between packages
      delay(packageDelay);
    }
    
    // Toggle message type for next transmission
    sendPressureMessage = !sendPressureMessage;
  }
}

void handleButtons() {
  unsigned long currentTime = millis();
  
  // Read button states
  bool startButtonState = digitalRead(START_BUTTON_PIN);
  bool stopButtonState = digitalRead(STOP_BUTTON_PIN);
  
  // Debounce start button
  if (startButtonState != lastStartButtonState) {
    lastDebounceTime = currentTime;
  }
  
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    if (startButtonState == LOW && !startButtonPressed) {
      startButtonPressed = true;
      Serial.println("Start button pressed");
    }
    if (startButtonState == HIGH && startButtonPressed) {
      startButtonPressed = false;
    }
  }
  
  // Debounce stop button
  if (stopButtonState != lastStopButtonState) {
    lastDebounceTime = currentTime;
  }
  
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    if (stopButtonState == LOW && !stopButtonPressed) {
      stopButtonPressed = true;
      Serial.println("Stop button pressed");
    }
    if (stopButtonState == HIGH && stopButtonPressed) {
      stopButtonPressed = false;
    }
  }
  
  lastStartButtonState = startButtonState;
  lastStopButtonState = stopButtonState;
}

void runControlStateMachine() {
  unsigned long currentTime = millis();
  
  // Read current sensor values
  float pressure1 = readSensor(0, PRESSURE_1_RANGE);
  float pressure2 = readSensor(1, PRESSURE_2_RANGE);
  float level = readSensor(2, LEVEL_RANGE);
  
  switch (currentState) {
    case IDLE:
      if (startButtonPressed && pressure1 > PRESSURE_1_START && level > LEVEL_START) {
        currentState = STARTING;
        stateStartTime = currentTime;
        autoControlActive = true;
        targetVoltage = 0.0;
        Serial.println("Starting automatic control");
      }
      break;
      
    case STARTING:
      if (stopButtonPressed) {
        currentState = STOPPING;
        stateStartTime = currentTime;
        Serial.println("Stopping automatic control");
        break;
      }
      
      // Check if conditions are still met
      if (pressure1 < PRESSURE_1_START || level < LEVEL_START) {
        currentState = IDLE;
        autoControlActive = false;
        setVoltage(0.0);
        Serial.println("Start conditions not met, returning to IDLE");
        break;
      }
      
      // Start increasing voltage
      currentState = RUNNING;
      targetVoltage = currentVoltage + VOLTAGE_STEP_UP;
      lastCheckTime = currentTime;
      Serial.println("Entering RUNNING state");
      break;
      
    case RUNNING:
      if (stopButtonPressed) {
        currentState = STOPPING;
        stateStartTime = currentTime;
        Serial.println("Stopping automatic control");
        break;
      }
      
      // Check if we need to adjust
      if (pressure1 < PRESSURE_1_MIN) {
        currentState = ADJUSTING_P1;
        stateStartTime = currentTime;
        targetVoltage = currentVoltage - VOLTAGE_STEP_P1_DOWN;
        Serial.println("Pressure 1 too low, adjusting down");
        break;
      }
      
      if (pressure2 > PRESSURE_2_MAX) {
        currentState = ADJUSTING_P2;
        stateStartTime = currentTime;
        targetVoltage = currentVoltage - VOLTAGE_STEP_P2_DOWN;
        Serial.println("Pressure 2 too high, adjusting down");
        break;
      }
      
      if (level < LEVEL_MIN) {
        currentState = ADJUSTING_LEVEL;
        stateStartTime = currentTime;
        targetVoltage = currentVoltage - VOLTAGE_STEP_LEVEL_DOWN;
        Serial.println("Water level too low, adjusting down");
        break;
      }
      
      // If all parameters are good, continue increasing voltage
      if (currentTime - lastCheckTime >= CHECK_DELAY) {
        targetVoltage = currentVoltage + VOLTAGE_STEP_UP;
        lastCheckTime = currentTime;
        Serial.println("All parameters OK, increasing voltage");
      }
      break;
      
    case ADJUSTING_P1:
      if (stopButtonPressed) {
        currentState = STOPPING;
        stateStartTime = currentTime;
        break;
      }
      
      // Set voltage and wait
      setVoltage(targetVoltage);
      
      if (currentTime - stateStartTime >= CHECK_DELAY) {
        if (pressure1 >= PRESSURE_1_MIN) {
          currentState = RUNNING;
          lastCheckTime = currentTime;
          Serial.println("Pressure 1 normalized, returning to RUNNING");
        } else {
          // Try again with lower voltage
          targetVoltage = currentVoltage - VOLTAGE_STEP_P1_DOWN;
          stateStartTime = currentTime;
          Serial.println("Pressure 1 still low, decreasing voltage further");
        }
      }
      break;
      
    case ADJUSTING_P2:
      if (stopButtonPressed) {
        currentState = STOPPING;
        stateStartTime = currentTime;
        break;
      }
      
      // Set voltage and wait
      setVoltage(targetVoltage);
      
      if (currentTime - stateStartTime >= CHECK_DELAY) {
        if (pressure2 <= PRESSURE_2_MAX) {
          currentState = RUNNING;
          lastCheckTime = currentTime;
          Serial.println("Pressure 2 normalized, returning to RUNNING");
        } else {
          // Try again with lower voltage
          targetVoltage = currentVoltage - VOLTAGE_STEP_P2_DOWN;
          stateStartTime = currentTime;
          Serial.println("Pressure 2 still high, decreasing voltage further");
        }
      }
      break;
      
    case ADJUSTING_LEVEL:
      if (stopButtonPressed) {
        currentState = STOPPING;
        stateStartTime = currentTime;
        break;
      }
      
      // Set voltage and wait
      setVoltage(targetVoltage);
      
      if (currentTime - stateStartTime >= LEVEL_CHECK_DELAY) {
        if (level >= LEVEL_MIN) {
          currentState = RUNNING;
          lastCheckTime = currentTime;
          Serial.println("Water level normalized, returning to RUNNING");
        } else {
          // Try again with lower voltage
          targetVoltage = currentVoltage - VOLTAGE_STEP_LEVEL_DOWN;
          stateStartTime = currentTime;
          Serial.println("Water level still low, decreasing voltage further");
        }
      }
      break;
      
    case STOPPING:
      setVoltage(0.0);
      autoControlActive = false;
      currentState = IDLE;
      Serial.println("Automatic control stopped");
      break;
  }
  
  // Update voltage if target has changed
  if (targetVoltage != currentVoltage) {
    setVoltage(targetVoltage);
  }
}

void setVoltage(float voltage) {
  // Constrain voltage to valid range
  voltage = constrain(voltage, MIN_VOLTAGE, MAX_VOLTAGE);
  
  // Convert voltage to PWM value
  int pwmValue = (int)((voltage / MAX_VOLTAGE) * ((1 << PWM_RESOLUTION) - 1));
  
  // Set PWM output
  ledcWrite(PWM_CHANNEL, pwmValue);
  currentVoltage = voltage;
  
  Serial.print("Setting voltage: ");
  Serial.print(voltage, 3);
  Serial.print("V (PWM: ");
  Serial.print(pwmValue);
  Serial.println(")");
}

float readSensor(int channel, float range) {
  // Read the ADC value (0-32767 for ±4.096V)
  int16_t adcValue = ads.readADC_SingleEnded(channel);
  
  // Convert to voltage (4.096V is full scale for GAIN_ONE)
  float voltage = (adcValue * 4.096) / 32767.0;
  
  // Convert to current (4-20mA with 100 ohm resistor)
  float current = (voltage / RESISTOR_VALUE) * 1000.0; // in mA
  
  // Convert to measurement (4mA = 0, 20mA = range)
  float value = 0.0;
  if (current >= 4.0) {
    value = ((current - 4.0) / 16.0) * range;
    value = constrain(value, 0, range); // Limit to range
  }
  
  return value;
}

float readMultiplexedValue(int channel, float ratio) {
  // Read the ADC value from the specified channel (0-32767 for ±4.096V)
  int16_t adcValue = ads.readADC_SingleEnded(channel);
  
  // Convert to voltage (4.096V is full scale for GAIN_ONE)
  float voltage = (adcValue * 4.096) / 32767.0;
  
  // Apply multiplex ratio
  float multiplexedValue = voltage * ratio;
  
  return multiplexedValue;
}

void updateLCD(float p1, float p2, float lvl, float multiplexed, float voltage, ControlState state) {
  lcd.clear();
  delay(2); // Small delay to allow clear to complete
  
  // Line 1: Pressure Sensor 1
  lcd.setCursor(0, 0);
  lcd.print("P1: ");
  lcd.print(p1, 2);
  lcd.print(" bar");
  
  // Line 2: Pressure Sensor 2 (still displayed but not transmitted)
  lcd.setCursor(0, 1);
  lcd.print("P2: ");
  lcd.print(p2, 2);
  lcd.print(" bar");
  
  // Line 3: Level Sensor
  lcd.setCursor(0, 2);
  lcd.print("Lvl: ");
  lcd.print(lvl, 2);
  lcd.print(" m");
  
  // Line 4: Control status and voltage
  lcd.setCursor(0, 3);
  lcd.print("V: ");
  lcd.print(voltage, 2);
  lcd.print("V ");
  
  // Show control state
  switch (state) {
    case IDLE:
      lcd.print("IDLE");
      break;
    case STARTING:
      lcd.print("START");
      break;
    case RUNNING:
      lcd.print("RUN");
      break;
    case ADJUSTING_P1:
      lcd.print("ADJ P1");
      break;
    case ADJUSTING_P2:
      lcd.print("ADJ P2");
      break;
    case ADJUSTING_LEVEL:
      lcd.print("ADJ LVL");
      break;
    case STOPPING:
      lcd.print("STOP");
      break;
  }
}

void sendMessage(String message) {
  // Prepare the message with headers (address and channel)
  uint8_t header[3] = {
    LORA_ADDRESS_H,  // High address byte
    LORA_ADDRESS_L,  // Low address byte
    LORA_CHANNEL     // Channel
  };
  
  // Send header
  loraSerial.write(header, sizeof(header));
  
  // Send message
  loraSerial.print(message);
  
  // Small delay since AUX isn't connected
  delay(50);
}

void configureModule() {
  // Set to Configuration mode (M0=1, M1=1)
  digitalWrite(M0_PIN, HIGH);
  digitalWrite(M1_PIN, HIGH);
  delay(1000); // Wait for mode change
  
  // Configuration command (0xC0 to write to module)
  uint8_t configCommand = 0xC0;
  
  // Configuration parameters
  uint8_t configParams[6] = {
    LORA_ADDRESS_H,      // Address high
    LORA_ADDRESS_L,      // Address low
    0x00 |              // Speed/Parity (8N1)
    (LORA_AIR_DATA_RATE << 3), // Air data rate
    LORA_CHANNEL,       // Channel
    0x44,               // Option (fixed transmission, pull-up enabled)
    0x00                // Wake-up time (not used in normal mode)
  };
  
  // Send configuration
  loraSerial.write(configCommand);
  loraSerial.write(configParams, sizeof(configParams));
  
  // Wait for configuration to complete
  delay(1000);
  
  // Return to Normal mode
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, LOW);
  delay(1000);
}