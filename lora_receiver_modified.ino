#include <HardwareSerial.h>
#include <ArduinoBLE.h>
#include <driver/ledc.h>

// E32 module pins
#define M0_PIN 10
#define M1_PIN 10  // Both M0 and M1 connected to IO10
#define BUZZER_PIN 9

// LED Pins and Configuration
#define LED_RED_PIN 2
#define LED_GREEN_PIN 3
#define LED_BLUE_PIN 4
#define VCC 3.3f  // Typical operating voltage

// Button pin with internal pullup
#define BUTTON_PIN 1

// LoRa Configuration (must match transmitter)
#define LORA_ADDRESS_H 0x00
#define LORA_ADDRESS_L 0x01
#define LORA_CHANNEL 0x17

// BLE Service and Characteristic UUIDs
#define BLE_UUID_SERVICE "19b10000-e8f2-537e-4f6c-d104768a1214"
#define BLE_UUID_MESSAGE "19b10001-e8f2-537e-4f6c-d104768a1214"

// Timing constants
#define DATA_TIMEOUT_MS 10000  // 10 seconds timeout
#define SILENT_MODE_MS 120000  // 2 minutes silent mode
#define LONG_BEEP_MS 1000      // Long beep duration
#define SHORT_BEEP_MS 300      // Short beep duration
#define LED_BLINK_MS 200       // LED blink duration

BLEService loraService(BLE_UUID_SERVICE);
BLECharacteristic messageCharacteristic(BLE_UUID_MESSAGE, BLERead | BLENotify, 40);

HardwareSerial loraSerial(1);  // UART1 (RX=20, TX=21)

// LED Status Variables
bool radioConnected = false;
unsigned long lastBlinkTime = 0;
bool ledState = false;

// Data parsing variables
String receivedData = "";
bool dataReceived = false;
unsigned long lastDataTime = 0;

// Alert system variables
bool silentMode = false;
unsigned long silentModeStart = 0;
bool alertActive = false;
unsigned long alertStartTime = 0;
String alertType = "";  // "timeout", "low_value", "ok"

// Button variables
bool buttonPressed = false;
bool lastButtonState = HIGH;
unsigned long buttonPressStart = 0;
const unsigned long LONG_PRESS_MS = 1000;  // 1 second for long press

void setupLEDs() {
  // Configure PWM timer
  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,  // 1024 steps
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000,  // 1kHz frequency
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  // Configure LED channels (initial duty cycle 0 = off)
  ledc_channel_config_t channel_conf_red = {
    .gpio_num = LED_RED_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf_red);

  ledc_channel_config_t channel_conf_green = {
    .gpio_num = LED_GREEN_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf_green);

  ledc_channel_config_t channel_conf_blue = {
    .gpio_num = LED_BLUE_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf_blue);

  Serial.println("LED PWM outputs configured");
}

void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
  // Convert 0-255 values to 0-1023 duty cycle
  ledcWrite(LEDC_CHANNEL_0, map(r, 0, 255, 0, 1023));
  ledcWrite(LEDC_CHANNEL_1, map(g, 0, 255, 0, 1023));
  ledcWrite(LEDC_CHANNEL_2, map(b, 0, 255, 0, 1023));
}

void updateLEDStatus() {
  if (alertActive) {
    // Handle active alerts
    if (alertType == "timeout") {
      // Long green blink for timeout
      if (millis() - lastBlinkTime > 500) {
        lastBlinkTime = millis();
        ledState = !ledState;
        if (ledState) {
          setLEDColor(0, 255, 0);  // Green on
        } else {
          setLEDColor(0, 0, 0);    // All off
        }
      }
    } else if (alertType == "low_value") {
      // Short blue blink for low values
      if (millis() - lastBlinkTime > 300) {
        lastBlinkTime = millis();
        ledState = !ledState;
        if (ledState) {
          setLEDColor(0, 0, 255);  // Blue on
        } else {
          setLEDColor(0, 0, 0);    // All off
        }
      }
    } else if (alertType == "ok") {
      // Solid green for OK
      setLEDColor(0, 255, 0);
    }
  } else if (radioConnected) {
    // Solid green when connected and receiving data
    setLEDColor(0, 255, 0);
  } else {
    // Blinking blue when not connected (500ms interval)
    if (millis() - lastBlinkTime > 500) {
      lastBlinkTime = millis();
      ledState = !ledState;
      if (ledState) {
        setLEDColor(0, 0, 255);  // Blue on
      } else {
        setLEDColor(0, 0, 0);    // All off
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  loraSerial.begin(9600, SERIAL_8N1, 20, 21);

  // Initialize LED control
  setupLEDs();

  // Set LoRa module to Normal mode
  pinMode(M0_PIN, OUTPUT);
  digitalWrite(M0_PIN, LOW);

  // Setup button with internal pullup
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Setup buzzer with PWM for louder sound
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(0, 2000, 8);  // 2000 Hz, 8-bit resolution
  ledcAttachPin(BUZZER_PIN, 0);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set advertised name and service
  BLE.setLocalName("LoRa-Receiver");
  BLE.setAdvertisedService(loraService);

  // Add characteristic to service
  loraService.addCharacteristic(messageCharacteristic);

  // Add service
  BLE.addService(loraService);

  // Start advertising
  BLE.advertise();

  Serial.println("BLE LoRa Receiver Ready");
  alertStartup();  // Loud startup beep sequence
  
  // Initialize timing
  lastDataTime = millis();
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
  } else {
    processLoRaMessages();
    checkButton();
    checkDataTimeout();
    updateLEDStatus();
    handleAlerts();
  }
}

void checkButton() {
  bool currentButtonState = digitalRead(BUTTON_PIN);
  
  // Detect button press (LOW because of pullup)
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    buttonPressStart = millis();
    buttonPressed = true;
  }
  
  // Detect button release
  if (currentButtonState == HIGH && lastButtonState == LOW && buttonPressed) {
    unsigned long pressDuration = millis() - buttonPressStart;
    
    if (pressDuration >= LONG_PRESS_MS) {
      // Long press - activate silent mode
      silentMode = true;
      silentModeStart = millis();
      alertActive = false;
      ledcWrite(0, 0);  // Stop buzzer
      setLEDColor(0, 0, 0);  // Turn off LEDs
      Serial.println("Silent mode activated for 2 minutes");
    }
    
    buttonPressed = false;
  }
  
  lastButtonState = currentButtonState;
  
  // Check if silent mode should end
  if (silentMode && (millis() - silentModeStart >= SILENT_MODE_MS)) {
    silentMode = false;
    Serial.println("Silent mode ended");
  }
}

void checkDataTimeout() {
  if (dataReceived && (millis() - lastDataTime >= DATA_TIMEOUT_MS)) {
    // Data timeout - trigger long beep and green blink
    if (!silentMode) {
      alertActive = true;
      alertType = "timeout";
      alertStartTime = millis();
      Serial.println("Data timeout - long beep and green blink");
    }
    dataReceived = false;
  }
}

void processLoRaMessages() {
  if (loraSerial.available() >= 3) {
    // Read header
    uint8_t addressHigh = loraSerial.read();
    uint8_t addressLow = loraSerial.read();
    uint8_t channel = loraSerial.read();
    
    // Check if it's our address and channel
    if (addressHigh == LORA_ADDRESS_H && addressLow == LORA_ADDRESS_L && channel == LORA_CHANNEL) {
      // Read data length
      uint8_t dataLength = loraSerial.read();
      
      // Read the actual data
      receivedData = "";
      for (int i = 0; i < dataLength; i++) {
        if (loraSerial.available()) {
          char c = loraSerial.read();
          receivedData += c;
        }
      }
      
      // Process the received data
      processReceivedData();
      
      // Update timing
      lastDataTime = millis();
      dataReceived = true;
      radioConnected = true;
    }
  } else {
    radioConnected = false;
  }
}

void processReceivedData() {
  Serial.print("Received data: ");
  Serial.println(receivedData);
  
  // Parse data format: "Pn1,n2" or "Ln3,n4"
  if (receivedData.startsWith("P")) {
    // Parse P format: Pn1,n2
    int commaIndex = receivedData.indexOf(',');
    if (commaIndex > 0) {
      String n1Str = receivedData.substring(1, commaIndex);
      String n2Str = receivedData.substring(commaIndex + 1);
      
      float n1 = n1Str.toFloat();
      float n2 = n2Str.toFloat();
      
      Serial.print("P data - n1: ");
      Serial.print(n1);
      Serial.print(", n2: ");
      Serial.println(n2);
      
      // Check if n2 is lower than 0.5
      if (n2 < 0.5) {
        if (!silentMode) {
          alertActive = true;
          alertType = "low_value";
          alertStartTime = millis();
          Serial.println("Low value alert - short beep and blue blink");
        }
      } else {
        // Values are OK
        if (!silentMode) {
          alertActive = true;
          alertType = "ok";
          alertStartTime = millis();
        }
      }
    }
  } else if (receivedData.startsWith("L")) {
    // Parse L format: Ln3,n4
    int commaIndex = receivedData.indexOf(',');
    if (commaIndex > 0) {
      String n3Str = receivedData.substring(1, commaIndex);
      String n4Str = receivedData.substring(commaIndex + 1);
      
      float n3 = n3Str.toFloat();
      float n4 = n4Str.toFloat();
      
      Serial.print("L data - n3: ");
      Serial.print(n3);
      Serial.print(", n4: ");
      Serial.println(n4);
      
      // Check if n3 is lower than 0.65
      if (n3 < 0.65) {
        if (!silentMode) {
          alertActive = true;
          alertType = "low_value";
          alertStartTime = millis();
          Serial.println("Low value alert - short beep and blue blink");
        }
      } else {
        // Values are OK
        if (!silentMode) {
          alertActive = true;
          alertType = "ok";
          alertStartTime = millis();
        }
      }
    }
  }
  
  // Send data via BLE if connected
  if (BLE.central() && messageCharacteristic.canNotify()) {
    messageCharacteristic.writeValue(receivedData.c_str());
  }
}

void handleAlerts() {
  if (alertActive && !silentMode) {
    if (alertType == "timeout") {
      // Long beep and green blink
      if (millis() - alertStartTime < LONG_BEEP_MS) {
        ledcWriteTone(0, 2000);  // 2000 Hz
        ledcWrite(0, 200);       // ~78% duty cycle
      } else {
        ledcWrite(0, 0);  // Stop buzzer
        alertActive = false;
      }
    } else if (alertType == "low_value") {
      // Short beep and blue blink
      if (millis() - alertStartTime < SHORT_BEEP_MS) {
        ledcWriteTone(0, 2500);  // 2500 Hz
        ledcWrite(0, 200);       // ~78% duty cycle
      } else {
        ledcWrite(0, 0);  // Stop buzzer
        alertActive = false;
      }
    } else if (alertType == "ok") {
      // No sound, just green LED
      ledcWrite(0, 0);  // Ensure buzzer is off
      if (millis() - alertStartTime > 1000) {  // Show OK for 1 second
        alertActive = false;
      }
    }
  } else if (silentMode) {
    // Ensure buzzer is off in silent mode
    ledcWrite(0, 0);
  }
}

void alertStartup() {
  // Startup sequence with LED feedback
  setLEDColor(255, 0, 0);  // Red
  delay(200);
  setLEDColor(0, 255, 0);  // Green
  delay(200);
  setLEDColor(0, 0, 255);  // Blue
  delay(200);
  setLEDColor(0, 0, 0);    // Off

  // Loud startup sequence (3 beeps)
  for (int i = 0; i < 3; i++) {
    ledcWriteTone(0, 2000);  // 2000 Hz
    ledcWrite(0, 128);       // 50% duty cycle
    delay(150);
    ledcWrite(0, 0);         // Off
    delay(100);
  }
}

void alertMessageReceived() {
  // LED feedback for message received
  setLEDColor(0, 255, 255);  // Cyan
  delay(100);
  setLEDColor(0, 255, 0);    // Green

  // More noticeable message alert pattern
  ledcWriteTone(0, 3000);  // 3000 Hz
  ledcWrite(0, 200);       // ~78% duty cycle
  delay(200);
  ledcWrite(0, 0);
  delay(50);

  ledcWriteTone(0, 2000);  // 2000 Hz
  ledcWrite(0, 200);
  delay(300);
  ledcWrite(0, 0);
  delay(50);

  ledcWriteTone(0, 2500);
  ledcWrite(0, 200);
  delay(100);
  ledcWrite(0, 0);
}

void beepBuzzer() {
  // Standard beep pattern
  for (int i = 0; i < 3; i++) {
    ledcWriteTone(0, 2500);
    ledcWrite(0, 200);
    delay(150);
    ledcWrite(0, 0);
    delay(50);
  }
}