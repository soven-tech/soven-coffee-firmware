// Soven Coffee Maker Firmware - MVP1 with NeoPixelBus RGBW
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <NeoPixelBus.h>
#include <Preferences.h>

Preferences preferences;
String deviceSerial = "";
String broadcastName = "";

// Pin definitions
#define RELAY_PIN 10        // Relay control
#define LED_DATA_PIN 4      // WS2812B data line
#define TEMP_SENSOR_PIN 7   // Thermistor ADC input
#define NUM_LEDS 3          // 3 LEDs in strip

// Temperature thresholds
#define EMPTY_RESERVOIR_TEMP 115.0  // °C - temp spike when dry
#define NORMAL_BREW_TEMP 100.0      // °C - normal boiling temp

// Global LED state array
bool ledStates[3] = {false, false, false};

// Hybrid LED control
bool appControlsLeds = false;
unsigned long lastAppLedUpdate = 0;
const unsigned long APP_LED_TIMEOUT = 5000; // 5 seconds - firmware takes back control after this

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define STATE_CHAR_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define TEMP_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define COMMAND_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26ab"
#define CONVO_CHAR_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26ac"
#define LED_STATE_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26ad"

BLECharacteristic* pLedStateChar = NULL;

// NeoPixelBus - RGBW with S3-optimized RMT driver
NeoPixelBus<NeoGrbwFeature, NeoEsp32Rmt0800KbpsMethod> strip(NUM_LEDS, LED_DATA_PIN);

// State machine
enum BrewState {
    IDLE,
    BREWING,
    KEEPING_WARM,
    ERROR
};

enum ConnectionState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED
};

// Global state
BrewState currentState = IDLE;
ConnectionState connState = DISCONNECTED;
float heatingElementTemp = 25.0;
unsigned long brewStartTime = 0;
unsigned long stateStartTime = 0;
unsigned long keepWarmStartTime = 0;
bool heatingElementOn = false;
uint8_t ledBrightness = 100;
const unsigned long KEEP_WARM_DURATION = 1800000;  // 30 minutes

// BLE objects
BLEServer* pServer = NULL;
BLECharacteristic* pStateChar = NULL;
BLECharacteristic* pTempChar = NULL;
BLECharacteristic* pCommandChar = NULL;
BLECharacteristic* pConvoChar = NULL;
bool deviceConnected = false;

// Thermistor constants for 10K NTC @ 25°C, B=3950
const float SERIES_RESISTOR = 10000.0;
const float THERMISTOR_NOMINAL = 10000.0;
const float TEMPERATURE_NOMINAL = 25.0;
const float B_COEFFICIENT = 3950.0;

float readTemperature() {
    int rawADC = analogRead(TEMP_SENSOR_PIN);
    
    if (rawADC == 0) return -999;
    
    float resistance = SERIES_RESISTOR / ((4095.0 / rawADC) - 1.0);
    
    // Steinhart-Hart equation
    float steinhart = resistance / THERMISTOR_NOMINAL;
    steinhart = log(steinhart);
    steinhart /= B_COEFFICIENT;
    steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;
    
    return steinhart;
}

void setHeatingElement(bool state) {
    static bool lastState = false;
    
    if (state != lastState) {
        heatingElementOn = state;
        digitalWrite(RELAY_PIN, state ? HIGH : LOW);
        Serial.printf("Heating element: %s\n", state ? "ON" : "OFF");
        lastState = state;
    } else {
        heatingElementOn = state;
        digitalWrite(RELAY_PIN, state ? HIGH : LOW);
    }
}

uint8_t getBrewProgress() {
    if (currentState != BREWING) return 0;
    
    unsigned long elapsed = millis() - brewStartTime;
    uint8_t progress = (elapsed * 100) / 480000;  // 8 minutes
    return min(progress, (uint8_t)100);
}

// Pulsing effect using simple sine approximation
uint8_t getPulseValue(uint8_t bpm, uint8_t minVal, uint8_t maxVal) {
    float beats = (millis() / 1000.0) * (bpm / 60.0);
    float sinVal = sin(beats * 2.0 * PI);
    uint8_t result = map(sinVal * 127 + 128, 0, 255, minVal, maxVal);
    return result;
}

void broadcastLedState() {
    if (!deviceConnected) return;
    
    uint8_t ledStates = 0;
    for (int i = 0; i < NUM_LEDS; i++) {
        RgbwColor color = strip.GetPixelColor(i);
        if (color.B > 50) { // LED is "on" (blue)
            ledStates |= (1 << i);
        }
    }
    pLedStateChar->setValue(&ledStates, 1);
    pLedStateChar->notify();
}

void updateLEDs() {
    // Check if app has timed out - take back control
    if (appControlsLeds && (millis() - lastAppLedUpdate > APP_LED_TIMEOUT)) {
        appControlsLeds = false;
        Serial.println(">>> FIRMWARE CONTROL: App timeout - firmware taking back LED control");
    }
    
    // If app controls LEDs, don't interfere
    if (appControlsLeds) {
        return;
    }
    
    // Firmware controls LEDs from here on...
    // Critical: Clear and latch FIRST
    strip.ClearTo(RgbwColor(0, 0, 0, 0));
    strip.Show();
    delayMicroseconds(500);
    
    // CONNECTING animation - sequential dots
    if (connState == CONNECTING) {
        static uint8_t connectPos = 0;
        static unsigned long lastConnectMove = 0;
        static uint8_t connectCycles = 0;
        
        if(millis() - lastConnectMove > 300) {
            connectPos = (connectPos + 1) % NUM_LEDS;
            lastConnectMove = millis();
            
            // After 2 full cycles (10 steps), switch to CONNECTED
            if (connectPos == 0) {
                connectCycles++;
                if (connectCycles >= 2) {
                    connState = CONNECTED;
                    connectCycles = 0;
                    Serial.println("Connection animation complete");
                }
            }
        }
        
        // Light only current position (electric blue)
        for(int i = 0; i < NUM_LEDS; i++) {
            if (i == connectPos) {
                strip.SetPixelColor(i, RgbwColor(0, 136, 255, 0)); // Bright blue
            }
        }
        
        strip.Show();
        broadcastLedState();
        return;
    }
    
    // CONNECTED - all bright for 2 seconds, then enter IDLE
    if (connState == CONNECTED) {
        static unsigned long connectedStartTime = 0;
        
        // Record start time on first entry
        if (connectedStartTime == 0) {
            connectedStartTime = millis();
            Serial.println("All LEDs bright - waiting 2 seconds");
        }
        
        if (millis() - connectedStartTime < 2000) {
            // All bright blue for 2 seconds
            for(int i = 0; i < NUM_LEDS; i++) {
                strip.SetPixelColor(i, RgbwColor(0, 136, 255, 0));
            }
        } else {
            // Switch to IDLE state
            Serial.println("Entering IDLE mode");
            currentState = IDLE;
            connState = DISCONNECTED;
            connectedStartTime = 0; // Reset for next connection
        }
        
        strip.Show();
        broadcastLedState();
        return;
    }

    // Normal state machine (IDLE, BREWING, etc)
    switch(currentState) {
        case IDLE:
            {
                uint8_t brightness = getPulseValue(20, 30, ledBrightness);
                for(int i = 0; i < NUM_LEDS; i++) {
                    strip.SetPixelColor(i, RgbwColor(0, brightness, brightness*2, 0)); // Blue
                }
            }
            break;
            
        case BREWING:
            {
                static uint8_t pos = 0;
                static unsigned long lastMove = 0;
                
                if(millis() - lastMove > 400) {
                    pos = (pos + 1) % NUM_LEDS;
                    lastMove = millis();
                }
                
                for(int i = 0; i < NUM_LEDS; i++) {
                    if(i == pos) {
                        strip.SetPixelColor(i, RgbwColor(0, ledBrightness/2, ledBrightness, 0));
                    } else if(i == ((pos + NUM_LEDS - 1) % NUM_LEDS)) {
                        strip.SetPixelColor(i, RgbwColor(0, ledBrightness/6, ledBrightness/3, 0));
                    } else {
                        strip.SetPixelColor(i, RgbwColor(0, 0, 0, 0));
                    }
                }
            }
            break;
            
        case KEEPING_WARM:
            {
                uint8_t brightness = getPulseValue(15, 50, 120);
                for(int i = 0; i < NUM_LEDS; i++) {
                    strip.SetPixelColor(i, RgbwColor(0, brightness/2, brightness, 0));
                }
            }
            break;
            
        case ERROR:
            {
                uint8_t brightness = (millis() % 400 < 200) ? 200 : 0;
                for(int i = 0; i < NUM_LEDS; i++) {
                    strip.SetPixelColor(i, RgbwColor(0, brightness/2, brightness, 0));
                }
            }
            break;
    }
    
    strip.Show();
    broadcastLedState();
}

const char* stateToString(BrewState state) {
    switch(state) {
        case IDLE: return "idle";
        case BREWING: return "brewing";
        case KEEPING_WARM: return "keeping_warm";
        case ERROR: return "error";
        default: return "unknown";
    }
}

void transitionToState(BrewState newState) {
    if (newState == currentState) return;
    
    Serial.printf("\nState transition: %s -> %s\n", 
                  stateToString(currentState), 
                  stateToString(newState));
    
    currentState = newState;
    stateStartTime = millis();
    
    if (newState == KEEPING_WARM) {
        keepWarmStartTime = millis();
    }
    
    if (pStateChar && deviceConnected) {
        pStateChar->setValue(stateToString(currentState));
        pStateChar->notify();
    }
}

void updateStateMachine() {
    switch(currentState) {
        case IDLE:
            setHeatingElement(false);
            break;
            
        case BREWING:
            setHeatingElement(true);
            
            heatingElementTemp = readTemperature();
            Serial.printf("\rTemp: %.1f°C  ", heatingElementTemp);
            
            // Check for empty reservoir (temp spike)
            if (heatingElementTemp > EMPTY_RESERVOIR_TEMP) {
                Serial.println("\nReservoir empty!");
                transitionToState(KEEPING_WARM);
                break;
            }
            
            // Backup timeout (8 min max)
            if (millis() - brewStartTime >= 480000) {
                Serial.println("\nBrew timer complete");
                transitionToState(KEEPING_WARM);
            }

            // Update temperature characteristic
            if (pTempChar && deviceConnected) {
                pTempChar->setValue(heatingElementTemp);
                pTempChar->notify();
            }
            break;
            
        case KEEPING_WARM:
            setHeatingElement(true);
            
            // Auto-shutoff after 30 minutes
            if (millis() - keepWarmStartTime >= KEEP_WARM_DURATION) {
                Serial.println("\nKeep-warm timeout, shutting off");
                transitionToState(IDLE);
            }
            break;
            
        case ERROR:
            setHeatingElement(false);
            break;
    }
    
    updateLEDs();
}

void handleCommand(const char* command) {
    String cmd = String(command);
    Serial.printf("Received command: %s\n", command);
    
    if (cmd == "start_brew") {
        if (currentState == IDLE) {
            brewStartTime = millis();
            transitionToState(BREWING);
        }
    } 
    else if (cmd == "stop_brew") {
        transitionToState(IDLE);
    }
    else if (cmd == "keep_warm_off") {
        if (currentState == KEEPING_WARM) {
            transitionToState(IDLE);
        }
    }
}

// BLE Callbacks
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        // Don't change connState here - let the connection stay in firmware control
        // App will take LED control explicitly when it wants to show connection animation
        Serial.println("Device connected");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        connState = DISCONNECTED;
        appControlsLeds = false; // Release app control on disconnect
        Serial.println("Device disconnected");
        BLEDevice::startAdvertising();
    }
};

class CommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    
        if (value.length() > 0) {
            String command = String(value.c_str());
            Serial.print("Received command: ");
            Serial.println(command);
            
            if (command.startsWith("start_brew")) {
                currentState = BREWING;
                Serial.println("Starting brew");
            }
            else if (command.startsWith("stop_brew")) {
                currentState = IDLE;
                Serial.println("Stopping brew");
            }
            else if (command.startsWith("set_name:")) {
                // Extract name after "set_name:"
                String newName = command.substring(9);
                Serial.print("Setting device name to: ");
                Serial.println(newName);
                
                // Save to EEPROM
                preferences.begin("soven", false);
                preferences.putString("ai_name", newName);
                preferences.end();
                
                Serial.println("Name saved! Restarting to apply...");
                delay(500);
                ESP.restart(); // Auto-restart to apply new name
            }
        }
    }
};

class LedStateCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        Serial.println(">>> LED STATE CALLBACK TRIGGERED");
        std::string value = pCharacteristic->getValue();
        
        if (value.length() > 0) {
            uint8_t ledByte = value[0];
            
            Serial.print(">>> Received LED byte: ");
            Serial.println(ledByte);
            
            // App takes control of LEDs
            appControlsLeds = true;
            lastAppLedUpdate = millis();
            Serial.println(">>> APP CONTROL: App now controls LEDs");

            // Update LED states from bits AND physically light them
            strip.ClearTo(RgbwColor(0, 0, 0, 0));
            
            for (int i = 0; i < NUM_LEDS; i++) {
                ledStates[i] = (ledByte & (1 << i)) != 0;
                
                if (ledStates[i]) {
                    strip.SetPixelColor(i, RgbwColor(0, 136, 255, 0));
                }
            }
            
            strip.Show();
            
            Serial.print(">>> LED state updated and displayed: ");
            for (int i = 0; i < NUM_LEDS; i++) {
                Serial.print(ledStates[i] ? "1" : "0");
            }
            Serial.println();
        }
    }
};

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Soven Coffee Maker starting...");
    
    //Factory Reset
    //preferences.begin("soven", false);
    //preferences.remove("ai_name");
    //preferences.end();
    //Serial.println(">>> FACTORY RESET: Name cleared");

    // Initialize pins
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    pinMode(TEMP_SENSOR_PIN, INPUT);
    
    // Initialize NeoPixelBus - simple startup
    strip.Begin();
    strip.Show();
    
    Serial.println("Hardware initialized");
    
    // Get unique serial from MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char serialStr[7];
    sprintf(serialStr, "%02X%02X%02X", mac[3], mac[4], mac[5]);
    deviceSerial = String(serialStr);
    
    // Check if device has been registered (has custom name)
    preferences.begin("soven", false);
    String aiName = preferences.getString("ai_name", "");
    preferences.end();
    
    // Broadcast format: "Soven-Coffee-SERIAL" or custom name if registered
    if (aiName.length() > 0) {
        broadcastName = aiName;
    } else {
        broadcastName = "Soven-Coffee-" + deviceSerial;
    }
    
    Serial.print("Broadcasting as: ");
    Serial.println(broadcastName);

    // Initialize BLE
    BLEDevice::init(broadcastName.c_str());
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    pStateChar = pService->createCharacteristic(
        STATE_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pStateChar->addDescriptor(new BLE2902());
    pStateChar->setValue(stateToString(currentState));
    
    pTempChar = pService->createCharacteristic(
        TEMP_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pTempChar->addDescriptor(new BLE2902());
       
    pCommandChar = pService->createCharacteristic(
        COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pCommandChar->setCallbacks(new CommandCallbacks());
    
    pConvoChar = pService->createCharacteristic(
        CONVO_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    pConvoChar->addDescriptor(new BLE2902());
    
    pLedStateChar = pService->createCharacteristic(
        LED_STATE_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_WRITE | 
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pLedStateChar->addDescriptor(new BLE2902());
    pLedStateChar->setCallbacks(new LedStateCallbacks());

    Serial.print("LED State characteristic created: ");
    Serial.println(LED_STATE_CHAR_UUID);

    pService->start();

    // ADD MANUFACTURER DATA with serial (NimBLE style)
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

    // Create manufacturer data
    std::string mfgData;
    mfgData += (char)0xFF;  // Company ID low byte
    mfgData += (char)0xFF;  // Company ID high byte
    mfgData += (char)mac[3]; // Serial byte 1
    mfgData += (char)mac[4]; // Serial byte 2
    mfgData += (char)mac[5]; // Serial byte 3

    // Get advertisement data object and set manufacturer data
    BLEAdvertisementData advData;
    advData.setManufacturerData(mfgData);
    pAdvertising->setAdvertisementData(advData);

    // Continue with existing advertising setup
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.print("Serial in manufacturer data: ");
    Serial.println(deviceSerial);
    Serial.println("BLE service started, advertising...");
    Serial.println("Ready to brew!");
    }

void loop() {
    updateStateMachine();
    delay(50);  // 20Hz update rate
}