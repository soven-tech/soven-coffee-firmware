// Soven HUB Firmware
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NeoPixelBus.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "SovenProtocol.h"
#include "DeviceRegistry.h"

Preferences preferences;
String deviceSerial = "";
String broadcastName = "";

// Pin definitions
#define RELAY_PIN 10
#define LED_DATA_PIN 4
#define TEMP_SENSOR_PIN 7
#define NUM_LEDS 3

// Temperature thresholds
#define EMPTY_RESERVOIR_TEMP 115.0
#define NORMAL_BREW_TEMP 100.0

// Global LED state
bool ledStates[3] = {false, false, false};
bool appControlsLeds = false;
unsigned long lastAppLedUpdate = 0;
const unsigned long APP_LED_TIMEOUT = 5000;

// Device registry for mesh
DeviceRegistry registry;
NimBLEScan* pBLEScan = nullptr;
unsigned long lastDeviceScan = 0;
const unsigned long DEVICE_SCAN_INTERVAL = 10000;
bool isScanning = false;

// NeoPixelBus
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

BrewState currentState = IDLE;
ConnectionState connState = DISCONNECTED;
float heatingElementTemp = 25.0;
unsigned long brewStartTime = 0;
unsigned long stateStartTime = 0;
unsigned long keepWarmStartTime = 0;
bool heatingElementOn = false;
uint8_t ledBrightness = 100;
const unsigned long KEEP_WARM_DURATION = 1800000;

// NimBLE objects
NimBLEServer* pServer = NULL;
NimBLECharacteristic* pCommandChar = NULL;
NimBLECharacteristic* pStatusChar = NULL;
NimBLECharacteristic* pBrewStateChar = NULL;
NimBLECharacteristic* pTempChar = NULL;
NimBLECharacteristic* pLedControlChar = NULL;
NimBLECharacteristic* pConvoChar = NULL;
bool deviceConnected = false;

// Thermistor constants
const float SERIES_RESISTOR = 10000.0;
const float THERMISTOR_NOMINAL = 10000.0;
const float TEMPERATURE_NOMINAL = 25.0;
const float B_COEFFICIENT = 3950.0;

// ============================================================================
// TEMPERATURE SENSING
// ============================================================================

float readTemperature() {
    int rawADC = analogRead(TEMP_SENSOR_PIN);
    if (rawADC == 0) return -999;
    
    float resistance = SERIES_RESISTOR / ((4095.0 / rawADC) - 1.0);
    float steinhart = resistance / THERMISTOR_NOMINAL;
    steinhart = log(steinhart);
    steinhart /= B_COEFFICIENT;
    steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;
    
    return steinhart;
}

// ============================================================================
// RELAY CONTROL
// ============================================================================

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

// ============================================================================
// LED CONTROL
// ============================================================================

uint8_t getPulseValue(uint8_t bpm, uint8_t minVal, uint8_t maxVal) {
    float beats = (millis() / 1000.0) * (bpm / 60.0);
    float sinVal = sin(beats * 2.0 * PI);
    uint8_t result = map(sinVal * 127 + 128, 0, 255, minVal, maxVal);
    return result;
}

void broadcastLedState() {
    if (!deviceConnected || !pLedControlChar) return;
    
    uint8_t ledStateByte = 0;
    for (int i = 0; i < NUM_LEDS; i++) {
        RgbwColor color = strip.GetPixelColor(i);
        if (color.B > 50) {
            ledStateByte |= (1 << i);
        }
    }
    pLedControlChar->setValue(&ledStateByte, 1);
    pLedControlChar->notify();
}

void updateLEDs() {
    if (appControlsLeds && (millis() - lastAppLedUpdate > APP_LED_TIMEOUT)) {
        appControlsLeds = false;
        Serial.println(">>> Firmware taking back LED control");
    }
    
    if (appControlsLeds) return;
    
    strip.ClearTo(RgbwColor(0, 0, 0, 0));
    strip.Show();
    delayMicroseconds(500);
    
    if (connState == CONNECTING) {
        static uint8_t connectPos = 0;
        static unsigned long lastConnectMove = 0;
        static uint8_t connectCycles = 0;
        
        if(millis() - lastConnectMove > 300) {
            connectPos = (connectPos + 1) % NUM_LEDS;
            lastConnectMove = millis();
            
            if (connectPos == 0) {
                connectCycles++;
                if (connectCycles >= 2) {
                    connState = CONNECTED;
                    connectCycles = 0;
                }
            }
        }
        
        for(int i = 0; i < NUM_LEDS; i++) {
            if (i == connectPos) {
                strip.SetPixelColor(i, RgbwColor(0, 136, 255, 0));
            }
        }
        strip.Show();
        broadcastLedState();
        return;
    }
    
    if (connState == CONNECTED) {
        static unsigned long connectedStartTime = 0;
        
        if (connectedStartTime == 0) {
            connectedStartTime = millis();
        }
        
        if (millis() - connectedStartTime < 2000) {
            for(int i = 0; i < NUM_LEDS; i++) {
                strip.SetPixelColor(i, RgbwColor(0, 136, 255, 0));
            }
        } else {
            currentState = IDLE;
            connState = DISCONNECTED;
            connectedStartTime = 0;
        }
        
        strip.Show();
        broadcastLedState();
        return;
    }

    switch(currentState) {
        case IDLE:
            {
                uint8_t brightness = getPulseValue(20, 30, ledBrightness);
                for(int i = 0; i < NUM_LEDS; i++) {
                    strip.SetPixelColor(i, RgbwColor(0, brightness, brightness*2, 0));
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

// ============================================================================
// STATE MACHINE
// ============================================================================

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
    
    Serial.printf("\nState: %s -> %s\n", stateToString(currentState), stateToString(newState));
    
    currentState = newState;
    stateStartTime = millis();
    
    if (newState == KEEPING_WARM) {
        keepWarmStartTime = millis();
    }
    
    if (pBrewStateChar && deviceConnected) {
        pBrewStateChar->setValue(stateToString(currentState));
        pBrewStateChar->notify();
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
            
            if (heatingElementTemp > EMPTY_RESERVOIR_TEMP) {
                Serial.println("\nReservoir empty!");
                transitionToState(KEEPING_WARM);
                break;
            }
            
            if (millis() - brewStartTime >= 480000) {
                Serial.println("\nBrew complete");
                transitionToState(KEEPING_WARM);
            }

            if (pTempChar && deviceConnected) {
                pTempChar->setValue(heatingElementTemp);
                pTempChar->notify();
            }
            break;
            
        case KEEPING_WARM:
            setHeatingElement(true);
            
            if (millis() - keepWarmStartTime >= KEEP_WARM_DURATION) {
                Serial.println("\nKeep-warm timeout");
                transitionToState(IDLE);
            }
            break;
            
        case ERROR:
            setHeatingElement(false);
            break;
    }
    
    updateLEDs();
}

// ============================================================================
// DEVICE CONTROL (Mesh)
// ============================================================================

bool turnOnDevice(String deviceName, String duration = "") {
    return registry.sendCommand(deviceName, "power_on", duration);
}

bool turnOffDevice(String deviceName) {
    return registry.sendCommand(deviceName, "power_off", "");
}

String getDeviceBattery(String deviceName) {
    String status = registry.getDeviceStatus(deviceName);
    StaticJsonDocument<768> doc;
    deserializeJson(doc, status);
    
    if (doc.containsKey("error")) return "unknown";
    return String((int)doc["battery"]["percentage"]);
}

void listDevices() {
    registry.printRegistry();
}

// ============================================================================
// NIMBLE CALLBACKS
// ============================================================================

class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Device connected");
    }

    void onDisconnect(NimBLEServer* pServer) {
        deviceConnected = false;
        connState = DISCONNECTED;
        appControlsLeds = false;
        Serial.println("Device disconnected");
        NimBLEDevice::startAdvertising();
    }
};

class CommandCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() == 0) return;
        
        String command = String(value.c_str());
        Serial.println("Command: " + command);
        
        if (command == "start_brew") {
            brewStartTime = millis();
            transitionToState(BREWING);
        }
        else if (command == "stop_brew") {
            transitionToState(IDLE);
        }
        else if (command == "keep_warm_off") {
            if (currentState == KEEPING_WARM) {
                transitionToState(IDLE);
            }
        }
        else if (command.startsWith("set_name:")) {
            String newName = command.substring(9);
            preferences.begin("soven", false);
            preferences.putString("ai_name", newName);
            preferences.end();
            Serial.println("Name saved, restarting...");
            delay(500);
            ESP.restart();
        }
        else if (command.startsWith("froth_on")) {
            String duration = "30";
            if (command.indexOf(':') > 0) {
                duration = command.substring(command.indexOf(':') + 1);
            }
            if (turnOnDevice("frother", duration)) {
                if (pStatusChar) {
                    pStatusChar->setValue("Frothing milk");
                    pStatusChar->notify();
                }
            }
        }
        else if (command == "froth_off") {
            turnOffDevice("frother");
        }
        else if (command == "froth_battery") {
            String battery = getDeviceBattery("frother");
            if (pStatusChar) {
                String msg = "Battery: " + battery + "%";
                pStatusChar->setValue(msg.c_str());
                pStatusChar->notify();
            }
        }
        else if (command == "list_devices") {
            listDevices();
        }
    }
};

class LedControlCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() == 0) return;
        
        uint8_t ledByte = value[0];
        appControlsLeds = true;
        lastAppLedUpdate = millis();
        
        strip.ClearTo(RgbwColor(0, 0, 0, 0));
        for (int i = 0; i < NUM_LEDS; i++) {
            if (ledByte & (1 << i)) {
                strip.SetPixelColor(i, RgbwColor(0, 136, 255, 0));
            }
        }
        strip.Show();
    }
};

// ============================================================================
// DEVICE SCANNING
// ============================================================================

class SovenDeviceScanCallbacks: public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        // Debug: Print every device we see
        Serial.print("📡 Saw: ");
        Serial.print(advertisedDevice->getName().c_str());
        Serial.print(" | ");
        Serial.println(advertisedDevice->getAddress().toString().c_str());
        
        if (advertisedDevice->haveServiceUUID()) {
            Serial.print("   Services: ");
            for (int i = 0; i < advertisedDevice->getServiceUUIDCount(); i++) {
                Serial.print(advertisedDevice->getServiceUUID(i).toString().c_str());
                Serial.print(" ");
            }
            Serial.println();
            
            if (advertisedDevice->isAdvertisingService(NimBLEUUID(Soven::SERVICE_UUID))) {
                Serial.println("   ✓ MATCH! Adding to registry");
                registry.addDevice(advertisedDevice);
            } else {
                Serial.println("   ✗ Not a Soven device");
            }
        } else {
            Serial.println("   (No service UUIDs)");
        }
    }
};

// ============================================================================
// SERIAL COMMANDS
// ============================================================================

void handleSerialCommands() {
    if (!Serial.available()) return;
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;
    
    if (input == "scan") {
        Serial.println("📡 Scanning...");
        pBLEScan->start(5, false);
    }
    else if (input == "devices") {
        listDevices();
    }
    else if (input == "froth on") {
        turnOnDevice("frother", "30");
    }
    else if (input.startsWith("froth on ")) {
        turnOnDevice("frother", input.substring(9));
    }
    else if (input == "froth off") {
        turnOffDevice("frother");
    }
    else if (input == "froth battery") {
        Serial.println("Battery: " + getDeviceBattery("frother") + "%");
    }
    else if (input == "brew") {
        brewStartTime = millis();
        transitionToState(BREWING);
    }
    else if (input == "stop") {
        transitionToState(IDLE);
    }
    else if (input == "help") {
        Serial.println("\n=== Commands ===");
        Serial.println("brew / stop");
        Serial.println("scan / devices");
        Serial.println("froth on [duration]");
        Serial.println("froth off / froth battery");
        Serial.println("===============\n");
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n╔════════════════════════════════════╗");
    Serial.println("║  SOVEN HUB - ALL NIMBLE v2.0      ║");
    Serial.println("╚════════════════════════════════════╝\n");
    
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    pinMode(TEMP_SENSOR_PIN, INPUT);
    
    strip.Begin();
    strip.Show();
    
    // Get device serial
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char serialStr[7];
    sprintf(serialStr, "%02X%02X%02X", mac[3], mac[4], mac[5]);
    deviceSerial = String(serialStr);
    
    // Check for custom name
    preferences.begin("soven", false);
    String aiName = preferences.getString("ai_name", "");
    preferences.end();
    
    broadcastName = (aiName.length() > 0) ? aiName : "Soven-Coffee-" + deviceSerial;
    Serial.println("Name: " + broadcastName);

    // Initialize NimBLE
    NimBLEDevice::init(broadcastName.c_str());
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    
    // Create server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    // Create service
    NimBLEService *pService = pServer->createService(Soven::SERVICE_UUID);
    
    // Universal characteristics
    pCommandChar = pService->createCharacteristic(
        Soven::COMMAND_UUID,
        NIMBLE_PROPERTY::WRITE
    );
    pCommandChar->setCallbacks(new CommandCallbacks());
    
    pStatusChar = pService->createCharacteristic(
        Soven::STATUS_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    
    // Hub-specific characteristics
    pBrewStateChar = pService->createCharacteristic(
        Soven::BREW_STATE_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pBrewStateChar->setValue(stateToString(currentState));
    
    pTempChar = pService->createCharacteristic(
        Soven::TEMPERATURE_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    
    pLedControlChar = pService->createCharacteristic(
        Soven::LED_CONTROL_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    pLedControlChar->setCallbacks(new LedControlCallbacks());
    
    pConvoChar = pService->createCharacteristic(
        Soven::CONVERSATION_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    
    pService->start();

    // Start advertising
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(Soven::SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();
    
    Serial.println("✓ BLE server started");

    // Initialize scanner
    Serial.println("🔍 Initializing scanner...");
    pBLEScan = NimBLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new SovenDeviceScanCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    
    Serial.println("📡 Scanning for devices...");
    pBLEScan->start(5, false);
    
    Serial.println("\n✓ Ready\n");
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    handleSerialCommands();
    
    if (!isScanning && (millis() - lastDeviceScan >= DEVICE_SCAN_INTERVAL)) {
        isScanning = true;
        pBLEScan->start(3, false);
        registry.removeStaleDevices();
        lastDeviceScan = millis();
    }
    
    updateStateMachine();
    delay(50);
}