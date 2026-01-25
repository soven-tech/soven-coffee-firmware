#ifndef DEVICE_REGISTRY_H
#define DEVICE_REGISTRY_H

#include <NimBLEDevice.h>
#include <vector>
#include <ArduinoJson.h>
#include "SovenProtocol.h"

struct SovenDevice {
  String deviceId;
  String deviceType;
  String loadType;
  String friendlyName;
  NimBLEAddress address;
  NimBLEClient* client;
  bool connected;
  unsigned long lastSeen;
  
  float batteryVoltage;
  int batteryPercentage;
  bool actuatorEnabled;
};

class DeviceRegistry {
private:
  std::vector<SovenDevice> devices;
  const unsigned long STALE_TIMEOUT = 60000;
  
public:
  
  void addDevice(NimBLEAdvertisedDevice* advertisedDevice) {
    NimBLEAddress addr = advertisedDevice->getAddress();
    for (auto& device : devices) {
      if (device.address == addr) {
        device.lastSeen = millis();
        return;
      }
    }
    
    SovenDevice newDevice;
    newDevice.deviceId = advertisedDevice->getName().c_str();
    newDevice.address = addr;
    newDevice.connected = false;
    newDevice.lastSeen = millis();
    newDevice.client = nullptr;
    
    String id = newDevice.deviceId;
    int underscorePos = id.indexOf('_');
    if (underscorePos > 0) {
      newDevice.loadType = id.substring(0, underscorePos);
      newDevice.friendlyName = newDevice.loadType;
      newDevice.friendlyName[0] = toupper(newDevice.friendlyName[0]);
    } else {
      newDevice.loadType = "unknown";
      newDevice.friendlyName = "Unknown Device";
    }
    
    newDevice.deviceType = "battery_actuator";
    devices.push_back(newDevice);
    Serial.println("📍 " + newDevice.friendlyName + " (" + newDevice.deviceId + ")");
  }
  
  SovenDevice* findDevice(String identifier) {
    identifier.toLowerCase();
    for (auto& device : devices) {
      String loadTypeLower = device.loadType;
      loadTypeLower.toLowerCase();
      if (loadTypeLower == identifier || device.deviceId.equalsIgnoreCase(identifier)) {
        return &device;
      }
    }
    return nullptr;
  }
  
  bool connectToDevice(SovenDevice* device) {
    if (device->connected && device->client != nullptr) return true;
    
    Serial.println("🔗 " + device->friendlyName + "...");
    device->client = NimBLEDevice::createClient();
    
    if (!device->client->connect(device->address)) {
      Serial.println("❌ Failed");
      return false;
    }
    
    NimBLERemoteService* pService = device->client->getService(Soven::SERVICE_UUID);
    if (pService == nullptr) {
      Serial.println("❌ No service");
      device->client->disconnect();
      return false;
    }
    
    device->connected = true;
    Serial.println("✓ Connected");
    return true;
  }
  
  bool sendCommand(String deviceIdentifier, String action, String params) {
      SovenDevice* device = findDevice(deviceIdentifier);
      if (device == nullptr) {
        Serial.println("❌ Not found: " + deviceIdentifier);
        return false;
      }
      
      if (!connectToDevice(device)) return false;
      
      Serial.println("DEBUG: Getting service...");  // ADD
      NimBLERemoteService* pService = device->client->getService(Soven::SERVICE_UUID);
      if (pService == nullptr) {
        Serial.println("❌ Service not found");  // ADD
        return false;
      }
      
      Serial.println("DEBUG: Getting command characteristic...");  // ADD
      NimBLERemoteCharacteristic* pCommandChar = pService->getCharacteristic(Soven::COMMAND_UUID);
      
      if (pCommandChar == nullptr) {
        Serial.println("❌ No command char");
        return false;
      }
      
      Serial.println("DEBUG: Command char found");  // ADD
      Serial.print("DEBUG: Can write? ");  // ADD
      Serial.println(pCommandChar->canWrite() ? "YES" : "NO");  // ADD
      
      StaticJsonDocument<256> cmd;
      cmd["action"] = action;
      cmd["params"] = params;
      cmd["source"] = "hub";
      cmd["timestamp"] = millis();
      
      String commandJSON;
      serializeJson(cmd, commandJSON);
      
      Serial.println("DEBUG: Writing: " + commandJSON);  // ADD
      
      bool writeSuccess = pCommandChar->writeValue(commandJSON.c_str(), commandJSON.length());
      
      Serial.print("DEBUG: Write result: ");  // ADD
      Serial.println(writeSuccess ? "SUCCESS" : "FAILED");  // ADD
      
      Serial.println("📤 " + device->friendlyName + ": " + action);
      
      return true;
  }
  
  String getDeviceStatus(String deviceIdentifier) {
    SovenDevice* device = findDevice(deviceIdentifier);
    if (device == nullptr) return "{\"error\":\"Not found\"}";
    if (!connectToDevice(device)) return "{\"error\":\"Connection failed\"}";
    
    NimBLERemoteService* pService = device->client->getService(Soven::SERVICE_UUID);
    NimBLERemoteCharacteristic* pTelemetryChar = pService->getCharacteristic(Soven::TELEMETRY_UUID);
    
    if (pTelemetryChar == nullptr) return "{\"error\":\"No telemetry\"}";
    
    String telemetry = pTelemetryChar->readValue().c_str();
    
    StaticJsonDocument<768> doc;
    deserializeJson(doc, telemetry);
    device->batteryVoltage = doc["battery"]["voltage"];
    device->batteryPercentage = doc["battery"]["percentage"];
    device->actuatorEnabled = doc["operation"]["enabled"];
    
    return telemetry;
  }
  
  void removeStaleDevices() {
    devices.erase(
      std::remove_if(devices.begin(), devices.end(),
        [this](const SovenDevice& d) { return (millis() - d.lastSeen) > STALE_TIMEOUT; }),
      devices.end()
    );
  }
  
  void printRegistry() {
    Serial.println("\n╔══════════════════════════════════╗");
    Serial.println("║     DEVICE REGISTRY              ║");
    Serial.println("╚══════════════════════════════════╝");
    
    if (devices.empty()) {
      Serial.println("  (No devices)");
    } else {
      for (auto& device : devices) {
        Serial.printf("  %s (%s)\n", device.friendlyName.c_str(), device.deviceId.c_str());
        Serial.printf("    %s | %s\n", 
          device.connected ? "Connected" : "Discovered",
          device.address.toString().c_str());
        if (device.connected) {
          Serial.printf("    Battery: %d%% | %s\n", 
            device.batteryPercentage,
            device.actuatorEnabled ? "ON" : "OFF");
        }
      }
    }
    Serial.println("══════════════════════════════════\n");
  }
  
  std::vector<SovenDevice>& getAllDevices() { return devices; }
  int getDeviceCount() { return devices.size(); }
};

#endif