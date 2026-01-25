#ifndef SOVEN_PROTOCOL_H
#define SOVEN_PROTOCOL_H

// Soven Protocol v2.0 - Universal BLE Protocol
// Used by ALL Soven devices (Tier 1 Hubs and Tier 2 Devices)

namespace Soven {
    // Base service (all Soven devices advertise this)
    const char* SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb";
    
    // Universal characteristics (all devices)
    const char* COMMAND_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb";      // Write: Send commands
    const char* STATUS_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb";       // Read/Notify: Get status
    const char* TELEMETRY_UUID = "0000ffe3-0000-1000-8000-00805f9b34fb";    // Notify: Stream metrics
    
    // Hub-specific characteristics (coffee maker only)
    const char* BREW_STATE_UUID = "0000ffe4-0000-1000-8000-00805f9b34fb";   // Notify: Brew state changes
    const char* TEMPERATURE_UUID = "0000ffe5-0000-1000-8000-00805f9b34fb";  // Notify: Temperature updates
    const char* LED_CONTROL_UUID = "0000ffe6-0000-1000-8000-00805f9b34fb";  // Write: LED control
    const char* CONVERSATION_UUID = "0000ffe7-0000-1000-8000-00805f9b34fb"; // Write/Notify: Voice/text
}

#endif // SOVEN_PROTOCOL_H