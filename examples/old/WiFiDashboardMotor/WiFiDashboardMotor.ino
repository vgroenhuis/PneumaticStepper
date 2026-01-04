/*
 * Pneumatic stepper control (without acceleration control) using WiFi dashboard
 */
#include "RoamingWiFiManager.h"
#include "PneumaticStepper.h"
#include "MotorDashboard.html"
#include <ArduinoJson.h>

#ifndef WIFI_SSID
#define WIFI_SSID "your-ssid"
#define WIFI_PASSWORD "your-password"
#endif

String ssid = WIFI_SSID;
String wifiPassword = WIFI_PASSWORD;

// The ESP32-C5-WIFI6-KIT-N16R8 (2025 version) has the red and green channels reversed
#ifdef RED_GREEN_REVERSED
#define LED2(R, G, B) rgbLedWrite(RGB_BUILTIN, G, R, B)
#else
#define LED2(R, G, B) rgbLedWrite(RGB_BUILTIN, R, G, B)
#endif


RoamingWiFiManager wiFiManager;
PneumaticStepper motor = PneumaticStepper::makeTwoCylinderStepper();
AsyncWebSocket ws("/ws");


void handleWebSocketCommand(String command) {
    // Parse JSON command
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, command);
    
    if (error) {
        Serial.println("JSON parse error");
        return;
    }

    String action = doc["action"] | "";

    if (action == "setSetpoint") {
        long setpoint = doc["value"];
        motor.setSetpoint(setpoint);
        Serial.printf("Setpoint changed to: %ld\n", setpoint);
    }
    else if (action == "addSteps") {
        long stepDelta = doc["value"];
        long newSetpoint = motor.getSetpoint() + stepDelta;
        motor.setSetpoint(newSetpoint);
        Serial.printf("Setpoint adjusted by %ld steps to: %ld\n", stepDelta, newSetpoint);
    }
    else if (action == "stop") {
        motor.setSetpoint(motor.getRoundedPosition());
    }
    else if (action == "home") {
        motor.setSetpoint(0);
    }
    else if (action == "setFrequency") {
        float maxVelocity = doc["value"];
        motor.setMaxVelocity(maxVelocity);
        Serial.printf("Max velocity changed to: %.1f Hz\n", maxVelocity);
    }
}

void sendMotorStatus() {
    if (ws.count() == 0) return;

    JsonDocument doc;
    doc["timestamp"] = millis();
    doc["position"] = motor.getRoundedPosition();
    doc["setpoint"] = motor.getSetpoint();
    doc["phase"] = motor.getPhaseNr();
    doc["running"] = motor.isRunning();
    doc["positionValid"] = motor.isPositionValid();
    
    // Add cylinder states
    JsonArray cylinders = doc["cylinders"].to<JsonArray>();
    for (int i = 0; i < motor.getCylinderCount(); i++) {
        cylinders.add(motor.getCylinderState(i));
    }

    String json;
    serializeJson(doc, json);
    ws.textAll(json);
}

// WebSocket event handler
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, 
               void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WebSocket client #%u connected\n", client->id());
    } 
    else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
    }
    else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            String cmd = (char*)data;
            handleWebSocketCommand(cmd);
        }
    }
}

void handleRoot(AsyncWebServerRequest *request) {
    request->send(200, "text/html", MOTOR_DASHBOARD_HTML);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting WiFi Dashboard Motor example");
    wiFiManager.init({{ssid, wifiPassword}});
    wiFiManager.setUseLEDIndicator(false);
    
    wiFiManager.server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        handleRoot(request);
    });

    // Setup WebSocket
    ws.onEvent(onWsEvent);
    wiFiManager.server.addHandler(&ws);

    pinMode(2,OUTPUT); // Use digital pins 2 and 3 for controlling the valves
    pinMode(3,OUTPUT);  
}

void checkSendMotorStatus() {
    // Send status update to all connected WebSocket clients
    // - Send when position changed and last send was over 50 ms ago
    // - Otherwise send every 200 ms (five times per second)
    static unsigned long lastSend = 0;
    static long lastPosition = 0;
    long currentPosition = motor.getRoundedPosition();
    bool positionChanged = (currentPosition != lastPosition);
    unsigned long timeSinceLastSend = millis() - lastSend;
    
    bool shouldSend = false;
    if (positionChanged && timeSinceLastSend >= 50) {
        shouldSend = true;
    } else if (!positionChanged && timeSinceLastSend >= 200) {
        shouldSend = true;
    }
    
    if (shouldSend) {
        sendMotorStatus();
        lastSend = millis();
        lastPosition = currentPosition;
    }
}

void loop() {
    wiFiManager.loop(); // Handle WiFi manager tasks
    motor.work(); // This function takes care of proper motor control and cylinder states

    digitalWrite(2,motor.getCylinderState(0)); // Copy motor's cylinder state to output pin
    digitalWrite(3,motor.getCylinderState(1));

    // LED indication: red = cylinder A, green = off, blue = cylinder B
    LED2(motor.getCylinderState(0) ? 40 : 0, 0, motor.getCylinderState(1) ? 40 : 0);

    checkSendMotorStatus();
}
