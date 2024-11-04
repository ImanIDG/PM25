#define SENSENET_DEBUG // enable debug on SerialMon
#define SerialMon Serial // if you need DEBUG SerialMon should be defined

#define FIRMWARE_TITLE "Arduino_Data_Collector"
#define FIRMWARE_VERSION "0.3.14"

#include <Arduino.h>
#include "sensenet.h"
#include "WiFi.h"
#include "Wire.h"
#include "SPI.h"
//#include "Preferences.h"
#include "Ticker.h"
#include <esp_task_wdt.h>
#include <ESP32Time.h>

#include "DEV_Config.h"

#define WIFI_SSID "Sensenet_2.4G"
#define WIFI_PASS "Sensenet123"
#define TB_URL "tb.sensenet.ca"

// for COM7
//#define TOKEN "SGP4xESP32_3"

// for COM8
//#define TOKEN "SGP4xESP32_2"

// for COM10
#define TOKEN "GPSESP32"

#include <SPI.h>
#include "Adafruit_PM25AQI.h"

// If your PM2.5 is UART only, for UNO and others (without hardware serial) 
// we must use software serial...
// pin #2 is IN from sensor (TX pin on sensor), leave pin #3 disconnected
// comment these two lines if using hardware serial
#include <SoftwareSerial.h>
#include "cozir.h"

ESP32Time internalRtc(0);  // offset in seconds GMT
Ticker restartTicker;
//Preferences preferences;
NetworkInterface wifiInterface("wifi", 2, 2);
NetworkInterfacesController networkController;
MQTTController mqttController;
MQTTOTA ota(&mqttController, 5120);

WiFiClient wiFiClient;

int i = 0;

void resetESP() {
    ESP.restart();
}

uint64_t getTimestamp() {
    if (internalRtc.getEpoch() < 946713600)
        return 0;
    uint64_t ts = internalRtc.getEpoch();
    ts = ts * 1000L;
    ts = ts + internalRtc.getMillis();
    return ts;
}

// Sampling interval in seconds
char errorMessage[32];

bool on_message(const String &topic, DynamicJsonDocument json) {
    Serial.print("Topic1: ");
    Serial.println(topic);
    Serial.print("Message1: ");
    Serial.println(json.as<String>());

    if (json.containsKey("shared")) {
        JsonObject sharedKeys = json["shared"].as<JsonObject>();
        for (JsonPair kv: sharedKeys)
            json[kv.key()] = sharedKeys[kv.key()];

    }

    if (json.containsKey("method")) {
        String method = json["method"].as<String>();

        bool handled = false;
        if (method.equalsIgnoreCase("restart_device")) {
            float seconds = 0;
            if (json["params"].containsKey("seconds"))
                seconds = json["params"]["seconds"];
            if (seconds == 0) seconds = 1;
            printDBGln("Device Will Restart in " + String(seconds) + " Seconds");
            restartTicker.once(seconds, resetESP);
            handled = true;
        }

        if (handled) {
            String responseTopic = String(topic);
            responseTopic.replace("request", "response");
            DynamicJsonDocument responsePayload(300);
            responsePayload["result"] = "true";
            mqttController.addToPublishQueue(responseTopic, responsePayload.as<String>(), true);
            return true;
        }
    }

    return false;
}

void connectToNetwork() {
    Serial.println("Added WiFi Interface");
    networkController.addNetworkInterface(&wifiInterface);

    networkController.setAutoReconnect(true, 10000);
    networkController.autoConnectToNetwork();
}

void connectToPlatform(Client &client, const bool enableOTA) {

    Serial.println("Trying to Connect Platform");
    mqttController.connect(client, "esp", TOKEN, "", TB_URL,
                           1883, on_message,
                           nullptr, [&]() {
                Serial.println("Connected To Platform");
                DynamicJsonDocument info(512);
                info["Token"] = TOKEN;
                info.shrinkToFit();
                mqttController.sendAttributes(info, true);
                if (enableOTA)
                    ota.begin(FIRMWARE_TITLE, FIRMWARE_VERSION);
                else
                    ota.stopHandleOTAMessages();

                DynamicJsonDocument requestKeys(512);
                requestKeys["sharedKeys"] = "desiredAllowSleep,desiredDisableIR,desiredSEN55TempOffset";
                requestKeys.shrinkToFit();
                mqttController.requestAttributesJson(requestKeys.as<String>());

                if (getTimestamp() == 0) {
                    DynamicJsonDocument requestTime(512);
                    requestTime["method"] = "requestTimestamp";
                    requestTime.shrinkToFit();
                    mqttController.requestRPC(requestTime.as<String>(),
                                              [](const String &rpcTopic, const DynamicJsonDocument &rpcJson) -> bool {
                                                  Serial.print("Updating Internal RTC to: ");
                                                  Serial.println(rpcJson.as<String>());
                                                  uint64_t tsFromCloud = rpcJson["timestamp"].as<uint64_t>();
                                                  tsFromCloud = tsFromCloud / 1000;
                                                  internalRtc.setTime(tsFromCloud);
                                                  Serial.print("Internal RTC updated to: ");
                                                  Serial.println(internalRtc.getDateTime(true));
                                                  getTimestamp();
                                                  return true;
                                              });
                } else {
                    Serial.print("Internal RTC updated to: ");
                    Serial.println(internalRtc.getDateTime(true));
                }
            });
}

int retry = 0;

void initInterfaces() {
    retry = 0;
    wifiInterface.setTimeoutMs(30000);
    wifiInterface.setConnectInterface([]() -> bool {
        Serial.println(String("Connecting To WiFi ") + WIFI_SSID);
        WiFi.mode(WIFI_MODE_NULL);
        delay(2000);
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        return true;
    });
    wifiInterface.setConnectionCheckInterfaceInterface([]() -> bool {
        return WiFi.status() == WL_CONNECTED;
    });
    wifiInterface.OnConnectingEvent([]() {
        Serial.print(".");
    }, 500);
    wifiInterface.OnConnectedEvent([]() {
        retry = 0;
        Serial.println(String("Connected to WIFI with IP: ") + WiFi.localIP().toString());
        connectToPlatform(wiFiClient, true);
        DynamicJsonDocument data(200);
        data["Connection Type"] = "WIFI";
        data["IP"] = WiFi.localIP().toString();
        data.shrinkToFit();
    });
    wifiInterface.OnTimeoutEvent([]() {
        retry++;
        Serial.println("WiFi Connecting Timeout! retrying for " + String(retry) + " Times");
        WiFi.mode(WIFI_MODE_NULL);

//        if (retry >= 20)
//            ESP.restart();
    });
}

SoftwareSerial pmSerial(2, 3);
COZIR czr(&Serial1);

Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
void setupPM25_Colzir();
void loopPM25_Colzir(DynamicJsonDocument &data) {
  PM25_AQI_Data dataT;
  
  if (! aqi.read(&dataT)) {
    Serial.println("Could not read from AQI");
    delay(500);  // try again in a bit!
    return;
  }
  Serial.println("AQI reading success");

  Serial.println();
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Concentration Units (standard)"));
  Serial.println(F("---------------------------------------"));
  uint16_t pm10_standard = dataT.pm10_standard;
  Serial.print(F("PM 1.0: ")); Serial.print(pm10_standard);
  data["pm10_standard"] = String(pm10_standard);
  
  uint16_t pm25_standard = dataT.pm25_standard;
  Serial.print(F("\t\tPM 2.5: ")); Serial.print(pm25_standard);
  data["pm25_standard"] = String(pm25_standard);
  
  uint16_t pm100_standard = dataT.pm100_standard;
  Serial.print(F("\t\tPM 10: ")); Serial.println(pm100_standard);
  data["pm100_standard"] = String(pm100_standard);
  Serial.println(F("Concentration Units (environmental)"));
  Serial.println(F("---------------------------------------"));
  uint16_t pm10_env = dataT.pm10_env;
  Serial.print(F("PM 1.0: ")); Serial.print(pm10_env);
  data["pm10_env"] = String(pm10_env);
  Serial.print(F("\t\tPM 2.5: ")); Serial.print(dataT.pm25_env);
  data["pm25_env"] = String(dataT.pm25_env);
  Serial.print(F("\t\tPM 10: ")); Serial.println(dataT.pm100_env);
  data["pm100_env"] = String(dataT.pm100_env);
  Serial.println(F("---------------------------------------"));
  Serial.print(F("Particles > 0.3um / 0.1L air:")); Serial.println(dataT.particles_03um);
  data["particles_03um"] = String(dataT.particles_03um);
  Serial.print(F("Particles > 0.5um / 0.1L air:")); Serial.println(dataT.particles_05um);
  data["particles_05um"] = String(dataT.particles_05um);
  Serial.print(F("Particles > 1.0um / 0.1L air:")); Serial.println(dataT.particles_10um);
  data["particles_10um"] = String(dataT.particles_10um);
  Serial.print(F("Particles > 2.5um / 0.1L air:")); Serial.println(dataT.particles_25um);
  data["particles_25um"] = String(dataT.particles_25um);
  Serial.print(F("Particles > 5.0um / 0.1L air:")); Serial.println(dataT.particles_50um);
  data["particles_50um"] = String(dataT.particles_50um);
  Serial.print(F("Particles > 10 um / 0.1L air:")); Serial.println(dataT.particles_100um);
  data["particles_100um"] = String(dataT.particles_100um);
  Serial.println(F("---------------------------------------"));
  
  uint32_t c = czr.CO2();
  c *= czr.getPPMFactor();  //  most of time PPM = one.
  Serial.print("CO2 =\t");
  Serial.println(c);
  data["czrCO2"] = String(c);
}

uint64_t lastPM25_Colzir = 0;

void core0Loop(void *parameter) {
    //Dont do anything 1
    esp_task_wdt_init(600, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
    //Dont do anything1

    lastPM25_Colzir = Uptime.getMilliseconds();
    uint64_t now = Uptime.getMilliseconds();
    DynamicJsonDocument data(5120);
    if (now - lastPM25_Colzir > 60000) {
        lastPM25_Colzir = now;
        loopPM25_Colzir(data);
    }

    if (data.size() > 0 && getTimestamp() > 0) {
        data.shrinkToFit();
        Serial.println("Data: " + data.as<String>());
        mqttController.sendTelemetry(data, true, getTimestamp());
    }
    delayMicroseconds(1);
    esp_task_wdt_reset();
}

void setup() {
    //Dont do anything in setup
    //Add setup SEN55
    internalRtc.setTime(1000);
    btStop();
    esp_task_wdt_init(60, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
    esp_task_wdt_reset();

    Serial.begin(115200);
//    preferences.begin("Configs", false);
//    Serial.println("Hello from: " + preferences.getString("token", "not-set"));
    mqttController.init();
    mqttController.sendSystemAttributes(true);
    initInterfaces();
    Wire.begin();
    esp_task_wdt_reset();

    uint16_t error;
    char errorMessage[256];
    delay(1000);  // needed on some Arduino boards in order to have Serial ready

    esp_task_wdt_reset();
    connectToNetwork();
    esp_task_wdt_reset();
    setupPM25_Colzir();
    delay(1000);
    xTaskCreatePinnedToCore(
            core0Loop, // Function to implement the task
            "Core0Loop", // Name of the task
            10000, // Stack size in words
            NULL,  // Task input parameter
            0, // Priority of the task
            NULL,  // Task handle.
            0); // Core where the task should run
    esp_task_wdt_reset();
}

uint64_t core1Heartbeat;

void loop() {
    //Dont do anything
    esp_task_wdt_reset();

    if (Serial.available()) {
        if (Serial.readString().indexOf("reboot") >= 0)
            resetESP();
    }

    if (networkController.getCurrentNetworkInterface() != nullptr &&
        networkController.getCurrentNetworkInterface()->lastConnectionStatus()) {
        mqttController.loop();
    }

    networkController.loop();

    if ((Uptime.getSeconds() - core1Heartbeat) > 10) {
        core1Heartbeat = Uptime.getSeconds();
        printDBGln("Core 1 Heartbeat");
    }
}

void setupPM25_Colzir() {

  Serial.println("Adafruit PMSA003I Air Quality Sensor");

  // Wait three seconds for sensor to boot up!
  delay(3000);

  // If using serial, initialize it and set baudrate before starting!
  // Uncomment one of the following
  //Serial1.begin(9600);
  //pmSerial.begin(9600);

  // There are 3 options for connectivity!
  //if (! aqi.begin_I2C()) {      // connect to the sensor over I2C
  //if (! aqi.begin_UART(&Serial1)) { // connect to the sensor over hardware serial
  if (! aqi.begin_UART(&pmSerial)) { // connect to the sensor over software serial 
    Serial.println("Could not find PM 2.5 sensor!");
    while (1) delay(10);
  }

  Serial.println("PM25 found!");
  czr.init();
  Serial.print("COZIR_LIB_VERSION: ");
  Serial.println(COZIR_LIB_VERSION);
  Serial.println();

  //  set to polling explicitly.
  czr.setOperatingMode(CZR_POLLING);
}
