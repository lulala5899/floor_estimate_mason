#include <Arduino.h>
#include <Wire.h>
#include <time.h>
#include <sys/time.h>
#include <WiFi.h>
#include "HP206C.h"
#include "config.h"

// Configuration
#define DATA_COLLECTION_INTERVAL 150            // 150 milliseconds
#define TIME_SYNC_INTERVAL (60 * 60 * 2 * 1000) // 2 hour in milliseconds

// define onboard LED pin
constexpr int kOnboardLedPin = 2;

inline void blinkLoop(int delay_ms) {
    while (true) {
        digitalWrite(kOnboardLedPin, !digitalRead(kOnboardLedPin));
        delay(delay_ms);
    }
}

// Global variables
HP206C barometer;
String deviceMac;
unsigned long lastDataCollectionTime = 0;
unsigned long lastTimeSync = 0;
bool timeIsSynchronized = false;

// Function declarations
void collectAndSendData();
void requestTimeSync();
void processTimeinfo(unsigned long long timestamp);
void sendMACAddress();
bool processSerialInput();

void setup()
{
    Serial.begin(115200);
    Wire.begin(BARO_I2C_SDA_PIN, BARO_I2C_SCL_PIN); // Initialize I2C
    Serial.printf("INFO: I2C init SDA=%d SCL=%d\n", BARO_I2C_SDA_PIN, BARO_I2C_SCL_PIN);
    delay(1000);  // Wait for serial to connect

    pinMode(kOnboardLedPin, OUTPUT);
    // ensure initial state is LOW
    digitalWrite(kOnboardLedPin, LOW);

    deviceMac = WiFi.macAddress();

    // Initialize barometer
    if (!barometer.begin())
    {
        Serial.println("ERROR: Sensor initialization failed. Please check wiring.");
        blinkLoop(1000);
    }
}

void loop()
{
    // Process any incoming messages
    processSerialInput();

    // Check if time is synchronized and request time sync if not
    unsigned long currentMillis = millis();
    if ((!timeIsSynchronized) || (currentMillis - lastTimeSync >= TIME_SYNC_INTERVAL))
    {
        requestTimeSync();
    }
    // Collect data every DATA_COLLECTION_INTERVAL
    if (currentMillis - lastDataCollectionTime >= DATA_COLLECTION_INTERVAL)
    {
        lastDataCollectionTime = currentMillis;
        collectAndSendData();
    }
}

/**
 * Process incoming serial messages.
 * Returns true if a message was processed.
 */
bool processSerialInput()
{
    if (!Serial.available())
    {
        return false;
    }
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0)
    { // If input is empty or just whitespace
        Serial.println("DEBUG: Received empty input");
        return false;
    }
    if (input.startsWith("TS>"))
    {
        // Extract and set time from PC
        Serial.println("DEBUG: Received time sync request: " + input);
        String timestampStr = input.substring(3);
        unsigned long long timestamp = 0;
        // Manual string to unsigned long long conversion
        for (int i = 0; i < timestampStr.length(); i++)
        {
            char c = timestampStr.charAt(i);
            if (isDigit(c))
            {
                timestamp = timestamp * 10 + (c - '0');
            }
        }
        processTimeinfo(timestamp);
        return timeIsSynchronized;
    }
    else if (input.startsWith("WHICH_MAC>"))
    {
        sendMACAddress();
        return true; // 返回 true 表示已处理该命令
    }
    else
    {
        // 可以选择打印未知命令用于调试
        Serial.print("DEBUG: Received unknown command: ");
        Serial.println(input);
        return false; // 返回 false 表示未处理该命令
    }
}

// Aquire current UNIX timestamp in milliseconds
unsigned long long getUnixMillis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    unsigned long long millisecondsSinceEpoch =
        (unsigned long long)(tv.tv_sec) * 1000 +
        (unsigned long long)(tv.tv_usec) / 1000;
    return millisecondsSinceEpoch;
}

/**
 * Process time information received from PC
 * and set the system time accordingly.
 */
void processTimeinfo(unsigned long long timestamp)
{
    // Check for invalid timestamp (e.g., integer overflow)
    if (timestamp > 4000000000000ULL)
    { // Approximately year 2096
        Serial.println("ERROR: Invalid timestamp received");
        return;
    }
    time_t ts_sec = timestamp / 1000;
    suseconds_t ts_usec = (timestamp % 1000) * 1000;
    struct timeval tv = {ts_sec, ts_usec};

    // Set the system time
    settimeofday(&tv, NULL);

    lastTimeSync = millis();
    timeIsSynchronized = true;
}

void collectAndSendData()
{
    float temperature, pressure;
    
    // Use the new readAll method which handles measurement start and data reading
    if (!barometer.readAll(&temperature, &pressure))
    {
        Serial.println("ERROR: Failed to read sensor data!");
        return;
    }

    // Check for valid readings
    if (isnan(temperature) || isnan(pressure))
    {
        Serial.println("ERROR: Invalid sensor readings!");
        return;
    }

    // Get current timestamp from system time
    unsigned long long now = getUnixMillis();

    // Format: DATA>timestamp,pressure,temperature
    Serial.print("BAROD>");
    Serial.print(now);
    Serial.print(",");
    Serial.print(pressure, 2);
    Serial.print(",");
    Serial.println(temperature, 2);
}

void requestTimeSync()
{
    Serial.println("BAROT>" + deviceMac);
    // Wait for time synchronization (with timeout 1s)
    unsigned long syncStartTime = millis();
    while (!timeIsSynchronized && (millis() - syncStartTime < 1000))
    {
        processSerialInput();
        if (timeIsSynchronized)
        {
            return;
        }
    }
}

void sendMACAddress()
{
    // Send the device MAC address to the PC
    Serial.println("BAROM>" + deviceMac);
    delay(100); // Give some time for the message to be sent
}