#include <M5Stack.h>
#include "Free_Fonts.h"
#include <Wire.h>
#include <DFRobot_SHT20.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include "Adafruit_SGP30.h"
#include <WiFi.h>
#include <PubSubClient.h>  // MQTT library


// Define a struct to hold PM and particle data
struct PMData {
    uint16_t frame_length;
    uint16_t standard_PM1_0;
    uint16_t standard_PM2_5;
    uint16_t standard_PM10;
    uint16_t atmospheric_PM1_0;
    uint16_t atmospheric_PM2_5;
    uint16_t atmospheric_PM10;
    uint16_t particles_0_3um;
    uint16_t particles_0_5um;
    uint16_t particles_1um;
    uint16_t particles_2_5um;
    uint16_t particles_5um;
    uint16_t particles_10um;
    uint16_t reserved;
    uint16_t checksum;
};
PMData pmData;  // Create an instance of PMData

//Same for SGP30 air sensor data
struct AirQData {
    uint16_t ethanol;
    uint16_t hydrogen;
    uint16_t eCo2;
};
AirQData airQData;

float temp = 0.0;
float humd = 0.0;
float co2Level = 0.0;
int ilumLevel=0;

DFRobot_SHT20 sht20;
SCD30 co2Sensor;
Adafruit_SGP30 airSensor;
#define TSL_MEASURMENT_PERIOD_US 100000


#define TFT_GREY 0x7BEF

#define DATA_LEN 32
#define X_LOCAL 40
#define Y_LOCAL 30
#define X_OFFSET 160
#define Y_OFFSET 23

#define OUT_BAUD 115200
#define SENSOR_BAUD 9600
#define FIRST_START_BIT 0x42  // 66 in hex
#define SECOND_START_BIT 0x4D // 77 in hex
#define FONT_SIZE 2

bool firstStartBitIsRead;
bool secondStartBitIsRead;
uint8_t serial_bytes[32] = {0};
uint8_t currentByte = 0;

// Wi-Fi credentials
const char* ssid = "Tenda_5B0D30";
const char* password = "Bochum2021";

// MQTT Broker details
const char* mqtt_server = "homeassistant.local";
WiFiClient espClient;
PubSubClient client(espClient);

// Function to connect to Wi-Fi
void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    client.setServer(mqtt_server, 1883);
}

// Function to connect to MQTT broker
void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("M5StackClient", "mqtt_pub_sub", "malina")) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" trying again in 5 seconds");
            delay(5000);
        }
    }
}

// Reset serial data buffer
void resetSerial() {
    memset(serial_bytes, 0, sizeof(serial_bytes));
    currentByte = 0;
    firstStartBitIsRead = false;
    secondStartBitIsRead = false;
}

// Print the header for the display
void header(const char* title, uint16_t color) {
    M5.Lcd.fillScreen(color);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLUE);
    M5.Lcd.fillRect(0, 0, 320, 30, TFT_BLUE);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString(title, 160, 3, 4);
}

// Function to display PM values and particle counts on the LCD
void displayPMandParticleValues() {
    M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    M5.Lcd.setCursor(X_LOCAL, Y_LOCAL, FONT_SIZE);
    M5.Lcd.print("Standard PM");

    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(X_LOCAL, Y_LOCAL + Y_OFFSET, FONT_SIZE);
    M5.Lcd.printf("PM1.0 : %d", pmData.standard_PM1_0);

    M5.Lcd.setCursor(X_LOCAL, Y_LOCAL + Y_OFFSET * 2, FONT_SIZE);
    M5.Lcd.printf("PM2.5 : %d", pmData.standard_PM2_5);

    M5.Lcd.setCursor(X_LOCAL, Y_LOCAL + Y_OFFSET * 3, FONT_SIZE);
    M5.Lcd.printf("PM10  : %d", pmData.standard_PM10);

    M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    M5.Lcd.setCursor(X_LOCAL + X_OFFSET, Y_LOCAL, FONT_SIZE);
    M5.Lcd.print("Atmospheric PM");

    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(X_LOCAL + X_OFFSET, Y_LOCAL + Y_OFFSET, FONT_SIZE);
    M5.Lcd.printf("PM1.0 : %d", pmData.atmospheric_PM1_0);

    M5.Lcd.setCursor(X_LOCAL + X_OFFSET, Y_LOCAL + Y_OFFSET * 2, FONT_SIZE);
    M5.Lcd.printf("PM2.5 : %d", pmData.atmospheric_PM2_5);

    M5.Lcd.setCursor(X_LOCAL + X_OFFSET, Y_LOCAL + Y_OFFSET * 3, FONT_SIZE);
    M5.Lcd.printf("PM10  : %d", pmData.atmospheric_PM10);

    M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    M5.Lcd.setCursor(X_LOCAL, Y_LOCAL + Y_OFFSET * 5, FONT_SIZE);
    M5.Lcd.print("Particle Counts");

    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(X_LOCAL, Y_LOCAL + Y_OFFSET * 6, FONT_SIZE);
    M5.Lcd.printf("0.3um : %u", pmData.particles_0_3um);

    M5.Lcd.setCursor(X_LOCAL, Y_LOCAL + Y_OFFSET * 7, FONT_SIZE);
    M5.Lcd.printf("0.5um : %d", pmData.particles_0_5um);

    M5.Lcd.setCursor(X_LOCAL, Y_LOCAL + Y_OFFSET * 8, FONT_SIZE);
    M5.Lcd.printf("1.0um : %d", pmData.particles_1um);

    M5.Lcd.setCursor(X_LOCAL + X_OFFSET, Y_LOCAL + Y_OFFSET * 6, FONT_SIZE);
    M5.Lcd.printf("2.5um : %d", pmData.particles_2_5um);

    M5.Lcd.setCursor(X_LOCAL + X_OFFSET, Y_LOCAL + Y_OFFSET * 7, FONT_SIZE);
    M5.Lcd.printf("5.0um : %d", pmData.particles_5um);

    M5.Lcd.setCursor(X_LOCAL + X_OFFSET, Y_LOCAL + Y_OFFSET * 8, FONT_SIZE);
    M5.Lcd.printf("10um  : %d", pmData.particles_10um);
}

// Read and process serial data from the PMS sensor
void readPMData() {
    pmData.frame_length = (serial_bytes[2] << 8) | serial_bytes[3];
    pmData.standard_PM1_0 = (serial_bytes[4] << 8) | serial_bytes[5];
    pmData.standard_PM2_5 = (serial_bytes[6] << 8) | serial_bytes[7];
    pmData.standard_PM10 = (serial_bytes[8] << 8) | serial_bytes[9];
    pmData.atmospheric_PM1_0 = (serial_bytes[10] << 8) | serial_bytes[11];
    pmData.atmospheric_PM2_5 = (serial_bytes[12] << 8) | serial_bytes[13];
    pmData.atmospheric_PM10 = (serial_bytes[14] << 8) | serial_bytes[15];
    pmData.particles_0_3um = (serial_bytes[16] << 8) | serial_bytes[17];
    pmData.particles_0_5um = (serial_bytes[18] << 8) | serial_bytes[19];
    pmData.particles_1um = (serial_bytes[20] << 8) | serial_bytes[21];
    pmData.particles_2_5um = (serial_bytes[22] << 8) | serial_bytes[23];
    pmData.particles_5um = (serial_bytes[24] << 8) | serial_bytes[25];
    pmData.particles_10um = (serial_bytes[26] << 8) | serial_bytes[27];
    pmData.reserved = (serial_bytes[28] << 8) | serial_bytes[29];
    pmData.checksum = (serial_bytes[30] << 8) | serial_bytes[31];
}

bool verifyChecksum(const uint8_t *data, uint16_t len) {
    uint16_t checksum = 0;
    for (int i = 0; i < len - 2; i++) {  // Exclude the checksum bytes from the calculation
        checksum += data[i];
    }
    uint16_t transmitted_checksum = (data[len - 2] << 8) | data[len - 1];
    return (checksum == transmitted_checksum);
}

// Read temperature and humidity from SHT20 sensor
void readTempHumidity() {
    temp = sht20.readTemperature();
    humd = sht20.readHumidity();
    
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    M5.Lcd.setCursor(X_LOCAL, Y_LOCAL + Y_OFFSET * 10, FONT_SIZE);
    M5.Lcd.printf("Temp : %.2f C", temp);

    M5.Lcd.setCursor(X_LOCAL + X_OFFSET, Y_LOCAL + Y_OFFSET * 10, FONT_SIZE);
    M5.Lcd.printf("Hum  : %.2f %%", humd);
}

// Send sensor data via MQTT (PM and particle number data)
void sendMQTTData() {
    if (!client.connected()) {
        reconnect();
    }

    char tempStr[8], humdStr[8], valueStr[8];
    dtostrf(temp, 6, 2, tempStr);
    dtostrf(humd, 6, 2, humdStr);

    // Publish temperature and humidity
    client.publish("home/sensor/m5stack/temperature", tempStr);
    client.publish("home/sensor/m5stack/humidity", humdStr);

    // Publish PM data
    snprintf(valueStr, sizeof(valueStr), "%d", pmData.standard_PM1_0);
    client.publish("home/sensor/m5stack/standard_pm1_0", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%d", pmData.standard_PM2_5);
    client.publish("home/sensor/m5stack/standard_pm2_5", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%d", pmData.standard_PM10);
    client.publish("home/sensor/m5stack/standard_pm10", valueStr);

    // Publish particle number data
    snprintf(valueStr, sizeof(valueStr), "%d", pmData.particles_0_3um);
    client.publish("home/sensor/m5stack/particles_0_3um", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%d", pmData.particles_0_5um);
    client.publish("home/sensor/m5stack/particles_0_5um", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%d", pmData.particles_1um);
    client.publish("home/sensor/m5stack/particles_1um", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%d", pmData.particles_2_5um);
    client.publish("home/sensor/m5stack/particles_2_5um", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%d", pmData.particles_5um);
    client.publish("home/sensor/m5stack/particles_5_0um", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%d", pmData.particles_10um);
    client.publish("home/sensor/m5stack/particles_10um", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%f", co2Level);
    client.publish("home/sensor/m5stack/co2Level", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%d", airQData.ethanol);
    client.publish("home/sensor/m5stack/ethanol", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%d", airQData.hydrogen);
    client.publish("home/sensor/m5stack/hydrogen", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%d", airQData.eCo2);
    client.publish("home/sensor/m5stack/eCO2", valueStr);

    snprintf(valueStr, sizeof(valueStr), "%d", ilumLevel);
    client.publish("home/sensor/m5stack/iluminationLevel", valueStr);
    Serial.print("Ilumination: ");
    Serial.println(ilumLevel);

    client.loop();
}

// Serial reading
void readCurrentBit() {
    uint8_t byte = Serial2.read();

    if (!firstStartBitIsRead) {
        if (byte != FIRST_START_BIT) {
            resetSerial();
            return;
        }
        firstStartBitIsRead = true;
    } else if (!secondStartBitIsRead) {
        if (byte != SECOND_START_BIT) {
            resetSerial();
            return;
        }
        secondStartBitIsRead = true;
    }

    if (currentByte < DATA_LEN) {
        serial_bytes[currentByte++] = byte;
    }
}

void readCO2(){
    co2Level = co2Sensor.getCO2();
    Serial.print("CO2: ");
    Serial.println(co2Level);
}


void readTVOC(){
    if (!(airSensor.IAQmeasure() && airSensor.IAQmeasureRaw())) {
        Serial.println("SGP30 read failed");
    }else{
        Serial.println("SGP30 read ok");
        Serial.print("Raw Ethanol: ");
        airQData.ethanol = airSensor.rawEthanol;
        Serial.println(airQData.ethanol);

        Serial.print("Raw H2: ");
        airQData.hydrogen=airSensor.rawH2;
        Serial.println(airQData.hydrogen);

        Serial.print("eCo2: ");
        airQData.eCo2 = airSensor.eCO2;
        Serial.println(airQData.eCo2);
   }
}

volatile double tslPulsesCount=0;
void tsl_handler(){
  tslPulsesCount++;
}

void readTSL(){
  tslPulsesCount=0;
  double startTime=micros();
  attachInterrupt(G5, tsl_handler, RISING);
  while((micros()-startTime)<TSL_MEASURMENT_PERIOD_US);
  detachInterrupt(G5);
  double durationSec = (micros()-startTime)/TSL_MEASURMENT_PERIOD_US;
  ilumLevel =  tslPulsesCount/durationSec; //hz = mW/cm^2
}


void setup() {
    M5.begin();

    Serial.begin(OUT_BAUD);
    Serial2.begin(SENSOR_BAUD, SERIAL_8N1, 16, 17);
    header("Sensors Init...", TFT_BLACK);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    //pinMode(10, OUTPUT);
    //digitalWrite(10, 1);

    //Init CO2 sensor
    Wire.begin(); //Start the wire hardware that may be supported by your platform
    if (co2Sensor.begin(Wire) == false) //Pass the Wire port to the .begin() function
    {
        Serial.println("SCD30(NDIR) not detected. Please check wiring. Freezing...");
        while (1);
    }

    // Initialize SHT20 temp/hum sensor
    sht20.initSHT20();
    delay(100);
    sht20.checkSHT20();

    //Initialize SGP30 Eco2 sensor
    airSensor.begin();
    airSensor.IAQinit();
    //TSL235
    pinMode(G5, INPUT);
    digitalWrite(G5, HIGH);
    

    // Initialize display
    M5.Lcd.fillScreen(TFT_BLACK);
    header("Connecting to network", TFT_BLACK);
    setup_wifi();
    resetSerial();
    header("Environmental sensor", TFT_BLACK);
 
}

void loop() {
    if (Serial2.available()) {
        readCurrentBit();
    }

    if (currentByte >= DATA_LEN) {
        if (verifyChecksum(serial_bytes, DATA_LEN)) {
            readPMData();
          
        } else {
            Serial.println("Checksum error");
        }
        displayPMandParticleValues();
        readTempHumidity();
        readCO2();
        readTVOC();
        readTSL();
        sendMQTTData();
        resetSerial();
    }
}

