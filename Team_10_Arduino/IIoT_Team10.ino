#include <HTTPClient.h>
#include <Wire.h>
#include <max32664.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <time.h>
#include <TimeLib.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Arduino.h>

#define MAIN_SSID "Vivo Y31"
#define MAIN_PASS "11111111"
#define CSE_IP "192.168.117.133"
#define CSE_PORT 5089
#define OM2M_ORGIN "admin:admin"
#define OM2M_MN "/~/in-cse/in-name/"
#define OM2M_AE "Patient_Monitor"
#define OM2M_DATA_TEMP "Temp/Data"
#define OM2M_DATA_spO2 "SpO2/Data"
#define OM2M_DATA_F1 "Flex/Data1"
#define OM2M_DATA_F2 "Flex/Data2"
#define OM2M_DATA_F3 "Flex/Data3"
#define OM2M_DATA_F4 "Flex/Data4"
#define OM2M_DATA_Heart "Heart_Rate/Data"
#define OM2M_DATA_BPS "BP-sys/Data"
#define OM2M_DATA_BPD "BP-dia/Data"
#define OM2M_DATA_Saline "Saline/Data"
#define OM2M_DATA_A1 "Accelerometer/Data1"
#define OM2M_DATA_A2 "Accelerometer/Data2"
#define OM2M_DATA_A3 "Accelerometer/Data3"

#define INTERVAL 1500L

int oneWireBus = 27;
// const int oneWireBus1=27;

const int bluetoothTx = 16;
const int bluetoothRx = 17;

OneWire oneWire(oneWireBus);

DallasTemperature sensors(&oneWire);

LiquidCrystal_I2C lcd(0x27, 16,2);

SoftwareSerial bluetoothSerial(bluetoothRx, bluetoothTx);

#define SENSOR_PIN 15

// Pin definitions
#define MAX32664_MFIO_PIN 05    // Replace 05 with the appropriate pin for the MAX32664 MFIO
#define MAX32664_RST_PIN 04     // Replace 04 with the appropriate pin for the MAX32664 RST
// #define MPU6050_ADDRESS 0x68    // Replace 0x68 with the MPU6050 I2C address


//SDA: 21
//SCL: 22
//Heart rate VCC:3.3V
//DHT VCC: 5V
//MPU VCC: 3.3V
//HC05 VCC: 5V
//LCD VCC: 5V

float sysBP;
float diaBP;
int heartRate;
float spo2Level;

const int flexSensorPin1 = 32;
const int flexSensorPin2 = 33;
const int flexSensorPin3 = 34;
const int flexSensorPin4 = 35;

const int wirePin1 = 32;  // GPIO pin connected to the wire
const int wirePin2 = 33;  
const int wirePinflex = 34;  // USE THE ORIGINAL FLEX CODE
const int wirePin3 = 35;  

float accelerationX;
float accelerationY;
float accelerationZ;

max32664 MAX32664(MAX32664_MFIO_PIN, MAX32664_RST_PIN, 4);
Adafruit_MPU6050 mpu;

// ThingSpeak configuration
char ssid[] = "Vivo Y31";
char password[] = "11111111";
unsigned long channelID = 2165858;
const char apiKey[] = "OIPOX3A8NAR68EAO";
WiFiClient client;

const char * ntpServer = "pool.ntp.org";
long int prev_millis = 0;
unsigned long epochTime;
HTTPClient http;
 
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime( & timeinfo)) {
    //Serial.println("Failed to obtain time");
    return (0);
  }
  time( & now);
  return now;
}

void connectToBluetooth() {
  // Set the baud rate of the Bluetooth module to match the app
  bluetoothSerial.println("AT+UART=9600,0,0");

  // Wait for response from Bluetooth module
  delay(100);

  // Print the response from the Bluetooth module
  while (bluetoothSerial.available()) {
    Serial.write(bluetoothSerial.read());
  }

  // Set the Bluetooth module to slave mode
  bluetoothSerial.println("AT+ROLE=0");

  // Wait for response from Bluetooth module
  delay(100);

  // Print the response from the Bluetooth module
  while (bluetoothSerial.available()) {
    Serial.write(bluetoothSerial.read());
  }

  // Set the Bluetooth module to accept connections from any device
  bluetoothSerial.println("AT+CMODE=1");

  // Wait for response from Bluetooth module
  delay(100);

  // Print the response from the Bluetooth module
  while (bluetoothSerial.available()) {
    Serial.write(bluetoothSerial.read());
  }

  // Print a message indicating the Bluetooth connection setup is complete
  Serial.println("Bluetooth connection setup complete.");
}

void mfioInterruptHndlr(){
}

void enableInterruptPin(){
  attachInterrupt(digitalPinToInterrupt(MAX32664.mfioPin), mfioInterruptHndlr, FALLING);  
}

void loadAlgomodeParameters(){
  algomodeInitialiser algoParameters;
  algoParameters.calibValSys[0] = 120;
  algoParameters.calibValSys[1] = 122;
  algoParameters.calibValSys[2] = 125;
  algoParameters.calibValDia[0] = 80;
  algoParameters.calibValDia[1] = 81;
  algoParameters.calibValDia[2] = 82;
  
  algoParameters.spo2CalibCoefA = 1.5958422;
  algoParameters.spo2CalibCoefB = -34.659664;
  algoParameters.spo2CalibCoefC = 112.68987;

  MAX32664.loadAlgorithmParameters(&algoParameters);
}

void ReadSaline() {
  int sensorValue = digitalRead(SENSOR_PIN);
  if(sensorValue == 0)
  {
    Serial.println("Saline almost Empty!!");
  }
  delay(100);
}

void setup() {
  pinMode(wirePin1, INPUT);
  pinMode(wirePin2, INPUT);
  pinMode(wirePin3, INPUT);
  delay(1000);
  Serial.begin(9600);
  analogReadResolution(12);
  Wire.begin();

  lcd.begin();
  lcd.backlight();

  lcd.clear();
  lcd.print("Wifi");
  WiFi.begin(MAIN_SSID, MAIN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("#");
    lcd.clear();
    lcd.print("Connecting...");
  }
  Serial.println("Connected");
  lcd.clear();
  lcd.print("Connected");
  configTime(0, 0, ntpServer);

  delay(500);

  lcd.clear();
  lcd.print("Bluetooth");
  bluetoothSerial.begin(9600);
  delay(1000);
  connectToBluetooth();

  delay(500);

  lcd.clear();
  lcd.print("Pulse Oximeter");
  loadAlgomodeParameters();
  int result = MAX32664.hubBegin();
  if (result == CMD_SUCCESS) {
    Serial.println("Sensorhub begin!");
  } else {
    while (1) {
      Serial.println("Could not communicate with the sensor! please make proper connections");
      delay(1000);
    }
  }

  delay(500);

  bool ret = MAX32664.startBPTcalibration();
  while (!ret) {
    delay(10000);
    Serial.println("Failed calib, please restart");
  }

  delay(500);

  ret = MAX32664.configAlgoInEstimationMode();
  while (!ret) {
    ret = MAX32664.configAlgoInEstimationMode();
    delay(10000);
  }
  Serial.println("Getting the device ready..");

  delay(500);

  lcd.clear();
  lcd.print("Accelerometer");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    lcd.clear();
    lcd.println("Accelerometer not");
    lcd.print("found!");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  delay(500);
  
  sensors.begin();

  lcd.clear();
  lcd.println("Thingspeak...");
  ThingSpeak.begin(client);
  lcd.print("connected");

  delay(500);
}

void Read_Accelerometer() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accelerationX = a.acceleration.x;
  accelerationY = a.acceleration.y;
  accelerationZ = a.acceleration.z;
  // Serial.print("X: ");
  // Serial.println(accelerationX);
  // bluetoothSerial.println(accelerationX);
  // Serial.print("Y: ");
  // Serial.println(accelerationY);
  // bluetoothSerial.println(accelerationY);
  // Serial.print("Z: ");
  // Serial.println(accelerationZ);
  // bluetoothSerial.println(accelerationZ);
  float acceleration=sqrt(accelerationX*accelerationX+accelerationY*accelerationY+accelerationZ*accelerationZ);
  // Serial.println(acceleration);
  if(acceleration>20 || acceleration<3) {
    Serial.println("Fallen!");
  }
}

void Read_Flex_Sensors()
{
  int wireStatus1 = digitalRead(wirePin1);
  int wireStatus2 = digitalRead(wirePin2);
  int wireStatus3 = digitalRead(wirePin3); 
  int flexValue4 = analogRead(wirePinflex);

  if (wireStatus1 == LOW)
  {
    Serial.println("Water!");
  }
  if (wireStatus2 == LOW)
  {
    Serial.println("Food!");
  }
  if (wireStatus3 == LOW)
  {
    Serial.println("Washroom!");
  }
  if (flexValue4 > 30) {
    Serial.println("Emergency");
  }
}

void loop() {
  Read_Accelerometer();
  Read_Flex_Sensors();

  delay(500);

  uint8_t num_samples = MAX32664.readSamples();
  if (num_samples) {
    if (MAX32664.max32664Output.sys != 0 && MAX32664.max32664Output.dia != 0 && MAX32664.max32664Output.hr != 0 && MAX32664.max32664Output.spo2 != 0) {
      // Store blood pressure, heart rate, and SpO2 values in variables
      sysBP = MAX32664.max32664Output.sys;
      diaBP = MAX32664.max32664Output.dia;
      heartRate = MAX32664.max32664Output.hr;
      spo2Level = MAX32664.max32664Output.spo2;
      // Serial.println(heartRate);
      // Serial.println(spo2Level);
      // Serial.println(sysBP);
      // Serial.println(diaBP);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(heartRate);
      lcd.print(" bpm");
      lcd.print(spo2Level);
      lcd.print("% Sys: ");
      lcd.setCursor(0, 1);
      lcd.print(sysBP);
      lcd.print(" Dia: ");
      lcd.print(diaBP);

      // Send blood pressure, heart rate, and SpO2 values to ThingSpeak
      ThingSpeak.setField(3, sysBP);
      ThingSpeak.setField(4, diaBP);
      ThingSpeak.setField(1, heartRate);
      ThingSpeak.setField(2, spo2Level);
      ThingSpeak.writeFields(channelID, apiKey);
      // Serial.println("Published");
    }
  }

  delay(500);
  Read_Accelerometer();
  Read_Flex_Sensors();

  delay(500);
  Read_Accelerometer();
  Read_Flex_Sensors();

  delay(500);
  Read_Accelerometer();
  Read_Flex_Sensors();

  delay(500);
  Read_Accelerometer();
  Read_Flex_Sensors();

  delay(100);

  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  // Serial.print(temperatureC);
  // Serial.println("ºC");
  // Serial.print(temperatureF);
  // Serial.println("ºF");
  ThingSpeak.setField(5, temperatureF);
  ThingSpeak.writeFields(channelID, apiKey);
  // Serial.println("Published");
  lcd.clear();
  lcd.print(temperatureC);
  lcd.print("C ");
  lcd.print(temperatureF);
  lcd.print("F");
  
  delay(500);
  Read_Accelerometer();
  Read_Flex_Sensors();

  delay(500);
  Read_Accelerometer();
  Read_Flex_Sensors();

  delay(500);
  Read_Accelerometer();
  Read_Flex_Sensors();

  delay(500);
  Read_Accelerometer();
  Read_Flex_Sensors();

  ReadSaline();

  if (millis() - prev_millis >= INTERVAL) {
        epochTime = getTime();
     
        String data;
        //temperature
        String server = "http://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;
        http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_TEMP + "/");
      	http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        http.addHeader("Content-Type", "application/json;ty=4");
        http.addHeader("Content-Length", "100");
        data=temperatureF;
        String req_data = String() + "{\"m2m:cin\": {"
     
          +
          "\"con\": \"" + data + "\","
     
          +
          "\"lbl\": \"" + "V1.0.0" + "\","
     
          //+ "\"rn\": \"" + "cin_"+String(i++) + "\","
     
          +
          "\"cnf\": \"text\""
     
          +
          "}}";
        int code = http.POST(req_data);
        // Serial.println(code);
        http.end();
        // Serial.println("Temperature sent");
        //spO2
         server = "http://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;
        http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_spO2 + "/");
      	http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        http.addHeader("Content-Type", "application/json;ty=4");
        http.addHeader("Content-Length", "100");
        data=spo2Level;
         req_data = String() + "{\"m2m:cin\": {"
     
          +
          "\"con\": \"" + data + "\","
     
          +
          "\"lbl\": \"" + "V1.0.0" + "\","
     
          //+ "\"rn\": \"" + "cin_"+String(i++) + "\","
     
          +
          "\"cnf\": \"text\""
     
          +
          "}}";
        code = http.POST(req_data);

        // Serial.println(code);
        http.end();
        // Serial.println("SpO2 sent");

        // //flex
        //  server = "http://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;
        // http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_F1 + "/");
      	// http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        // http.addHeader("Content-Type", "application/json;ty=4");
        // http.addHeader("Content-Length", "100");
        // data = 50;
        //  req_data = String() + "{\"m2m:cin\": {"
     
        //   +
        //   "\"con\": \"" + data + "\","
     
        //   +
        //   "\"lbl\": \"" + "V1.0.0" + "\","
     
        //   //+ "\"rn\": \"" + "cin_"+String(i++) + "\","
     
        //   +
        //   "\"cnf\": \"text\""
     
        //   +
        //   "}}";
        // code = http.POST(req_data);
        // http.end();
        // Serial.println("Flex1 sent");

        // //flex
        //  server = "http://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;
        // http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_F2 + "/");
      	// http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        // http.addHeader("Content-Type", "application/json;ty=4");
        // http.addHeader("Content-Length", "100");
        // data = 50;
        //  req_data = String() + "{\"m2m:cin\": {"
     
        //   +
        //   "\"con\": \"" + data + "\","
     
        //   +
        //   "\"lbl\": \"" + "V1.0.0" + "\","
     
        //   //+ "\"rn\": \"" + "cin_"+String(i++) + "\","
     
        //   +
        //   "\"cnf\": \"text\""
     
        //   +
        //   "}}";
        // code = http.POST(req_data);
        // http.end();
        // Serial.println("Flex2 sent");

        // //flex
        //  server = "http://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;
        // http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_F3 + "/");
      	// http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        // http.addHeader("Content-Type", "application/json;ty=4");
        // http.addHeader("Content-Length", "100");
        // data = 50;
        //  req_data = String() + "{\"m2m:cin\": {"
     
        //   +
        //   "\"con\": \"" + data + "\","
     
        //   +
        //   "\"lbl\": \"" + "V1.0.0" + "\","
     
        //   //+ "\"rn\": \"" + "cin_"+String(i++) + "\","
     
        //   +
        //   "\"cnf\": \"text\""
     
        //   +
        //   "}}";
        // code = http.POST(req_data);
        // http.end();
        // Serial.println("Flex3 sent");

        // //flex
        //  server = "http://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;
        // http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_F4 + "/");
      	// http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        // http.addHeader("Content-Type", "application/json;ty=4");
        // http.addHeader("Content-Length", "100");
        // data = 50;
        //  req_data = String() + "{\"m2m:cin\": {"
     
        //   +
        //   "\"con\": \"" + data + "\","
     
        //   +
        //   "\"lbl\": \"" + "V1.0.0" + "\","
     
        //   //+ "\"rn\": \"" + "cin_"+String(i++) + "\","
     
        //   +
        //   "\"cnf\": \"text\""
     
        //   +
        //   "}}";
        // code = http.POST(req_data);
        // http.end();
        // Serial.println("Flex4 sent");

        //heart
         server = "http://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;
        http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_Heart + "/");
      	http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        http.addHeader("Content-Type", "application/json;ty=4");
        http.addHeader("Content-Length", "100");
        data=heartRate;
         req_data = String() + "{\"m2m:cin\": {"
     
          +
          "\"con\": \"" + data + "\","
     
          +
          "\"lbl\": \"" + "V1.0.0" + "\","
     
          //+ "\"rn\": \"" + "cin_"+String(i++) + "\","
     
          +
          "\"cnf\": \"text\""
     
          +
          "}}";
        code = http.POST(req_data);
        http.end();
        // Serial.println("Heart rate sent");

        //bp sys
         server = "http://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;
        http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_BPS + "/");
      	http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        http.addHeader("Content-Type", "application/json;ty=4");
        http.addHeader("Content-Length", "100");
        data=sysBP;
         req_data = String() + "{\"m2m:cin\": {"
     
          +
          "\"con\": \"" + data + "\","
     
          +
          "\"lbl\": \"" + "V1.0.0" + "\","
     
          //+ "\"rn\": \"" + "cin_"+String(i++) + "\","
     
          +
          "\"cnf\": \"text\""
     
          +
          "}}";
        code = http.POST(req_data);
        http.end();
        // Serial.println("Systolic BP sent");

        // //accel
        //  server = "http://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;
        // http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_A1 + "/");
      	// http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        // http.addHeader("Content-Type", "application/json;ty=4");
        // http.addHeader("Content-Length", "100");
        // data=accelerationX;
        //  req_data = String() + "{\"m2m:cin\": {"
     
        //   +
        //   "\"con\": \"" + data + "\","
     
        //   +
        //   "\"lbl\": \"" + "V1.0.0" + "\","
     
        //   //+ "\"rn\": \"" + "cin_"+String(i++) + "\","
     
        //   +
        //   "\"cnf\": \"text\""
     
        //   +
        //   "}}";
        // code = http.POST(req_data);
        // http.end();
        // Serial.println("Accel1 sent");

        // //accel
        //  server = "http://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;
        // http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_A2 + "/");
      	// http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        // http.addHeader("Content-Type", "application/json;ty=4");
        // http.addHeader("Content-Length", "100");
        // data=accelerationY;
        //  req_data = String() + "{\"m2m:cin\": {"
     
        //   +
        //   "\"con\": \"" + data + "\","
     
        //   +
        //   "\"lbl\": \"" + "V1.0.0" + "\","
     
        //   //+ "\"rn\": \"" + "cin_"+String(i++) + "\","
     
        //   +
        //   "\"cnf\": \"text\""
     
        //   +
        //   "}}";
        // code = http.POST(req_data);
        // http.end();
        // Serial.println("Accel2 sent");

        // //accel
        //  server = "http://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;
        // http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_A3 + "/");
      	// http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        // http.addHeader("Content-Type", "application/json;ty=4");
        // http.addHeader("Content-Length", "100");
        // data=accelerationZ;
        //  req_data = String() + "{\"m2m:cin\": {"
     
        //   +
        //   "\"con\": \"" + data + "\","
     
        //   +
        //   "\"lbl\": \"" + "V1.0.0" + "\","
     
        //   //+ "\"rn\": \"" + "cin_"+String(i++) + "\","
     
        //   +
        //   "\"cnf\": \"text\""
     
        //   +
        //   "}}";
        // code = http.POST(req_data);
        // http.end();
        // Serial.println("Accel3 sent");

        //saline
         server = "http://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;
        http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_Saline + "/");
      	http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        http.addHeader("Content-Type", "application/json;ty=4");
        http.addHeader("Content-Length", "100");
        data = 50;
         req_data = String() + "{\"m2m:cin\": {"
     
          +
          "\"con\": \"" + data + "\","
     
          +
          "\"lbl\": \"" + "V1.0.0" + "\","
     
          //+ "\"rn\": \"" + "cin_"+String(i++) + "\","
     
          +
          "\"cnf\": \"text\""
     
          +
          "}}";
        code = http.POST(req_data);
        http.end();
        // Serial.println("Saline level sent");

        //bp dia
         server = "http://" + String() + CSE_IP + ":" + String() + CSE_PORT + String() + OM2M_MN;
        http.begin(server + String() + OM2M_AE + "/" + OM2M_DATA_BPD + "/");
      	http.addHeader("X-M2M-Origin", OM2M_ORGIN);
        http.addHeader("Content-Type", "application/json;ty=4");
        http.addHeader("Content-Length", "100");
        data=diaBP;
         req_data = String() + "{\"m2m:cin\": {"
     
          +
          "\"con\": \"" + data + "\","
     
          +
          "\"lbl\": \"" + "V1.0.0" + "\","
     
          //+ "\"rn\": \"" + "cin_"+String(i++) + "\","
     
          +
          "\"cnf\": \"text\""
     
          +
          "}}";
        code = http.POST(req_data);
        http.end();
        // Serial.println("Diastolic BP sent");

        // Serial.println(code);
        prev_millis = millis();
}
 
}