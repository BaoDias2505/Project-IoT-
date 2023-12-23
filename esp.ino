#include <Adafruit_Fingerprint.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"                      //DHT
#include <LiquidCrystal_I2C.h>        //LCD
#include <Adafruit_Fingerprint.h>     //Finger
#include <Servo.h>
#include <Wire.h>

int servo_time = 0;
int Stranger_begin = 0, Stranger_end = 0;
String Stranger_status = "0";
String servo_status = "Dong";
int buzzer = D0;
int warning = 0;
int count_open = 3;

//PUSH NOTIs
const char* host = "maker.ifttt.com";
const int port = 80;
const char* request = "/trigger/SLnoti/with/key/l9Pu7kbjrh7bB-AxXeNssh_SVLmRjRtXFVDlfw-Q3U1";

void sendRequest() {
  WiFiClient client;
  while(!client.connect(host, port)) {
    Serial.println("connection fail");
    delay(1000);
  }
  client.print(String("GET ") + request + " HTTP/1.1\r\n"
              + "Host: " + host + "\r\n"
              + "Connection: close\r\n\r\n");
  delay(500);

  while(client.available()) {
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
}

//Servo
Servo myServo;

//DHT 
#define DHTPIN D5     // what digital pin we're connected to
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);

//KK
int sensorValue;

//UltraSonic
const int trig = D8;     // chân trig của HC-SR04
const int echo = D7;     // chân echo của HC-SR04

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
int sdaPin = D3;  // Chân D3 là chân SDA
int sclPin = D4;  // Chân D4 là chân SCL

//Finger
volatile int finger_status = -1;

#define Finger_Rx 4
#define Finger_Tx 5

#if (defined(_AVR_) || defined(ESP8266)) && !defined(_AVR_ATmega2560_)
SoftwareSerial mySerial(Finger_Rx, Finger_Tx);
#else
#define mySerial Serial1
#endif
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

uint8_t id;
uint8_t readnumber(void) {
  uint8_t num = 0;

  while (num == 0) {
    while (! Serial.available());
    num = Serial.parseInt();
  }
  return num;
}

//***Set server***
const char* mqttServer = "broker.mqtt-dashboard.com"; 
int port = 1883;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void wifiConnect() {
  WiFi.begin("i180", "CaoConNgua@1!");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print(" Connected!");
}

void mqttConnect() {
  while(!mqttClient.connected()) {
    Serial.print("Attemping MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if(mqttClient.connect(clientId.c_str())) {
      Serial.print("connected");

      //***Subscribe all topic you need***
      mqttClient.subscribe("21127583/ser_web");
      mqttClient.subscribe("21127583/fin_web");
      mqttClient.subscribe("21127583/warning_web");
    }
    else {
      Serial.print("try again in 5 seconds");
      delay(5000);
    }
  }
}

//***MQTT Receiver***
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print(topic);
  String strMsg;
  for(int i=0; i<length; i++) {
    strMsg += (char)message[i];
  }
  Serial.print(strMsg);

    //***Insert code here to control other devices***
  if(String(topic) == "21127583/ser_web") {
    if(strMsg == "Mo") {
      servo_open();
    }
    else {
      servo_close();
    }
  }

  if(String(topic) == "21127583/fin_web") {
    int id = strMsg.toInt();
    bool result = fingerprint_registration(id);
    if(result == 1){
      char buffer_result[5] = "true";
      mqttClient.publish("21127583/fin", buffer_result);
    }
    else{
      char buffer_result[6] = "false";
      mqttClient.publish("21127583/fin", buffer_result);
    }
  }

  if(String(topic) == "21127583/warning_web") {
    count_open = strMsg.toInt();
    Serial.print(count_open);
  }
}

// Temperature
void temperature(){
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.print("Failed to read from DHT sensor!");
    return;
  }

  char buffer_tem[50];
  sprintf(buffer_tem, "%f", t);
  mqttClient.publish("21127583/tem", buffer_tem);

  char buffer_hum[50];
  sprintf(buffer_hum, "%f", h);
  mqttClient.publish("21127583/hum", buffer_hum);
  
  int distance = ultrasonic();
  if(distance <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print("Humidity: ");
  lcd.setCursor(0,1);
  lcd.print(h);
  lcd.print("%");
  if(distance <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }
  delay(1000);
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print("Temperature: ");
  lcd.setCursor(0,1);
  lcd.print(t);
  lcd.print(" Celsius");
  if(distance <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }
  delay(1000);
}

// KK
void air_quality(){
  sensorValue = analogRead(A0);  
  char buffer_air[50];
  sprintf(buffer_air, "%d", sensorValue);
  mqttClient.publish("21127583/air", buffer_air);

  int distance = ultrasonic();
  if(distance <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }
  lcd.setCursor(0,0);
  lcd.clear();     
  lcd.print("Air Quality=");
  lcd.setCursor(0,1);
  lcd.print(sensorValue);
  lcd.print(" PPM");
  delay(1000);
}

//Ultrasonic
long getDistance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH);
  long distanceCm = duration * 0.034 / 2;
  return distanceCm;
}

int ultrasonic(){ 
  int distanceCm = getDistance();
  if(distanceCm <=50){
    lcd.backlight();
  }
  else{
    lcd.noBacklight();
  }
  /* In kết quả ra Serial Monitor */
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print("Distance: ");
  lcd.setCursor(0,1);
  lcd.print(distanceCm);
  delay(1000);
  return distanceCm;
}

void setup() {
  Serial.begin(9600);
  myServo.attach(D6);
  servo_close_web();

  //Buzzer
  pinMode(buzzer, OUTPUT);

  //DHT
  dht.begin();
  //UltraSonic
  pinMode(trig, OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(echo, INPUT);    // chân echo sẽ nhận tín hiệu

  //LCD
  Wire.begin(sdaPin, sclPin);
  lcd.init();                    
  lcd.backlight();
  lcd.setCursor(2,0);
  lcd.print("Hello Everyone");
  lcd.setCursor(0,1);
  lcd.print("Xin chao cac ban");
  Serial.print("Connecting to WiFi");
  // pinMode(2, OUTPUT);

  while (!Serial);  // For Yun/Leo/Micro/Zero/...
  delay(100);
  Serial.print("\n\nAdafruit Fingerprint sensor enrollment");

  // set the data rate for the sensor serial port
  finger.begin(57600);

  if (finger.verifyPassword()) {
    Serial.print("Found fingerprint sensor!");
  } else {
    Serial.print("Did not find fingerprint sensor");
    while (1) { delay(1); }
  }

  Serial.print(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.print(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.print(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.print(finger.capacity);
  Serial.print(F("Security level: ")); Serial.print(finger.security_level);
  Serial.print(F("Device address: ")); Serial.print(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.print(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.print(finger.baud_rate);
  //connect
  while (!Serial);  // For Yun/Leo/Micro/Zero/...
  delay(100);
  Serial.print("\n\nAdafruit finger detect test");

  // set the data rate for the sensor serial port
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    Serial.print("Found fingerprint sensor!");
  } else {
    Serial.print("Did not find fingerprint sensor ");
    while (1) { delay(1); }
  }

  Serial.print(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.print(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.print(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.print(finger.capacity);
  Serial.print(F("Security level: ")); Serial.print(finger.security_level);
  Serial.print(F("Device address: ")); Serial.print(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.print(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.print(finger.baud_rate);

  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor doesn't contain any fingerprint data. Please run the 'enroll' example.");
  }
  else {
    Serial.print("Waiting for valid finger...");
      Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.print(" templates");
  }

  wifiConnect();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive(90);
}

void loop() {
  if(!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();

  temperature();
  air_quality();
  
  // FINGER 
  
  //connect
  if(servo_status == "Mo" && millis()-servo_time>15000){
    servo_close_web();
  }

  int check = getFingerprintID();
  if(check >= 1 && check <= 127){
    servo_open_web();
  }
  else if(check == 254){
    Serial.print("___");
    checkStranger();
  }
  rfid();
  if( servo_status == "Mo"){
    Serial.println(3);
  }
  else if(servo_status == "Dong"){
    Serial.println(4);
  }
}

void checkStranger() {
  Stranger_end = millis();

  if ((Stranger_end - Stranger_begin) >= 60000) {
    warning = 0;
  }
  warning++;

  if (warning == 1) {
    Stranger_begin = millis();
  }
  
  if(warning == count_open){
    digitalWrite(D0, HIGH);
    delay(5000);
    digitalWrite(D0, LOW);
    warning = 0;
    Stranger_web();
    sendRequest();
  }
  else{
    digitalWrite(D0, HIGH);
    delay(500);
    digitalWrite(D0, LOW);
  }
}

bool fingerprint_registration(int id){
  // enroll
  if (id == 0) {// ID #0 not allowed, try again!
     return false;
  }
  Serial.print("Enrolling ID #");
  Serial.print(id);

  int count = 0;
  while (!  getFingerprintEnroll(id) ){
    count++;
    if(count>=3){
      return false;
    }
  }
  return true;
}

void rfid(){
  if (Serial.available() > 0) {
    int id = Serial.parseInt(); // Đọc giá trị số từ cổng serial
    // Bây giờ bạn có thể sử dụng giá trị cảm biến (sensorValue) trong Arduino
    if(id == 1){
      servo_open_web();
    } else if(id == 2){
      Serial.print("___");
      checkStranger();
    }
  }
}

void servo_open(){
  myServo.write(180);
  servo_status = "Mo";
  servo_time = millis();
}

void servo_close(){
  myServo.write(0);
  servo_status = "Dong";
}

void servo_open_web(){
  myServo.write(180);
  servo_status = "Mo";
  servo_time = millis();
  char buffer_ser[servo_status.length() + 1]; // +1 để chứa ký tự kết thúc chuỗi (\0)
  servo_status.toCharArray(buffer_ser, sizeof(buffer_ser));
  mqttClient.publish("21127583/ser", buffer_ser);
}

void servo_close_web(){
  myServo.write(0);
  servo_status = "Dong";
  char buffer_ser[servo_status.length() + 1]; // +1 để chứa ký tự kết thúc chuỗi (\0)
  servo_status.toCharArray(buffer_ser, sizeof(buffer_ser));
  mqttClient.publish("21127583/ser", buffer_ser);
}

void Stranger_web(){
  Stranger_status = count_open;
  char buffer_ser[Stranger_status.length() + 1]; // +1 để chứa ký tự kết thúc chuỗi (\0)
  servo_status.toCharArray(buffer_ser, sizeof(buffer_ser));
  mqttClient.publish("21127583/stranger", buffer_ser);
}

//FINGER enroll
bool getFingerprintEnroll(int id) { 

  int p = -1;
  Serial.print("Waiting for valid finger to enroll as #"); Serial.print(id);
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      Serial.print("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.print(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.print("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.print("Imaging error");
      break;
    default:
      Serial.print("Unknown error");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(1);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.print("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.print("Image too messy");
      return false;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.print("Communication error");
      return false;
    case FINGERPRINT_FEATUREFAIL:
      Serial.print("Could not find fingerprint features");
      return false;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.print("Could not find fingerprint features");
      return false;
    default:
      Serial.print("Unknown error");
      return false;
  }

  Serial.print("Remove finger");
  delay(2000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }
  Serial.print("ID "); Serial.print(id);
  p = -1;
  Serial.print("Place same finger again");
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      Serial.print("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.print(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.print("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.print("Imaging error");
      break;
    default:
      Serial.print("Unknown error");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(2);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.print("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.print("Image too messy");
      return false;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.print("Communication error");
      return false;
    case FINGERPRINT_FEATUREFAIL:
      Serial.print("Could not find fingerprint features");
      return false;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.print("Could not find fingerprint features");
      return false;
    default:
      Serial.print("Unknown error");
      return false;
  }

  // OK converted!
  Serial.print("Creating model for #");  Serial.print(id);

  p = finger.createModel();
  if (p == FINGERPRINT_OK) {
    Serial.print("Prints matched!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.print("Communication error");
    return false;
  } else if (p == FINGERPRINT_ENROLLMISMATCH) {
    Serial.print("Fingerprints did not match");
    return false;
  } else {
    Serial.print("Unknown error");
    return false;
  }

  Serial.print("ID "); Serial.print(id);
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    Serial.print("Stored!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.print("Communication error");
    return false;
  } else if (p == FINGERPRINT_BADLOCATION) {
    Serial.print("Could not store in that location");
    return false;
  } else if (p == FINGERPRINT_FLASHERR) {
    Serial.print("Error writing to flash");
    return false;
  } else {
    Serial.print("Unknown error");
    return false;
  }

  return true;
}

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.print("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.print("No finger detected");
      return 253;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.print("Communication error");
      return 253;
    case FINGERPRINT_IMAGEFAIL:
      Serial.print("Imaging error");
      return 254;
    default:
      Serial.print("Unknown error");
      return 253;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.print("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.print("Image too messy");
      return 254;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.print("Communication error");
      return 254;
    case FINGERPRINT_FEATUREFAIL:
      Serial.print("Could not find fingerprint features");
      return 254;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.print("Could not find fingerprint features");
      return 254;
    default:
      Serial.print("Unknown error");
      return 253;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.print("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.print("Communication error");
    return 254;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.print("Did not find a match");
    return 254;
  } else {
    Serial.print("Unknown error");
    return 253;
  }

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.print(finger.confidence);

  return finger.fingerID;
}
