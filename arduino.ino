#include "SPI.h" // SPI library
#include "MFRC522.h" // RFID library (https://github.com/miguelbalboa/rfid)

const int pinRST = 5;
const int pinSDA = 10;
const int pinLed = 4;

MFRC522 mfrc522(pinSDA, pinRST); // Set up mfrc522 on the Arduino

void setup() {
  SPI.begin(); // open SPI connection
  mfrc522.PCD_Init(); // Initialize Proximity Coupling Device (PCD)
  Serial.begin(9600); // open serial connection
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, LOW);
}
// 1269511016,2231869837
void loop() {
  if (mfrc522.PICC_IsNewCardPresent()) { // (true, if RFID tag/card is present ) PICC = Proximity Integrated Circuit Card
    if(mfrc522.PICC_ReadCardSerial()) { // true, if RFID tag/card was 
      String s = "";
      for (byte i = 0; i < mfrc522.uid.size; ++i) { // read id (in parts)
        s += mfrc522.uid.uidByte[i];
      }
      if(s == "1269511016" || s == "2231869837"){
        Serial.println(1);
        delay(5000);
      }
      else{
        Serial.println(2);
        delay(5000);
      }
    }
  }
  if (Serial.available() > 0) {
    int status = Serial.parseInt(); // Đọc giá trị số từ cổng serial
    // Bây giờ bạn có thể sử dụng giá trị cảm biến (sensorValue) trong Arduino
    if(status == 3){
      digitalWrite(pinLed, HIGH);
    } else if(status == 4){
      digitalWrite(pinLed, LOW);
    }
  }
}