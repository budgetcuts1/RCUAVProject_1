#include <HardwareSerial.h>
HardwareSerial LoRa(1);

#define TX_PIN 25
#define RX_PIN 26

struct ServoValues {
  uint8_t throttle;
  uint8_t A_ang;
  uint8_t elev_ang;
  uint8_t rud_ang;
  uint8_t fpvH_ang;
  uint8_t fpvV_ang;
  uint8_t thr_arm;
};

ServoValues servoVals = { 0, 90, 90, 90, 90, 90, 0};

unsigned long lastServoSend = 0;
const unsigned long servoInterval = 50;

void setup() {
  Serial.begin(115200);
  LoRa.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  delay(500);

  LoRa.println("AT+ADDRESS=1");
  delay(100);
  LoRa.println("AT+NETWORKID=5");
  delay(100);
  LoRa.println("AT+PARAMETER=7,7,1,4"); // SF7, BW125kHz, CR4/5
  delay(100);

  Serial.println("Transmitter ready");
}

void loop() {
  unsigned long currentMillis = millis();

  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);
    int thirdComma = data.indexOf(',', secondComma + 1);
    int fourthComma = data.indexOf(',', thirdComma + 1);
    int fifthComma = data.indexOf(',', fourthComma + 1);
    int sixthComma = data.indexOf(',', fifthComma + 1);

    if (firstComma > 0 && secondComma > firstComma && thirdComma > secondComma && fourthComma > thirdComma && fifthComma > fourthComma && sixthComma > fifthComma) {
      int throttle = data.substring(0, firstComma).toInt();
      int A_ang = data.substring(firstComma + 1, secondComma).toInt();
      int elev_ang = data.substring(secondComma + 1, thirdComma).toInt();
      int rud_ang = data.substring(thirdComma + 1, fourthComma).toInt();
      int fpvH_ang = data.substring(fourthComma + 1, fifthComma).toInt();
      int fpvV_ang = data.substring(fifthComma + 1, sixthComma).toInt();
      int thr_arm = data.substring(sixthComma + 1).toInt();

      throttle = constrain(throttle, 0, 180);
      A_ang = constrain(A_ang, 0, 180);
      elev_ang = constrain(elev_ang, 0, 180);
      rud_ang = constrain(rud_ang, 0, 180);
      fpvH_ang = constrain(fpvH_ang, 0, 180);
      fpvV_ang = constrain(fpvV_ang, 0, 180);

      servoVals.throttle = throttle;
      servoVals.A_ang = A_ang;
      servoVals.elev_ang = elev_ang;
      servoVals.rud_ang = rud_ang;
      servoVals.fpvH_ang = fpvH_ang;
      servoVals.fpvV_ang = fpvV_ang;
      servoVals.thr_arm = thr_arm;
    }

  if (currentMillis - lastServoSend >= servoInterval) {
    lastServoSend = currentMillis;
    String payload = String(servoVals.throttle) + "," + String(servoVals.A_ang) + ","
    + String(servoVals.elev_ang) + "," + String(servoVals.rud_ang) + "," + String(servoVals.fpvH_ang) 
    + "," + String(servoVals.fpvV_ang) + "," + String(servoVals.thr_arm);

    String cmd = "AT+SEND=2," + String(payload.length()) + "," + payload;
    LoRa.println(cmd);
    Serial.println("Servo data sent: " + payload);
    }
  }
}