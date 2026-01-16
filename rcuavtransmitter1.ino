#include <HardwareSerial.h>
HardwareSerial LoRa(1);

#define TX_PIN 25
#define RX_PIN 26

struct __attribute__((packed)) ServoValues {
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

  if (Serial.available() >= (int)sizeof(ServoValues)) {
    size_t recieve = Serial.readBytes((char*)&servoVals, sizeof(ServoValues));
    if(recieve == sizeof(ServoValues)) {
      int throttle = constrain(servoVals.throttle, 0, 180);
      int A_ang = constrain(servoVals.A_ang, 0, 180);
      int elev_ang = constrain(servoVals.elev_ang, 0, 180);
      int rud_ang = constrain(servoVals.rud_ang, 0, 180);
      int fpvH_ang = constrain(servoVals.fpvH_ang, 0, 180);
      int fpvV_ang = constrain(servoVals.fpvV_ang, 0, 180);
      int thr_arm = servoVals.thr_arm;
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
