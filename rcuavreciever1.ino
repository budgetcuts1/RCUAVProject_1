#include <HardwareSerial.h>
#include <Wire.h>
#include <ESP32Servo.h>
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

ServoValues servoVals = { 0, 90, 90, 90, 90, 90, 0 };

Servo esc;

Servo leftAileron;
Servo rightAileron;

Servo elevator;
Servo rudder1;
Servo rudder2;

Servo fpvHorizontal;
Servo fpvVertical;

Servo frontWheel;

const int throttlePin = 19;
const int LaileronPin = 18;
const int RaileronPin = 4;
const int elevatorPin = 17;
const int rudder1Pin = 5;
const int rudder2Pin = 13;
const int fpvHPin = 21;
const int fpvVPin = 22;
const int frontWheelPin = 33;

bool started = false;

bool armed_throttle = false;
bool prevSignal = false;

void setup() {
  Serial.begin(115200);
  LoRa.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  esc.attach(throttlePin);
  leftAileron.attach(LaileronPin);
  rightAileron.attach(RaileronPin);
  elevator.attach(elevatorPin);
  rudder1.attach(rudder1Pin);
  rudder2.attach(rudder2Pin);
  fpvHorizontal.attach(fpvHPin);
  fpvVertical.attach(fpvVPin);
  frontWheel.attach(frontWheelPin);


  delay(500);

  LoRa.println("AT+ADDRESS=2");
  delay(100);
  LoRa.println("AT+NETWORKID=5");
  delay(100);
  LoRa.println("AT+PARAMETER=7,7,1,4");  // SF7, BW125kHz, CR4/5

  Serial.println("Receiver ready");
}

void loop() {
  while (LoRa.available()) {
    String line = LoRa.readStringUntil('\n');
    line.trim();
    if(line.length() == 0) continue;

    Serial.println("RAW: " + line); // debug raw input

    if (line.startsWith("+RCV=")) {
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);
       if (secondComma > 0) {
        String payload = line.substring(secondComma + 1);
        payload.trim();

        int values[7];
        int start = 0;

        for (int i = 0; i < 7; i++) {
          int comma = payload.indexOf(',', start);
          if (comma == -1) comma = payload.length();
          values[i] = payload.substring(start, comma).toInt();
          start = comma + 1;
        }
        servoVals.throttle = values[0];
        servoVals.A_ang = values[1];
        servoVals.elev_ang = values[2];
        servoVals.rud_ang = values[3];
        servoVals.fpvH_ang = values[4];
        servoVals.fpvV_ang = values[5];
        servoVals.thr_arm = values[6];
      }
    }
  }

  if (!started) {
    if (servoVals.throttle > 10) {
      esc.writeMicroseconds(1000);
      Serial.println("Throttle too high at start!");
      return;
    } else if (servoVals.throttle <= 10) {
      started = true;
      Serial.println("Throttle has reached safe level before start");
    }
  }

  char status[32];

  if (servoVals.thr_arm == 1 && !prevSignal) {
    armed_throttle = !armed_throttle;
    Serial.print("Armed state:");
    Serial.println(armed_throttle ? "ARMED" : "DISARMED");
  }

  prevSignal = (servoVals.thr_arm == 1);


  if (armed_throttle) {
    int throttlePulse = map(servoVals.throttle, 0, 180, 1000, 2000);
    esc.writeMicroseconds(throttlePulse);
  } else {
    esc.writeMicroseconds(1000);
  };

  leftAileron.write(constrain(servoVals.A_ang, 0, 180));
  rightAileron.write(constrain(servoVals.A_ang, 0, 180));
  elevator.write(constrain(servoVals.elev_ang, 0, 180));
  rudder1.write(constrain(servoVals.rud_ang, 0, 180));
  rudder2.write(constrain(servoVals.rud_ang, 0, 180));
  fpvHorizontal.write(constrain(servoVals.fpvH_ang, 0, 180));
  fpvVertical.write(constrain(servoVals.fpvV_ang, 0, 180));
  frontWheel.write(constrain(servoVals.rud_ang, 0, 180));
}