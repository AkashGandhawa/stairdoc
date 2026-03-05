#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_1 0

#define SERVO_MIN 150
#define SERVO_MAX 600

char command;

void setup() {

  Serial.begin(115200);
  SerialBT.begin("ServoTest");

  Serial.println("ESP32 Servo Control Ready");

  Wire.begin(21,22);

  pwm.begin();
  pwm.setPWMFreq(50);

  delay(100);

  setServoAngle(SERVO_1,0);
}

void loop() {

  if (SerialBT.available()) {

    command = SerialBT.read();

    Serial.print("Command received: ");
    Serial.println(command);

    switch(command){

      case 'u':
        Serial.println("Moving UP");
        sweepServo(SERVO_1,0,180);
      break;

      case 'd':
        Serial.println("Moving DOWN");
        sweepServo(SERVO_1,180,0);
      break;

    }

  }
}

void setServoAngle(int servo,int angle){

  angle = constrain(angle,0,180);

  int pulse = map(angle,0,180,SERVO_MIN,SERVO_MAX);

  pwm.setPWM(servo,0,pulse);
}

void sweepServo(int servo,int startAngle,int endAngle){

  if(startAngle < endAngle){

    for(int pos=startAngle; pos<=endAngle; pos+=5){

      setServoAngle(servo,pos);
      delay(40);
    }

  }
  else{

    for(int pos=startAngle; pos>=endAngle; pos-=5){

      setServoAngle(servo,pos);
      delay(40);
    }

  }

}
