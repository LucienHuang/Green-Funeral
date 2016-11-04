#include <Servo.h>

Servo myServo;
int servoSpeed = 90;

int magPinA = 2;
int magPinB = 4;
int magPinC = 7;

int sensorA = 0;
int sensorB = 0;
int sensorC = 0;

int ledVal0, ledVal1, ledVal2;
int ledA, ledB;

int ledPin[3] = {3, 5, 6};
int ledPin1 = 8;
int ledPin2 = 9;

int waitTime[7] = {50, 200, 50, 20, 20, 50, 500};
long unsigned lastBlinkTime = 0;
int n = 0;

bool B = false;
bool C = false;

int echoPin = 12;
int trigPin = 11;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(magPinA, INPUT);
  pinMode(magPinB, INPUT);
  pinMode(magPinC, INPUT);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  for (int i = 0; i < 3; i++) {
    pinMode(ledPin[i], OUTPUT);
  }
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myServo.attach(13);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorA = digitalRead(magPinA);
  sensorB = digitalRead(magPinB);
  sensorC = digitalRead(magPinC);

  //  Serial.print(sensorA);
  //  Serial.print('\t');
  //  Serial.print(sensorB);
  //  Serial.print('\t');
  //  Serial.println(sensorC);

  if (sensorA && !sensorB && !sensorC) {
    Serial.println('A');
    myServo.detach();
    B = false;
    C = false;

    float amplitude = 120;
    float frequency = 0.05;
    //radians把180°转化为π
    float T = radians(millis());
    float baseline = 120;

    ledVal0 = amplitude * sin(frequency * T) + baseline;
    ledVal1 = amplitude * sin(frequency * (T + 90)) + baseline;
    ledVal2 = amplitude * sin(frequency * (T + 180)) + baseline;
    ledA = 1;
    ledB = 1;

    analogWrite(ledPin[0], ledVal0);
    analogWrite(ledPin[1], ledVal1);
    analogWrite(ledPin[2], ledVal2);
    digitalWrite(ledPin1, ledA);
    digitalWrite(ledPin2, ledB);

  } else if (sensorB && !sensorA && !sensorC) {
    Serial.println('B');
    C = false;

    if (B == false) {
      myServo.attach(13); B = true;
    }
    Serial.print(ledA);
    Serial.print('\t');
    myServo.write(servoSpeed);

    float amplitude = 120;
    float frequency = 1;
    //radians把180°转化为π
    float T = radians(millis());
    float baseline = 120;

    servoSpeed = 2 * sin(T) + 90;
    ledVal0 = amplitude * sin(frequency * T) + baseline;
    ledVal1 = amplitude * sin(frequency * (T + 90)) + baseline;
    ledVal2 = amplitude * sin(frequency * (T + 180)) + baseline;
    if (millis() - lastBlinkTime > waitTime[n]) {
      lastBlinkTime = millis();
      ledB = ledA;
      ledA = !ledA;
      n++;
      if (n > 6) n = 0;
    }

    analogWrite(ledPin[0], ledVal0);
    analogWrite(ledPin[1], ledVal1);
    analogWrite(ledPin[2], ledVal2);
    digitalWrite(ledPin1, ledA);
    digitalWrite(ledPin2, ledB);
  } else if (sensorC && !sensorB && !sensorA) {
    //    Serial.println('C');
    if (C == false) {
      myServo.attach(13); C = true;
    }
    //Ultrasonic
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(trigPin, LOW);
    int duration = pulseIn(echoPin, HIGH);
    int distance = duration / 58.2;
      Serial.println(distance);
    if (distance < 10) {
      C = false;
      int servoSpeed = map(distance, 2, 10, 0, 90);
      Serial.println(servoSpeed);
      myServo.write(servoSpeed);
    } else {
      C = true;
      myServo.detach();
    }
    analogWrite(ledPin[0], random(0, 255));
    analogWrite(ledPin[1], random(0, 255));
    analogWrite(ledPin[2], random(0, 255));
    digitalWrite(ledPin1, ledA);
    digitalWrite(ledPin2, ledB);
    delay(20);
    ledA = !ledA;
    ledB = !ledB;

  } else {
    B = false;
    C = false;
    myServo.detach();
    Serial.print(sensorA);
    Serial.print('\t');
    Serial.print(sensorB);
    Serial.print('\t');
    Serial.println(sensorC);

    analogWrite(ledPin[0], 0);
    analogWrite(ledPin[1], 0);
    analogWrite(ledPin[2], 0);
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
  }
}
