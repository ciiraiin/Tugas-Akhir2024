#include <Wire.h>
#include <HMC5883L.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GPS.h>
#include <TinyGPS++.h>
#include <NewPing.h>

// Komponen LCD dan Kompas
LiquidCrystal_I2C lcd(0x27, 16, 2);
HMC5883L compass;
Adafruit_GPS GPS(&Serial3);
TinyGPSPlus gps;

// Pin untuk sensor encoder
const int encoderPin1 = 16;
const int encoderPin2 = 17;
float distanceTravelled = 0;
float jaraktempuh = 0;

// Pin untuk sensor ultrasonik
const int trigFrontLeft = A10;
const int echoFrontLeft = A11;
const int trigFrontRight = A12;
const int echoFrontRight = A13;
const int trigLeft = A6;
const int echoLeft = A7;
const int trigRight = A0;
const int echoRight = A1;

int Start = 31;        // Tombol hijau
int Stop = 29;         // Tombol Kuning
bool tbStart = LOW;
bool tbStop = LOW;
bool kondisirobot = false;

// LED indikator
int ledPins[3] = {38, 40, 42}; //Merah, Biru Hijau

#define stopwp 3 //detik
float currentLat, currentLong;
String cekgps = "";

// Target Waypoint
#define MAX_TARGETS 4
int targetAngles[MAX_TARGETS];
float targetDistances[MAX_TARGETS];
int currentTargetIndex = 0; // Indeks target aktif
String inputData = "";
bool target = 1;

// Variabel untuk pengukuran encoder
int encoderCount1 = 0;
int encoderCount2 = 0;
int encoderCount = 0;
float distancePerTick = 0.05;

// Variabel status gerakan
bool isMovingForward = false;
bool isTurningLeft = false;
bool isTurningRight = false;
bool isMovingBackward = false;
bool isRotatingLeft = false;
bool isRotatingRight = false;
bool isSwipeLeft = false;
bool isSwipeRight = false;

// Timer variables
unsigned long previousCompassMillis = 0;
unsigned long previousEncoderMillis = 0;
unsigned long previousUltrasonicMillis = 0;
unsigned long previousBluetoothMillis = 0;
unsigned long compassInterval = 50;
unsigned long encoderInterval = 10;
unsigned long ultrasonicInterval = 100;
unsigned long bluetoothInterval = 200;

int currentAngle = 0;
bool obstacleDetected[4] = {false, false, false, false};
bool avoidingObstacle = false;
bool halangan = 0;

// Variabel untuk penghindaran halangan
unsigned long avoidStartMillis = 0;
int avoidStep = 0;

bool parseInput(String data) {
data.trim(); // Menghapus spasi atau karakter ekstra
int commaIndex = 0;
  for (int i = 0; i < MAX_TARGETS; i++) {
  // Ekstrak sudut
    commaIndex = data.indexOf(',');
    if (commaIndex == -1) return false; // Tidak ada koma
    targetAngles[i] = data.substring(0, commaIndex).toInt();
    data = data.substring(commaIndex + 1);

    // Ekstrak jarak
    commaIndex = data.indexOf(';');
    if (i == MAX_TARGETS - 1) { 
    // Target terakhir tidak memerlukan koma
    targetDistances[i] = data.toFloat();
    } else {
    if (commaIndex == -1) return false; // Tidak ada koma
      targetDistances[i] = data.substring(0, commaIndex).toFloat();
      data = data.substring(commaIndex + 1);
    }
  } return true;
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);  // Bluetooth     
  Serial3.begin(9600);  // GPS
  lcd.init();
  lcd.backlight();
  setup_motor();

  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(Start, INPUT);
  pinMode(Stop, INPUT);

  for (int i = 0; i < 3; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
  digitalWrite(ledPins[0], HIGH);
  digitalWrite(ledPins[1], HIGH);
  digitalWrite(ledPins[2], HIGH);

  setupUltrasonic(trigFrontLeft, echoFrontLeft);
  setupUltrasonic(trigFrontRight, echoFrontRight);
  setupUltrasonic(trigLeft, echoLeft);
  setupUltrasonic(trigRight, echoRight);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Mobile Robot");
  lcd.setCursor(2,1);
  lcd.print("Waypoint GPS");
  delay(2000);

  GPS.begin(9600);                               // 9600 NMEA default speed
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // turns on RMC and GGA (fix data)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // 1 Hz update rate
  GPS.sendCommand(PGCMD_NOANTENNA);              // turn off antenna status info
  delay(1000);

  Wire.begin();
  compass = HMC5883L();
  if (compass.begin()) {
    Serial.println("Kompas berhasil diinisialisasi!");
  } else {
    Serial.println("Kompas tidak terdeteksi.");
    while (1);
  }
  updateCompass();
  digitalWrite(ledPins[0], LOW);
  digitalWrite(ledPins[1], LOW);
  digitalWrite(ledPins[2], LOW);
  Serial.println("Start");
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("Heading : ");
  lcd.print(currentAngle);
  lcd.setCursor(1,1);
  lcd.print("Input Target!");
  
  while (true) {
  if (Serial1.available()) {
    inputData = Serial1.readStringUntil('\n'); // Membaca data hingga newline
    if (parseInput(inputData)) {
      Serial.println("Data diterima:");
      for (int i = 0; i < MAX_TARGETS; i++) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Wp1 : ");
        lcd.print(targetAngles[0]);  lcd.print(",");
        lcd.print(targetDistances[0]);
        lcd.setCursor(0,1);
        lcd.print("Wp2 : ");
        lcd.print(targetAngles[1]);  lcd.print(",");
        lcd.print(targetDistances[1]);
        delay(700);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Wp3 : ");
        lcd.print(targetAngles[2]);  lcd.print(",");
        lcd.print(targetDistances[2]);
        lcd.setCursor(0,1);
        lcd.print("Wp4 : ");
        lcd.print(targetAngles[3]);  lcd.print(",");
        lcd.print(targetDistances[3]);
        delay(700);
      }
      break;
      } else {
        Serial.println("Format data salah");
      }
    }
  }
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("< MULAI >");
  delay(1500);
}

void loop() {
  tbStart = digitalRead(Start);
  tbStop = digitalRead(Stop);

  if (tbStart == HIGH) {
    kondisirobot = false;
  }
  if (tbStop == HIGH) {
    kondisirobot = true;
  }
  if (kondisirobot) {
    stop();
  }
  else {
  while (Serial3.available() > 0)
    if (gps.encode(Serial3.read()))
      processGPS();

  if (currentTargetIndex < MAX_TARGETS) {
    float targetAngle = targetAngles[currentTargetIndex];
    float targetDistance = targetDistances[currentTargetIndex];
    
    if (!isFacingTarget(currentAngle, targetAngle)) {
    halangan = 0;
    digitalWrite(ledPins[0], LOW);
    digitalWrite(ledPins[1], LOW);
    digitalWrite(ledPins[2], HIGH);
    rotateToTargetAngle(targetAngle);
    }
    else if (encoderCount*distancePerTick < targetDistances[currentTargetIndex]) {
    halangan = 0;
    digitalWrite(ledPins[0], LOW);
    digitalWrite(ledPins[1], HIGH);
    digitalWrite(ledPins[2], HIGH);
    moveToTargetWithCorrection(targetDistance, targetAngle);
    }
    else {
    encoderCount1 = 0;
    encoderCount2 = 0;
    encoderCount = 0;
    currentTargetIndex++;
    digitalWrite(ledPins[0], LOW);
    digitalWrite(ledPins[1], LOW);
    digitalWrite(ledPins[2], LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Target");
    lcd.print(currentTargetIndex);
    lcd.print(" Tercapai");
    stop();
    delay(stopwp*1000);
    }
  }
  else {  // Setelah semua target tercapai, hentikan robot
    stop(); 
    digitalWrite(ledPins[0], HIGH);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Target Tercapai");
    delay(1000);
    lcd.clear();
    lcd.setCursor(5,0);
    lcd.print("FINISH");
    while (1); // Robot berhenti di akhir
    }
  }
}

void rotateToTargetAngle(float targetAngle) {
  while (true) {
    updateCompass();

    float angleError = targetAngle - currentAngle;
    if (angleError > 180) angleError -= 360;
    if (angleError < -180) angleError += 360;

    if (abs(angleError) < 3) {
      stop();
      break;
    }

    if (angleError > 0) {
      putarkanan();
    } else {
      putarkiri();
    }
    LCD();
    sendBluetoothData();
  }
}

void moveToTargetWithCorrection(float targetDistance, float targetAngle) {
  encoderCount = 0;
  distanceTravelled = 0;

  while (distanceTravelled < targetDistance) {
    updateUltrasonic();

     if (checkObstacle()) {
      avoidObstacle(); // Jalankan logika penghindaran
      halangan = 1;
      if (!avoidingObstacle) {
        resetObstacles();
        halangan = 0;
      }
      continue;
    }

    updateCompass();
    updateEncoder();

    float angleError = targetAngle - currentAngle;
    if (angleError > 180) angleError -= 360;
    if (angleError < -180) angleError += 360;

    // Koreksi arah robot jika melenceng
    if (abs(angleError) > 2) {
      if (angleError > 0) {
        belokkanan();
      } else {
        belokkiri();
      }
    } else {
      maju();
    }
    distanceTravelled = encoderCount*distancePerTick;
    jaraktempuh = targetDistances[currentTargetIndex]-distanceTravelled;
    LCD();
    sendBluetoothData();
  }
  stop();
}

bool isFacingTarget(float currentAngle, float targetAngle) {
  int error = targetAngle - currentAngle;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  return abs(error) <= 2;
}

void processGPS() {
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLong = gps.location.lng();
    Serial.print(F("Latitude: "));
    Serial.println(currentLat,5);
    Serial.print(F("Langitude: "));
    Serial.println(currentLong,5);
    cekgps = "<#>";
  } else {
    Serial.println(F("Location: INVALID"));
    cekgps = "<?>";
  }
}

void updateEncoder() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousEncoderMillis >= encoderInterval) {
    previousEncoderMillis = currentMillis;

    static int lastEncoderState1 = LOW;
    static int lastEncoderState2 = LOW;
    int currentEncoderState1 = digitalRead(encoderPin1);
    int currentEncoderState2 = digitalRead(encoderPin2);

    if (currentEncoderState1 == HIGH && lastEncoderState1 == LOW || currentEncoderState2 == HIGH && lastEncoderState2 == LOW) {
      if (isTurningLeft || isTurningRight || isMovingForward) {
         encoderCount1++;
         encoderCount2++;
      }
      else if (isMovingBackward) {
         encoderCount1--;
         encoderCount2--;
      }
      encoderCount = (encoderCount1+encoderCount2)/2;
      Serial.print("Pulsa : ");
      Serial.println(encoderCount);
    }
    lastEncoderState1 = currentEncoderState1;
    lastEncoderState2 = currentEncoderState2;
  }
}

void updateCompass() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousCompassMillis >= compassInterval) {
    previousCompassMillis = currentMillis;

    Vector norm = compass.readNormalize();
    currentAngle = atan2(norm.YAxis, norm.XAxis) * 180 / PI;
    if (currentAngle < 0) currentAngle += 360;
    Serial.print("Angle : ");
    Serial.println(currentAngle);
  }
}

void setupUltrasonic(int trig, int echo) {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

float readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH);
  return duration * 0.034 / 2;
}

void updateUltrasonic() {
  obstacleDetected[0] = readUltrasonic(trigFrontLeft, echoFrontLeft) < 10;
  obstacleDetected[1] = readUltrasonic(trigFrontRight, echoFrontRight) < 10;
  obstacleDetected[2] = readUltrasonic(trigLeft, echoLeft) < 10;
  obstacleDetected[3] = readUltrasonic(trigRight, echoRight) < 10; 
}

bool checkObstacle() {
  for (int i = 0; i < 4; i++) {
    if (obstacleDetected[i]) return true;
  }
  return false;
}

void resetObstacles() {
  for (int i = 0; i < 4; i++) {
    obstacleDetected[i] = false;
  }
}

void avoidObstacle() {
  if (obstacleDetected[0]) {
    halangan = 1;
    geserkiri();
    delay(900);
  } else if (obstacleDetected[1]) {
    geserkanan();
    halangan = 1;
    delay(900);
  } else if (obstacleDetected[0] || obstacleDetected[1]) { // Depan
    mundur();
    halangan = 1;
    delay(200);
  } else if (obstacleDetected[2]) { // Kiri
    halangan = 1;
    geserkanan();
    delay(200);
  } else if (obstacleDetected[3]) { // Kanan
    halangan = 1;
    geserkiri();
    delay(200);
  }
}

void LCD() {
  static unsigned long LCDMillis = 0;
  unsigned long currentLCD = millis();
  if (currentLCD - LCDMillis >= 100) {
  LCDMillis = currentLCD;
  updateCompass();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Wp");
  lcd.print(currentTargetIndex+1);
  lcd.setCursor(4,0);
  lcd.print("Ch:");
  lcd.print(currentAngle);
  lcd.setCursor(0,1);
  lcd.print(cekgps);
  lcd.setCursor(4,1);
  lcd.print("Th:");
  lcd.print(targetAngles[currentTargetIndex]);
  lcd.setCursor(11,0);
  lcd.print("e");
  lcd.print(targetAngles[currentTargetIndex]-currentAngle);
  lcd.setCursor(11,1);
  lcd.print("d:");
  lcd.print(jaraktempuh);
  }
}

void sendBluetoothData() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousBluetoothMillis >= bluetoothInterval) {
    previousBluetoothMillis = currentMillis;
    updateCompass();
    Serial1.print(currentTargetIndex+1);
    Serial1.print(";");
    Serial1.print(currentLat);
    Serial1.print(";");
    Serial1.print(currentLong);
    Serial1.print(";");
    Serial1.print(targetAngles[currentTargetIndex]);
    Serial1.print(";");
    Serial1.print(currentAngle);
    Serial1.print(";");
    Serial1.print(targetAngles[currentTargetIndex]-currentAngle);
    Serial1.print(";");
    Serial1.print(jaraktempuh);
    Serial1.print(";");
    Serial1.print(halangan);
    Serial1.println(";");

    //Pengujian Sensor GPS
    // Serial1.print("Lat  : ");
    // Serial1.println(currentLat);
    // Serial1.print("Long : ");
    // Serial1.println(currentLong);

    //Pengujian Durasi
    // Serial1.print("Wp ");
    // Serial1.print(currentTargetIndex+1);
    // Serial1.print(" Ch ");
    // Serial1.print(currentAngle);
    // Serial1.print(" Th ");
    // Serial1.print(targetAngles[currentTargetIndex]);
    // Serial1.print(" Er ");
    // Serial1.print(targetAngles[currentTargetIndex]-currentAngle);
    // Serial1.print(" Ds ");
    // Serial1.println(jaraktempuh);
  }
}