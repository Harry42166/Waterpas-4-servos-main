#include <Arduino.h>
#include <Wire.h> // Voor I2C communicatie met MPU6050
#include <MPU6050.h> // De bibliotheek voor de MPU-6050
#include <ESP32Servo.h> // De bibliotheek voor de servo's
#include <Adafruit_Sensor.h> // Voor sensors_event_t definitie

// --- Definieer de GPIO-pins voor de servo's ---
#define SERVO_LF_PIN 13 // Linksvoor
#define SERVO_RF_PIN 12 // Rechtsvoor
#define SERVO_LB_PIN 14 // Linksachter
#define SERVO_RB_PIN 27 // Rechtsachter hallo

// --- MPU6050 Object aanmaken ---
MPU6050 mpu;

// --- Servo Objecten aanmaken ---
Servo servoLF; // Linksvoor
Servo servoRF; // Rechtsvoor
Servo servoLB; // Linksachter
Servo servoRB; // Rechtsachter

// --- Variabelen voor MPU6050 data ---
float pitch = 0; // Kanteling voorover/achterover (X-as)
float roll = 0;  // Kanteling zijwaarts (Y-as)

// --- Basis servo instellingen (aanpasbaar) ---
// Dit zijn de middelste posities van je servo's.
// Experimenteer hiermee om de waterpas stand te vinden.
const int servoCenterAngle = 90; // Typische middenstand van een servo (0-180 graden)
const int servoMinAngle = 45;    // Minimale hoek die een servo kan aannemen
const int servoMaxAngle = 135;   // Maximale hoek die een servo kan aannemen

// --- Gevoeligheid van de correctie (Proportionele Gain) ---
// Een hogere waarde = snellere/agressievere correctie.
// Een lagere waarde = langzamere/stabielere correctie.
// Begin met een kleine waarde en verhoog indien nodig.
const float Kp_pitch = 1.0; // Proportionele gain voor pitch correctie
const float Kp_roll = 1.0;  // Proportionele gain voor roll correctie

// --- Functie om servo hoek te beperken ---
// Zorgt ervoor dat de servo niet buiten zijn bereik beweegt.
int constrainServoAngle(int angle) {
  return constrain(angle, servoMinAngle, servoMaxAngle);
}

void setup() {
  // --- Seriële communicatie starten voor debugging ---
  Serial.begin(115200);
  while (!Serial); // Wacht tot seriële poort klaar is (niet altijd nodig op ESP32)
  Serial.println("Start initialisatie...");

  // --- MPU6050 initialiseren ---
  Wire.begin(); // Start I2C communicatie
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 niet gevonden! Controleer aansluitingen.");
    while (1); // Stop het programma als MPU6050 niet reageert
  }
  Serial.println("MPU6050 succesvol geïnitialiseerd.");

  // Optioneel: Kalibreer de MPU6050. Dit kan even duren (15 seconden)
  // en moet gedaan worden als het platform helemaal stil ligt.
  // mpu.calibrateGyro();
  // mpu.setAccelerometerRange(MPU6050::ACCEL_RANGE_8G);
  // mpu.setGyroRange(MPU6050::GYRO_RANGE_500DEG);

  // --- Servo's koppelen aan pins ---
  servoLF.attach(SERVO_LF_PIN);
  servoRF.attach(SERVO_RF_PIN);
  servoLB.attach(SERVO_LB_PIN);
  servoRB.attach(SERVO_RB_PIN);

  // --- Initialiseer servo's naar de middenstand ---
  servoLF.write(servoCenterAngle);
  servoRF.write(servoCenterAngle);
  servoLB.write(servoCenterAngle);
  servoRB.write(servoCenterAngle);
  delay(1000); // Geef servo's even tijd om te bewegen
  Serial.println("Servo's geïnitialiseerd naar middenstand.");
  Serial.println("Klaar!");
}

void loop() {
  // --- MPU6050 data uitlezen ---
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // Bereken pitch en roll uit de versnellingsmeter data
  // Dit is een simpele benadering, zonder filtering kan dit ruis bevatten.
  // Voor een stabielere meting is een Kalman filter of complementair filter nodig.
  pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180 / PI;
  roll = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;

  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.println(roll);

  // --- Bereken de benodigde correcties voor elke servo ---
  // Dit is de kern van de regeling. We corrigeren op basis van de gemeten pitch en roll.
  // Hoe dit precies werkt, hangt af van de fysieke opstelling van je servo's.
  // Dit voorbeeld gaat ervan uit dat:
  // - Positieve pitch = platform kantelt voorover (voorkant omlaag)
  // - Positieve roll = platform kantelt naar rechts (rechterkant omlaag)

  // Aanpassing voor elke servo
  float correctionLF = (-pitch * Kp_pitch) + (-roll * Kp_roll); // Linksvoor: reageert op zowel pitch als roll
  float correctionRF = (-pitch * Kp_pitch) + (roll * Kp_roll);  // Rechtsvoor: reageert op pitch en tegengestelde roll
  float correctionLB = (pitch * Kp_pitch) + (-roll * Kp_roll);  // Linksachter: reageert op tegengestelde pitch en roll
  float correctionRB = (pitch * Kp_pitch) + (roll * Kp_roll);   // Rechtsachter: reageert op tegengestelde pitch en tegengestelde roll

  // Nieuwe servo hoeken berekenen
  int angleLF = constrainServoAngle(servoCenterAngle + (int)correctionLF);
  int angleRF = constrainServoAngle(servoCenterAngle + (int)correctionRF);
  int angleLB = constrainServoAngle(servoCenterAngle + (int)correctionLB);
  int angleRB = constrainServoAngle(servoCenterAngle + (int)correctionRB);

  // Stuur servo's aan
  servoLF.write(angleLF);
  servoRF.write(angleRF);
  servoLB.write(angleLB);
  servoRB.write(angleRB);

  // Optioneel: print de servo hoeken voor debugging
  Serial.print("Servo LF: "); Serial.print(angleLF);
  Serial.print(" RF: "); Serial.print(angleRF);
  Serial.print(" LB: "); Serial.print(angleLB);
  Serial.print(" RB: "); Serial.println(angleRB);

  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  const unsigned long interval = 50; // 50 ms interval

  if (now - lastUpdate < interval) {
    return;
  }
  lastUpdate = now;
}