#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// Broches ESP32
#define SDA_PIN 21
#define SCL_PIN 22
#define SW_PIN 19
#define BUZZER_PIN 2
#define LIGHT_PIN 18
#define BLINKER_SOURCE_PIN 17
#define BLINKER_LIGHT_PIN 4

// Broches de commande du moteur via le L293D
#define L293D_INPUT4_PIN 16
#define L293D_INPUT3_PIN 5

// Adresses I2C
#define ADS7830_ADDR 0x4B
#define LCD_ADDR 0x3F  // Remplacez par 0x27 si c'est l'adresse de votre LCD

// Canaux ADS7830
#define JOYSTICK_X 1
#define JOYSTICK_Y 0
#define LIGHT_SENSOR 2

// Seuils d'hystérésis du capteur de lumière
const int LOW_THRESHOLD = 100;
const int HIGH_THRESHOLD = 150;

// Calibrage du joystick
const int JOYSTICK_CENTER = 128;  // À ajuster selon la valeur au repos
const int JOYSTICK_DEADZONE = 20;

// Réglages PWM du moteur
const int MOTOR_PWM_FREQUENCY = 1000;
const int MOTOR_PWM_RESOLUTION = 8;  // Valeurs PWM de 0 à 255

bool lightOn = true;

// Vitesse : -50 en marche arrière, +100 en marche avant
float vehicleSpeed = 0.0f;
unsigned long lastSpeedUpdate = 0;
unsigned long lastLcdUpdate = 0;

// Clignotant non bloquant
bool blinkerLightState = false;
unsigned long lastBlinkerChange = 0;

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// Flèche vers l'avant
byte forwardArrow[8] = {
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00100,
  B00100,
  B00000
};

// Flèche vers l'arrière
byte reverseArrow[8] = {
  B00000,
  B00100,
  B00100,
  B00100,
  B00100,
  B10101,
  B01110,
  B00100
};

// Lecture d'un canal de l'ADS7830
int ads7830_read_selected_channel(int addr, int channel)
{
  const int readCommandBase = 0x84;

  unsigned int argChannel =
      (((channel << 2) | (channel >> 1)) & 0x07) << 4;

  unsigned char command =
      (unsigned char)((readCommandBase | argChannel) & 0xFF);

  Wire.beginTransmission(addr);
  Wire.write(command);

  if (Wire.endTransmission() != 0) {
    return -1;
  }

  Wire.requestFrom(addr, 1);

  if (!Wire.available()) {
    return -1;
  }

  return Wire.read();
}

// Mise à jour de la lumière avec hystérésis
void updateLight(int lightLevel)
{
  if (lightLevel < 0) {
    return;  // Ignore une lecture invalide
  }

  if (lightLevel < LOW_THRESHOLD) {
    lightOn = true;
  } else if (lightLevel > HIGH_THRESHOLD) {
    lightOn = false;
  }

  digitalWrite(LIGHT_PIN, lightOn ? HIGH : LOW);
}

// Mise à jour de la vitesse selon le joystick
void updateVehicleSpeed(int joystickY)
{
  unsigned long now = millis();

  if (lastSpeedUpdate == 0) {
    lastSpeedUpdate = now;
    return;
  }

  float dt = (now - lastSpeedUpdate) / 1000.0f;
  lastSpeedUpdate = now;

  // Évite un grand changement de vitesse après une pause.
  if (dt > 0.1f) {
    dt = 0.1f;
  }

  float acceleration = 0.0f;
  int offset = joystickY - JOYSTICK_CENTER;

  if (abs(offset) > JOYSTICK_DEADZONE) {
    if (offset > 0) {
      // Accélération avant : maximum 5 unités par seconde.
      int usableOffset = offset - JOYSTICK_DEADZONE;
      float range =
          255.0f - JOYSTICK_CENTER - JOYSTICK_DEADZONE;

      if (range > 0.0f) {
        acceleration = 5.0f * usableOffset / range;
      }
    } else {
      // Accélération arrière : maximum 3 unités par seconde.
      int usableOffset = -offset - JOYSTICK_DEADZONE;
      float range = JOYSTICK_CENTER - JOYSTICK_DEADZONE;

      if (range > 0.0f) {
        acceleration = -3.0f * usableOffset / range;
      }
    }
  }

  // Accélération commandée par le joystick.
  vehicleSpeed += acceleration * dt;

  // Décélération naturelle vers zéro : 1 unité par seconde.
  if (vehicleSpeed > 0.0f) {
    vehicleSpeed -= 1.0f * dt;

    if (vehicleSpeed < 0.0f) {
      vehicleSpeed = 0.0f;
    }
  } else if (vehicleSpeed < 0.0f) {
    vehicleSpeed += 1.0f * dt;

    if (vehicleSpeed > 0.0f) {
      vehicleSpeed = 0.0f;
    }
  }

  // Limites : -50 en reculons, +100 en avant.
  if (vehicleSpeed > 100.0f) {
    vehicleSpeed = 100.0f;
  } else if (vehicleSpeed < -50.0f) {
    vehicleSpeed = -50.0f;
  }
}

// Commande du moteur : PWM sur INPUT 3 ou INPUT 4
void updateMotor()
{
  int pwm = 0;

  if (vehicleSpeed > 0.0f) {
    // Avant : INPUT 3 reçoit le PWM, INPUT 4 reste à LOW.
    pwm = (int)roundf(vehicleSpeed * 255.0f / 100.0f);
    pwm = constrain(pwm, 0, 255);

    ledcWrite(L293D_INPUT3_PIN, pwm);
    ledcWrite(L293D_INPUT4_PIN, 0);
  } else if (vehicleSpeed < 0.0f) {
    // Arrière : INPUT 4 reçoit le PWM, INPUT 3 reste à LOW.
    pwm = (int)roundf(fabsf(vehicleSpeed) * 255.0f / 50.0f);
    pwm = constrain(pwm, 0, 255);

    ledcWrite(L293D_INPUT3_PIN, 0);
    ledcWrite(L293D_INPUT4_PIN, pwm);
  } else {
    // Arrêt : les deux entrées sont à LOW.
    ledcWrite(L293D_INPUT3_PIN, 0);
    ledcWrite(L293D_INPUT4_PIN, 0);
  }
}

// Affichage de la vitesse absolue et de la direction
void updateSpeedLcd()
{
  lcd.setCursor(0, 1);
  lcd.print("Vitesse: ");

  int displayedSpeed = (int)roundf(fabsf(vehicleSpeed));
  lcd.print(displayedSpeed);
  lcd.print("    ");  // Efface les anciens chiffres

  lcd.setCursor(15, 0);

  if (vehicleSpeed < 0.0f) {
    lcd.write(byte(1));  // Flèche arrière
  } else {
    lcd.write(byte(0));  // Flèche avant
  }
}

void setup()
{
  Serial.begin(115200);

  // Initialisation du bus I2C avant l'écran LCD
  Wire.begin(SDA_PIN, SCL_PIN);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.createChar(0, forwardArrow);
  lcd.createChar(1, reverseArrow);

  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(BLINKER_LIGHT_PIN, OUTPUT);

  // Suppose que l'interrupteur relie GPIO 17 à GND lorsqu'il est activé.
  pinMode(BLINKER_SOURCE_PIN, INPUT_PULLUP);

  // Les broches du moteur reçoivent le PWM.
  // Enable 3,4 du L293D doit être relié au +5 V selon l'énoncé.
  ledcAttach(
      L293D_INPUT3_PIN,
      MOTOR_PWM_FREQUENCY,
      MOTOR_PWM_RESOLUTION
  );

  ledcAttach(
      L293D_INPUT4_PIN,
      MOTOR_PWM_FREQUENCY,
      MOTOR_PWM_RESOLUTION
  );

  ledcWrite(L293D_INPUT3_PIN, 0);
  ledcWrite(L293D_INPUT4_PIN, 0);

  digitalWrite(LIGHT_PIN, HIGH);
  digitalWrite(BLINKER_LIGHT_PIN, LOW);

  lastSpeedUpdate = millis();
  lastLcdUpdate = millis();

  Serial.println("Joystick, capteur, buzzer et moteur prêts");
}

void loop()
{
  // Lectures de l'ADS7830
  int x = ads7830_read_selected_channel(ADS7830_ADDR, JOYSTICK_X);
  int y = ads7830_read_selected_channel(ADS7830_ADDR, JOYSTICK_Y);
  int lightLevel =
      ads7830_read_selected_channel(ADS7830_ADDR, LIGHT_SENSOR);

  // Bouton du joystick
  bool pressed = (digitalRead(SW_PIN) == LOW);

  // Avec INPUT_PULLUP, l'interrupteur du clignotant est actif à LOW.
  bool isSwitchedOn = (digitalRead(BLINKER_SOURCE_PIN) == LOW);

  updateLight(lightLevel);

  // Buzzer lorsque le bouton du joystick est appuyé.
  if (pressed) {
    tone(BUZZER_PIN, 2000);
  } else {
    noTone(BUZZER_PIN);
  }

  // Mise à jour de la vitesse si la lecture du joystick est valide.
  if (y >= 0) {
    updateVehicleSpeed(y);
  }

  // Commande du moteur.
  updateMotor();

  // Clignotement sans delay() bloquant : 500 ms allumé, 250 ms éteint.
  unsigned long now = millis();

  if (isSwitchedOn) {
    unsigned long interval = blinkerLightState ? 500 : 250;

    if (now - lastBlinkerChange >= interval) {
      lastBlinkerChange = now;
      blinkerLightState = !blinkerLightState;

      digitalWrite(
          BLINKER_LIGHT_PIN,
          blinkerLightState ? HIGH : LOW
      );
    }
  } else {
    blinkerLightState = false;
    digitalWrite(BLINKER_LIGHT_PIN, LOW);
    lastBlinkerChange = now;
  }

  // LCD rafraîchi 10 fois par seconde.
  if (now - lastLcdUpdate >= 100) {
    lastLcdUpdate = now;
    updateSpeedLcd();
  }

  // Informations de débogage.
  Serial.print("X: ");
  Serial.print(x);

  Serial.print(" | Y: ");
  Serial.print(y);

  Serial.print(" | SW: ");
  Serial.print(pressed ? "PRESSED" : "RELEASED");

  Serial.print(" | Lumiere: ");
  Serial.print(lightLevel);

  Serial.print(" | Lampe: ");
  Serial.print(lightOn ? "ON" : "OFF");

  Serial.print(" | Clignotant: ");
  Serial.print(isSwitchedOn ? "ON" : "OFF");

  Serial.print(" | Vitesse: ");
  Serial.println(vehicleSpeed, 1);

  delay(5);
}