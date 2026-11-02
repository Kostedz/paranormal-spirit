#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <math.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define SW_PIN 19
#define BUZZER_PIN 2
#define BLINKER_SOURCE_PIN 17
#define BLINKER_LIGHT_PIN 4
#define LIGHT_PIN 18

#define L293D_INPUT4_PIN 16
#define L293D_INPUT3_PIN 5

#define SERVO_PIN 23

#define ADS7830_ADDR 0x4B
#define LCD_ADDR 0x3F

#define JOYSTICK_X 1
#define JOYSTICK_Y 0
#define LIGHT_SENSOR 2  

const int LOW_THRESHOLD = 100;
const int HIGH_THRESHOLD = 150;

const int JOYSTICK_CENTER = 128;
const int JOYSTICK_DEADZONE = 20;

const int MOTOR_PWM_FREQUENCY = 1000;
const int MOTOR_PWM_RESOLUTION = 8;

const unsigned long SERVO_UPDATE_INTERVAL = 50;
const unsigned long LCD_UPDATE_INTERVAL = 100;
const unsigned long DEBUG_INTERVAL = 500;

bool lightOn = true;
bool servoAttached = false;

float vehicleSpeed = 0.0f;

unsigned long lastSpeedUpdate = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastServoUpdate = 0;
unsigned long lastDebugUpdate = 0;

bool blinkerLightState = false;
unsigned long lastBlinkerChange = 0;

int lastServoAngle = -1;

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
Servo steeringServo;

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

byte lightbulb[8] = {
  B00100,
  B10101,
  B01110,
  B10001,
  B10101,
  B01110,
  B00100,
  B01110
};

int ads7830_read_selected_channel(int addr, int channel)
{
  const int readCommandBase = 0x84;

  unsigned int argChannel =
    (((channel << 2) | (channel >> 1)) & 0x07) << 4;

  uint8_t command =
    (uint8_t)(readCommandBase | argChannel);

  Wire.beginTransmission(addr);
  Wire.write(command);

  if (Wire.endTransmission() != 0) {
    return -1;
  }

  if (Wire.requestFrom(addr, 1) != 1 || !Wire.available()) {
    return -1;
  }

  return Wire.read();
}

void updateLight(int lightLevel)
{
  if (lightLevel < 0) {
    return;
  }

  if (lightLevel < LOW_THRESHOLD) {
    lightOn = true;
  }
  else if (lightLevel > HIGH_THRESHOLD) {
    lightOn = false;
  }

  digitalWrite(LIGHT_PIN, lightOn ? HIGH : LOW);
}

void updateServo(int joystickX)
{
  if (joystickX < 0) {
    Serial.println("Servo update skipped: invalid joystick value.");
    return;
  }

  int angle = map(joystickX, 0, 255, 0, 180);
  angle = constrain(angle, 0, 180);

  steeringServo.write(angle);
  lastServoAngle = angle;
}

void updateVehicleSpeed(int joystickY)
{
  unsigned long now = millis();

  if (lastSpeedUpdate == 0) {
    lastSpeedUpdate = now;
    return;
  }

  float dt = (now - lastSpeedUpdate) / 1000.0f;
  lastSpeedUpdate = now;

  if (dt > 0.1f) {
    dt = 0.1f;
  }

  float acceleration = 0.0f;
  int offset = joystickY - JOYSTICK_CENTER;

  if (abs(offset) > JOYSTICK_DEADZONE) {
    if (offset > 0) {
      int usableOffset = offset - JOYSTICK_DEADZONE;

      float range =
        255.0f - JOYSTICK_CENTER - JOYSTICK_DEADZONE;

      if (range > 0.0f) {
        acceleration = 5.0f * usableOffset / range;
      }
    }
    else {
      int usableOffset =
        -offset - JOYSTICK_DEADZONE;

      float range =
        JOYSTICK_CENTER - JOYSTICK_DEADZONE;

      if (range > 0.0f) {
        acceleration = -3.0f * usableOffset / range;
      }
    }
  }

  vehicleSpeed += acceleration * dt;

  if (vehicleSpeed > 0.0f) {
    vehicleSpeed -= dt;

    if (vehicleSpeed < 0.0f) {
      vehicleSpeed = 0.0f;
    }
  }
  else if (vehicleSpeed < 0.0f) {
    vehicleSpeed += dt;

    if (vehicleSpeed > 0.0f) {
      vehicleSpeed = 0.0f;
    }
  }

  vehicleSpeed = constrain(vehicleSpeed, -50.0f, 100.0f);
}

void updateMotor()
{
  if (vehicleSpeed > 0.0f) {
    int pwm = (int)roundf(
      vehicleSpeed * 255.0f / 100.0f
    );

    pwm = constrain(pwm, 0, 255);

    ledcWrite(L293D_INPUT3_PIN, pwm);
    ledcWrite(L293D_INPUT4_PIN, 0);
  }
  else if (vehicleSpeed < 0.0f) {
    int pwm = (int)roundf(
      fabsf(vehicleSpeed) * 255.0f / 50.0f
    );

    pwm = constrain(pwm, 0, 255);

    ledcWrite(L293D_INPUT3_PIN, 0);
    ledcWrite(L293D_INPUT4_PIN, pwm);
  }
  else {
    ledcWrite(L293D_INPUT3_PIN, 0);
    ledcWrite(L293D_INPUT4_PIN, 0);
  }
}

void updateSpeedLcd()
{
  lcd.setCursor(15, 0);
  lcd.write(vehicleSpeed < 0.0f ? byte(1) : byte(0));

  lcd.setCursor(0, 1);
  lcd.print("Vitesse: ");

  int displayedSpeed = (int)roundf(fabsf(vehicleSpeed));

  if (displayedSpeed < 100) {
    lcd.print(" ");
  }

  if (displayedSpeed < 10) {
    lcd.print(" ");
  }

  lcd.print(displayedSpeed);
  lcd.print("   ");

  lcd.setCursor(15, 1);

  if (blinkerLightState) {
    lcd.write(byte(2));
  }
  else {
    lcd.print(" ");
  }
}

void updateBlinker(bool enabled, unsigned long now)
{
  if (!enabled) {
    blinkerLightState = false;
    digitalWrite(BLINKER_LIGHT_PIN, LOW);
    lastBlinkerChange = now;
    return;
  }

  unsigned long interval = blinkerLightState ? 500 : 250;

  if (now - lastBlinkerChange >= interval) {
    lastBlinkerChange = now;
    blinkerLightState = !blinkerLightState;

    digitalWrite(
      BLINKER_LIGHT_PIN,
      blinkerLightState ? HIGH : LOW
    );
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.createChar(0, forwardArrow);
  lcd.createChar(1, reverseArrow);
  lcd.createChar(2, lightbulb);

  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(BLINKER_LIGHT_PIN, OUTPUT);
  pinMode(BLINKER_SOURCE_PIN, INPUT_PULLUP);

  Serial.println("Starting servo initialization...");

  steeringServo.setPeriodHertz(50);

  Serial.printf(
    "Attempting to attach servo to GPIO %d...\n",
    SERVO_PIN
  );

  steeringServo.setPeriodHertz(50);
  servoAttached = steeringServo.attach(SERVO_PIN, 500, 2400);
  servoAttached = true;
  Serial.printf(
    "Servo attach on GPIO %d: %s\n",
    SERVO_PIN,
    servoAttached ? "OK" : "FAILED"
  );

  if (servoAttached) {
    steeringServo.write(89);
    lastServoAngle = 89;
    Serial.println("Servo initialized at 90 degrees.");
  }
  else {
    Serial.println("ERROR: Servo failed to attach!");
  }

  Serial.println("Starting motor PWM initialization...");

  bool motorPwm3Attached = ledcAttach(
    L293D_INPUT3_PIN,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION
  );

  Serial.printf(
    "Motor PWM GPIO %d: %s\n",
    L293D_INPUT3_PIN,
    motorPwm3Attached ? "OK" : "FAILED"
  );

  bool motorPwm4Attached = ledcAttach(
    L293D_INPUT4_PIN,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION
  );

  Serial.printf(
    "Motor PWM GPIO %d: %s\n",
    L293D_INPUT4_PIN,
    motorPwm4Attached ? "OK" : "FAILED"
  );

  ledcWrite(L293D_INPUT3_PIN, 0);
  ledcWrite(L293D_INPUT4_PIN, 0);

  Serial.println("Motor PWM initialization complete.");

  digitalWrite(LIGHT_PIN, HIGH);
  digitalWrite(BLINKER_LIGHT_PIN, LOW);

  unsigned long now = millis();

  lastSpeedUpdate = now;
  lastLcdUpdate = now;
  lastServoUpdate = now;
  lastDebugUpdate = now;

  Serial.println("System ready.");
  Serial.println("Servo: SG90 positional mode");
  Serial.println("Open Serial Monitor at 115200 baud.");
}

void loop()
{
  int x = ads7830_read_selected_channel(
    ADS7830_ADDR,
    JOYSTICK_X
  );

  int y = ads7830_read_selected_channel(
    ADS7830_ADDR,
    JOYSTICK_Y
  );

  int lightLevel = ads7830_read_selected_channel(
    ADS7830_ADDR,
    LIGHT_SENSOR
  );

  unsigned long now = millis();

  if (now - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {
    lastServoUpdate = now;

    if (x >= 0 && servoAttached) {
      updateServo(x);
    }
  }

  bool pressed = digitalRead(SW_PIN) == LOW;
  bool blinkerEnabled =
    digitalRead(BLINKER_SOURCE_PIN) == LOW;

  updateLight(lightLevel);
  updateBlinker(blinkerEnabled, now);

  if (pressed) {
    tone(BUZZER_PIN, 2000);
  }
  else {
    noTone(BUZZER_PIN);
  }

  if (y >= 0) {
    updateVehicleSpeed(y);
  }

  updateMotor();

  if (now - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
    lastLcdUpdate = now;
    updateSpeedLcd();
  }

  if (now - lastDebugUpdate >= DEBUG_INTERVAL) {
    lastDebugUpdate = now;

    Serial.printf(
      "X: %d | Servo: %s | Angle: %d | Y: %d | "
      "Light: %d | Speed: %.1f\n",
      x,
      servoAttached ? "YES" : "NO",
      lastServoAngle,
      y,
      lightLevel,
      vehicleSpeed
    );
  }

  delay(5);
}