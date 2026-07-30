#include <Arduino.h>

// Left motor
#define LEFT_PWM_PIN   25
#define LEFT_DIR_PIN   26
#define LEFT_ENC_A     34
#define LEFT_ENC_B     35

// Right motor
#define RIGHT_PWM_PIN  27
#define RIGHT_DIR_PIN  14
#define RIGHT_ENC_A    36
#define RIGHT_ENC_B    39

// ===== PWM (LEDC) config =====
#define PWM_FREQ  20000
#define PWM_RESOLUTION 8
#define LEFT_PWM_CHANNEL 0
#define RIGHT_PWM_CHANNEL 1

//Encoders:
volatile long leftTicks = 0;
volatile long rightTicks = 0;


void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B) == HIGH) {
    leftTicks++;
  } else {
  leftTicks--;
  }
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) {
    rightTicks++;
  } else {
    rightTicks--;
  }
}

//takes speed val and drives 1 motor; positive forward negative reverse

void setMotor(int pwmChannel, int dirPin, int speed) {
  bool forward = speed >=0;
  int duty = constrain(abs(speed), 0, 255);

  digitalWrite(dirPin, forward ? HIGH : LOW);
  ledcWrite(pwmChannel, duty);
}
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  // Encoder pins are input-only on ESP32 (34/35/36/39), no internal pull-up available
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_PWM_PIN, LEFT_PWM_CHANNEL);

  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_PWM_CHANNEL);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  Serial.println("Motor control firmware boot OK - pins configured");
}

void loop() {
  Serial.print("Left ticks: ");
  Serial.print(leftTicks);
  Serial.print("  |  Right ticks: ");
  Serial.println(rightTicks);

  // Temporary test: run both motors at low fixed speed forward.
  // Remove once PID is wired in.
  setMotor(LEFT_PWM_CHANNEL, LEFT_DIR_PIN, 100);
  setMotor(RIGHT_PWM_CHANNEL, RIGHT_DIR_PIN, 100);

  delay(500);
}