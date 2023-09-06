#include <Arduino.h>

// švytavimo periodas
constexpr double period_s = 3.87;
constexpr uint32_t max_half_period_steps = 9; //17
constexpr uint32_t start_half_period_microsteps = 32;

// sukimosi laikas tarp dviejų stepų
#define ROTATION_STEP_PERIOD_MS 1

#define direction_pin 0
#define step_pin 1
#define enable_pin 10

#define rotation_direction_pin 3
#define rotation_step_pin 2

#define ENCODER_A 3  // Pin for Encoder A
#define ENCODER_B 2  // Pin for Encoder B

volatile int encoder_value = 0;
int last_encoder_value = 0;
void encoder_isr() {
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);
  // If the state of A changed, it means the encoder has been rotated
  if ((A == HIGH) != (B == LOW)) {
    encoder_value--;
  } else {
    encoder_value++;
  }
}

void setup() {
  Serial.begin(9600);
  // pinMode(ENCODER_A, INPUT_PULLUP);
  // pinMode(ENCODER_B, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr, CHANGE);

  pinMode(direction_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(enable_pin, OUTPUT);

  digitalWrite(enable_pin, LOW);
  delay(10000);
  digitalWrite(enable_pin, HIGH);

  pinMode(rotation_direction_pin, OUTPUT);
  pinMode(rotation_step_pin, OUTPUT);
  digitalWrite(rotation_direction_pin, LOW);
}

double constexpr sqrtNewtonRaphson(double x, double curr, double prev) {
  return curr == prev ? curr
                      : sqrtNewtonRaphson(x, 0.5 * (curr + x / curr), curr);
}

// pendulum length in meters
// double constexpr L = 3.704 - 0.194;

// constexpr double g = 9.807;
//be skriestuvo
// constexpr double period_s = 3.015;


// 2 * PI * sqrtNewtonRaphson(L / g, L / g, 0)
constexpr uint32_t period_ms = static_cast<uint32_t>(period_s * 1000);

// constexpr uint32_t half_period_steps = 1;
constexpr uint32_t microsteps_per_step = 64;
uint32_t half_period_microsteps = start_half_period_microsteps;

#define CURR_ANGLE TWO_PI * (millis() % period_ms) / period_ms

void drive_sine() {
  static int current_position = 0;

  int intended_position = int(round(half_period_microsteps * sin(CURR_ANGLE)));

  while (intended_position != current_position) {
    bool direction = intended_position > current_position;
    digitalWrite(direction_pin, direction);

    digitalWrite(step_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(step_pin, LOW);

    current_position += direction ? +1 : -1;
  }
}

uint32_t last_step = 0;

void drive_rotation(){
  if(millis() - last_step > ROTATION_STEP_PERIOD_MS){
    digitalWrite(rotation_step_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(rotation_step_pin, LOW);
    last_step = millis();
  }
}

bool is_target_ampliture_reached(){
    return half_period_microsteps / microsteps_per_step >= max_half_period_steps;
}

uint32_t last_ampliture_update = 0;
void loop() {
  drive_sine();
  // drive_rotation();
  if (millis() - last_ampliture_update > period_ms / 8 &&
      !is_target_ampliture_reached()) {
    half_period_microsteps +=1;
    last_ampliture_update = millis();
    Serial.println(half_period_microsteps);
  }
}
