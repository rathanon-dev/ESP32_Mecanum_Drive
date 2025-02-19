#include <Bluepad32.h>

// Pin Definitions
#define STBY1 32
#define PWM_FL 27
#define PWM_FR 26
#define PWM_RL 25
#define PWM_RR 33

#define IN1_FL 12
#define IN2_FL 14
#define IN1_FR 4
#define IN2_FR 16
#define IN1_RL 18
#define IN2_RL 19
#define IN1_RR 5
#define IN2_RR 17

// PWM Configuration
#define PWM_FREQ 1000
#define PWM_RES 8
#define PWM_CHANNELS { 0, 1, 2, 3 }

ControllerPtr controllers[BP32_MAX_GAMEPADS];
const int pwm_channels[] = PWM_CHANNELS;
int speedLevels[] = { 64, 96, 128, 160, 192 };
int currentSpeedIndex = 2;  // Default speed level (128)
bool lbPressed = false, rbPressed = false;

void setup() {
  pinMode(STBY1, OUTPUT);
  for (int pin : { IN1_FL, IN2_FL, IN1_FR, IN2_FR, IN1_RL, IN2_RL, IN1_RR, IN2_RR }) {
    pinMode(pin, OUTPUT);
  }

  for (int i = 0; i < 4; i++) {
    ledcSetup(pwm_channels[i], PWM_FREQ, PWM_RES);
    ledcAttachPin((int[]){ PWM_FL, PWM_FR, PWM_RL, PWM_RR }[i], pwm_channels[i]);
  }

  stop();
  BP32.setup(onConnectedController, onDisconnectedController);
  BP32.enableVirtualDevice(false);
  delay(1500);
}

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (!controllers[i]) {
      controllers[i] = ctl;
      return;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (controllers[i] == ctl) {
      controllers[i] = nullptr;
      return;
    }
  }
}

void move(int fl, int fr, int rl, int rr) {
  int speed = speedLevels[currentSpeedIndex];
  digitalWrite(STBY1, HIGH);
  int pwm_speeds[] = { fl * speed / 128, fr * speed / 128, rl * speed / 128, rr * speed / 128 };
  int states[][2] = { { IN1_FL, IN2_FL }, { IN1_FR, IN2_FR }, { IN1_RL, IN2_RL }, { IN1_RR, IN2_RR } };

  for (int i = 0; i < 4; i++) {
    digitalWrite(states[i][0], pwm_speeds[i] > 0);
    digitalWrite(states[i][1], pwm_speeds[i] < 0);
    ledcWrite(pwm_channels[i], abs(pwm_speeds[i]));
  }
}

void stop() { move(0, 0, 0, 0); }
void forward() { move(128, 128, 128, 128); }
void backward() { move(-128, -128, -128, -128); }
void left() { move(-128, 128, 128, -128); }
void right() { move(128, -128, -128, 128); }
void strafeLeft() { move(-128, 128, -128, 128); }
void strafeRight() { move(128, -128, 128, -128); }
void curverRotateLeft() { move(64, 128, 128, 128); }
void curverRotateRight() { move(128, 64, 128, 128); }
void lateralLeft() { move(0, 0, 128, -128); }
void lateralRight() { move(0, 0, -128, 128); }
void diagonalUpRight() { move(128, 0, 0, 128); }
void diagonalUpLeft() { move(0, 128, 128, 0); }
void diagonalDownRight() { move(0, -128, -128, 0); }
void diagonalDownLeft() { move(-128, 0, 0, -128); }

void processGamepad(ControllerPtr ctl) {
  uint8_t dpad = ctl->dpad(), buttons = ctl->buttons();
  bool btnLB = buttons & 0x0020, btnRB = buttons & 0x0010;

  if (btnLB && !lbPressed) {
    currentSpeedIndex = min(currentSpeedIndex + 1, 4);
  }
  lbPressed = btnLB;

  if (btnRB && !rbPressed) {
    currentSpeedIndex = max(currentSpeedIndex - 1, 0);
  }
  rbPressed = btnRB;

  if (dpad == 0x05) diagonalUpRight();
  else if (dpad == 0x09) diagonalUpLeft();
  else if (dpad == 0x06) diagonalDownRight();
  else if (dpad == 0x0A) diagonalDownLeft();
  else if (dpad & DPAD_UP) forward();
  else if (dpad & DPAD_DOWN) backward();
  else if (dpad & DPAD_LEFT) left();
  else if (dpad & DPAD_RIGHT) right();
  else if (buttons == 0x0004) curverRotateRight();
  else if (buttons == 0x0002) curverRotateLeft();
  else if (buttons == 0x0008) lateralLeft();
  else if (buttons == 0x0001) lateralRight();
  else if (ctl->throttle() > 50) strafeLeft();
  else if (ctl->brake() > 50) strafeRight();
  else stop();
}

void processControllers() {
  for (auto ctl : controllers) {
    if (ctl && ctl->isConnected() && ctl->hasData()) {
      processGamepad(ctl);
    }
  }
}

void loop() {
  if (BP32.update()) processControllers();
  vTaskDelay(1);
}
