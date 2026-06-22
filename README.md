# Camera-Aided-Line-Following-Robot
// ============================================
// LINE FOLLOWER - ESP32-CAM - SMOOTH FINAL
// ============================================

#include "esp_camera.h"

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define MOTOR_A_IN1 13
#define MOTOR_A_IN2 12
#define MOTOR_B_IN1 14
#define MOTOR_B_IN2 15
#define ENA_PIN     4
#define ENB_PIN     2

// ── Tuning ────────────────────────────────────────────────────────────────────
int   FORWARD_SPEED = 110;  // max speed 0-255
int   MIN_SPEED     = 60;   // minimum motor speed (below this motors stall)
int   DEAD_ZONE     = 45;   // pixels — within this = straight
int   LOST_LIMIT    = 10;   // frames before full stop

// PD controller weights — tune these for your track
float Kp = 0.5;   // proportional: how hard to correct error
float Kd = 0.3;   // derivative:   damps oscillation (reduce if sluggish)
// ─────────────────────────────────────────────────────────────────────────────

int   lastError  = 0;
int   lostCount  = 0;

// ── Camera ────────────────────────────────────────────────────────────────────
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM; config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM; config.pin_href  = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera failed: 0x%x\n", err);
    while(1) { Serial.println("CAMERA ERROR!"); delay(1000); }
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_hmirror(s, 0);
  s->set_brightness(s, -1);
  s->set_contrast(s, 2);
  s->set_saturation(s, 0);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_ae_level(s, -2);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 0);
  s->set_gainceiling(s, (gainceiling_t)2);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);

  Serial.println("Camera OK!");
}

// ── Motors ────────────────────────────────────────────────────────────────────
void setupMotors() {
  pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT); pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);     pinMode(ENB_PIN, OUTPUT);
  analogWrite(ENA_PIN, 0);      analogWrite(ENB_PIN, 0);
  digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, LOW);
  Serial.println("Motors ready!");
}

void setMotorA(int speed) {
  speed = constrain(speed, 0, 255);
  if (speed == 0) {
    analogWrite(ENA_PIN, 0);
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
  } else {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    analogWrite(ENA_PIN, speed);
  }
}

void setMotorB(int speed) {
  speed = constrain(speed, 0, 255);
  if (speed == 0) {
    analogWrite(ENB_PIN, 0);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
  } else {
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
    analogWrite(ENB_PIN, speed);
  }
}

void stopMotors() { setMotorA(0); setMotorB(0); }

// ── PD Motor Drive ────────────────────────────────────────────────────────────
// Core idea: both motors always run, just at different speeds
// correction > 0 → steer right (speed up left, slow right)
// correction < 0 → steer left  (speed up right, slow left)
void drive(int error) {
  int derivative  = error - lastError;          // rate of change
  float correction = Kp * error + Kd * derivative; // PD output

  int speedA = FORWARD_SPEED - (int)correction; // left motor
  int speedB = FORWARD_SPEED + (int)correction; // right motor

  // Never let either motor drop below MIN_SPEED (stall zone)
  speedA = constrain(speedA, MIN_SPEED, 255);
  speedB = constrain(speedB, MIN_SPEED, 255);

  // On sharp turns slow overall speed to prevent overshoot
  int absError = abs(error);
  if (absError > 100) {
    // very sharp — scale both down
    float scale = map(absError, 100, 160, 90, 60) / 100.0;
    speedA = (int)(speedA * scale);
    speedB = (int)(speedB * scale);
    speedA = constrain(speedA, MIN_SPEED, 255);
    speedB = constrain(speedB, MIN_SPEED, 255);
  }

  Serial.printf("DRIVE A:%d B:%d (err=%d d=%d)\n",
                speedA, speedB, error, derivative);

  setMotorA(speedA);
  setMotorB(speedB);
}

// ── Line Detection ────────────────────────────────────────────────────────────
int findLine() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { Serial.println("NO FRAME!"); return -2; }

  int scanRows[]  = {180, 200, 220};
  int numRows     = 3;

  int bestPos      = 160;
  int bestContrast = 0;
  int bestDark     = 255;
  int bestAvg      = 255;

  for (int r = 0; r < numRows; r++) {
    int row = scanRows[r];
    if (row < 0 || row >= 240) continue;

    int brightness[320];
    for (int col = 0; col < 320; col++)
      brightness[col] = fb->buf[row * 320 + col];

    int darkVal = 255, darkPos = 160;
    for (int col = 10; col < 310; col++) {
      if (brightness[col] < darkVal) {
        darkVal = brightness[col];
        darkPos = col;
      }
    }

    int rowSum = 0;
    for (int col = 0; col < 320; col++) rowSum += brightness[col];
    int rowAvg   = rowSum / 320;
    int contrast = rowAvg - darkVal;

    if (contrast > bestContrast) {
      bestContrast = contrast;
      bestPos      = darkPos;
      bestDark     = darkVal;
      bestAvg      = rowAvg;
    }
  }

  esp_camera_fb_return(fb);

  Serial.printf("Pos:%d Dark:%d Avg:%d Contrast:%d\n",
                bestPos, bestDark, bestAvg, bestContrast);

  if (bestContrast < 30) return -1;

  return bestPos;
}

// ── Decision ──────────────────────────────────────────────────────────────────
void decide(int linePos) {
  if (linePos == -2) return;

  if (linePos == -1) {
    lostCount++;
    if (lostCount <= LOST_LIMIT) {
      // Keep steering in last known direction but slower
      Serial.printf("LOST(%d): recovering\n", lostCount);
      drive(lastError);
    } else {
      Serial.println("LOST: STOPPED");
      stopMotors();
    }
    return;
  }

  lostCount = 0;
  int error  = linePos - 160;

  drive(error);          // PD handles everything — forward, left, right
  lastError = error;     // update AFTER drive so derivative is correct
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== LINE FOLLOWER ===");

  setupMotors();
  setupCamera();

  Serial.println("Warming up camera...");
  for (int i = 0; i < 20; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(100);
  }

  camera_fb_t* test = esp_camera_fb_get();
  if (test) {
    int sum = 0;
    for (int i = 0; i < (int)test->len; i++) sum += test->buf[i];
    Serial.printf("Avg brightness: %d (good=100-180)\n", sum / test->len);
    esp_camera_fb_return(test);
  }

  Serial.println("Starting in 2s...");
  delay(2000);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  decide(findLine());
  delay(60);  // slightly faster = better reaction on curves
}
