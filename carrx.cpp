#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <driver/ledc.h>   // ESP-IDF LEDC (PWM) API

// ESC pins (change if needed)
#define LEFT_ESC_PIN  18
#define RIGHT_ESC_PIN 19

// PWM settings
#define PWM_FREQ_HZ        50          // 50Hz for hobby ESCs
#define PWM_RESOLUTION_BITS 16         // 16-bit resolution
#define PWM_MIN_US         1000u
#define PWM_NEUTRAL_US     1500u
#define PWM_MAX_US         2000u

// LEDC configuration
#define LEDC_TIMER         LEDC_TIMER_0
#define LEDC_MODE          LEDC_HIGH_SPEED_MODE
#define LEFT_CHANNEL       LEDC_CHANNEL_0
#define RIGHT_CHANNEL      LEDC_CHANNEL_1

// control parameters
#define MAX_ANGLE_DEG      45.0f       // expected max tilt
#define DEADZONE_DEG       4.0f        // deadzone around neutral
#define SIGNAL_TIMEOUT_MS  500         // failsafe: ms before returning to neutral

unsigned long lastPacketTime = 0;

typedef struct {
  float yaw;
  float pitch;
  float roll;
} DataPacket;

static DataPacket receivedPacket;

// convert microseconds (1000..2000) to LEDC duty (0..top)
static uint32_t usToDuty(uint32_t us) {
  const uint32_t top = ((1UL << PWM_RESOLUTION_BITS) - 1);
  // period in microseconds for 50Hz = 20000
  uint64_t tmp = (uint64_t)us * (uint64_t)top;
  tmp /= 20000ULL;
  return (uint32_t)tmp;
}

static void writeESC_us(uint32_t left_us, uint32_t right_us) {
  uint32_t leftDuty  = usToDuty(left_us);
  uint32_t rightDuty = usToDuty(right_us);
  ledc_set_duty(LEDC_MODE, LEFT_CHANNEL, leftDuty);
  ledc_update_duty(LEDC_MODE, LEFT_CHANNEL);
  ledc_set_duty(LEDC_MODE, RIGHT_CHANNEL, rightDuty);
  ledc_update_duty(LEDC_MODE, RIGHT_CHANNEL);
}

// ESP-NOW receive callback (IDF v5 signature)
void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  (void)info; // unused here, but available (RSSI, MAC, channel)
  if (len != (int)sizeof(DataPacket)) {
    // unexpected packet size -> ignore
    return;
  }

  memcpy(&receivedPacket, incomingData, sizeof(receivedPacket));
  lastPacketTime = millis();

  float yaw   = receivedPacket.yaw;
  float pitch = receivedPacket.pitch;

  // apply deadzone
  if (fabs(pitch) < DEADZONE_DEG) pitch = 0.0f;
  if (fabs(yaw)   < DEADZONE_DEG) yaw   = 0.0f;

  // clamp angles
  pitch = constrain(pitch, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
  yaw   = constrain(yaw,   -MAX_ANGLE_DEG, MAX_ANGLE_DEG);

  // mapping: scale angles to microsecond offsets around neutral
  // tune these spans for responsiveness vs safety
  const float throttleSpan_us = 400.0f; // pitch -> ±400µs (safe)
  const float turnSpan_us     = 300.0f; // yaw -> ±300µs

  float throttle_delta = (pitch / MAX_ANGLE_DEG) * throttleSpan_us; // -400..+400
  float turn_delta     = (yaw   / MAX_ANGLE_DEG) * turnSpan_us;     // -300..+300

  float left_delta  = throttle_delta + turn_delta;
  float right_delta = throttle_delta - turn_delta;

  left_delta = -left_delta;

  // clamp final deltas to safe bounds
  float maxDelta = throttleSpan_us + turnSpan_us;
  left_delta  = constrain(left_delta,  -maxDelta, maxDelta);
  right_delta = constrain(right_delta, -maxDelta, maxDelta);

  uint32_t left_us  = (uint32_t)constrain(round(PWM_NEUTRAL_US + left_delta),  PWM_MIN_US, PWM_MAX_US);
  uint32_t right_us = (uint32_t)constrain(round(PWM_NEUTRAL_US + right_delta), PWM_MIN_US, PWM_MAX_US);

  writeESC_us(left_us, right_us);
}

void setupPWM() {
  // configure LEDC timer
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_MODE,
    .duty_resolution  = (ledc_timer_bit_t)PWM_RESOLUTION_BITS,
    .timer_num        = LEDC_TIMER,
    .freq_hz          = PWM_FREQ_HZ,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  // configure left channel
  ledc_channel_config_t left_ch = {
    .gpio_num   = LEFT_ESC_PIN,
    .speed_mode = LEDC_MODE,
    .channel    = LEFT_CHANNEL,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = LEDC_TIMER,
    .duty       = 0,
    .hpoint     = 0
  };
  ledc_channel_config(&left_ch);

  // configure right channel
  ledc_channel_config_t right_ch = {
    .gpio_num   = RIGHT_ESC_PIN,
    .speed_mode = LEDC_MODE,
    .channel    = RIGHT_CHANNEL,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = LEDC_TIMER,
    .duty       = 0,
    .hpoint     = 0
  };
  ledc_channel_config(&right_ch);

  // start at neutral
  writeESC_us(PWM_NEUTRAL_US, PWM_NEUTRAL_US);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // WiFi + ESP-NOW init
  WiFi.mode(WIFI_STA);
  delay(50);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(500);
  }

  esp_now_register_recv_cb(onDataReceive);

  // PWM / ESC setup
  setupPWM();

  lastPacketTime = millis();
  Serial.println("Receiver ready - awaiting packets");
}

void loop() {
  // failsafe: hold neutral if no packet recently
  if (millis() - lastPacketTime > SIGNAL_TIMEOUT_MS) {
    writeESC_us(PWM_NEUTRAL_US, PWM_NEUTRAL_US);
  }
  delay(100);
}
