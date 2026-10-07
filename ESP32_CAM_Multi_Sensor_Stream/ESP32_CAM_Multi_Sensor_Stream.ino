/*
  ESP32-CAM Multi-Sensor Stream v1.0
  Hardware: AI-Thinker ESP32-CAM + OV2640 + MPU-6050 + VL53L0X
  Target: embedded vision, robotics and synchronized sensor streaming

  IMPORTANT PIN MAP
  -----------------
  OV2640 camera (fixed by AI-Thinker PCB):
    PWDN  GPIO32
    RESET -1
    XCLK  GPIO0
    SIOD  GPIO26
    SIOC  GPIO27
    Y9    GPIO35
    Y8    GPIO34
    Y7    GPIO39
    Y6    GPIO36
    Y5    GPIO21
    Y4    GPIO19
    Y3    GPIO18
    Y2    GPIO5
    VSYNC GPIO25
    HREF  GPIO23
    PCLK  GPIO22

  External I2C bus (microSD MUST NOT be used):
    GPIO13 = SDA -> MPU6050 SDA + VL53L0X SDA
    GPIO14 = SCL -> MPU6050 SCL + VL53L0X SCL

  Host UART (UART0, 3.3 V TTL):
    GPIO1 / U0T -> host RX
    GPIO3 / U0R <- host TX
    GND         <-> host GND

  Flash/programming:
    GPIO0 -> GND ONLY while uploading firmware.
    Disconnect GPIO0 from GND and reset after upload.

  Power:
    ESP32-CAM: stable 5 V supply recommended.
    Sensors: use 3.3 V if the actual breakout boards support 3.3 V.
    All grounds must be common.

  Protocol:
    One UART carries binary JPEG frames + telemetry packets.
    Commands from host are ASCII lines beginning with '#', e.g.:
      #STATUS
      #START
      #STOP
      #FPS 10
      #QUALITY 12
      #FRAMESIZE QVGA
      #TELEMETRY 100
      #CALIBRATE_GYRO
      #SAVE
      #DEFAULTS
      #REBOOT

  Required Arduino libraries:
    - ESP32 board package by Espressif Systems
    - Adafruit MPU6050
    - Adafruit Unified Sensor
    - VL53L0X by Pololu
*/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "esp_camera.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>

// ---------- AI-THINKER OV2640 FIXED CAMERA PINS ----------
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

// ---------- EXTERNAL SENSOR PINS ----------
#define SENSOR_SDA_PIN    13
#define SENSOR_SCL_PIN    14

// UART0 is physically GPIO1 TX / GPIO3 RX.
#define HOST_BAUD         2000000UL

// ---------- PROTOCOL ----------
#define SYNC0             0xAA
#define SYNC1             0x55
#define PROTOCOL_VERSION  1

enum PacketType : uint8_t {
  PKT_TELEMETRY = 0x01,
  PKT_JPEG      = 0x02,
  PKT_STATUS    = 0x03,
  PKT_ACK       = 0x10,
  PKT_ERROR     = 0x11
};

struct __attribute__((packed)) TelemetryPayload {
  float ax_mps2;
  float ay_mps2;
  float az_mps2;
  float gx_rads;
  float gy_rads;
  float gz_rads;
  float imu_temp_c;
  uint16_t distance_mm;
  uint8_t distance_valid;
  uint8_t imu_valid;
  uint32_t last_frame_id;
};

struct RuntimeConfig {
  uint16_t telemetry_hz = 100;
  uint16_t camera_fps = 10;
  uint8_t jpeg_quality = 12;
  framesize_t frame_size = FRAMESIZE_QVGA;
  bool video_enabled = true;
};

Adafruit_MPU6050 mpu;
VL53L0X tof;
Preferences prefs;
RuntimeConfig cfg;

bool imuOK = false;
bool tofOK = false;
bool cameraOK = false;

uint32_t packetSequence = 0;
uint32_t frameId = 0;
uint64_t lastTelemetryUs = 0;
uint64_t lastFrameUs = 0;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

// ---------- CRC16-CCITT ----------
uint16_t crc16Update(uint16_t crc, uint8_t data) {
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; ++i) {
    crc = (crc & 0x8000)
      ? (uint16_t)((crc << 1) ^ 0x1021)
      : (uint16_t)(crc << 1);
  }
  return crc;
}

void serialWriteCrc(const uint8_t *data, size_t len, uint16_t &crc) {
  Serial.write(data, len);
  for (size_t i = 0; i < len; ++i) {
    crc = crc16Update(crc, data[i]);
  }
}

void sendPacket(
  uint8_t type,
  uint8_t flags,
  const uint8_t *payload,
  uint32_t payloadLen
) {
  const uint32_t seq = packetSequence++;
  const uint64_t timestampUs = esp_timer_get_time();

  Serial.write(SYNC0);
  Serial.write(SYNC1);

  uint16_t crc = 0xFFFF;
  const uint8_t version = PROTOCOL_VERSION;

  serialWriteCrc(&version, 1, crc);
  serialWriteCrc(&type, 1, crc);
  serialWriteCrc(&flags, 1, crc);
  serialWriteCrc((const uint8_t *)&seq, sizeof(seq), crc);
  serialWriteCrc((const uint8_t *)&timestampUs, sizeof(timestampUs), crc);
  serialWriteCrc((const uint8_t *)&payloadLen, sizeof(payloadLen), crc);

  if (payload != nullptr && payloadLen > 0) {
    serialWriteCrc(payload, payloadLen, crc);
  }

  Serial.write((const uint8_t *)&crc, sizeof(crc));
}

void sendTextPacket(uint8_t type, const String &text) {
  sendPacket(
    type,
    0,
    (const uint8_t *)text.c_str(),
    (uint32_t)text.length()
  );
}

// ---------- CAMERA ----------
bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;

  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;

  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;

  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;

  c.frame_size = cfg.frame_size;
  c.jpeg_quality = cfg.jpeg_quality;
  c.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.fb_count = 2;
  } else {
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    return false;
  }
  return true;
}

void applyCameraSettings() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    return;
  }

  s->set_framesize(s, cfg.frame_size);
  s->set_quality(s, cfg.jpeg_quality);
}

void sendJpegFrame() {
  if (!cameraOK) {
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    return;
  }

  sendPacket(
    PKT_JPEG,
    0,
    fb->buf,
    (uint32_t)fb->len
  );

  frameId++;
  esp_camera_fb_return(fb);
}

// ---------- MPU6050 + VL53L0X ----------
void initExternalSensors() {
  Wire.begin(
    SENSOR_SDA_PIN,
    SENSOR_SCL_PIN,
    400000
  );

  imuOK = mpu.begin(0x68, &Wire);

  if (imuOK) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  }

  tof.setTimeout(40);
  tofOK = tof.init();

  if (tofOK) {
    tof.setMeasurementTimingBudget(20000);
    tof.startContinuous(25);
  }
}

void calibrateGyro() {
  if (!imuOK) {
    sendTextPacket(
      PKT_ERROR,
      "MPU6050_NOT_FOUND"
    );
    return;
  }

  const uint16_t samples = 500;
  double sx = 0;
  double sy = 0;
  double sz = 0;

  sensors_event_t a, g, t;

  for (uint16_t i = 0; i < samples; ++i) {
    mpu.getEvent(&a, &g, &t);

    sx += g.gyro.x;
    sy += g.gyro.y;
    sz += g.gyro.z;

    delay(4);
  }

  gyroBiasX = (float)(sx / samples);
  gyroBiasY = (float)(sy / samples);
  gyroBiasZ = (float)(sz / samples);

  prefs.putFloat("gbx", gyroBiasX);
  prefs.putFloat("gby", gyroBiasY);
  prefs.putFloat("gbz", gyroBiasZ);

  sendTextPacket(
    PKT_ACK,
    "GYRO_CAL_OK"
  );
}

void sendTelemetry() {
  TelemetryPayload p = {};

  p.last_frame_id = frameId;
  p.distance_mm = 0xFFFF;

  if (imuOK) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    p.ax_mps2 = a.acceleration.x;
    p.ay_mps2 = a.acceleration.y;
    p.az_mps2 = a.acceleration.z;

    p.gx_rads = g.gyro.x - gyroBiasX;
    p.gy_rads = g.gyro.y - gyroBiasY;
    p.gz_rads = g.gyro.z - gyroBiasZ;

    p.imu_temp_c = t.temperature;
    p.imu_valid = 1;
  }

  if (tofOK) {
    uint16_t mm =
      tof.readRangeContinuousMillimeters();

    bool valid =
      !tof.timeoutOccurred()
      && mm > 0
      && mm < 8190;

    p.distance_valid = valid ? 1 : 0;
    p.distance_mm = valid ? mm : 0xFFFF;
  }

  sendPacket(
    PKT_TELEMETRY,
    0,
    (const uint8_t *)&p,
    sizeof(p)
  );
}

// ---------- SETTINGS ----------
framesize_t parseFrameSize(String value) {
  value.trim();
  value.toUpperCase();

  if (value == "QQVGA") {
    return FRAMESIZE_QQVGA;
  }

  if (value == "QVGA") {
    return FRAMESIZE_QVGA;
  }

  if (value == "VGA") {
    return FRAMESIZE_VGA;
  }

  return cfg.frame_size;
}

void saveConfig() {
  prefs.putUShort(
    "telem",
    cfg.telemetry_hz
  );

  prefs.putUShort(
    "fps",
    cfg.camera_fps
  );

  prefs.putUChar(
    "quality",
    cfg.jpeg_quality
  );

  prefs.putUInt(
    "fsize",
    (uint32_t)cfg.frame_size
  );
}

void loadConfig() {
  cfg.telemetry_hz =
    prefs.getUShort("telem", 100);

  cfg.camera_fps =
    prefs.getUShort("fps", 10);

  cfg.jpeg_quality =
    prefs.getUChar("quality", 12);

  cfg.frame_size =
    (framesize_t)prefs.getUInt(
      "fsize",
      (uint32_t)FRAMESIZE_QVGA
    );

  gyroBiasX =
    prefs.getFloat("gbx", 0.0f);

  gyroBiasY =
    prefs.getFloat("gby", 0.0f);

  gyroBiasZ =
    prefs.getFloat("gbz", 0.0f);
}

void restoreDefaults() {
  cfg.telemetry_hz = 100;
  cfg.camera_fps = 10;
  cfg.jpeg_quality = 12;
  cfg.frame_size = FRAMESIZE_QVGA;
  cfg.video_enabled = true;

  applyCameraSettings();
}

void sendStatus() {
  String s;
  s.reserve(220);

  s += "FW=ESP32_CAM_MULTI_SENSOR_STREAM_1.0;";
  s += "BOARD=AI_THINKER_ESP32_CAM;";
  s += "CAM=OV2640;";
  s += "MPU6050=";
  s += imuOK ? "OK;" : "FAIL;";
  s += "VL53L0X=";
  s += tofOK ? "OK;" : "FAIL;";
  s += "PSRAM=";
  s += psramFound() ? "YES;" : "NO;";
  s += "UART=2000000;";
  s += "PROTO=ECMS-1;";

  sendTextPacket(
    PKT_STATUS,
    s
  );
}

// ---------- COMMAND CHANNEL ----------
void processCommand(String cmd) {
  cmd.trim();

  if (!cmd.startsWith("#")) {
    return;
  }

  cmd.remove(0, 1);

  String upper = cmd;
  upper.toUpperCase();

  if (upper == "START") {
    cfg.video_enabled = true;
    sendTextPacket(PKT_ACK, "STREAM_ON");

  } else if (upper == "STOP") {
    cfg.video_enabled = false;
    sendTextPacket(PKT_ACK, "STREAM_OFF");

  } else if (upper.startsWith("FPS ")) {
    cfg.camera_fps =
      constrain(
        cmd.substring(4).toInt(),
        1,
        30
      );

    sendTextPacket(PKT_ACK, "FPS_OK");

  } else if (upper.startsWith("QUALITY ")) {
    cfg.jpeg_quality =
      constrain(
        cmd.substring(8).toInt(),
        6,
        40
      );

    applyCameraSettings();
    sendTextPacket(PKT_ACK, "QUALITY_OK");

  } else if (upper.startsWith("FRAMESIZE ")) {
    cfg.frame_size =
      parseFrameSize(
        cmd.substring(10)
      );

    applyCameraSettings();
    sendTextPacket(PKT_ACK, "FRAMESIZE_OK");

  } else if (upper.startsWith("TELEMETRY ")) {
    cfg.telemetry_hz =
      constrain(
        cmd.substring(10).toInt(),
        1,
        200
      );

    sendTextPacket(
      PKT_ACK,
      "TELEMETRY_OK"
    );

  } else if (upper == "CALIBRATE_GYRO") {
    calibrateGyro();

  } else if (upper == "STATUS") {
    sendStatus();

  } else if (upper == "SAVE") {
    saveConfig();
    sendTextPacket(PKT_ACK, "SAVE_OK");

  } else if (upper == "DEFAULTS") {
    restoreDefaults();
    sendTextPacket(PKT_ACK, "DEFAULTS_OK");

  } else if (upper == "REBOOT") {
    sendTextPacket(PKT_ACK, "REBOOTING");
    delay(100);
    ESP.restart();

  } else {
    sendTextPacket(
      PKT_ERROR,
      "UNKNOWN_COMMAND"
    );
  }
}

void pollHostCommands() {
  static String line;

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n') {
      processCommand(line);
      line = "";

    } else if (
      c != '\r'
      && line.length() < 128
    ) {
      line += c;
    }
  }
}

// ---------- ARDUINO ----------
void setup() {
  Serial.begin(HOST_BAUD);
  delay(250);

  prefs.begin(
    "camstream",
    false
  );

  loadConfig();

  initExternalSensors();
  cameraOK = initCamera();

  sendStatus();
}

void loop() {
  pollHostCommands();

  const uint64_t nowUs =
    esp_timer_get_time();

  const uint32_t telemetryPeriodUs =
    1000000UL
    / max(
        (uint16_t)1,
        cfg.telemetry_hz
      );

  if (
    nowUs - lastTelemetryUs
    >= telemetryPeriodUs
  ) {
    lastTelemetryUs = nowUs;
    sendTelemetry();
  }

  const uint32_t framePeriodUs =
    1000000UL
    / max(
        (uint16_t)1,
        cfg.camera_fps
      );

  if (
    cfg.video_enabled
    && nowUs - lastFrameUs
       >= framePeriodUs
  ) {
    lastFrameUs = nowUs;
    sendJpegFrame();
  }

  delay(1);
}
