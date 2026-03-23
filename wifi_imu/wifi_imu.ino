/*
 * WiFi IMU - ESP8266 + MPU6500/BMP280 (GY-91)
 * Streams 7-DOF sensor data (accel, gyro, pressure, temp) over UDP at 100 Hz.
 * Magnetometer fields are zeroed (MPU6500 has no built-in AK8963).
 * See PROTOCOL.md for the binary packet format.
 */

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <MPU6500_WE.h>

// ── Network selector pin ────────────────────────────────────────────────────
#define NET_SEL_PIN D7  // HIGH (3.3V) = ScorpionIPX, LOW (GND) = Reach

// ── WiFi credentials & network config (selected at boot via D5) ─────────────
const char* WIFI_SSID;
const char* WIFI_PASSWORD;
IPAddress staticIP;
IPAddress gateway;
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);
IPAddress targetIP;
const uint16_t TARGET_PORT = 4269;

// ── I2C / sensor addresses ──────────────────────────────────────────────────
#define SDA_PIN D2
#define SCL_PIN D1
#define MPU9250_ADDR 0x69
#define BMP280_ADDR  0x77

// ── Timing ──────────────────────────────────────────────────────────────────
const uint32_t SAMPLE_INTERVAL_US = 10000;  // 10 ms → 100 Hz

// ── BMP280 register definitions ─────────────────────────────────────────────
#define BMP280_REG_CALIB     0x88
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_PRESS_MSB 0xF7

// ── BMP280 calibration data ─────────────────────────────────────────────────
struct BMP280Calib {
  uint16_t dig_T1;
  int16_t  dig_T2, dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
  int32_t  t_fine;
};

// ── UDP packet structure (packed, little-endian) ────────────────────────────
#pragma pack(push, 1)
struct IMUPacket {
  uint8_t  sync[2];        // 0xAA 0x55
  uint32_t packet_id;      // rolling counter
  uint32_t timestamp_ms;   // millis()
  float    accel[3];       // X Y Z  [g]
  float    gyro[3];        // X Y Z  [deg/s]
  float    pressure;       // [Pa]
  float    temperature;    // [°C]
  uint8_t  checksum;       // XOR of all preceding bytes
};
#pragma pack(pop)

// ── Globals ─────────────────────────────────────────────────────────────────
WiFiUDP udp;
MPU6500_WE mpu(MPU9250_ADDR);
BMP280Calib bmpCal;
IMUPacket pkt;
uint32_t packetCounter = 0;
uint32_t lastSampleTime = 0;

// ── BMP280 helper functions (direct register access) ────────────────────────

static void bmp280WriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t bmp280ReadReg(uint8_t reg) {
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)1);
  return Wire.read();
}

static void bmp280ReadCalibration() {
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(BMP280_REG_CALIB);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)26);

  uint8_t buf[26];
  for (int i = 0; i < 26; i++) buf[i] = Wire.read();

  bmpCal.dig_T1 = (uint16_t)(buf[1] << 8 | buf[0]);
  bmpCal.dig_T2 = (int16_t)(buf[3] << 8 | buf[2]);
  bmpCal.dig_T3 = (int16_t)(buf[5] << 8 | buf[4]);
  bmpCal.dig_P1 = (uint16_t)(buf[7] << 8 | buf[6]);
  bmpCal.dig_P2 = (int16_t)(buf[9] << 8 | buf[8]);
  bmpCal.dig_P3 = (int16_t)(buf[11] << 8 | buf[10]);
  bmpCal.dig_P4 = (int16_t)(buf[13] << 8 | buf[12]);
  bmpCal.dig_P5 = (int16_t)(buf[15] << 8 | buf[14]);
  bmpCal.dig_P6 = (int16_t)(buf[17] << 8 | buf[16]);
  bmpCal.dig_P7 = (int16_t)(buf[19] << 8 | buf[18]);
  bmpCal.dig_P8 = (int16_t)(buf[21] << 8 | buf[20]);
  bmpCal.dig_P9 = (int16_t)(buf[23] << 8 | buf[22]);
}

static void bmp280Init() {
  // Reset
  bmp280WriteReg(0xE0, 0xB6);
  delay(10);

  bmp280ReadCalibration();

  // Config: standby 0.5 ms, filter coeff 4, no SPI
  bmp280WriteReg(BMP280_REG_CONFIG, 0x08);
  // Ctrl_meas: temp oversampling x1, press oversampling x4, normal mode
  bmp280WriteReg(BMP280_REG_CTRL_MEAS, 0x2F);
}

static void bmp280Read(float* pressure, float* temperature) {
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(BMP280_REG_PRESS_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)6);

  uint8_t buf[6];
  for (int i = 0; i < 6; i++) buf[i] = Wire.read();

  int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
  int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);

  // Temperature compensation (from BMP280 datasheet)
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmpCal.dig_T1 << 1))) * ((int32_t)bmpCal.dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmpCal.dig_T1)) *
                    ((adc_T >> 4) - ((int32_t)bmpCal.dig_T1))) >> 12) *
                  ((int32_t)bmpCal.dig_T3)) >> 14;
  bmpCal.t_fine = var1 + var2;
  *temperature = (float)((bmpCal.t_fine * 5 + 128) >> 8) / 100.0f;

  // Pressure compensation (from BMP280 datasheet)
  int64_t v1 = ((int64_t)bmpCal.t_fine) - 128000;
  int64_t v2 = v1 * v1 * (int64_t)bmpCal.dig_P6;
  v2 = v2 + ((v1 * (int64_t)bmpCal.dig_P5) << 17);
  v2 = v2 + (((int64_t)bmpCal.dig_P4) << 35);
  v1 = ((v1 * v1 * (int64_t)bmpCal.dig_P3) >> 8) + ((v1 * (int64_t)bmpCal.dig_P2) << 12);
  v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)bmpCal.dig_P1) >> 33;

  if (v1 == 0) {
    *pressure = 0;
    return;
  }

  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - v2) * 3125) / v1;
  v1 = (((int64_t)bmpCal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  v2 = (((int64_t)bmpCal.dig_P8) * p) >> 19;
  p = ((p + v1 + v2) >> 8) + (((int64_t)bmpCal.dig_P7) << 4);

  *pressure = (float)((uint32_t)p) / 256.0f;
}

// ── Checksum ────────────────────────────────────────────────────────────────
static uint8_t computeChecksum(const uint8_t* data, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; i++) cs ^= data[i];
  return cs;
}

// ── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n[WiFi IMU] Starting...");

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400 kHz fast mode

  // ── MPU6500 ──
  if (!mpu.init()) {
    Serial.println("[MPU6500] INIT FAILED");
    while (1) { delay(1000); }
  }
  Serial.println("[MPU6500] OK");

  mpu.setAccRange(MPU6500_ACC_RANGE_4G);
  mpu.setGyrRange(MPU6500_GYRO_RANGE_500);
  mpu.enableGyrDLPF();
  mpu.setGyrDLPF(MPU6500_DLPF_3);  // ~41 Hz bandwidth
  mpu.setSampleRateDivider(0);       // max sample rate

  // ── BMP280 ──
  uint8_t bmpId = bmp280ReadReg(0xD0);
  if (bmpId != 0x58) {
    Serial.printf("[BMP280] WRONG ID: 0x%02X\n", bmpId);
    while (1) { delay(1000); }
  }
  bmp280Init();
  Serial.println("[BMP280] OK");

  // ── Network selection via D7 ──
  pinMode(NET_SEL_PIN, INPUT_PULLUP);
  delay(10);  // let pin settle

  if (digitalRead(NET_SEL_PIN) == HIGH) {
    WIFI_SSID     = "Reach";
    WIFI_PASSWORD = "RememberReach";
    staticIP  = IPAddress(192, 168, 68, 132);
    gateway   = IPAddress(192, 168, 68, 1);
    targetIP  = IPAddress(192, 168, 68, 139);
  } else {
    WIFI_SSID     = "ScorpionIPX";
    WIFI_PASSWORD = "Qwerty123";
    staticIP  = IPAddress(192, 168, 0, 132);
    gateway   = IPAddress(192, 168, 0, 1);
    targetIP  = IPAddress(192, 168, 0, 69);
  }
  Serial.printf("[NET] D7=%s → SSID: %s\n",
                digitalRead(NET_SEL_PIN) ? "HIGH" : "LOW", WIFI_SSID);

  // ── WiFi ──
  WiFi.mode(WIFI_STA);
  WiFi.config(staticIP, gateway, subnet, dns);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("[WiFi] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected – IP: %s\n", WiFi.localIP().toString().c_str());

  // ── UDP ──
  udp.begin(TARGET_PORT);

  // Prepare static packet fields
  pkt.sync[0] = 0xAA;
  pkt.sync[1] = 0x55;

  Serial.printf("[UDP] Sending to %s:%d every %d us\n",
                targetIP.toString().c_str(), TARGET_PORT, SAMPLE_INTERVAL_US);

  lastSampleTime = micros();
}

// ── Loop ────────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = micros();
  if (now - lastSampleTime < SAMPLE_INTERVAL_US) return;
  lastSampleTime += SAMPLE_INTERVAL_US;

  // ── Read MPU6500 ──
  xyzFloat accel = mpu.getGValues();
  xyzFloat gyro  = mpu.getGyrValues();

  // ── Read BMP280 ──
  float pressure, temperature;
  bmp280Read(&pressure, &temperature);

  // ── Build packet ──
  pkt.packet_id    = packetCounter++;
  pkt.timestamp_ms = millis();

  pkt.accel[0] = accel.x;
  pkt.accel[1] = accel.y;
  pkt.accel[2] = accel.z;

  pkt.gyro[0] = gyro.x;
  pkt.gyro[1] = gyro.y;
  pkt.gyro[2] = gyro.z;

  pkt.pressure    = pressure;
  pkt.temperature = temperature;

  // Checksum: XOR all bytes before the checksum field
  pkt.checksum = computeChecksum((const uint8_t*)&pkt, sizeof(pkt) - 1);

  // ── Send UDP ──
  udp.beginPacket(targetIP, TARGET_PORT);
  udp.write((const uint8_t*)&pkt, sizeof(pkt));
  udp.endPacket();
}