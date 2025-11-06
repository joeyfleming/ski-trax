// IMU Tilt test: ADXL345 (0x53) + ITG3205 (0x68)
// Heltec ESP32-S3, SDA=GPIO4, SCL=GPIO5
// Prints accel (g), gyro (deg/s), pitch & roll (deg) using complementary filter.

#include <Wire.h>
#include <math.h>

#define SDA_PIN 4
#define SCL_PIN 5

// I2C addresses
const uint8_t ADXL = 0x53;
const uint8_t ITG  = 0x68;

// ADXL registers
const uint8_t ADXL_DEVID = 0x00;
const uint8_t ADXL_DATA  = 0x32;
const uint8_t ADXL_POWER_CTL = 0x2D;
const float ADXL_SENSITIVITY = 0.0039f; // 3.9 mg/LSB for +/-2g

// ITG registers
const uint8_t ITG_WHOAMI = 0x00;
const uint8_t ITG_GYRO_XOUT_H = 0x1D; // XH, XL, YH, YL, ZH, ZL
const uint8_t ITG_PWR_MGM = 0x3E;
const float ITG_SENSITIVITY = 14.375f; // LSB per °/s

// Complementary filter param
const float alpha = 0.98f; // gyro weight
const float dt_default = 0.1f; // default dt fallback

// running state
float pitch = 0.0f, roll = 0.0f;
float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;

unsigned long last_t = 0;

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((int)addr, (int)len);
  if (got != len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

int16_t be16(uint8_t hi, uint8_t lo) { return (int16_t)((hi << 8) | lo); }

void setupSensors() {
  // ADXL345 - enable measurement mode
  uint8_t tmp = 0;
  if (i2cRead(ADXL, ADXL_DEVID, &tmp, 1)) {
    // set measure bit
    i2cWrite(ADXL, ADXL_POWER_CTL, 0x08);
    Serial.println("ADXL345: measurement mode set");
  } else {
    Serial.println("ADXL345 not found!");
  }

  // ITG3205 - wake up, clock source set to internal oscillator (0)
  tmp = 0;
  if (i2cRead(ITG, ITG_WHOAMI, &tmp, 1)) {
    (void)i2cWrite(ITG, ITG_PWR_MGM, 0x00); // wake
    Serial.println("ITG3205: awakened");
  } else {
    Serial.println("ITG3205 not found!");
  }
}

void calibrateGyro(int samples = 200) {
  Serial.print("Calibrating gyro ");
  Serial.print(samples);
  Serial.println(" samples...");
  long sx=0, sy=0, sz=0;
  for (int i=0;i<samples;i++) {
    uint8_t buf[6];
    if (i2cRead(ITG, ITG_GYRO_XOUT_H, buf, 6)) {
      int16_t gx = be16(buf[0], buf[1]);
      int16_t gy = be16(buf[2], buf[3]);
      int16_t gz = be16(buf[4], buf[5]);
      sx += gx; sy += gy; sz += gz;
    }
    delay(5);
  }
  gyro_bias_x = sx / float(samples);
  gyro_bias_y = sy / float(samples);
  gyro_bias_z = sz / float(samples);
  Serial.printf("Gyro bias: X=%.2f Y=%.2f Z=%.2f\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);
}

void setup() {
  Serial.begin(115200);
  // do NOT block waiting for Serial
  delay(10);
  Serial.println("\nIMU tilt test starting...");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(50);

  setupSensors();
  delay(100);

  calibrateGyro(200);

  // initial read to seed angles
  uint8_t buf[6];
  if (i2cRead(ADXL, ADXL_DATA, buf, 6)) {
    int16_t ax_raw = (int16_t)((buf[1] << 8) | buf[0]); // little-endian
    int16_t ay_raw = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t az_raw = (int16_t)((buf[5] << 8) | buf[4]);
    // convert to g
    float ax = ax_raw * ADXL_SENSITIVITY;
    float ay = ay_raw * ADXL_SENSITIVITY;
    float az = az_raw * ADXL_SENSITIVITY;
    // seed angles
    pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
    roll  = atan2f(ay, az) * 180.0f / PI;
  }

  last_t = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - last_t) / 1e6f;
  if (dt <= 0 || dt > 0.5) dt = dt_default; // sanity
  last_t = now;

  // read accel
  uint8_t abuf[6];
  bool okA = i2cRead(ADXL, ADXL_DATA, abuf, 6);
  int16_t ax_raw = 0, ay_raw = 0, az_raw = 0;
  if (okA) {
    // ADXL345 data order: DATAX0 (low), DATAX1 (high), DATAY0, DATAY1, DATAZ0, DATAZ1
    ax_raw = (int16_t)((abuf[1] << 8) | abuf[0]);
    ay_raw = (int16_t)((abuf[3] << 8) | abuf[2]);
    az_raw = (int16_t)((abuf[5] << 8) | abuf[4]);
  }

  // read gyro
  uint8_t gbuf[6];
  bool okG = i2cRead(ITG, ITG_GYRO_XOUT_H, gbuf, 6);
  int16_t gx_raw=0, gy_raw=0, gz_raw=0;
  if (okG) {
    gx_raw = be16(gbuf[0], gbuf[1]);
    gy_raw = be16(gbuf[2], gbuf[3]);
    gz_raw = be16(gbuf[4], gbuf[5]);
  }

  // convert
  float ax = ax_raw * ADXL_SENSITIVITY; // g
  float ay = ay_raw * ADXL_SENSITIVITY;
  float az = az_raw * ADXL_SENSITIVITY;

  float gx = (gx_raw - gyro_bias_x) / ITG_SENSITIVITY; // deg/s
  float gy = (gy_raw - gyro_bias_y) / ITG_SENSITIVITY;
  float gz = (gz_raw - gyro_bias_z) / ITG_SENSITIVITY;

  // accel angles
  float pitch_a = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
  float roll_a  = atan2f(ay, az) * 180.0f / PI;

  // integrate gyro into angle estimates (note axis orientation may need sign flips)
  float pitch_g = pitch + gx * dt; // gx rotates around X -> affects pitch
  float roll_g  = roll  + gy * dt; // gy rotates around Y -> affects roll

  // complementary filter
  pitch = alpha * pitch_g + (1.0f - alpha) * pitch_a;
  roll  = alpha * roll_g  + (1.0f - alpha) * roll_a;

  // print
  Serial.printf("A(g): X=%.3f Y=%.3f Z=%.3f  |  G(dps): X=%.2f Y=%.2f Z=%.2f  |  Pitch=%.2f Roll=%.2f  dt=%.3f\n",
                ax, ay, az, gx, gy, gz, pitch, roll, dt);

  delay(90); // ~100 ms loop (plus time taken)
}
