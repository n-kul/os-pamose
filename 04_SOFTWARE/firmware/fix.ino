/*
 * ╔══════════════════════════════════════════════════════════════╗
 *  CYBERDECK GROUND STATION — ESP32 Firmware v2.0
 *  Hardware : ESP32 + nRF24L01+ PA+LNA + 16×2 LCD (I2C backpack)
 *
 *  SPI  (nRF24) : SCK=18  MISO=19  MOSI=23  CE=16  CSN=17
 *  I2C  (LCD)   : SDA=21  SCL=22   addr=0x27 (PCF8574 backpack)
 *
 * ── MODES ────────────────────────────────────────────────────
 *  SCAN   Core 0 sweeps ch 0-125, RPD occupancy, serves graph
 *  LOCK   Core 0 parks on one channel, receives any packet
 *  TX     Core 0 transmits payload on chosen channel
 *
 * ── API ENDPOINTS ────────────────────────────────────────────
 *  GET /                → dashboard HTML
 *  GET /scan            → {mode,locked_ch,ch[126],peak[126]}
 *  GET /status          → {uptime,mode,locked_ch,packets,loss,heap,cycles}
 *  GET /setMode?mode=SCAN|LOCK|TX
 *  GET /setChannel?ch=N             → lock+switch to ch N
 *  GET /setPipe?addr=AABBCCDDEE     → set 5-byte pipe address (hex)
 *  GET /resetPeak
 *  GET /telemetry       → latest decoded/raw packet (LOCK mode)
 *  GET /transmit?ch=N&data=PAYLOAD  → send payload on ch N
 *  GET /txstatus        → {sent,acked,loss_pct,last_ack_ms}
 *
 * ── PACKET HANDLING ──────────────────────────────────────────
 *  LOCK mode receives ANY packet regardless of format.
 *  Known format: flight computer TelemetryPacket (31 bytes)
 *    byte 0-3  : timestamp (uint32)
 *    byte 4    : flight state (uint8)
 *    byte 5-8  : altitude AGL (float)
 *    byte 9-12 : velocity (float)
 *    byte 13-16: acceleration (float)
 *    byte 17-20: max altitude (float)
 *    byte 21-24: max velocity (float)
 *    byte 25-28: max accel (float)
 *    byte 29   : battery % (uint8)
 *    byte 30   : checksum XOR (uint8)
 *  Unknown format: raw hex served via /telemetry
 *
 * ── LCD LAYOUT ───────────────────────────────────────────────
 *  SCAN mode:
 *    Row 0: GS SCAN  CY:12345
 *    Row 1: ACT:3 PK:CH037
 *  LOCK mode (raw):
 *    Row 0: LCK:072 PKTS:482
 *    Row 1: [first 16 hex chars of payload]
 *  LOCK mode (flight telemetry):
 *    Row 0: ALT:342m V:+48m/s
 *    Row 1: ACC:3.2g ST:COAST
 *  TX mode:
 *    Row 0: TX:072 SENT:123
 *    Row 1: ACK:120 LOSS:2%
 * ══════════════════════════════════════════════════════════════╝
 */

// ── Libraries ────────────────────────────────────────────────
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_task_wdt.h>

// ── Pin Definitions ──────────────────────────────────────────
// VSPI shared bus
#define NRF_CE    16
#define NRF_CSN   17
#define NRF_SCK   18
#define NRF_MISO  19
#define NRF_MOSI  23
#define MPU_CS    5    // MPU6500 on same VSPI bus

// I2C shared bus
#define LCD_SDA   21
#define LCD_SCL   22
// BME280 on same I2C bus as LCD

// Audio
#define BUZZER_PIN 26

// ── WiFi ─────────────────────────────────────────────────────
#define AP_SSID   "GROUNDSTATION-01"
#define AP_PASS   "telemetry"

// ── LCD I2C ──────────────────────────────────────────────────
#define LCD_ADDR  0x27   // PCF8574 I2C backpack — try 0x3F if blank
#define LCD_COLS  16
#define LCD_ROWS  2

// ── BME280 I2C ───────────────────────────────────────────────
#define BME_ADDR  0x76
#define BME280_CHIP_ID 0x60
#define BMP280_CHIP_ID 0x58

// ── MPU6500 SPI ──────────────────────────────────────────────
#define MPU_WHO_AM_I   0x75
#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_CFG  0x1C
#define MPU_ACCEL_CFG2 0x1D
#define MPU_GYRO_CFG   0x1B
#define MPU_CONFIG_REG 0x1A
#define MPU_ACCEL_OUT  0x3B
#define MPU_GYRO_OUT   0x43
#define MPU6500_ID     0x70
#define MPU6000_ID     0x68

// Sensor update interval
#define SENSOR_INTERVAL_MS 20    // 50Hz IMU + baro

// ── Joystick ADC pins ────────────────────────────────────────
// Left stick  → spectrum pan (X) + zoom (Y)
// Right stick → channel navigation (X) + lock/scan trigger (Y)
#define JOY_LX  34   // Left  X — pan spectrum
#define JOY_LY  35   // Left  Y — zoom spectrum
#define JOY_RX  32   // Right X — scroll channel
#define JOY_RY  33   // Right Y — push full up=LOCK, full down=SCAN

// ADC centre ~2048, deadzone ±200, full deflection ~4095 or 0
#define JOY_DEAD    200   // ignore within this distance of centre
#define JOY_CENTRE 2048
#define JOY_MAX    4095

// Joystick shared state (written by joystick task, read by HTTP + LCD)
volatile int16_t g_joy_lx = 0;   // -100..+100 normalised
volatile int16_t g_joy_ly = 0;
volatile int16_t g_joy_rx = 0;
volatile int16_t g_joy_ry = 0;
// Pan/zoom state published for browser via /joystate
volatile int16_t g_zoom_start = 0;    // channel index
volatile int16_t g_zoom_end   = 125;
volatile uint8_t g_joy_ch_sel = 76;   // right-stick selected channel

// ── nRF24 Registers ──────────────────────────────────────────
#define R_CONFIG      0x00
#define R_EN_AA       0x01
#define R_EN_RXADDR   0x02
#define R_SETUP_AW    0x03
#define R_SETUP_RETR  0x04
#define R_RF_CH       0x05
#define R_RF_SETUP    0x06
#define R_STATUS      0x07
#define R_RX_ADDR_P0  0x0A
#define R_TX_ADDR     0x10
#define R_RX_PW_P0    0x11
#define R_RPD         0x09
#define R_FIFO_STATUS 0x17
#define R_DYNPD       0x1C
#define R_FEATURE     0x1D

#define CMD_R_REG     0x00
#define CMD_W_REG     0x20
#define CMD_R_RX_PL   0x61
#define CMD_W_TX_PL   0xA0
#define CMD_FLUSH_TX  0xE1
#define CMD_FLUSH_RX  0xE2
#define CMD_NOP       0xFF

// ── Scanner ──────────────────────────────────────────────────
#define NUM_CH        126
#define DWELL_US      200
#define PEAK_DECAY    300

// ── TX config ────────────────────────────────────────────────
#define TX_RETRY_COUNT   5     // retransmit attempts
#define TX_RETRY_DELAY   3     // 1000µs units (3 = 1ms between retries)
#define TX_TIMEOUT_MS    30    // give up waiting for ACK after 30ms

// ── Flight computer packet ────────────────────────────────────
// Must match flight computer TelemetryPacket exactly
struct __attribute__((packed)) TelemetryPacket {
  uint32_t timestamp;
  uint8_t  state;
  float    altitude;
  float    velocity;
  float    acceleration;
  float    max_altitude;
  float    max_velocity;
  float    max_accel;
  uint8_t  battery_percent;
  uint8_t  checksum;
};  // 31 bytes

static const char* FLIGHT_STATES[] = {
  "BOOT","SINIT","CALIB","IDLE","ARMED",
  "BOOST","COAST","APOGEE","DESCENT","LANDED","ERROR"
};

// ── Mode ─────────────────────────────────────────────────────
typedef enum { MODE_SCAN = 0, MODE_LOCK = 1, MODE_TX = 2, MODE_STELLAR = 3 } rf_mode_t;

// ── Shared state (mutex protected) ───────────────────────────
SemaphoreHandle_t g_mutex;

// Scanner data
volatile uint8_t  g_ch_avg[NUM_CH]  = {0};
volatile uint8_t  g_ch_peak[NUM_CH] = {0};

// Mode/channel
volatile rf_mode_t g_mode       = MODE_SCAN;
volatile uint8_t   g_locked_ch  = 76;
volatile uint8_t   g_tx_ch      = 76;

// Pipe address — default matches flight computer txAddress
volatile uint8_t g_pipe_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
volatile bool    g_pipe_changed = false;

// RX packet buffer (raw, up to 32 bytes)
#define PKT_BUF_LEN 32
volatile uint8_t  g_rx_buf[PKT_BUF_LEN]  = {0};
volatile uint8_t  g_rx_len               = 0;
volatile bool     g_rx_new               = false;  // new unread packet
volatile uint32_t g_rx_packet_count      = 0;
volatile bool     g_rx_is_telemetry      = false;  // matches TelemetryPacket format
volatile TelemetryPacket g_telem         = {0};

// TX stats
volatile uint32_t g_tx_sent     = 0;
volatile uint32_t g_tx_acked    = 0;
volatile uint32_t g_tx_last_ack = 0;   // ms since last ack

// TX payload buffer (from UI, written under mutex)
#define TX_PAYLOAD_MAX 32
volatile uint8_t  g_tx_payload[TX_PAYLOAD_MAX] = {0};
volatile uint8_t  g_tx_payload_len = 0;
volatile bool     g_tx_trigger     = false;   // set by HTTP handler, cleared by task
volatile bool     g_tx_repeat      = false;
volatile uint32_t g_tx_repeat_ms   = 1000;

// System
volatile uint32_t g_scan_cycles  = 0;
WebServer server(80);

//PEWO
#define PEWO_MAX_APS 16

typedef struct
{
  char ssid[33];
  int8_t rssi;
  uint8_t channel;
  uint32_t last_seen;
}
pewo_ap_t;
pewo_ap_t g_pewo_aps[
  PEWO_MAX_APS
];
volatile int
g_pewo_ap_count=0;

volatile bool
g_pewo_active=false;

SemaphoreHandle_t
g_pewo_mutex;

// ── STELLAR MAP ─────────────────────

#define STAR_COUNT 12

struct StarSystem
{
const char* name;

float x;

float y;

bool visited;
};

StarSystem stars[STAR_COUNT]=
{
{"SOL",0,0,true},

{"TAU",60,-30,false},

{"VEGA",120,50,false},

{"SIRIUS",-90,40,false},

{"PROXIMA",150,-80,false},

{"TRAPPIST",-140,100,false},

{"ALTAIR",80,140,false},

{"DENEB",-170,-90,false},

{"ROSS128",200,40,false},

{"WOLF359",-220,20,false},

{"LHS1140",0,200,false},

{"ARC",250,-150,false}
};

volatile float g_star_camx=0;

volatile float g_star_camy=0;

volatile int g_star_selected=0;

// ── Sensor shared state (written by Core 1, read by HTTP) ────
// Gyro angles — integrated from MPU6500 gyro (degrees)
volatile float g_roll  = 0.0f;
volatile float g_pitch = 0.0f;
volatile float g_yaw   = 0.0f;
// Raw accel (g)
volatile float g_ax = 0.0f, g_ay = 0.0f, g_az = 0.0f;
// BME280
volatile float g_temperature = 0.0f;  // °C
volatile float g_pressure    = 0.0f;  // hPa
volatile float g_humidity    = 0.0f;  // %
volatile float g_baro_alt    = 0.0f;  // m (from pressure)
volatile bool  g_imu_ok      = false;
volatile bool  g_bme_ok      = false;
SemaphoreHandle_t g_sensor_mutex;

// ── Buzzer ───────────────────────────────────────────────────
// Non-blocking buzzer state machine (no delay() calls)
struct BuzzerState {
  bool     active;
  uint16_t freq;
  uint32_t on_until;   // millis() when to stop current tone
};
static BuzzerState g_buz = {false, 0, 0};

// Queue a tone (non-blocking — replaces current tone)
void buz_tone(uint16_t freq_hz, uint32_t dur_ms) {
  if (freq_hz == 0) {
    noTone(BUZZER_PIN);
    g_buz.active = false;
    return;
  }
  tone(BUZZER_PIN, freq_hz);
  g_buz.active   = true;
  g_buz.freq     = freq_hz;
  g_buz.on_until = millis() + dur_ms;
}

// Call from loop() — stops tone when duration expires
void buz_update() {
  if (g_buz.active && millis() >= g_buz.on_until) {
    noTone(BUZZER_PIN);
    g_buz.active = false;
  }
}

// Startup melody — played once at boot (blocking, only in setup)
void buz_startup() {
  uint16_t notes[] = {880, 1047, 1319, 1568};
  uint16_t durs[]  = {100,  100,  100,  200};
  for (int i = 0; i < 4; i++) {
    tone(BUZZER_PIN, notes[i]);
    delay(durs[i]);
    noTone(BUZZER_PIN);
    delay(30);
  }
}

// Mode switch beep: 1 beep = SCAN, 2 = LOCK, 3 = TX
void buz_mode(rf_mode_t m) {
  // Simple rising pip sequence — non-blocking via buz_tone for first note,
  // but we need short gaps. Use a quick blocking sequence since mode switches
  // are rare and take <150ms total.
  int count = (m == MODE_SCAN) ? 1 : (m == MODE_LOCK) ? 2 : 3;
  for (int i = 0; i < count; i++) {
    tone(BUZZER_PIN, 1200 + i * 200);
    delay(60);
    noTone(BUZZER_PIN);
    if (i < count - 1) delay(40);
  }
}

// Lock acquired / signal lost beeps
void buz_lock_acquired() { tone(BUZZER_PIN, 1800); delay(80); noTone(BUZZER_PIN); delay(40); tone(BUZZER_PIN, 2200); delay(120); noTone(BUZZER_PIN); }
void buz_signal_lost()   { tone(BUZZER_PIN, 800);  delay(200); noTone(BUZZER_PIN); }

// Flight event beeps (non-blocking single tone)
void buz_apogee()  { buz_tone(2500, 300); }
void buz_landed()  { buz_tone(1800, 500); }

// ── LCD driver (bitbang PCF8574 — no library needed) ─────────
// PCF8574 pin map to HD44780:
//   P7=D7 P6=D6 P5=D5 P4=D4 P3=BL P2=EN P1=RW P0=RS
static bool lcd_backlight = true;

void lcd_i2c_write(uint8_t data) {
  Wire.beginTransmission(LCD_ADDR);
  Wire.write(data | (lcd_backlight ? 0x08 : 0x00));
  Wire.endTransmission();
}

void lcd_pulse_enable(uint8_t data) {
  lcd_i2c_write(data | 0x04);  // EN high
  delayMicroseconds(1);
  lcd_i2c_write(data & ~0x04); // EN low
  delayMicroseconds(50);
}

void lcd_write4(uint8_t data) {
  lcd_i2c_write(data);
  lcd_pulse_enable(data);
}

void lcd_send(uint8_t value, uint8_t mode) {
  // mode: 0=command, 1=data (RS pin)
  uint8_t hi = (value & 0xF0) | mode;
  uint8_t lo = ((value << 4) & 0xF0) | mode;
  lcd_write4(hi);
  lcd_write4(lo);
}

void lcd_command(uint8_t cmd)  { lcd_send(cmd, 0x00); delayMicroseconds(37); }
void lcd_data(uint8_t ch)      { lcd_send(ch,  0x01); delayMicroseconds(37); }

void lcd_init() {
  Wire.begin(LCD_SDA, LCD_SCL);
  Wire.setClock(100000);
  delay(50);

  // HD44780 4-bit init sequence
  lcd_write4(0x30); delay(5);
  lcd_write4(0x30); delayMicroseconds(150);
  lcd_write4(0x30); delayMicroseconds(150);
  lcd_write4(0x20); delayMicroseconds(150);  // switch to 4-bit

  lcd_command(0x28);  // 4-bit, 2 lines, 5x8
  lcd_command(0x0C);  // display on, cursor off
  lcd_command(0x06);  // entry mode: increment, no shift
  lcd_command(0x01);  // clear
  delay(2);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
  uint8_t offsets[] = {0x00, 0x40};
  lcd_command(0x80 | (col + offsets[row % 2]));
}

void lcd_print(const char* str) {
  while (*str) lcd_data(*str++);
}

void lcd_clear() {
  lcd_command(0x01);
  delay(2);
}

// Write exactly LCD_COLS chars to one row (pads with spaces)
void lcd_write_row(uint8_t row, const char* str) {
  lcd_set_cursor(0, row);
  char buf[LCD_COLS + 1];
  snprintf(buf, sizeof(buf), "%-16s", str);
  buf[LCD_COLS] = '\0';
  for (int i = 0; i < LCD_COLS; i++) lcd_data((uint8_t)buf[i]);
}

// ── MPU6500 SPI driver (shares VSPI with nRF24) ──────────────
// Always keep NRF_CSN HIGH while talking to MPU and vice versa.
static SPISettings mpu_spi_cfg(1000000, MSBFIRST, SPI_MODE0);

void mpu_write(uint8_t reg, uint8_t val) {
  digitalWrite(NRF_CSN, HIGH);  // ensure nRF24 deselected
  SPI.beginTransaction(mpu_spi_cfg);
  digitalWrite(MPU_CS, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(MPU_CS, HIGH);
  SPI.endTransaction();
}

uint8_t mpu_read(uint8_t reg) {
  digitalWrite(NRF_CSN, HIGH);
  SPI.beginTransaction(mpu_spi_cfg);
  digitalWrite(MPU_CS, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(MPU_CS, HIGH);
  SPI.endTransaction();
  return v;
}

void mpu_read_buf(uint8_t reg, uint8_t* buf, uint8_t len) {
  digitalWrite(NRF_CSN, HIGH);
  SPI.beginTransaction(mpu_spi_cfg);
  digitalWrite(MPU_CS, LOW);
  SPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(MPU_CS, HIGH);
  SPI.endTransaction();
}

bool mpu_init() {
  pinMode(MPU_CS, OUTPUT);
  digitalWrite(MPU_CS, HIGH);
  delay(100);

  uint8_t who = mpu_read(MPU_WHO_AM_I);
  if (who != MPU6500_ID && who != MPU6000_ID) {
    Serial.printf("[MPU] WHO_AM_I fail: 0x%02X\n", who);
    return false;
  }

  mpu_write(MPU_PWR_MGMT_1, 0x80); delay(100);  // reset
  mpu_write(MPU_PWR_MGMT_1, 0x01); delay(10);   // clock = PLL gyro X
  mpu_write(MPU_GYRO_CFG,   0x18); // ±2000°/s
  mpu_write(MPU_ACCEL_CFG,  0x18); // ±16g
  mpu_write(MPU_CONFIG_REG, 0x02); // DLPF 92Hz
  mpu_write(MPU_ACCEL_CFG2, 0x02); // accel DLPF 99Hz
  delay(10);

  Serial.printf("[MPU] OK (ID 0x%02X)\n", who);
  return true;
}

// Read accel (g) and gyro (°/s) — 12 bytes starting at ACCEL_OUT
void mpu_read_all(float* ax, float* ay, float* az,
                  float* gx, float* gy, float* gz) {
  uint8_t buf[12];
  mpu_read_buf(MPU_ACCEL_OUT, buf, 12);  // ACCEL_OUT × 6, skip TEMP × 2 skipped inside

  int16_t axr = (buf[0]<<8)|buf[1];
  int16_t ayr = (buf[2]<<8)|buf[3];
  int16_t azr = (buf[4]<<8)|buf[5];
  // buf[6-7] = TEMP_H/L — skipped
  uint8_t gbuf[6];
  mpu_read_buf(MPU_GYRO_OUT, gbuf, 6);
  int16_t gxr = (gbuf[0]<<8)|gbuf[1];
  int16_t gyr = (gbuf[2]<<8)|gbuf[3];
  int16_t gzr = (gbuf[4]<<8)|gbuf[5];

  *ax = axr / 2048.0f;   // ±16g → 2048 LSB/g
  *ay = ayr / 2048.0f;
  *az = azr / 2048.0f;
  *gx = gxr / 16.4f;    // ±2000°/s → 16.4 LSB/°/s
  *gy = gyr / 16.4f;
  *gz = gzr / 16.4f;
}

// ── BME280 I2C driver ────────────────────────────────────────
// Calibration data
static uint16_t bme_T1; static int16_t bme_T2, bme_T3;
static uint16_t bme_P1; static int16_t bme_P2,bme_P3,bme_P4,bme_P5,bme_P6,bme_P7,bme_P8,bme_P9;
static uint8_t  bme_H1; static int16_t bme_H2; static uint8_t bme_H3;
static int16_t  bme_H4, bme_H5; static int8_t bme_H6;
static int32_t  bme_t_fine;

uint8_t bme_read_reg(uint8_t reg) {
  Wire.beginTransmission(BME_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return 0;
  Wire.requestFrom(BME_ADDR, 1);
  return Wire.read();
}

void bme_read_regs(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(BME_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BME_ADDR, (int)len);
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
}

void bme_write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BME_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

bool bme_init() {
  uint8_t id = bme_read_reg(0xD0);
  if (id != BME280_CHIP_ID && id != BMP280_CHIP_ID) {
    Serial.printf("[BME] chip ID fail: 0x%02X\n", id);
    return false;
  }

  // Reset
  bme_write_reg(0xE0, 0xB6); delay(10);

  // Read calibration T + P
  uint8_t cal[26];
  bme_read_regs(0x88, cal, 26);
  bme_T1=cal[0]|(cal[1]<<8); bme_T2=cal[2]|(cal[3]<<8); bme_T3=cal[4]|(cal[5]<<8);
  bme_P1=cal[6]|(cal[7]<<8); bme_P2=cal[8]|(cal[9]<<8); bme_P3=cal[10]|(cal[11]<<8);
  bme_P4=cal[12]|(cal[13]<<8); bme_P5=cal[14]|(cal[15]<<8); bme_P6=cal[16]|(cal[17]<<8);
  bme_P7=cal[18]|(cal[19]<<8); bme_P8=cal[20]|(cal[21]<<8); bme_P9=cal[22]|(cal[23]<<8);

  // Read calibration H (BME280 only — skipped gracefully on BMP280)
  if (id == BME280_CHIP_ID) {
    bme_H1 = bme_read_reg(0xA1);
    uint8_t hcal[7]; bme_read_regs(0xE1, hcal, 7);
    bme_H2 = hcal[0]|(hcal[1]<<8);
    bme_H3 = hcal[2];
    bme_H4 = ((int16_t)hcal[3]<<4)|(hcal[4]&0x0F);
    bme_H5 = ((int16_t)hcal[5]<<4)|(hcal[4]>>4);
    bme_H6 = (int8_t)hcal[6];
  }

  // osrs_t×2 osrs_p×16 mode=normal (ctrl_meas=0x57)
  // standby 0.5ms, filter×16 (config=0x1C)
  // osrs_h×1 (ctrl_hum=0x01)
  bme_write_reg(0xF2, 0x01);  // ctrl_hum — must write before ctrl_meas
  bme_write_reg(0xF5, 0x1C);  // config
  bme_write_reg(0xF4, 0x57);  // ctrl_meas: start normal mode

  delay(100);
  Serial.printf("[BME] OK (ID 0x%02X)\n", id);
  return true;
}

void bme_read_data(float* temp_c, float* press_hpa, float* hum_pct, float* alt_m) {
  uint8_t buf[8];
  bme_read_regs(0xF7, buf, 8);

  int32_t adc_P = ((int32_t)buf[0]<<12)|((int32_t)buf[1]<<4)|(buf[2]>>4);
  int32_t adc_T = ((int32_t)buf[3]<<12)|((int32_t)buf[4]<<4)|(buf[5]>>4);
  int32_t adc_H = ((int32_t)buf[6]<<8)|buf[7];

  // Temperature
  int32_t var1 = ((((adc_T>>3)-((int32_t)bme_T1<<1)))*bme_T2)>>11;
  int32_t var2 = (((((adc_T>>4)-(int32_t)bme_T1)*((adc_T>>4)-(int32_t)bme_T1))>>12)*bme_T3)>>14;
  bme_t_fine = var1+var2;
  *temp_c = (bme_t_fine*5+128)>>8;
  *temp_c /= 100.0f;

  // Pressure
  int64_t p;
  int64_t v1 = (int64_t)bme_t_fine-128000;
  int64_t v2 = v1*v1*(int64_t)bme_P6;
  v2 += (v1*(int64_t)bme_P5)<<17;
  v2 += ((int64_t)bme_P4)<<35;
  v1 = ((v1*v1*(int64_t)bme_P3)>>8)+((v1*(int64_t)bme_P2)<<12);
  v1 = (((((int64_t)1)<<47)+v1)*(int64_t)bme_P1)>>33;
  if (v1 == 0) { *press_hpa=0; *alt_m=0; }
  else {
    p = 1048576-adc_P;
    p = (((p<<31)-v2)*3125)/v1;
    v1 = (((int64_t)bme_P9)*(p>>13)*(p>>13))>>25;
    v2 = (((int64_t)bme_P8)*p)>>19;
    p = ((p+v1+v2)>>8)+((int64_t)bme_P7<<4);
    *press_hpa = (float)p/25600.0f;
    *alt_m = 44330.0f*(1.0f - powf(*press_hpa/1013.25f, 0.1903f));
  }

  // Humidity (BME280 only — returns 0 on BMP280)
  int32_t h = bme_t_fine - 76800;
  h = (((((adc_H<<14)-(((int32_t)bme_H4)<<20)-(((int32_t)bme_H5)*h))+
         16384)>>15)*
        (((((((h*((int32_t)bme_H6))>>10)*
              (((h*((int32_t)bme_H3))>>11)+32768))>>10)+2097152)*
             ((int32_t)bme_H2)+8192)>>14));
  h = h-(((((h>>15)*(h>>15))>>7)*((int32_t)bme_H1))>>4);
  h = h<0?0:(h>419430400?419430400:h);
  *hum_pct = (float)(h>>12)/1024.0f;
}

// ── nRF24 SPI helpers ────────────────────────────────────────

static inline void nrf_csn_lo() { digitalWrite(NRF_CSN, LOW);  }
static inline void nrf_csn_hi() { digitalWrite(NRF_CSN, HIGH); }
static inline void nrf_ce_lo()  { digitalWrite(NRF_CE,  LOW);  }
static inline void nrf_ce_hi()  { digitalWrite(NRF_CE,  HIGH); }

uint8_t nrf_spi(uint8_t b)      { return SPI.transfer(b); }

uint8_t nrf_read_reg(uint8_t reg) {
  nrf_csn_lo();
  nrf_spi(CMD_R_REG | (reg & 0x1F));
  uint8_t v = nrf_spi(0xFF);
  nrf_csn_hi();
  return v;
}

void nrf_write_reg(uint8_t reg, uint8_t val) {
  nrf_csn_lo();
  nrf_spi(CMD_W_REG | (reg & 0x1F));
  nrf_spi(val);
  nrf_csn_hi();
}

void nrf_write_reg_buf(uint8_t reg, const uint8_t* buf, uint8_t len) {
  nrf_csn_lo();
  nrf_spi(CMD_W_REG | (reg & 0x1F));
  for (uint8_t i = 0; i < len; i++) nrf_spi(buf[i]);
  nrf_csn_hi();
}

void nrf_flush_tx() { nrf_csn_lo(); nrf_spi(CMD_FLUSH_TX); nrf_csn_hi(); }
void nrf_flush_rx() { nrf_csn_lo(); nrf_spi(CMD_FLUSH_RX); nrf_csn_hi(); }

uint8_t nrf_status() { return nrf_read_reg(R_STATUS); }

// ── nRF24 mode configurators ──────────────────────────────────

// SCAN mode: PRX, no AA, no dynamic payload, listen for RPD
void nrf_config_scan() {
  nrf_ce_lo();
  nrf_write_reg(R_CONFIG,     0x0F);  // PWR_UP | PRIM_RX | EN_CRC 2-byte
  nrf_write_reg(R_EN_AA,      0x00);  // no auto-ack
  nrf_write_reg(R_EN_RXADDR,  0x01);  // pipe 0 enabled
  nrf_write_reg(R_SETUP_RETR, 0x00);  // no retries
  nrf_write_reg(R_RF_SETUP,   0x03);  // 1 Mbps, 0dBm (PA+LNA handles gain)
  nrf_write_reg(R_DYNPD,      0x00);
  nrf_write_reg(R_FEATURE,    0x00);
  nrf_flush_rx();
  nrf_flush_tx();
  nrf_write_reg(R_STATUS, 0x70);
  delayMicroseconds(130);
}

// LOCK/RX mode: PRX on locked channel, with or without AA
// We use AA=off (one-way broadcast from flight computer), pipe 0
// Pipe address is configurable via /setPipe
void nrf_config_rx(uint8_t ch, const uint8_t* addr, bool use_aa) {
  nrf_ce_lo();
  nrf_write_reg(R_CONFIG,     0x0F);
  nrf_write_reg(R_EN_AA,      use_aa ? 0x01 : 0x00);
  nrf_write_reg(R_EN_RXADDR,  0x01);
  nrf_write_reg(R_SETUP_AW,   0x03);  // 5-byte address
  nrf_write_reg(R_SETUP_RETR, 0x00);
  nrf_write_reg(R_RF_CH,      ch);
  nrf_write_reg(R_RF_SETUP,   0x03);
  nrf_write_reg(R_RX_PW_P0,   PKT_BUF_LEN);  // fixed 32-byte payload
  nrf_write_reg_buf(R_RX_ADDR_P0, addr, 5);
  nrf_write_reg(R_DYNPD,      0x00);
  nrf_write_reg(R_FEATURE,    0x00);
  nrf_flush_rx();
  nrf_flush_tx();
  nrf_write_reg(R_STATUS, 0x70);
  nrf_ce_hi();  // start listening
  delayMicroseconds(130);
}

// TX mode: PTX, AA enabled (for ACK tracking), pipe 0
void nrf_config_tx(uint8_t ch, const uint8_t* addr) {
  nrf_ce_lo();
  nrf_write_reg(R_CONFIG,     0x0E);  // PWR_UP | PTX | EN_CRC 2-byte
  nrf_write_reg(R_EN_AA,      0x01);
  nrf_write_reg(R_EN_RXADDR,  0x01);
  nrf_write_reg(R_SETUP_AW,   0x03);
  // TX_RETRY_DELAY in 250µs units, TX_RETRY_COUNT attempts
  nrf_write_reg(R_SETUP_RETR, (TX_RETRY_DELAY << 4) | TX_RETRY_COUNT);
  nrf_write_reg(R_RF_CH,      ch);
  nrf_write_reg(R_RF_SETUP,   0x03);
  nrf_write_reg(R_RX_PW_P0,   PKT_BUF_LEN);
  nrf_write_reg_buf(R_TX_ADDR,     addr, 5);
  nrf_write_reg_buf(R_RX_ADDR_P0,  addr, 5);  // ACK pipe must match TX addr
  nrf_write_reg(R_DYNPD,      0x00);
  nrf_write_reg(R_FEATURE,    0x00);
  nrf_flush_tx();
  nrf_flush_rx();
  nrf_write_reg(R_STATUS, 0x70);
  delayMicroseconds(130);
}

// Send one packet in TX mode, return true if ACKed
bool nrf_send_packet(const uint8_t* payload, uint8_t len) {
  // Load payload
  nrf_csn_lo();
  nrf_spi(CMD_W_TX_PL);
  for (uint8_t i = 0; i < len && i < PKT_BUF_LEN; i++) nrf_spi(payload[i]);
  // pad to 32 bytes
  for (uint8_t i = len; i < PKT_BUF_LEN; i++) nrf_spi(0x00);
  nrf_csn_hi();

  // Pulse CE to fire
  nrf_ce_hi();
  delayMicroseconds(15);
  nrf_ce_lo();

  // Wait for TX_DS (acked) or MAX_RT (failed)
  unsigned long t = millis();
  while (millis() - t < TX_TIMEOUT_MS) {
    uint8_t st = nrf_status();
    if (st & 0x20) {  // TX_DS — data sent + acked
      nrf_write_reg(R_STATUS, 0x20);
      nrf_flush_tx();
      return true;
    }
    if (st & 0x10) {  // MAX_RT — no ack
      nrf_write_reg(R_STATUS, 0x10);
      nrf_flush_tx();
      return false;
    }
    delayMicroseconds(100);
  }
  nrf_flush_tx();
  return false;
}

// ── Checksum (matches flight computer) ───────────────────────
uint8_t xor_checksum(const uint8_t* buf, uint8_t len) {
  uint8_t s = 0;
  for (uint8_t i = 0; i < len; i++) s ^= buf[i];
  return s;
}

// ── Telemetry packet decoder ──────────────────────────────────
// Returns true if buf looks like a valid TelemetryPacket
bool decode_telemetry(const uint8_t* buf, uint8_t len, TelemetryPacket* out) {
  if (len < (int)sizeof(TelemetryPacket)) return false;

  // Verify checksum (XOR of all bytes except last)
  uint8_t csum = xor_checksum(buf, sizeof(TelemetryPacket) - 1);
  if (csum != buf[sizeof(TelemetryPacket) - 1]) return false;

  // Sanity check: state byte must be in valid range
  uint8_t st = buf[4];
  if (st > 10) return false;

  // Sanity check: altitude must be in plausible range (-500..15000m)
  float alt;
  memcpy(&alt, buf + 5, 4);
  if (alt < -500.0f || alt > 15000.0f) return false;

  memcpy(out, buf, sizeof(TelemetryPacket));
  return true;
}

// ── LCD refresh (called from RF task, safe since Wire is I2C) ─
static char lcd_r0[LCD_COLS + 1] = {0};
static char lcd_r1[LCD_COLS + 1] = {0};

void lcd_refresh_scan(uint8_t peak_ch, uint32_t cycles, uint8_t active_count) {
  char r0[LCD_COLS + 1], r1[LCD_COLS + 1];
  snprintf(r0, sizeof(r0), "GS SCAN  CY:%-5lu", (unsigned long)cycles % 100000UL);
  snprintf(r1, sizeof(r1), "ACT:%-2u PK:CH%03u", active_count, peak_ch);
  if (memcmp(r0, lcd_r0, LCD_COLS)) { lcd_write_row(0, r0); memcpy(lcd_r0, r0, LCD_COLS); }
  if (memcmp(r1, lcd_r1, LCD_COLS)) { lcd_write_row(1, r1); memcpy(lcd_r1, r1, LCD_COLS); }
}

void lcd_refresh_lock_raw(uint8_t ch, uint32_t pkts, const uint8_t* buf, uint8_t blen) {
  char r0[LCD_COLS + 1], r1[LCD_COLS + 1];
  snprintf(r0, sizeof(r0), "LCK:%03u PKTS:%-5lu", ch, (unsigned long)pkts % 100000UL);
  // First 8 bytes as hex on row 1
  char hex[LCD_COLS + 1] = {0};
  int pos = 0;
  for (int i = 0; i < 8 && i < blen && pos < LCD_COLS - 1; i++) {
    pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X", buf[i]);
  }
  snprintf(r1, sizeof(r1), "%-16s", hex);
  if (memcmp(r0, lcd_r0, LCD_COLS)) { lcd_write_row(0, r0); memcpy(lcd_r0, r0, LCD_COLS); }
  if (memcmp(r1, lcd_r1, LCD_COLS)) { lcd_write_row(1, r1); memcpy(lcd_r1, r1, LCD_COLS); }
}

void lcd_refresh_lock_telem(uint8_t ch, const TelemetryPacket* t) {
  char r0[LCD_COLS + 1], r1[LCD_COLS + 1];
  // Alt and velocity on row 0
  int alt_i = (int)t->altitude;
  int vel_i = (int)(t->velocity * 10);  // 1 decimal as int
  snprintf(r0, sizeof(r0), "A:%4dm V:%+4d.%d",
           alt_i, vel_i / 10, abs(vel_i % 10));
  // Accel and state on row 1
  const char* sname = (t->state <= 10) ? FLIGHT_STATES[t->state] : "???";
  int acc_i = (int)(t->acceleration * 10);
  snprintf(r1, sizeof(r1), "AC:%+3d.%dg %s",
           acc_i / 10, abs(acc_i % 10), sname);
  if (memcmp(r0, lcd_r0, LCD_COLS)) { lcd_write_row(0, r0); memcpy(lcd_r0, r0, LCD_COLS); }
  if (memcmp(r1, lcd_r1, LCD_COLS)) { lcd_write_row(1, r1); memcpy(lcd_r1, r1, LCD_COLS); }
}

void lcd_refresh_tx(uint8_t ch, uint32_t sent, uint32_t acked) {
  char r0[LCD_COLS + 1], r1[LCD_COLS + 1];
  uint8_t loss = (sent > 0) ? (uint8_t)(100 - ((uint32_t)acked * 100 / sent)) : 0;
  snprintf(r0, sizeof(r0), "TX:%03u SENT:%-5lu", ch, (unsigned long)sent % 100000UL);
  snprintf(r1, sizeof(r1), "ACK:%-5lu LOSS:%2u%%", (unsigned long)acked % 100000UL, loss);
  if (memcmp(r0, lcd_r0, LCD_COLS)) { lcd_write_row(0, r0); memcpy(lcd_r0, r0, LCD_COLS); }
  if (memcmp(r1, lcd_r1, LCD_COLS)) { lcd_write_row(1, r1); memcpy(lcd_r1, r1, LCD_COLS); }
}

// ── RF task — Core 0 ─────────────────────────────────────────
void task_rf(void* pv) {
  esp_task_wdt_add(NULL);

  // Local scanner buffers
  uint8_t  loc_avg[NUM_CH]  = {0};
  uint8_t  loc_peak[NUM_CH] = {0};
  uint16_t loc_timer[NUM_CH]= {0};

  // SPI init (called from Core 0 task)
  pinMode(NRF_CE,  OUTPUT);
  pinMode(NRF_CSN, OUTPUT);
  nrf_ce_lo();
  nrf_csn_hi();
  SPI.begin(NRF_SCK, NRF_MISO, NRF_MOSI, NRF_CSN);
  SPI.setFrequency(10000000);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  delay(5);

  nrf_config_scan();

  rf_mode_t cur_mode    = MODE_SCAN;
  uint8_t   cur_ch      = 76;
  uint8_t   cur_pipe[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
  bool      cur_aa      = false;

  // TX repeat state
  unsigned long last_repeat_tx = 0;

  while (true) {

    // ── Read shared config under mutex ──────────────────────
    rf_mode_t want_mode;
    uint8_t   want_ch;
    uint8_t   want_pipe[5];
    bool      pipe_changed;
    bool      tx_trigger;
    uint8_t   tx_pl[TX_PAYLOAD_MAX];
    uint8_t   tx_pl_len;
    bool      tx_repeat;
    uint32_t  tx_repeat_ms;

    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      want_mode    = g_mode;
      want_ch      = (want_mode == MODE_TX) ? g_tx_ch : g_locked_ch;
      memcpy(want_pipe, (void*)g_pipe_addr, 5);
      pipe_changed = g_pipe_changed;
      tx_trigger   = g_tx_trigger;
      tx_pl_len    = g_tx_payload_len;
      memcpy(tx_pl, (void*)g_tx_payload, tx_pl_len);
      tx_repeat    = g_tx_repeat;
      tx_repeat_ms = g_tx_repeat_ms;
      // clear one-shot flags
      if (tx_trigger)   g_tx_trigger   = false;
      if (pipe_changed) g_pipe_changed = false;
      xSemaphoreGive(g_mutex);
    } else {
      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // ── Reconfigure if mode/channel/pipe changed ────────────
    bool need_reconfig = (want_mode != cur_mode)
                      || (want_ch   != cur_ch && want_mode != MODE_SCAN)
                      || pipe_changed;

    if (need_reconfig) {
      cur_mode = want_mode;
      cur_ch   = want_ch;
      memcpy(cur_pipe, want_pipe, 5);

      if (cur_mode == MODE_SCAN) {
        nrf_config_scan();
      } else if (cur_mode == MODE_LOCK) {
        // Flight computer uses AA=off (fire-and-forget broadcast).
        // If user configured a different pipe, assume they know what they're
        // doing and keep AA off for maximum compatibility.
        cur_aa = false;
        nrf_config_rx(cur_ch, cur_pipe, cur_aa);
      } else if (cur_mode == MODE_TX) {
        nrf_config_tx(cur_ch, cur_pipe);
      }
    }

    // ════════════════════════════════════════════════════════
    //  SCAN MODE
    // ════════════════════════════════════════════════════════
    if (cur_mode == MODE_SCAN) {

      uint8_t peak_ch    = 0;
      uint8_t peak_val   = 0;
      uint8_t active_cnt = 0;

      for (uint8_t ch = 0; ch < NUM_CH; ch++) {
        nrf_write_reg(R_RF_CH, ch);
        nrf_ce_hi();
        delayMicroseconds(DWELL_US);
        uint8_t hit = nrf_read_reg(R_RPD) & 0x01;
        nrf_ce_lo();

        // EMA: fast attack (+40), slow decay (-8)
        if (hit) {
          loc_avg[ch] = (loc_avg[ch] > 200) ? 255 : loc_avg[ch] + 40;
        } else {
          loc_avg[ch] = (loc_avg[ch] < 8)   ? 0   : loc_avg[ch] - 8;
        }

        // Peak hold
        if (loc_avg[ch] >= loc_peak[ch]) {
          loc_peak[ch]  = loc_avg[ch];
          loc_timer[ch] = 0;
        } else {
          if (++loc_timer[ch] > PEAK_DECAY) {
            if (loc_peak[ch]) loc_peak[ch]--;
            loc_timer[ch] = 0;
          }
        }

        if (loc_avg[ch] > 25) active_cnt++;
        if (loc_peak[ch] > peak_val) { peak_val = loc_peak[ch]; peak_ch = ch; }
      }

      g_scan_cycles++;

      // Publish scan data
      if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy((void*)g_ch_avg,  loc_avg,  NUM_CH);
        memcpy((void*)g_ch_peak, loc_peak, NUM_CH);
        xSemaphoreGive(g_mutex);
      }

      // LCD update (every 5 cycles to avoid I2C hammering)
      if (g_scan_cycles % 5 == 0) {
        lcd_refresh_scan(peak_ch, g_scan_cycles, active_cnt);
      }

      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    // ════════════════════════════════════════════════════════
    //  LOCK / RX MODE
    // ════════════════════════════════════════════════════════
    else if (cur_mode == MODE_LOCK) {

      // Poll FIFO status — non-blocking
      uint8_t fifo = nrf_read_reg(R_FIFO_STATUS);

      if (!(fifo & 0x01)) {  // RX FIFO not empty
        // Read payload
        uint8_t buf[PKT_BUF_LEN] = {0};
        nrf_csn_lo();
        nrf_spi(CMD_R_RX_PL);
        for (uint8_t i = 0; i < PKT_BUF_LEN; i++) buf[i] = nrf_spi(0xFF);
        nrf_csn_hi();
        nrf_write_reg(R_STATUS, 0x40);  // clear RX_DR

        // Try to decode as flight computer telemetry
        TelemetryPacket tpkt = {0};
        bool is_telem = decode_telemetry(buf, PKT_BUF_LEN, &tpkt);

        // Publish under mutex
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          memcpy((void*)g_rx_buf, buf, PKT_BUF_LEN);
          g_rx_len          = PKT_BUF_LEN;
          g_rx_new          = true;
          g_rx_packet_count++;
          g_rx_is_telemetry = is_telem;
          if (is_telem) memcpy((void*)&g_telem, &tpkt, sizeof(TelemetryPacket));
          xSemaphoreGive(g_mutex);
        }

        // LCD update
        if (is_telem) {
          lcd_refresh_lock_telem(cur_ch, &tpkt);
        } else {
          uint32_t pkt_count;
          if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            pkt_count = g_rx_packet_count;
            xSemaphoreGive(g_mutex);
          } else {
            pkt_count = 0;
          }
          lcd_refresh_lock_raw(cur_ch, pkt_count, buf, PKT_BUF_LEN);
        }
      }

      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(2));  // 2ms poll — fast enough for 20Hz telemetry
    }

    // ════════════════════════════════════════════════════════
    //  TX MODE
    // ════════════════════════════════════════════════════════
    else if (cur_mode == MODE_TX) {

      bool do_send = false;

      if (tx_trigger && tx_pl_len > 0) {
        do_send = true;
      } else if (tx_repeat && tx_pl_len > 0) {
        unsigned long now = millis();
        if (now - last_repeat_tx >= tx_repeat_ms) {
          last_repeat_tx = now;
          do_send = true;
        }
      }

      if (do_send) {
        bool acked = nrf_send_packet(tx_pl, tx_pl_len);

        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          g_tx_sent++;
          if (acked) {
            g_tx_acked++;
            g_tx_last_ack = millis();
          }
          xSemaphoreGive(g_mutex);
        }

        uint32_t sent, acked_cnt;
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
          sent      = g_tx_sent;
          acked_cnt = g_tx_acked;
          xSemaphoreGive(g_mutex);
        } else {
          sent = acked_cnt = 0;
        }
        lcd_refresh_tx(cur_ch, sent, acked_cnt);
      }

      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(5));
    }

    else if (cur_mode == MODE_STELLAR) {
      // STELLAR mode: keep RF task alive but do not run scanning or
      // reconfigure the radio. This preserves RF state (graphs, lock, TX).
      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    else {
      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

// ── Sensor task — Core 1 (runs alongside loop/HTTP) ──────────
// Reads MPU6500 via SPI and BME280 via I2C at 50Hz.
// Integrates gyro to get roll/pitch/yaw angles.
// Complementary filter blends accel angles for roll/pitch drift correction.
// Yaw is gyro-only (no magnetometer used).

void task_sensors(void* pv) {
  esp_task_wdt_add(NULL);

  // Local calibration offsets (taken at startup)
  float gx_off = 0, gy_off = 0, gz_off = 0;
  float ax_off = 0, ay_off = 0;  // accel offsets for level

  // Calibrate gyro — average 100 samples at rest
  Serial.println("[SENS] Calibrating gyro...");
  float gxs=0,gys=0,gzs=0;
  for (int i = 0; i < 100; i++) {
    float ax,ay,az,gx,gy,gz;
    mpu_read_all(&ax,&ay,&az,&gx,&gy,&gz);
    gxs+=gx; gys+=gy; gzs+=gz;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  gx_off = gxs/100.0f;
  gy_off = gys/100.0f;
  gz_off = gzs/100.0f;
  Serial.printf("[SENS] Gyro offsets: %.2f %.2f %.2f\n", gx_off, gy_off, gz_off);

  // Integration state
  float roll=0, pitch=0, yaw=0;
  unsigned long last_t = millis();

  // Track last flight state to detect state changes for buzzer
  uint8_t last_fc_state = 0xFF;

  while (true) {
    unsigned long now = millis();
    float dt = (now - last_t) / 1000.0f;
    last_t = now;
    if (dt < 0.001f || dt > 0.5f) dt = 0.02f;

    // ── MPU6500 read ─────────────────────────────────────
    float ax,ay,az,gx,gy,gz;
    mpu_read_all(&ax,&ay,&az,&gx,&gy,&gz);
    gx -= gx_off;
    gy -= gy_off;
    gz -= gz_off;

    // Integrate gyro (degrees)
    roll  += gx * dt;
    pitch += gy * dt;
    yaw   += gz * dt;

    // Complementary filter for roll/pitch (accel corrects gyro drift)
    // Alpha = 0.96 → 96% gyro, 4% accel
    float accel_roll  =  atan2f(ay, az) * 57.2958f;
    float accel_pitch = -atan2f(ax, sqrtf(ay*ay+az*az)) * 57.2958f;
    roll  = 0.96f*roll  + 0.04f*accel_roll;
    pitch = 0.96f*pitch + 0.04f*accel_pitch;

    // Wrap yaw to ±180
    while (yaw >  180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;

    // Publish IMU data
    if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_roll  = roll;
      g_pitch = pitch;
      g_yaw   = yaw;
      g_ax    = ax; g_ay = ay; g_az = az;
      g_imu_ok = true;
      xSemaphoreGive(g_sensor_mutex);
    }

    // ── BME280 read (every 5th cycle ~10Hz is enough) ────
    static int bme_tick = 0;
    if (++bme_tick >= 5) {
      bme_tick = 0;
      float temp, press, hum, alt;
      bme_read_data(&temp, &press, &hum, &alt);
      if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_temperature = temp;
        g_pressure    = press;
        g_humidity    = hum;
        g_baro_alt    = alt;
        g_bme_ok      = true;
        xSemaphoreGive(g_sensor_mutex);
      }
    }

    // ── Flight state change buzzer alerts ─────────────────
    // Read latest telemetry state under main mutex
    uint8_t fc_state = 0xFF;
    bool    is_telem = false;
    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      is_telem = g_rx_is_telemetry;
      if (is_telem) fc_state = g_telem.state;
      xSemaphoreGive(g_mutex);
    }

    if (is_telem && fc_state != last_fc_state) {
      last_fc_state = fc_state;
      // state 6=APOGEE, 8=LANDED
      if (fc_state == 6) buz_apogee();
      if (fc_state == 8) buz_landed();
    }

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL_MS));
  }
}

// ── JSON helpers ─────────────────────────────────────────────

void send_json(const char* body) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", body);
}

void send_ok()  { send_json("{\"ok\":true}"); }
void send_err(const char* msg) {
  char buf[128];
  snprintf(buf, sizeof(buf), "{\"error\":\"%s\"}", msg);
  server.send(400, "application/json", buf);
}
void handle_stellar()
{
char buf[2000];

int p=0;

p+=sprintf(
buf,

"{\"cx\":%.1f,\"cy\":%.1f,\"sel\":%d,\"stars\":[",

g_star_camx,

g_star_camy,

g_star_selected
);

for(int i=0;i<STAR_COUNT;i++)
{
p+=sprintf(

buf+p,

"{\"n\":\"%s\",\"x\":%.1f,\"y\":%.1f}%s",

stars[i].name,

stars[i].x,

stars[i].y,

i<STAR_COUNT-1?",":""

);
}

sprintf(
buf+p,
"]}"
);

send_json(buf);
}
// ── Route: /scan ─────────────────────────────────────────────
void handle_scan() {
  static char buf[2300];
  uint8_t snap_avg[NUM_CH], snap_peak[NUM_CH];
  rf_mode_t snap_mode;
  uint8_t snap_ch;

  if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    memcpy(snap_avg,  (void*)g_ch_avg,  NUM_CH);
    memcpy(snap_peak, (void*)g_ch_peak, NUM_CH);
    snap_mode = g_mode;
    snap_ch   = (snap_mode == MODE_TX) ? g_tx_ch : g_locked_ch;
    xSemaphoreGive(g_mutex);
  } else {
    server.send(503, "application/json", "{\"error\":\"busy\"}");
    return;
  }

  const char* mstr = snap_mode == MODE_SCAN   ? "SCAN" :
                     snap_mode == MODE_LOCK   ? "LOCK" :
                     snap_mode == MODE_TX     ? "TX" :
                     snap_mode == MODE_STELLAR? "STELLAR" : "UNKNOWN";

  int p = snprintf(buf, sizeof(buf),
    "{\"mode\":\"%s\",\"locked_ch\":%d,\"ch\":[", mstr, snap_ch);

  for (int i = 0; i < NUM_CH; i++) {
    p += snprintf(buf+p, sizeof(buf)-p, "%d%s", snap_avg[i], i<NUM_CH-1?",":"");
    if (p > (int)sizeof(buf) - 20) break;
  }
  p += snprintf(buf+p, sizeof(buf)-p, "],\"peak\":[");
  for (int i = 0; i < NUM_CH; i++) {
    p += snprintf(buf+p, sizeof(buf)-p, "%d%s", snap_peak[i], i<NUM_CH-1?",":"");
    if (p > (int)sizeof(buf) - 10) break;
  }
  snprintf(buf+p, sizeof(buf)-p, "]}");

  send_json(buf);
}

// ── Route: /status ───────────────────────────────────────────
void handle_status() {
  char buf[320];
  rf_mode_t m;
  uint8_t lch;
  uint32_t pkts, sent, acked;
  uint32_t last_ack;

  if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    m        = g_mode;
    lch      = (m == MODE_TX) ? g_tx_ch : g_locked_ch;
    pkts     = g_rx_packet_count;
    sent     = g_tx_sent;
    acked    = g_tx_acked;
    last_ack = g_tx_last_ack;
    xSemaphoreGive(g_mutex);
  } else {
    server.send(503, "application/json", "{\"error\":\"busy\"}");
    return;
  }

  const char* mstr = m==MODE_SCAN?"SCAN":
                     m==MODE_LOCK?"LOCK":
                     m==MODE_TX?"TX":
                     m==MODE_STELLAR?"STELLAR":"UNKNOWN";
  uint8_t loss = (sent > 0) ? (uint8_t)(100 - (acked*100/sent)) : 0;

  snprintf(buf, sizeof(buf),
    "{\"uptime\":%lu,\"mode\":\"%s\",\"locked_ch\":%d,"
    "\"packets\":%lu,\"tx_sent\":%lu,\"tx_acked\":%lu,"
    "\"tx_loss_pct\":%u,\"last_ack_ms\":%lu,"
    "\"scan_cycles\":%lu,\"free_heap\":%u}",
    millis()/1000, mstr, lch,
    (unsigned long)pkts, (unsigned long)sent, (unsigned long)acked,
    loss, (unsigned long)(millis()-last_ack),
    (unsigned long)g_scan_cycles, (unsigned)ESP.getFreeHeap());

  send_json(buf);
}

// ── Route: /telemetry ────────────────────────────────────────
void handle_telemetry() {
  char buf[512];
  bool is_telem;
  bool has_new;
  uint8_t raw[PKT_BUF_LEN];
  uint8_t rlen;
  TelemetryPacket t = {0};

  if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    is_telem = g_rx_is_telemetry;
    has_new  = g_rx_new;
    rlen     = g_rx_len;
    memcpy(raw, (void*)g_rx_buf, rlen);
    if (is_telem) memcpy(&t, (void*)&g_telem, sizeof(TelemetryPacket));
    g_rx_new = false;  // mark as read
    xSemaphoreGive(g_mutex);
  } else {
    server.send(503, "application/json", "{\"error\":\"busy\"}");
    return;
  }

  if (!has_new && rlen == 0) {
    send_json("{\"type\":\"none\"}");
    return;
  }

  if (is_telem) {
    const char* sname = (t.state <= 10) ? FLIGHT_STATES[t.state] : "UNKNOWN";
    snprintf(buf, sizeof(buf),
      "{\"type\":\"flight_telemetry\","
      "\"timestamp\":%lu,\"state\":%u,\"state_name\":\"%s\","
      "\"altitude\":%.2f,\"velocity\":%.2f,\"acceleration\":%.2f,"
      "\"max_altitude\":%.2f,\"max_velocity\":%.2f,\"max_accel\":%.2f,"
      "\"battery\":%u}",
      (unsigned long)t.timestamp, t.state, sname,
      t.altitude, t.velocity, t.acceleration,
      t.max_altitude, t.max_velocity, t.max_accel,
      t.battery_percent);
  } else {
    // Unknown format — return raw hex
    int p = snprintf(buf, sizeof(buf), "{\"type\":\"raw\",\"len\":%d,\"hex\":\"", rlen);
    for (int i = 0; i < rlen && p < (int)sizeof(buf)-10; i++) {
      p += snprintf(buf+p, sizeof(buf)-p, "%02X", raw[i]);
    }
    snprintf(buf+p, sizeof(buf)-p, "\"}");
  }

  send_json(buf);
}

// ── Route: /setMode ──────────────────────────────────────────
void handle_set_mode()
{
  if(!server.hasArg("mode"))
  {
    send_err("missing mode");
    return;
  }

  String m =
  server.arg("mode");

  rf_mode_t nm;

  if(m=="SCAN")
  {
    nm=MODE_SCAN;
  }

  else if(m=="LOCK")
  {
    nm=MODE_LOCK;
  }

  else if(m=="TX")
  {
    nm=MODE_TX;
  }

  else if(m=="STELLAR")
  {
    nm=MODE_STELLAR;
  }

  else
  {
    send_err(
    "invalid mode: SCAN|LOCK|TX|STELLAR"
    );

    return;
  }

  if(
  xSemaphoreTake(
  g_mutex,
  pdMS_TO_TICKS(20)
  )==pdTRUE)
  {
    g_mode=nm;

    xSemaphoreGive(
    g_mutex
    );
  }

  send_ok();
}

// ── Route: /setChannel ───────────────────────────────────────
void handle_set_channel() {
  if (!server.hasArg("ch")) { send_err("missing ch"); return; }
  int ch = server.arg("ch").toInt();
  if (ch < 0 || ch > 125) { send_err("ch must be 0-125"); return; }

  if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_locked_ch = (uint8_t)ch;
    g_mode      = MODE_LOCK;  // switching channel auto-engages LOCK
    // Clear RX state for fresh lock
    g_rx_packet_count = 0;
    g_rx_new          = false;
    g_rx_len          = 0;
    g_rx_is_telemetry = false;
    xSemaphoreGive(g_mutex);
  }
  send_ok();
}

// ── Route: /setPipe ──────────────────────────────────────────
// Accepts 5-byte address as 10 hex chars, e.g. /setPipe?addr=E7E7E7E7E7
void handle_set_pipe() {
  if (!server.hasArg("addr")) { send_err("missing addr"); return; }
  String a = server.arg("addr");
  if (a.length() != 10) { send_err("addr must be 10 hex chars (5 bytes)"); return; }

  uint8_t newaddr[5];
  for (int i = 0; i < 5; i++) {
    char hex[3] = {a[i*2], a[i*2+1], 0};
    newaddr[i] = (uint8_t)strtol(hex, nullptr, 16);
  }

  if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    memcpy((void*)g_pipe_addr, newaddr, 5);
    g_pipe_changed = true;
    xSemaphoreGive(g_mutex);
  }
  send_ok();
}

// ── Route: /resetPeak ────────────────────────────────────────
void handle_reset_peak() {
  if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    memset((void*)g_ch_avg,  0, NUM_CH);
    memset((void*)g_ch_peak, 0, NUM_CH);
    xSemaphoreGive(g_mutex);
  }
  send_ok();
}

// ── Route: /transmit ─────────────────────────────────────────
// /transmit?ch=72&data=HELLO
// data can be plain ASCII or hex if prefixed with 0x: 0xDEADBEEF
void handle_transmit() {
  if (!server.hasArg("ch"))   { send_err("missing ch");   return; }
  if (!server.hasArg("data")) { send_err("missing data"); return; }

  int ch = server.arg("ch").toInt();
  if (ch < 0 || ch > 125) { send_err("ch must be 0-125"); return; }

  String dataStr = server.arg("data");
  uint8_t payload[TX_PAYLOAD_MAX] = {0};
  uint8_t plen = 0;

  if (dataStr.startsWith("0x") || dataStr.startsWith("0X")) {
    // Hex mode
    String hex = dataStr.substring(2);
    for (int i = 0; i < (int)hex.length() && plen < TX_PAYLOAD_MAX; i += 2) {
      char hx[3] = {hex[i], (i+1<(int)hex.length())?hex[i+1]:'0', 0};
      payload[plen++] = (uint8_t)strtol(hx, nullptr, 16);
    }
  } else {
    // ASCII mode
    plen = min((int)dataStr.length(), TX_PAYLOAD_MAX);
    memcpy(payload, dataStr.c_str(), plen);
  }

  if (plen == 0) { send_err("empty payload"); return; }

  bool repeat = server.hasArg("repeat") && server.arg("repeat") == "1";
  uint32_t rep_ms = 1000;
  if (server.hasArg("interval")) rep_ms = (uint32_t)server.arg("interval").toInt();

  if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_tx_ch = (uint8_t)ch;
    memcpy((void*)g_tx_payload, payload, plen);
    g_tx_payload_len = plen;
    g_tx_trigger     = true;
    g_tx_repeat      = repeat;
    g_tx_repeat_ms   = rep_ms;
    g_mode           = MODE_TX;
    xSemaphoreGive(g_mutex);
  }
  send_ok();
}

// ── Route: /txstatus ─────────────────────────────────────────
void handle_tx_status() {
  char buf[256];
  uint32_t sent, acked, last_ack;
  uint8_t tx_ch;

  if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    sent     = g_tx_sent;
    acked    = g_tx_acked;
    last_ack = g_tx_last_ack;
    tx_ch    = g_tx_ch;
    xSemaphoreGive(g_mutex);
  } else {
    server.send(503, "application/json", "{\"error\":\"busy\"}");
    return;
  }

  uint8_t loss = (sent > 0) ? (uint8_t)(100 - (acked*100/sent)) : 0;
  snprintf(buf, sizeof(buf),
    "{\"ch\":%u,\"sent\":%lu,\"acked\":%lu,\"loss_pct\":%u,\"last_ack_ms\":%lu}",
    tx_ch, (unsigned long)sent, (unsigned long)acked,
    loss, (unsigned long)(millis()-last_ack));
  send_json(buf);
}

// ── Route: /sensors ──────────────────────────────────────────
// Returns ground station's own BME280 data
void handle_sensors() {
  char buf[256];
  float temp, press, hum, alt;
  bool bme_ok;

  if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    temp   = g_temperature;
    press  = g_pressure;
    hum    = g_humidity;
    alt    = g_baro_alt;
    bme_ok = g_bme_ok;
    xSemaphoreGive(g_sensor_mutex);
  } else {
    server.send(503, "application/json", "{\"error\":\"busy\"}");
    return;
  }

  snprintf(buf, sizeof(buf),
    "{\"ok\":%s,\"temp_c\":%.2f,\"pressure_hpa\":%.2f,"
    "\"humidity_pct\":%.1f,\"baro_alt_m\":%.1f}",
    bme_ok?"true":"false", temp, press, hum, alt);
  send_json(buf);
}

// ── Route: /orientation ──────────────────────────────────────
// Returns ground station's own MPU6500 orientation angles
void handle_orientation() {
  char buf[200];
  float roll, pitch, yaw, ax, ay, az;
  bool imu_ok;

  if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    roll   = g_roll;
    pitch  = g_pitch;
    yaw    = g_yaw;
    ax     = g_ax; ay = g_ay; az = g_az;
    imu_ok = g_imu_ok;
    xSemaphoreGive(g_sensor_mutex);
  } else {
    server.send(503, "application/json", "{\"error\":\"busy\"}");
    return;
  }

  snprintf(buf, sizeof(buf),
    "{\"ok\":%s,\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f}",
    imu_ok?"true":"false", roll, pitch, yaw, ax, ay, az);
  send_json(buf);
}

// ── Route: /resetOrientation ─────────────────────────────────
// Resets yaw to zero (call when pointing north / at pad)
volatile bool g_imu_reset = false;
void handle_reset_orientation() {
  g_imu_reset = true;
  send_ok();
}
void handle_pewo_start()
{
  g_pewo_active = true;

  send_ok();
}


void handle_pewo_stop()
{
  g_pewo_active = false;

  send_ok();
}


void handle_pewo_beacons()
{
  server.send(
    200,
    "application/json",
    "{\"count\":0,\"aps\":[]}"
  );
}
// ── Dashboard HTML (PROGMEM) ──────────────────────────────────
extern const char DASHBOARD_HTML[];
void handle_root() {
  server.sendHeader("Cache-Control", "max-age=3600");
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

void handle_not_found() {
  server.send(404, "text/plain", "Not found");
}
void handle_pewo_start();

void handle_pewo_stop();

void handle_pewo_beacons();
// ── setup() ──────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n[GND-STATION v3] Booting...");

  // Buzzer first — startup melody confirms power
  pinMode(BUZZER_PIN, OUTPUT);
  buz_startup();

  g_mutex        = xSemaphoreCreateMutex();
  g_sensor_mutex = xSemaphoreCreateMutex();
  g_pewo_mutex   = xSemaphoreCreateMutex();

  // I2C — shared by LCD and BME280
  Wire.begin(LCD_SDA, LCD_SCL);
  Wire.setClock(400000);

  // LCD init
  lcd_init();
  lcd_write_row(0, "GND-STATION v3.0");
  lcd_write_row(1, "Booting...");

  // SPI bus init — both MPU6500 and nRF24 share VSPI
  pinMode(NRF_CE,  OUTPUT); digitalWrite(NRF_CE,  LOW);
  pinMode(NRF_CSN, OUTPUT); digitalWrite(NRF_CSN, HIGH);
  pinMode(MPU_CS,  OUTPUT); digitalWrite(MPU_CS,  HIGH);
  SPI.begin(NRF_SCK, NRF_MISO, NRF_MOSI);
  // Note: RF task will call SPI.begin again on Core 0 — that's fine,
  // ESP32 SPI.begin is idempotent after first call.

  // MPU6500 init
  lcd_write_row(1, "Init MPU6500...");
  if (!mpu_init()) {
    lcd_write_row(1, "MPU FAIL");
    Serial.println("[MPU] Init failed — orientation disabled");
  }

  // BME280 init
  lcd_write_row(1, "Init BME280...");
  if (!bme_init()) {
    lcd_write_row(1, "BME FAIL");
    Serial.println("[BME] Init failed — weather disabled");
  }

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP("OSGS-01","telemetry",1,0,4);
  Serial.print("[WiFi] AP IP: ");
  Serial.println(WiFi.softAPIP());
  lcd_write_row(1, AP_SSID);

  // Web routes
  server.on("/",                  HTTP_GET, handle_root);
  server.on("/scan",              HTTP_GET, handle_scan);
  server.on("/stellar",           HTTP_GET, handle_stellar);
  server.on("/status",            HTTP_GET, handle_status);
  server.on("/telemetry",         HTTP_GET, handle_telemetry);
  server.on("/sensors",           HTTP_GET, handle_sensors);
  server.on("/orientation",       HTTP_GET, handle_orientation);
  server.on("/resetOrientation",  HTTP_GET, handle_reset_orientation);
  server.on("/setMode",           HTTP_GET, handle_set_mode);
  server.on("/setChannel",        HTTP_GET, handle_set_channel);
  server.on("/setPipe",           HTTP_GET, handle_set_pipe);
  server.on("/resetPeak",         HTTP_GET, handle_reset_peak);
  server.on("/transmit",          HTTP_GET, handle_transmit);
  server.on("/joystate",          HTTP_GET, handle_joystate);
  server.on("/txstatus",          HTTP_GET, handle_tx_status);
  server.on("/pewo/start",        HTTP_GET, handle_pewo_start);
  server.on("/pewo/stop",         HTTP_GET, handle_pewo_stop);
  server.on("/pewo/beacons",      HTTP_GET, handle_pewo_beacons);
  server.onNotFound(handle_not_found);
  server.begin();
  Serial.println("[HTTP] Server started");

  // Joystick ADC pins (input-only, no pinMode needed on 34/35)
  // GPIO32/33 are bidirectional so set as input
  pinMode(JOY_RX, INPUT);
  pinMode(JOY_RY, INPUT);
  analogReadResolution(12);   // 0-4095 range
  analogSetAttenuation(ADC_11db);  // full 0-3.3V range

  // RF task on Core 0, priority 2
  xTaskCreatePinnedToCore(task_rf, "RF_TASK", 8192, NULL, 2, NULL, 0);

  // Sensor task on Core 1, priority 3
  xTaskCreatePinnedToCore(task_sensors, "SENS_TASK", 4096, NULL, 3, NULL, 1);

  // Joystick task on Core 1, priority 1 (lowest — yields to everything)
  xTaskCreatePinnedToCore(task_joystick, "JOY_TASK", 2048, NULL, 1, NULL, 1);

  lcd_write_row(0, "GND-STATION v3.0");
  lcd_write_row(1, "Ready.");
  Serial.println("[GND-STATION v3] Ready.");
}

// ── Joystick task — Core 1 alongside sensors/HTTP ────────────
// Reads all 4 ADC channels at 20Hz, applies deadzone + normalisation,
// drives pan/zoom and channel selection without any UI polling needed.
//
// Left stick:
//   X → pan spectrum left/right (speed proportional to deflection)
//   Y → zoom in (up) / zoom out (down)
// Right stick:
//   X → scroll selected channel up/down
//   Y → full up (~>80%) = engage LOCK on g_joy_ch_sel
//        full down (~<20%) = return to SCAN

static int16_t joy_normalise(int raw) {
  // Returns -100..+100, 0 in deadzone
  int d = raw - JOY_CENTRE;
  if (d > -JOY_DEAD && d < JOY_DEAD) return 0;
  int range = JOY_CENTRE - JOY_DEAD;
  int norm = (d * 100) / range;
  if (norm >  100) norm =  100;
  if (norm < -100) norm = -100;
  return (int16_t)norm;
}

void task_joystick(void* pv) {
  esp_task_wdt_add(NULL);

  auto clamp = [](int v, int lo, int hi) -> int {
    return v < lo ? lo : (v > hi ? hi : v);
  };

  int zs = 0, ze = 125;
  uint8_t ch_sel = 76;

  uint8_t pan_tick  = 0;
  uint8_t zoom_tick = 0;
  uint8_t ch_tick   = 0;

  bool last_lock_trigger = false;
  bool last_scan_trigger = false;

  while (true) {
    int16_t lx = joy_normalise(analogRead(JOY_LX));
    int16_t ly = joy_normalise(analogRead(JOY_LY));
    int16_t rx = joy_normalise(analogRead(JOY_RX));
    int16_t ry = joy_normalise(analogRead(JOY_RY));

    g_joy_lx = lx; g_joy_ly = ly;
    g_joy_rx = rx; g_joy_ry = ry;

    // ── Read current mode safely ──────────────────────────────
    rf_mode_t cur_mode;
    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      cur_mode = g_mode;
      xSemaphoreGive(g_mutex);
    } else {
      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // ── STELLAR: consume sticks and skip RF controls ──────────
    if (cur_mode == MODE_STELLAR) {
      float dcx = 0, dcy = 0;
      if (lx >  20) dcx = +2;
      if (lx < -20) dcx = -2;
      if (ly >  20) dcy = +2;
      if (ly < -20) dcy = -2;

      if (dcx != 0 || dcy != 0) {
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          g_star_camx += dcx;
          g_star_camy += dcy;

          float best = 999999;
          int nearest = 0;
          for (int i = 0; i < STAR_COUNT; i++) {
            float dx = stars[i].x - g_star_camx;
            float dy = stars[i].y - g_star_camy;
            float d  = sqrtf(dx*dx + dy*dy);
            if (d < best) { best = d; nearest = i; }
          }
          g_star_selected = nearest;
          xSemaphoreGive(g_mutex);
        }
      }

      // Right Y still allows escaping back to SCAN
      bool scan_trigger = (ry < -80);
      if (scan_trigger && !last_scan_trigger) {
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          g_mode = MODE_SCAN;
          xSemaphoreGive(g_mutex);
        }
        buz_tone(1000, 80);
      }
      last_lock_trigger = false;  // reset so re-entry to other modes is clean
      last_scan_trigger = scan_trigger;

      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;  // <-- skip all RF pan/zoom/lock logic below
    }

    // ── Left X: pan (RF modes only) ───────────────────────────
    if (lx != 0) {
      pan_tick++;
      uint8_t rate = (uint8_t)(10 - (abs(lx) * 9 / 100));
      if (rate < 1) rate = 1;
      if (pan_tick >= rate) {
        pan_tick = 0;
        int visible = ze - zs + 1;
        int shift = (lx > 0) ? 1 : -1;
        if (abs(lx) > 70) shift *= 3;
        else if (abs(lx) > 40) shift *= 2;
        int ns = zs + shift, ne = ze + shift;
        if (ns < 0)   { ns = 0;   ne = visible - 1; }
        if (ne > 125) { ne = 125; ns = ne - visible + 1; }
        zs = ns; ze = ne;
      }
    } else { pan_tick = 0; }

    // ── Left Y: zoom (RF modes only) ──────────────────────────
    if (ly != 0) {
      zoom_tick++;
      uint8_t rate = (uint8_t)(8 - (abs(ly) * 7 / 100));
      if (rate < 1) rate = 1;
      if (zoom_tick >= rate) {
        zoom_tick = 0;
        int mid = (zs + ze) / 2;
        int half = (ze - zs) / 2;
        if (ly < 0) {
          half = half - 2;
          if (half < 5) half = 5;
        } else {
          half = half + 2;
          if (half > 63) half = 63;
        }
        zs = clamp(mid - half, 0, 120);
        ze = clamp(mid + half, 5, 125);
      }
    } else { zoom_tick = 0; }

    g_zoom_start = (int16_t)zs;
    g_zoom_end   = (int16_t)ze;

    // ── Right X: channel select (RF modes only) ───────────────
    if (rx != 0) {
      ch_tick++;
      uint8_t rate = (uint8_t)(12 - (abs(rx) * 10 / 100));
      if (rate < 1) rate = 1;
      if (ch_tick >= rate) {
        ch_tick = 0;
        int delta = (rx > 0) ? 1 : -1;
        if (abs(rx) > 70) delta *= 5;
        else if (abs(rx) > 40) delta *= 2;
        ch_sel = (uint8_t)clamp((int)ch_sel + delta, 0, 125);
        g_joy_ch_sel = ch_sel;

        if (ch_sel < zs || ch_sel > ze) {
          int visible = ze - zs + 1;
          if (ch_sel < zs) { zs = ch_sel; ze = zs + visible - 1; }
          else              { ze = ch_sel; zs = ze - visible + 1; }
          zs = clamp(zs, 0, 120);
          ze = clamp(ze, 5, 125);
          g_zoom_start = zs; g_zoom_end = ze;
        }
      }
    } else { ch_tick = 0; }

    // ── Right Y: lock/scan trigger ────────────────────────────
    bool lock_trigger = (ry > 80);
    bool scan_trigger = (ry < -80);

    if (lock_trigger && !last_lock_trigger) {
      if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_locked_ch       = ch_sel;
        g_mode            = MODE_LOCK;
        g_rx_packet_count = 0;
        g_rx_new          = false;
        g_rx_is_telemetry = false;
        xSemaphoreGive(g_mutex);
      }
      buz_lock_acquired();
    }
    if (scan_trigger && !last_scan_trigger) {
      if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_mode = MODE_SCAN;
        xSemaphoreGive(g_mutex);
      }
      buz_tone(1000, 80);
    }
    last_lock_trigger = lock_trigger;
    last_scan_trigger = scan_trigger;

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ── Route: /joystate ─────────────────────────────────────────
// Returns joystick values and current zoom/channel state for browser sync
void handle_joystate() {
  char buf[200];
  snprintf(buf, sizeof(buf),
    "{\"lx\":%d,\"ly\":%d,\"rx\":%d,\"ry\":%d,"
    "\"zs\":%d,\"ze\":%d,\"sel_ch\":%d}",
    (int)g_joy_lx, (int)g_joy_ly,
    (int)g_joy_rx, (int)g_joy_ry,
    (int)g_zoom_start, (int)g_zoom_end,
    (int)g_joy_ch_sel);
  send_json(buf);
}

// ── Serial output (structured CSV for laptop logging) ────────
// Format: GS,<ms>,<mode>,<ch>,<pkts>,<roll>,<pitch>,<yaw>,<temp>,<press>,<hum>,<alt>,<cycles>
// FC telemetry line (when locked and receiving flight computer):
// FC,<ms>,<state>,<alt_agl>,<vel>,<accel>,<max_alt>,<max_vel>,<batt>
void serial_print_state() {
  static unsigned long last_serial = 0;
  unsigned long now = millis();
  if (now - last_serial < 1000) return;
  last_serial = now;

  // Snapshot RF state
  rf_mode_t m;
  uint8_t lch;
  uint32_t pkts, cycles;
  bool is_telem;
  TelemetryPacket tpkt = {0};

  if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    m       = g_mode;
    lch     = (m == MODE_TX) ? g_tx_ch : g_locked_ch;
    pkts    = g_rx_packet_count;
    cycles  = g_scan_cycles;
    is_telem = g_rx_is_telemetry;
    if (is_telem) memcpy(&tpkt, (void*)&g_telem, sizeof(TelemetryPacket));
    xSemaphoreGive(g_mutex);
  } else { return; }

  // Snapshot sensors
  float roll, pitch, yaw, temp, press, hum, balt;
  if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    roll  = g_roll;  pitch = g_pitch; yaw = g_yaw;
    temp  = g_temperature; press = g_pressure;
    hum   = g_humidity;   balt  = g_baro_alt;
    xSemaphoreGive(g_sensor_mutex);
  } else { return; }

  const char* mstr = m==MODE_SCAN?"SCAN":
                     m==MODE_LOCK?"LOCK":
                     m==MODE_TX?"TX":
                     m==MODE_STELLAR?"STELLAR":"UNKNOWN";

  // Ground station line
  Serial.printf("GS,%lu,%s,%u,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%lu\n",
    now, mstr, lch, (unsigned long)pkts,
    roll, pitch, yaw,
    temp, press, hum, balt,
    (unsigned long)cycles);

  // Flight computer line (only when valid telemetry)
  if (is_telem && m == MODE_LOCK) {
    const char* sname = (tpkt.state <= 10) ? FLIGHT_STATES[tpkt.state] : "UNK";
    Serial.printf("FC,%lu,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%u\n",
      (unsigned long)tpkt.timestamp, sname,
      tpkt.altitude, tpkt.velocity, tpkt.acceleration,
      tpkt.max_altitude, tpkt.max_velocity,
      tpkt.battery_percent);
  }
}

// ── Serial command parser ────────────────────────────────────
// Commands typed into serial terminal (or sent by laptop script):
//   scan          → switch to SCAN mode
//   lock <ch>     → lock to channel N
//   tx <ch> <msg> → transmit msg on channel N
//   pipe <hex>    → set pipe address (10 hex chars)
//   peak          → reset peak hold
//   orient        → reset yaw to zero
//   status        → print one-shot status dump
//   help          → print command list
void serial_handle_commands() {
  static char cmd_buf[80];
  static uint8_t cmd_pos = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd_pos == 0) continue;
      cmd_buf[cmd_pos] = '\0';
      cmd_pos = 0;

      // Parse command
      char* tok = strtok(cmd_buf, " ");
      if (!tok) continue;

      if (strcmp(tok, "scan") == 0) {
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g_mode = MODE_SCAN; xSemaphoreGive(g_mutex);
        }
        Serial.println("OK mode=SCAN");

      } else if (strcmp(tok, "lock") == 0) {
        char* ch_s = strtok(NULL, " ");
        if (ch_s) {
          int ch = atoi(ch_s);
          if (ch >= 0 && ch <= 125) {
            if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
              g_locked_ch = ch; g_mode = MODE_LOCK;
              g_rx_packet_count = 0; g_rx_new = false;
              g_rx_is_telemetry = false;
              xSemaphoreGive(g_mutex);
            }
            Serial.printf("OK mode=LOCK ch=%d freq=%dMHz\n", ch, 2400+ch);
          } else { Serial.println("ERR ch must be 0-125"); }
        } else { Serial.println("ERR usage: lock <ch>"); }

      } else if (strcmp(tok, "tx") == 0) {
        char* ch_s = strtok(NULL, " ");
        char* msg  = strtok(NULL, "");  // rest of line
        if (ch_s && msg) {
          int ch = atoi(ch_s);
          if (ch >= 0 && ch <= 125) {
            uint8_t plen = (uint8_t)min((int)strlen(msg), TX_PAYLOAD_MAX);
            if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
              g_tx_ch = ch;
              memcpy((void*)g_tx_payload, msg, plen);
              g_tx_payload_len = plen;
              g_tx_trigger = true;
              g_mode = MODE_TX;
              xSemaphoreGive(g_mutex);
            }
            Serial.printf("OK tx ch=%d payload=%s\n", ch, msg);
          } else { Serial.println("ERR ch must be 0-125"); }
        } else { Serial.println("ERR usage: tx <ch> <message>"); }

      } else if (strcmp(tok, "pipe") == 0) {
        char* addr = strtok(NULL, " ");
        if (addr && strlen(addr) == 10) {
          uint8_t na[5];
          for (int i = 0; i < 5; i++) {
            char hx[3] = {addr[i*2], addr[i*2+1], 0};
            na[i] = (uint8_t)strtol(hx, nullptr, 16);
          }
          if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            memcpy((void*)g_pipe_addr, na, 5);
            g_pipe_changed = true;
            xSemaphoreGive(g_mutex);
          }
          Serial.printf("OK pipe=%s\n", addr);
        } else { Serial.println("ERR usage: pipe <10hexchars>  e.g. pipe E7E7E7E7E7"); }

      } else if (strcmp(tok, "peak") == 0) {
        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          memset((void*)g_ch_avg,  0, NUM_CH);
          memset((void*)g_ch_peak, 0, NUM_CH);
          xSemaphoreGive(g_mutex);
        }
        Serial.println("OK peak reset");

      } else if (strcmp(tok, "orient") == 0) {
        if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          g_yaw = 0.0f; xSemaphoreGive(g_sensor_mutex);
        }
        Serial.println("OK yaw zeroed");

      } else if (strcmp(tok, "status") == 0) {
        Serial.printf("uptime=%lus heap=%uB mode=%s ch=%u cycles=%lu\n",
          millis()/1000, ESP.getFreeHeap(),
          g_mode==MODE_SCAN?"SCAN":g_mode==MODE_LOCK?"LOCK":"TX",
          (unsigned)(g_mode==MODE_TX?g_tx_ch:g_locked_ch),
          (unsigned long)g_scan_cycles);

      } else if (strcmp(tok, "help") == 0) {
        Serial.println("Commands:");
        Serial.println("  scan              - switch to scan mode");
        Serial.println("  lock <0-125>      - lock to channel");
        Serial.println("  tx <0-125> <msg>  - transmit message");
        Serial.println("  pipe <10hex>      - set pipe address");
        Serial.println("  peak              - reset peak hold");
        Serial.println("  orient            - zero yaw");
        Serial.println("  status            - print status");
        Serial.println("CSV output: GS,ms,mode,ch,pkts,roll,pitch,yaw,temp,press,hum,alt,cycles");
        Serial.println("            FC,ms,state,alt,vel,accel,maxalt,maxvel,batt");

      } else {
        Serial.printf("ERR unknown command '%s' — type 'help'\n", tok);
      }

    } else {
      if (cmd_pos < sizeof(cmd_buf) - 1) cmd_buf[cmd_pos++] = c;
    }
  }
}

// ── loop() — Core 1, HTTP + buzzer update ────────────────────
void loop() {
  server.handleClient();
  buz_update();
  serial_print_state();
  serial_handle_commands();

  // Orientation reset from HTTP
  if (g_imu_reset) {
    g_imu_reset = false;
    if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      g_yaw = 0.0f;
      xSemaphoreGive(g_sensor_mutex);
    }
  }

  // Mode switch beep
  static rf_mode_t last_mode = MODE_SCAN;
  rf_mode_t cur_m;
  if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    cur_m = g_mode; xSemaphoreGive(g_mutex);
  } else { cur_m = last_mode; }
  if (cur_m != last_mode) { buz_mode(cur_m); last_mode = cur_m; }

  delay(1);
}

// ═══════════════════════════════════════════════════════════════
//  DASHBOARD HTML v3 — Galaxy Ace GT-S5830 optimised
//  320×480 HVGA · Android 2.3.6 WebKit · Fixed pixel layout
//  Tabs: SCAN · LOCK · TX · ORIENT · SENSORS
// ═══════════════════════════════════════════════════════════════
const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=320,user-scalable=no">
<title>GND-01</title>
<style>
*{margin:0;padding:0;-webkit-box-sizing:border-box;box-sizing:border-box}
html,body{width:320px;background:#000;color:#00ff41;font-family:monospace;font-size:11px;overflow:hidden}
#hdr{width:312px;height:24px;margin:3px 4px 2px 4px;border:1px solid #00cc33;padding:2px 5px;display:table}
#hl{display:table-cell;font-size:12px;letter-spacing:2px;font-weight:bold;vertical-align:middle}
#hr{display:table-cell;text-align:right;font-size:9px;color:#00aa2a;vertical-align:middle;line-height:1.4}
/* 5 tabs */
#tabs{width:312px;height:26px;margin:0 4px 2px 4px;display:table;table-layout:fixed}
.tab{display:table-cell;border:1px solid #004d12;background:#001a00;color:#006622;text-align:center;vertical-align:middle;font-family:monospace;font-size:9px;cursor:pointer;letter-spacing:0px}
.tab.on{border-color:#00ff41;background:#002600;color:#00ff41}
.tab.am{border-color:#664400;color:#664400}
.tab.am.on{border-color:#ffaa00;background:#1a0e00;color:#ffaa00}
/* spectrum canvas area */
#sw{width:312px;height:190px;margin:0 4px 2px 4px;border:1px solid #00cc33;position:relative;overflow:hidden}
#cvs{display:block}
/* overlays */
#lkovr{display:none;position:absolute;left:0;bottom:0;width:310px;background:#000;border-top:1px solid #00aa2a;padding:3px 5px;z-index:10}
#lkovr.on{display:block}
#lkovr-t{display:table;width:100%;table-layout:fixed}
.lc{display:table-cell;text-align:center}
.ll{font-size:8px;color:#005510;letter-spacing:1px}
.lv{font-size:13px;font-weight:bold;color:#00ff41}
#txovr{display:none;position:absolute;left:0;bottom:0;width:310px;background:#000;border-top:1px solid #ffaa00;padding:3px 5px;z-index:10}
#txovr.on{display:block}
#txovr-t{display:table;width:100%;table-layout:fixed}
.tc{display:table-cell;text-align:center}
.tl{font-size:8px;color:#664400;letter-spacing:1px}
.tv{font-size:13px;font-weight:bold;color:#ffaa00}
#flovr{display:none;position:absolute;left:0;bottom:0;width:310px;background:#000;border-top:1px solid #00ff41;padding:3px 5px;z-index:11}
#flovr.on{display:block}
/* info bar */
#ib{width:312px;height:17px;margin:0 4px 2px 4px;border:1px solid #003300;padding:1px 4px;display:table;table-layout:fixed}
#ibl{display:table-cell;font-size:9px;color:#00aa2a;vertical-align:middle}
#ibr{display:table-cell;text-align:right;font-size:9px;color:#00aa2a;vertical-align:middle}
/* controls */
#cr{width:312px;height:34px;margin:0 4px 2px 4px;display:table;table-layout:fixed}
.cc{display:table-cell;vertical-align:middle;text-align:center;padding:1px}
.btn{width:100%;height:30px;background:#001a00;border:1px solid #00aa2a;color:#00ff41;font-family:monospace;font-size:9px;cursor:pointer}
.btn:active{background:#003300}
.btn.am{border-color:#ffaa00;color:#ffaa00}
#chi{width:100%;height:30px;background:#000;border:1px solid #00aa2a;color:#00ff41;font-family:monospace;font-size:11px;text-align:center}
/* TX data row */
#txrow{width:312px;height:34px;margin:0 4px 2px 4px;display:none;table-layout:fixed}
#txrow.on{display:table}
.tc2{display:table-cell;vertical-align:middle;padding:1px}
#txdata{width:100%;height:30px;background:#000;border:1px solid #ffaa00;color:#ffaa00;font-family:monospace;font-size:10px;padding:2px 3px}
/* orientation page */
#orient-page{display:none;width:312px;padding:4px}
#orient-page.on{display:block}
#ocvs{display:block;background:#000;border:1px solid #003300}
#oinfo{display:table;width:100%;table-layout:fixed;margin-top:4px}
.oc{display:table-cell;text-align:center}
.ol{font-size:8px;color:#005510}
.ov{font-size:14px;font-weight:bold;color:#00ff41}
/* sensors page */
#sens-page{display:none;width:312px;padding:4px}
#sens-page.on{display:block}
.scard{border:1px solid #003300;padding:4px 6px;margin-bottom:4px;background:#020902}
.srow{display:table;width:100%;table-layout:fixed;padding:2px 0}
.scell{display:table-cell;vertical-align:middle}
.sk{font-size:9px;color:#005510}
.sv{font-size:15px;font-weight:bold;color:#00ff41}
.su{font-size:9px;color:#006622}
/* map page */
#stellar-page{
display:none;
width:312px;
padding:3px 4px
}

#stellar-page.on{
display:block
}

#stellar-cvs{
display:block;
background:#000;
border:1px solid #003300
}
#map-hud{display:table;width:100%;table-layout:fixed;margin-top:3px;background:#00000a;border:1px solid #001a40;padding:2px 0}
.mhc{display:table-cell;text-align:center;padding:2px 0}
.mhl{font-size:8px;color:#004488;letter-spacing:1px}
.mhv{font-size:11px;font-weight:bold;color:#4499ff;font-family:monospace}
#map-info{margin-top:3px;font-size:9px;color:#002244;font-family:monospace;height:14px;overflow:hidden;background:#00000a;border:1px solid #001a40;padding:2px 4px}
</style>
</head>
<body>

<div id="hdr">
  <div id="hl">&#9632; GND-01</div>
  <div id="hr"><span id="hm">SCAN</span><br><span id="ht">--:--:--</span></div>
</div>

<div id="tabs" style="width:312px;height:26px;margin:0 4px 2px 4px;display:table;table-layout:fixed">
  <div class="tab on" id="t0" onclick="showTab(0)">SCAN</div>
  <div class="tab"    id="t1" onclick="showTab(1)">LOCK</div>
  <div class="tab am" id="t2" onclick="showTab(2)">TX</div>
  <div class="tab"    id="t3" onclick="showTab(3)">ORNT</div>
  <div class="tab"    id="t4" onclick="showTab(4)">SENS</div>
  <div class="tab"    id="t5" onclick="showTab(5)">STELLAR</div>
</div>

<!-- ── SCAN / LOCK / TX view (tabs 0,1,2 share this layout) ── -->
<div id="rf-view">
  <div id="sw">
    <canvas id="cvs" width="310" height="188"></canvas>
    <div id="lkovr">
      <div id="lkovr-t">
        <div class="lc"><div class="ll">CH</div><div class="lv" id="lk-ch">--</div></div>
        <div class="lc"><div class="ll">FREQ</div><div class="lv" id="lk-fr">----</div></div>
        <div class="lc"><div class="ll">PKTS</div><div class="lv" id="lk-pk">0</div></div>
        <div class="lc"><div class="ll">TYPE</div><div class="lv" id="lk-ty">--</div></div>
      </div>
    </div>
    <div id="flovr">
      <div style="display:table;width:100%;table-layout:fixed">
        <div style="display:table-cell;text-align:center"><div class="ll">ALT m</div><div class="lv" id="fl-alt">--</div></div>
        <div style="display:table-cell;text-align:center"><div class="ll">VEL m/s</div><div class="lv" id="fl-vel">--</div></div>
        <div style="display:table-cell;text-align:center"><div class="ll">ACC g</div><div class="lv" id="fl-acc">--</div></div>
        <div style="display:table-cell;text-align:center"><div class="ll">STATE</div><div class="lv" id="fl-st">--</div></div>
      </div>
      <div style="display:table;width:100%;table-layout:fixed;margin-top:2px">
        <div style="display:table-cell;text-align:center"><div class="ll">MAXALT</div><div style="font-size:10px;color:#00aa2a" id="fl-ma">--</div></div>
        <div style="display:table-cell;text-align:center"><div class="ll">MAXVEL</div><div style="font-size:10px;color:#00aa2a" id="fl-mv">--</div></div>
        <div style="display:table-cell;text-align:center"><div class="ll">MAXACC</div><div style="font-size:10px;color:#00aa2a" id="fl-mc">--</div></div>
        <div style="display:table-cell;text-align:center"><div class="ll">BATT%</div><div style="font-size:10px;color:#00aa2a" id="fl-bt">--</div></div>
      </div>
    </div>
    <div id="txovr">
      <div id="txovr-t">
        <div class="tc"><div class="tl">TX CH</div><div class="tv" id="tx-ch">--</div></div>
        <div class="tc"><div class="tl">SENT</div><div class="tv" id="tx-sn">0</div></div>
        <div class="tc"><div class="tl">ACKED</div><div class="tv" id="tx-ak">0</div></div>
        <div class="tc"><div class="tl">LOSS%</div><div class="tv" id="tx-ls">0</div></div>
      </div>
    </div>
  </div>
  <div id="ib">
    <div id="ibl">ACT:<span id="ach" style="color:#00ff41">--</span></div>
    <div id="ibr">PK:<span id="pch" style="color:#00ff41">--</span>&nbsp;<span id="zrng" style="color:#004d12">2400M</span></div>
  </div>
  <div id="cr">
    <div class="cc" style="width:13%"><input id="chi" type="number" min="0" max="125" value="76"></div>
    <div class="cc" style="width:13%"><button class="btn" onclick="doLock()">LCK</button></div>
    <div class="cc" style="width:13%"><button class="btn" onclick="goScan()">SCN</button></div>
    <div class="cc" style="width:13%"><button class="btn" onclick="doReset()">RST</button></div>
    <div class="cc" style="width:10%"><button class="btn" onclick="zIn()">+</button></div>
    <div class="cc" style="width:10%"><button class="btn" onclick="zOut()">-</button></div>
    <div class="cc" style="width:28%"><button class="btn" onclick="zFit()">FIT</button></div>
  </div>
  <div id="txrow">
    <div class="tc2" style="width:62%"><input id="txdata" type="text" placeholder="ASCII or 0xHEX" value="PING"></div>
    <div class="tc2" style="width:19%"><button class="btn am" onclick="doSend()">SND</button></div>
    <div class="tc2" style="width:19%"><button class="btn am" id="repbtn" onclick="toggleRepeat()">RPT</button></div>
  </div>
</div>

<!-- ── ORIENTATION view (tab 3) ── -->
<div id="orient-page">
  <!-- 3D wireframe cyberdeck drawn on canvas -->
  <canvas id="ocvs" width="310" height="200"></canvas>
  <div id="oinfo">
    <div class="oc"><div class="ol">ROLL</div><div class="ov" id="o-roll">0.0</div></div>
    <div class="oc"><div class="ol">PITCH</div><div class="ov" id="o-pitch">0.0</div></div>
    <div class="oc"><div class="ol">YAW</div><div class="ov" id="o-yaw">0.0</div></div>
  </div>
  <div style="margin-top:4px;display:table;width:100%;table-layout:fixed">
    <div style="display:table-cell;padding:2px">
      <button class="btn" style="width:100%;height:28px;font-size:9px" onclick="resetOrient()">ZERO YAW</button>
    </div>
    <div style="display:table-cell;padding:2px;font-size:9px;color:#005510;vertical-align:middle">
      SRC: <span id="o-src" style="color:#006622">GS-IMU</span>
    </div>
  </div>
  <!-- Rocket orientation when in LOCK+telemetry mode -->
  <div id="o-fc-row" style="display:none;margin-top:3px;border-top:1px solid #003300;padding-top:3px">
    <div style="font-size:8px;color:#005510;margin-bottom:2px">ROCKET FC ACCEL (g)</div>
    <div style="display:table;width:100%;table-layout:fixed">
      <div style="display:table-cell;text-align:center"><div class="ol">AX</div><div style="font-size:12px;color:#00aa2a" id="o-ax">--</div></div>
      <div style="display:table-cell;text-align:center"><div class="ol">ALT</div><div style="font-size:12px;color:#00aa2a" id="o-falt">--</div></div>
      <div style="display:table-cell;text-align:center"><div class="ol">STATE</div><div style="font-size:12px;color:#00aa2a" id="o-fst">--</div></div>
    </div>
  </div>
</div>

<!-- ── SENSORS view (tab 4) ── -->
<div id="sens-page">
  <div class="scard">
    <div style="font-size:9px;color:#005510;letter-spacing:1px;margin-bottom:3px">BME280 · LOCAL WEATHER</div>
    <div class="srow">
      <div class="scell" style="width:50%"><div class="sk">TEMPERATURE</div><div><span class="sv" id="s-temp">--</span><span class="su"> °C</span></div></div>
      <div class="scell" style="width:50%"><div class="sk">HUMIDITY</div><div><span class="sv" id="s-hum">--</span><span class="su"> %</span></div></div>
    </div>
    <div class="srow">
      <div class="scell" style="width:50%"><div class="sk">PRESSURE</div><div><span class="sv" id="s-pres">--</span><span class="su"> hPa</span></div></div>
      <div class="scell" style="width:50%"><div class="sk">BARO ALT</div><div><span class="sv" id="s-alt">--</span><span class="su"> m</span></div></div>
    </div>
  </div>
  <div class="scard">
    <div style="font-size:9px;color:#005510;letter-spacing:1px;margin-bottom:3px">MPU6500 · GS ORIENTATION</div>
    <div class="srow">
      <div class="scell" style="width:33%"><div class="sk">ROLL</div><div><span class="sv" id="s-roll">--</span><span class="su">°</span></div></div>
      <div class="scell" style="width:33%"><div class="sk">PITCH</div><div><span class="sv" id="s-pitch">--</span><span class="su">°</span></div></div>
      <div class="scell" style="width:34%"><div class="sk">YAW</div><div><span class="sv" id="s-yaw">--</span><span class="su">°</span></div></div>
    </div>
    <div class="srow">
      <div class="scell" style="width:33%"><div class="sk">AX</div><div style="font-size:12px;color:#006622" id="s-ax">--</div></div>
      <div class="scell" style="width:33%"><div class="sk">AY</div><div style="font-size:12px;color:#006622" id="s-ay">--</div></div>
      <div class="scell" style="width:34%"><div class="sk">AZ</div><div style="font-size:12px;color:#006622" id="s-az">--</div></div>
    </div>
  </div>
  <!-- FC telemetry sensor data when locked -->
  <div class="scard" id="s-fc-card" style="display:none">
    <div style="font-size:9px;color:#005510;letter-spacing:1px;margin-bottom:3px">ROCKET FC · TELEMETRY</div>
    <div class="srow">
      <div class="scell" style="width:50%"><div class="sk">ALT AGL</div><div><span class="sv" id="s-falt">--</span><span class="su"> m</span></div></div>
      <div class="scell" style="width:50%"><div class="sk">VELOCITY</div><div><span class="sv" id="s-fvel">--</span><span class="su"> m/s</span></div></div>
    </div>
    <div class="srow">
      <div class="scell" style="width:50%"><div class="sk">MAX ALT</div><div style="font-size:12px;color:#006622" id="s-fma">--</div></div>
      <div class="scell" style="width:50%"><div class="sk">STATE</div><div style="font-size:12px;color:#006622" id="s-fst">--</div></div>
    </div>
  </div>
</div>

<!-- ── MAP view (tab 5) ── -->
<!-- Left stick moves crosshair · Right stick places/clears markers -->
<!-- RF active channels appear as blips at their grid X position     -->
<!-- ── STELLAR view (tab 5) ── -->

<div id="stellar-page">

<canvas
id="stellar-cvs"
width="304"
height="210">
</canvas>

<div id="map-hud">

<div class="mhc">
<div class="mhl">
SYSTEM
</div>
<div class="mhv"
id="st-name">
SOL
</div>
</div>

<div class="mhc">
<div class="mhl">
X
</div>
<div class="mhv"
id="st-x">
0
</div>
</div>

<div class="mhc">
<div class="mhl">
Y
</div>
<div class="mhv"
id="st-y">
0
</div>
</div>

<div class="mhc">
<div class="mhl">
VISITED
</div>
<div class="mhv"
id="st-vis">
NO
</div>
</div>

</div>

<div id="map-info">
STAR MAP READY
</div>

</div>

<div id="sb">
  <div id="sbl">UP:0s CY:0</div>
  <div id="sbr">J:<span id="sb-joy" style="color:#006622">--</span> IMU:<span id="sb-imu">--</span></div>
</div>

<script>
// ── constants ─────────────────────────────────────────────────
var N=126,CVS_W=310,CVS_H=188,OCVS_W=310,OCVS_H=200;
var gCh=[],gPeak=[];
var gMode='SCAN',gLCh=76,gTxCh=76;
var zS=0,zE=125;
var dragX=-1,dragZS=0;
var gStat={},gTelem={};
var gRoll=0,gPitch=0,gYaw=0;
var txRepeat=false;
var curTab=0;
for(var i=0;i<N;i++){gCh[i]=0;gPeak[i]=0;}

// ── canvas handles ────────────────────────────────────────────
var scvs=null;
var sctx=null;
var ocvs=document.getElementById('ocvs');
var octx=ocvs.getContext('2d');

// Main spectrum canvas
var cvs=document.getElementById('cvs');
var ctx=cvs.getContext('2d');

var WIFI_CH=[12,37,62],WIFI_LBL=['W1','W6','W11'];

function p2(n){return n<10?'0'+n:''+n;}
function fix1(v){return parseFloat(v).toFixed(1);}
function fix2(v){return parseFloat(v).toFixed(2);}

function get(url,cb){
  var r=new XMLHttpRequest();r.open('GET',url,true);
  r.onreadystatechange=function(){if(r.readyState===4&&r.status===200){try{cb(JSON.parse(r.responseText));}catch(e){}}};
  r.send();
}

// ── map state ─────────────────────────────────────────────────
var MAP_W=304,MAP_H=210;
var MAP_COLS=38,MAP_ROWS=26;    // grid cells
var MAP_CW=Math.floor(MAP_W/MAP_COLS);
var MAP_CH=Math.floor(MAP_H/MAP_ROWS);

// Crosshair position in grid cells (float for smooth movement)
var mpx=MAP_COLS/2, mpy=MAP_ROWS/2;
// Velocity accumulator driven by joystick
var mvx=0, mvy=0;
// Trail: array of {x,y} grid positions
var mtrail=[];
var MAX_TRAIL=80;
// Markers placed by right stick push (RY full up)
var mmarks=[];
// Last joy state for edge detection
var last_ry_up=false, last_ry_down=false;
// Sweep angle for radar effect
var msweep=0;
// RF blips: channels with signal strength > threshold
var mblips=[];  // {gx,gy,freq,strength,age}

var SECTOR_NAMES=[
  ['A1','A2','A3','A4'],['B1','B2','B3','B4'],
  ['C1','C2','C3','C4'],['D1','D2','D3','D4']
];
function mapSector(x,y){
  var sc=Math.floor(x/(MAP_COLS/4));
  var sr=Math.floor(y/(MAP_ROWS/4));
  sc=sc<0?0:sc>3?3:sc;
  sr=sr<0?0:sr>3?3:sr;
  return SECTOR_NAMES[sr][sc];
}

// Rebuild RF blips from current channel data
// Each active channel maps to a grid X position (0-125 → 0-MAP_COLS)
// Y position is pseudo-random but stable (seeded by channel number)
function rebuildBlips(){
  mblips=[];
  for(var i=0;i<N;i++){
    if(gCh[i]>30){
      var gx=Math.round(i/125*(MAP_COLS-1));
      // Deterministic Y from channel number — looks like scatter plot
      var gy=Math.floor(((i*7+13)%17)/(17)*(MAP_ROWS-2))+1;
      mblips.push({gx:gx,gy:gy,freq:2400+i,strength:gCh[i],age:0,ch:i});
    }
  }
}

function drawStellar() {
  scvs = document.getElementById('stellar-cvs');
  if (!scvs) return;
  sctx = scvs.getContext('2d');
  if (!sctx) return;

  get('/stellar', function(d) {
    if (curTab !== 5) return;

    var W = 304, H = 210;
    sctx.fillStyle = '#000008';
    sctx.fillRect(0, 0, W, H);

    // ── Nebula clouds (static, seeded by position) ────────────
    var nebulas = [
      {x:60,  y:80,  rx:55, ry:30, col:'rgba(60,0,120,0.18)'},
      {x:220, y:130, rx:70, ry:35, col:'rgba(0,30,100,0.20)'},
      {x:140, y:40,  rx:40, ry:20, col:'rgba(80,0,60,0.15)'},
      {x:260, y:60,  rx:35, ry:25, col:'rgba(0,60,80,0.18)'}
    ];
    for (var n = 0; n < nebulas.length; n++) {
      var nb = nebulas[n];
      sctx.save();
      sctx.translate(nb.x, nb.y);
      sctx.scale(1, nb.ry / nb.rx);
      sctx.beginPath();
      sctx.arc(0, 0, nb.rx, 0, 6.2832);
      sctx.fillStyle = nb.col;
      sctx.fill();
      sctx.restore();
    }

    // ── Background star field (static, seeded) ────────────────
    var seed = 42;
    function rand() { seed = (seed * 1664525 + 1013904223) & 0xffffffff; return (seed >>> 0) / 4294967296; }
    for (var s = 0; s < 120; s++) {
      var sx = rand() * W;
      var sy = rand() * H;
      var sb = rand();
      var sr = sb > 0.97 ? 1.2 : sb > 0.90 ? 0.8 : 0.4;
      var sa = 0.3 + sb * 0.5;
      // Tint: mostly white, some blue, some red
      var stint = rand();
      var scol = stint > 0.85 ? 'rgba(150,180,255,' :
                 stint > 0.70 ? 'rgba(255,160,140,' :
                                'rgba(200,210,255,';
      sctx.beginPath();
      sctx.arc(sx, sy, sr, 0, 6.2832);
      sctx.fillStyle = scol + sa + ')';
      sctx.fill();
    }

    // ── Grid (very subtle) ────────────────────────────────────
    sctx.strokeStyle = 'rgba(0,60,120,0.15)';
    sctx.lineWidth = 0.5;
    for (var gx = 0; gx < W; gx += 24) {
      sctx.beginPath(); sctx.moveTo(gx, 0); sctx.lineTo(gx, H); sctx.stroke();
    }
    for (var gy = 0; gy < H; gy += 24) {
      sctx.beginPath(); sctx.moveTo(0, gy); sctx.lineTo(W, gy); sctx.stroke();
    }

    // ── Camera offset ─────────────────────────────────────────
    var cx = d.cx || 0, cy = d.cy || 0;

    // ── Draw connection lines to nearby stars ─────────────────
    for (var i = 0; i < d.stars.length; i++) {
      for (var j = i + 1; j < d.stars.length; j++) {
        var si = d.stars[i], sj = d.stars[j];
        var dx = si.x - sj.x, dy = si.y - sj.y;
        var dist = Math.sqrt(dx*dx + dy*dy);
        if (dist < 120) {
          var ax = 152 + si.x - cx, ay = 105 + si.y - cy;
          var bx = 152 + sj.x - cx, by = 105 + sj.y - cy;
          var alpha = (1 - dist / 120) * 0.25;
          sctx.strokeStyle = 'rgba(80,120,200,' + alpha + ')';
          sctx.lineWidth = 0.5;
          sctx.beginPath(); sctx.moveTo(ax, ay); sctx.lineTo(bx, by); sctx.stroke();
        }
      }
    }

    // ── Draw stars ────────────────────────────────────────────
    for (var i = 0; i < d.stars.length; i++) {
      var st = d.stars[i];
      var sx = 152 + st.x - cx;
      var sy = 105 + st.y - cy;

      // Skip if off-canvas (with margin)
      if (sx < -20 || sx > W+20 || sy < -20 || sy > H+20) continue;

      var isSel = (i === d.sel);

      // Star colour — alternate between blue-white and red-orange by index
      var isRed = (i % 3 === 1);
      var isBlue = (i % 3 === 2);
      var starR = isSel ? 4 : isRed ? 2.5 : isBlue ? 2 : 2;
      var coreCol = isSel    ? '#ffffff' :
                   isRed    ? '#ff6644' :
                   isBlue   ? '#88aaff' :
                              '#ccddff';
      var glowCol = isSel ? 'rgba(160,200,255,0.35)' :
                   isRed  ? 'rgba(255,80,40,0.25)'  :
                   isBlue ? 'rgba(80,120,255,0.25)' :
                            'rgba(140,160,255,0.20)';

      // Glow ring
      sctx.beginPath();
      sctx.arc(sx, sy, isSel ? 10 : 6, 0, 6.2832);
      sctx.fillStyle = glowCol;
      sctx.fill();

      // Star core
      sctx.beginPath();
      sctx.arc(sx, sy, starR, 0, 6.2832);
      sctx.fillStyle = coreCol;
      sctx.fill();

      // Selection ring — pulsing effect via two rings
      if (isSel) {
        sctx.strokeStyle = 'rgba(100,180,255,0.9)';
        sctx.lineWidth = 1;
        sctx.beginPath();
        sctx.arc(sx, sy, 8, 0, 6.2832);
        sctx.stroke();
        sctx.strokeStyle = 'rgba(100,180,255,0.35)';
        sctx.lineWidth = 0.5;
        sctx.beginPath();
        sctx.arc(sx, sy, 14, 0, 6.2832);
        sctx.stroke();

        // Cross-hair spikes
        sctx.strokeStyle = 'rgba(100,180,255,0.6)';
        sctx.lineWidth = 0.5;
        sctx.beginPath();
        sctx.moveTo(sx - 18, sy); sctx.lineTo(sx - 10, sy);
        sctx.moveTo(sx + 10, sy); sctx.lineTo(sx + 18, sy);
        sctx.moveTo(sx, sy - 18); sctx.lineTo(sx, sy - 10);
        sctx.moveTo(sx, sy + 10); sctx.lineTo(sx, sy + 18);
        sctx.stroke();
      }

      // Label — selected gets full name, others get short name offset
      if (isSel) {
        sctx.fillStyle = '#ffffff';
        sctx.font = 'bold 9px monospace';
        sctx.fillText(st.n, sx + 16, sy + 4);
        // Coord readout under name
        sctx.fillStyle = 'rgba(100,160,255,0.8)';
        sctx.font = '8px monospace';
        sctx.fillText(Math.round(st.x)+','+Math.round(st.y), sx + 16, sy + 13);
      } else if (sx > 5 && sx < W-5 && sy > 5 && sy < H-20) {
        sctx.fillStyle = isRed  ? 'rgba(255,140,100,0.7)' :
                         isBlue ? 'rgba(140,180,255,0.7)' :
                                  'rgba(160,180,220,0.6)';
        sctx.font = '7px monospace';
        sctx.fillText(st.n, sx + 5, sy - 4);
      }
    }

    // ── Scan line (moving horizontal line for radar feel) ─────
    var scanY = Math.floor((Date.now() / 30) % H);
    sctx.strokeStyle = 'rgba(0,180,80,0.12)';
    sctx.lineWidth = 1;
    sctx.beginPath();
    sctx.moveTo(0, scanY);
    sctx.lineTo(W, scanY);
    sctx.stroke();

    // ── Centre crosshair ──────────────────────────────────────
    sctx.strokeStyle = 'rgba(0,200,80,0.3)';
    sctx.lineWidth = 0.5;
    sctx.beginPath();
    sctx.moveTo(152 - 8, 105); sctx.lineTo(152 + 8, 105);
    sctx.moveTo(152, 105 - 8); sctx.lineTo(152, 105 + 8);
    sctx.stroke();

    // ── HUD corners ───────────────────────────────────────────
    var corners = [[0,0,1,1],[W,0,-1,1],[0,H,1,-1],[W,H,-1,-1]];
    sctx.strokeStyle = 'rgba(0,180,80,0.5)';
    sctx.lineWidth = 1;
    for (var c = 0; c < corners.length; c++) {
      var co = corners[c];
      sctx.beginPath();
      sctx.moveTo(co[0] + co[2]*12, co[1]);
      sctx.lineTo(co[0],            co[1]);
      sctx.lineTo(co[0],            co[1] + co[3]*12);
      sctx.stroke();
    }

    // ── Update HUD panels ─────────────────────────────────────
    if (d.stars && d.sel !== undefined && d.stars[d.sel]) {
      var sel = d.stars[d.sel];
      document.getElementById('st-name').innerHTML = sel.n;
      document.getElementById('st-x').innerHTML    = Math.round(sel.x);
      document.getElementById('st-y').innerHTML    = Math.round(sel.y);
      document.getElementById('st-vis').innerHTML  = sel.v ? 'YES' : 'NO';
    }
  });
}

function updateMapHUD(){
  var px=Math.round(mpx), py=Math.round(mpy);
  document.getElementById('mp-pos').innerHTML=px+','+py;
  document.getElementById('mp-sec').innerHTML=mapSector(px,py);
  document.getElementById('mp-mrk').innerHTML=mmarks.length;
  // Find nearest RF blip
  var nearDist=999, nearCh='--';
  for(var b=0;b<mblips.length;b++){
    var d=Math.abs(mblips[b].gx-px)+Math.abs(mblips[b].gy-py);
    if(d<nearDist){nearDist=d;nearCh=mblips[b].ch;}
  }
  document.getElementById('mp-rf').innerHTML=nearDist<10?nearCh+'ch':'--';
}


// Joystick values used by map (set by pollJoy)
var gJoyLX=0,gJoyLY=0,gJoyRX=0,gJoyRY=0;

// Map runs at 100ms regardless — setInterval handles it
setInterval(drawStellar,150);
function showTab(n) {
  curTab = n;

  var tabs = document.querySelectorAll('.tab');
  tabs[0].className = 'tab' + (n===0 ? ' on' : '');
  tabs[1].className = 'tab' + (n===1 ? ' on' : '');
  tabs[2].className = 'tab am' + (n===2 ? ' on' : '');
  tabs[3].className = 'tab' + (n===3 ? ' on' : '');
  tabs[4].className = 'tab' + (n===4 ? ' on' : '');
  tabs[5].className = 'tab' + (n===5 ? ' on' : '');

  var rfv = document.getElementById('rf-view');
  var orp = document.getElementById('orient-page');
  var snp = document.getElementById('sens-page');
  var stp = document.getElementById('stellar-page');

  rfv.style.display = (n <= 2) ? 'block' : 'none';
  orp.className = n===3 ? 'on' : '';
  snp.className = n===4 ? 'on' : '';
  stp.className = n===5 ? 'on' : '';

  if      (n===0) { goScan(); get('/setMode?mode=SCAN',    function(){}); }
  else if (n===1) { goLock(); get('/setMode?mode=LOCK',    function(){}); }
  else if (n===2) { goTX();   get('/setMode?mode=TX',      function(){}); }
  else if (n===5) {           get('/setMode?mode=STELLAR', function(){}); }
}
// ── spectrum draw ─────────────────────────────────────────────
function draw(){
  var W=CVS_W,H=CVS_H,vis=zE-zS+1,bw=W/vis;
  ctx.fillStyle='#000';ctx.fillRect(0,0,W,H);
  ctx.strokeStyle='#001800';ctx.lineWidth=1;
  for(var g=1;g<=3;g++){ctx.beginPath();ctx.moveTo(0,Math.round(H*g/4));ctx.lineTo(W,Math.round(H*g/4));ctx.stroke();}
  ctx.strokeStyle='#001d00';
  for(var c=0;c<=125;c+=10){
    if(c<zS||c>zE)continue;
    var cx2=Math.round((c-zS)*bw);
    ctx.beginPath();ctx.moveTo(cx2,0);ctx.lineTo(cx2,H);ctx.stroke();
    ctx.fillStyle='#004400';ctx.font='8px monospace';ctx.fillText(c,cx2+1,H-1);
  }
  ctx.strokeStyle='#332200';
  for(var w=0;w<WIFI_CH.length;w++){
    var wc=WIFI_CH[w];if(wc<zS||wc>zE)continue;
    var wx=Math.round((wc-zS)*bw);
    ctx.beginPath();ctx.moveTo(wx,0);ctx.lineTo(wx,H);ctx.stroke();
    ctx.fillStyle='#554400';ctx.font='8px monospace';ctx.fillText(WIFI_LBL[w],wx+1,10);
  }
  for(var i=zS;i<=zE;i++){
    var val=gCh[i]||0,pk=gPeak[i]||0;
    var bx=Math.round((i-zS)*bw),bww=Math.ceil(bw);if(bww<1)bww=1;
    var bh=Math.round((val/255)*(H-14));
    if(bh>0){ctx.fillStyle='#004d18';ctx.fillRect(bx,H-bh,bww,bh);ctx.fillStyle='#00ff41';ctx.fillRect(bx,H-bh,bww,Math.min(2,bh));}
    if(pk>0){var ph=Math.round((pk/255)*(H-14));if(ph>0){ctx.fillStyle='#00ffaa';ctx.fillRect(bx,H-ph-1,bww,1);}}
  }
  if(gMode==='LOCK'||gMode==='TX'){
    var lc=gMode==='TX'?gTxCh:gLCh;
    if(lc>=zS&&lc<=zE){
      var lx=Math.round((lc-zS)*bw+bw/2);
      var lcol=gMode==='TX'?'#ffaa00':'#00ffaa';
      ctx.strokeStyle=lcol;ctx.lineWidth=2;
      ctx.beginPath();ctx.moveTo(lx,0);ctx.lineTo(lx,H-68);ctx.stroke();
      ctx.lineWidth=1;ctx.fillStyle=lcol;ctx.font='9px monospace';
      ctx.fillText((gMode==='TX'?'TX:':'LK:')+lc,lx+2,14);
    }
  }
  ctx.fillStyle='#005522';ctx.font='8px monospace';
  ctx.fillText((2400+zS)+'M',2,9);
  var rl=(2400+zE)+'M';ctx.fillText(rl,W-(rl.length*5)-1,9);
}

// ── 3D orientation renderer ───────────────────────────────────
// Draws a wireframe box representing the cyberdeck.
// Vertices of a rectangular box (W×H×D in canvas units):
// The box represents the cyberdeck: wider than tall, shallow depth.
// Rotated by roll/pitch/yaw using rotation matrix applied to each vertex.
// Android 2.3 WebKit has no typed arrays — use plain arrays.

var BOX_W=100,BOX_H=60,BOX_D=30;  // half-extents
var CX=155,CY=100;  // canvas centre

// 8 vertices of the box (centred at origin)
var VERTS=[
  [-BOX_W,-BOX_H,-BOX_D],[ BOX_W,-BOX_H,-BOX_D],
  [ BOX_W, BOX_H,-BOX_D],[-BOX_W, BOX_H,-BOX_D],
  [-BOX_W,-BOX_H, BOX_D],[ BOX_W,-BOX_H, BOX_D],
  [ BOX_W, BOX_H, BOX_D],[-BOX_W, BOX_H, BOX_D]
];
// Edges: pairs of vertex indices
var EDGES=[
  [0,1],[1,2],[2,3],[3,0],  // back face
  [4,5],[5,6],[6,7],[7,4],  // front face
  [0,4],[1,5],[2,6],[3,7]   // connecting edges
];
// Front face highlight (face towards viewer when unrotated)
var FRONT_FACE=[4,5,6,7];
// Screen icon positions on front face (relative to face centre)
// Screen is top-left area of front face, keyboard bottom
var SCREEN_PTS=[[-70,-30,-BOX_D+1],[20,-30,-BOX_D+1],[20,10,-BOX_D+1],[-70,10,-BOX_D+1]];

function deg2rad(d){return d*Math.PI/180;}

function rotatePoint(v,roll,pitch,yaw){
  var r=deg2rad(roll),p=deg2rad(pitch),y=deg2rad(yaw);
  var cr=Math.cos(r),sr=Math.sin(r);
  var cp=Math.cos(p),sp=Math.sin(p);
  var cy2=Math.cos(y),sy=Math.sin(y);
  // Rotation order: yaw → pitch → roll
  var x=v[0],yv=v[1],z=v[2];
  // Yaw (around Z)
  var x1=x*cy2-yv*sy, y1=x*sy+yv*cy2, z1=z;
  // Pitch (around X)
  var x2=x1, y2=y1*cp-z1*sp, z2=y1*sp+z1*cp;
  // Roll (around Y)
  var x3=x2*cr+z2*sr, y3=y2, z3=-x2*sr+z2*cr;
  return [x3,y3,z3];
}

function project(v){
  // Simple perspective projection
  var fov=400;
  var z=v[2]+300;  // viewer distance
  if(z<1)z=1;
  return [CX+v[0]*fov/z, CY+v[1]*fov/z];
}

function drawOrient(){
  var W=OCVS_W,H=OCVS_H;
  octx.fillStyle='#000';octx.fillRect(0,0,W,H);

  // Grid floor
  octx.strokeStyle='#001800';octx.lineWidth=1;
  for(var g=-3;g<=3;g++){
    octx.beginPath();octx.moveTo(0,H/2+g*20);octx.lineTo(W,H/2+g*20);octx.stroke();
  }

  var roll=gRoll,pitch=gPitch,yaw=gYaw;

  // Project all vertices
  var proj=[];
  for(var i=0;i<8;i++){
    var rv=rotatePoint(VERTS[i],roll,pitch,yaw);
    proj.push(project(rv));
  }

  // Compute Z depth for each vertex (for depth sorting)
  var zdepth=[];
  for(var i=0;i<8;i++){
    var rv=rotatePoint(VERTS[i],roll,pitch,yaw);
    zdepth.push(rv[2]);
  }

  // Draw back edges (darker)
  octx.strokeStyle='#003300';octx.lineWidth=1;
  for(var e=0;e<EDGES.length;e++){
    var a=EDGES[e][0],b=EDGES[e][1];
    if(zdepth[a]<0&&zdepth[b]<0){  // both behind centre = back face
      octx.beginPath();
      octx.moveTo(proj[a][0],proj[a][1]);
      octx.lineTo(proj[b][0],proj[b][1]);
      octx.stroke();
    }
  }

  // Draw front edges (bright)
  octx.strokeStyle='#00ff41';octx.lineWidth=1.5;
  for(var e=0;e<EDGES.length;e++){
    var a=EDGES[e][0],b=EDGES[e][1];
    if(zdepth[a]>=0||zdepth[b]>=0){
      octx.beginPath();
      octx.moveTo(proj[a][0],proj[a][1]);
      octx.lineTo(proj[b][0],proj[b][1]);
      octx.stroke();
    }
  }

  // Draw screen rectangle on front face
  var scr=[];
  for(var i=0;i<4;i++){
    var rv=rotatePoint(SCREEN_PTS[i],roll,pitch,yaw);
    scr.push(project(rv));
  }
  octx.strokeStyle='#00aa2a';octx.lineWidth=1;
  octx.beginPath();
  octx.moveTo(scr[0][0],scr[0][1]);
  for(var i=1;i<4;i++) octx.lineTo(scr[i][0],scr[i][1]);
  octx.closePath();octx.stroke();
  // Fill screen with dim green
  octx.fillStyle='rgba(0,80,20,0.4)';octx.fill();

  // Axis indicator (top right corner)
  var axO=[W-30,30];
  var axLen=20;
  var axes=[[axLen,0,0],[0,axLen,0],[0,0,axLen]];
  var axCols=['#ff4444','#44ff44','#4444ff'];
  var axLbls=['X','Y','Z'];
  for(var a=0;a<3;a++){
    var rv=rotatePoint(axes[a],roll,pitch,yaw);
    var ep=project([rv[0]+axO[0]-CX, rv[1]+axO[1]-CY, rv[2]]);
    // approximate: just use rotated vector directly scaled
    var ex=axO[0]+rv[0]*0.5, ey=axO[1]+rv[1]*0.5;
    octx.strokeStyle=axCols[a];octx.lineWidth=1.5;
    octx.beginPath();octx.moveTo(axO[0],axO[1]);octx.lineTo(ex,ey);octx.stroke();
    octx.fillStyle=axCols[a];octx.font='8px monospace';octx.fillText(axLbls[a],ex+1,ey+3);
  }

  // Horizon line
  var hx1=rotatePoint([-150,0,0],roll,pitch,yaw);
  var hx2=rotatePoint([ 150,0,0],roll,pitch,yaw);
  var hp1=project(hx1),hp2=project(hx2);
  octx.strokeStyle='#004400';octx.lineWidth=1;
  octx.beginPath();octx.moveTo(hp1[0],hp1[1]);octx.lineTo(hp2[0],hp2[1]);octx.stroke();

  // Angle readouts
  octx.fillStyle='#005510';octx.font='9px monospace';
  octx.fillText('R:'+fix1(roll),4,12);
  octx.fillText('P:'+fix1(pitch),60,12);
  octx.fillText('Y:'+fix1(yaw),116,12);
}

// ── mode control ──────────────────────────────────────────────
function goScan(){get('/setMode?mode=SCAN',function(){gMode='SCAN';refreshModeUI();});}
function goLock(){get('/setMode?mode=LOCK',function(){gMode='LOCK';refreshModeUI();});}
function goTX()  {get('/setMode?mode=TX',  function(){gMode='TX';  refreshModeUI();});}
function doLock(){
  var v=parseInt(document.getElementById('chi').value);
  if(isNaN(v)||v<0||v>125)return;
  gLCh=v;gTelem={};
  get('/setChannel?ch='+v,function(){gMode='LOCK';refreshModeUI();});
}
function doReset(){get('/resetPeak',function(){for(var i=0;i<N;i++){gCh[i]=0;gPeak[i]=0;}});}
function resetOrient(){get('/resetOrientation',function(){gYaw=0;});}
function doSend(){
  var data=document.getElementById('txdata').value;
  var ch=parseInt(document.getElementById('chi').value);
  if(isNaN(ch)||ch<0||ch>125)ch=gTxCh;
  gTxCh=ch;
  get('/transmit?ch='+ch+'&data='+encodeURIComponent(data)+(txRepeat?'&repeat=1':''),function(){gMode='TX';refreshModeUI();});
}
function toggleRepeat(){txRepeat=!txRepeat;document.getElementById('repbtn').style.background=txRepeat?'#1a0e00':'#001a00';}
function zIn(){var m=Math.round((zS+zE)/2),h=Math.max(5,Math.round((zE-zS)/4));zS=m-h<0?0:m-h;zE=m+h>125?125:m+h;}
function zOut(){var m=Math.round((zS+zE)/2),h=Math.round((zE-zS)/2)+15;if(h>63)h=63;zS=m-h<0?0:m-h;zE=m+h>125?125:m+h;}
function zFit(){zS=0;zE=125;}

var sw=document.getElementById('sw');
sw.ontouchstart=function(e){dragX=e.touches[0].clientX;dragZS=zS;};
sw.ontouchmove=function(e){
  if(dragX<0)return;
  var vis=zE-zS+1,dx=e.touches[0].clientX-dragX;
  var sh=Math.round(-dx/CVS_W*vis);
  var ns=dragZS+sh,ne=ns+vis-1;
  if(ns<0){ns=0;ne=vis-1;}if(ne>125){ne=125;ns=ne-vis+1;}
  zS=ns;zE=ne;
};
sw.ontouchend=function(){dragX=-1;};

// ── UI refresh ────────────────────────────────────────────────
function refreshModeUI(){
  document.getElementById('hm').innerHTML=gMode;
  document.getElementById('lkovr').className=gMode==='LOCK'?'on':'';
  document.getElementById('txovr').className=gMode==='TX'?'on':'';
  var stpElem = document.getElementById('stellar-page');
  if (stpElem) stpElem.className = (gMode==='STELLAR') ? 'on' : '';
  document.getElementById('txrow').className=gMode==='TX'?'on':'';
  document.getElementById('flovr').className=(gMode==='LOCK'&&gTelem&&gTelem.type==='flight_telemetry')?'on':'';
  if(gMode==='LOCK'){document.getElementById('lk-ch').innerHTML=gLCh;document.getElementById('lk-fr').innerHTML=(2400+gLCh);}
  if(gMode==='TX'){document.getElementById('tx-ch').innerHTML=gTxCh;}
}

function refreshInfo(){
  var active=[],pkV=0,pkC=0;
  for(var i=0;i<N;i++){if(gCh[i]>25)active.push(i);if(gPeak[i]>pkV){pkV=gPeak[i];pkC=i;}}
  document.getElementById('ach').innerHTML=active.length>7?active.length+'ch':active.join(' ')||'--';
  document.getElementById('pch').innerHTML=pkV>0?pkC:'--';
  document.getElementById('zrng').innerHTML=(2400+zS)+'-'+(2400+zE)+'M';
}

function refreshTelem(){
  if(!gTelem||gTelem.type!=='flight_telemetry')return;
  var t=gTelem;
  document.getElementById('fl-alt').innerHTML=fix1(t.altitude);
  document.getElementById('fl-vel').innerHTML=fix1(t.velocity);
  document.getElementById('fl-acc').innerHTML=fix2(t.acceleration);
  document.getElementById('fl-st').innerHTML=t.state_name||'--';
  document.getElementById('fl-ma').innerHTML=fix1(t.max_altitude);
  document.getElementById('fl-mv').innerHTML=fix1(t.max_velocity);
  document.getElementById('fl-mc').innerHTML=fix2(t.max_accel);
  document.getElementById('fl-bt').innerHTML=(t.battery||'--')+'%';
  // sensors page FC card
  document.getElementById('s-fc-card').style.display='block';
  document.getElementById('s-falt').innerHTML=fix1(t.altitude);
  document.getElementById('s-fvel').innerHTML=fix1(t.velocity);
  document.getElementById('s-fma').innerHTML=fix1(t.max_altitude);
  document.getElementById('s-fst').innerHTML=t.state_name||'--';
  // orient page FC row
  document.getElementById('o-fc-row').style.display='block';
  document.getElementById('o-falt').innerHTML=fix1(t.altitude);
  document.getElementById('o-fst').innerHTML=t.state_name||'--';
}

// ── polling ───────────────────────────────────────────────────
function pollScan(){
  get('/scan',function(d){
    if(d.ch&&d.ch.length===N){for(var i=0;i<N;i++){gCh[i]=d.ch[i]||0;gPeak[i]=d.peak[i]||0;}}
    if(d.mode)gMode=d.mode;
    if(d.locked_ch!==undefined)gLCh=d.locked_ch;
    if(curTab<=2){draw();refreshInfo();refreshModeUI();}
  });
}

function pollTelemetry(){
  if(gMode!=='LOCK')return;
  get('/telemetry',function(d){
    gTelem=d;
    if(d.type==='flight_telemetry'){
      document.getElementById('lk-ty').innerHTML='FC';
      document.getElementById('flovr').className='on';
      refreshTelem();
    } else if(d.type==='raw'){
      document.getElementById('lk-ty').innerHTML='RAW';
      document.getElementById('flovr').className='';
      document.getElementById('s-fc-card').style.display='none';
      document.getElementById('o-fc-row').style.display='none';
    }
  });
}

// Duplicate/old drawStellar() removed — using the robust drawStellar
// implementation (which references 'stellar-cvs') defined earlier.
function pollOrientation(){
  if(curTab!==3&&curTab!==4)return;
  get('/orientation',function(d){
    if(!d.ok)return;
    gRoll=d.roll;gPitch=d.pitch;gYaw=d.yaw;
    document.getElementById('o-roll').innerHTML=fix1(d.roll);
    document.getElementById('o-pitch').innerHTML=fix1(d.pitch);
    document.getElementById('o-yaw').innerHTML=fix1(d.yaw);
    document.getElementById('s-roll').innerHTML=fix1(d.roll);
    document.getElementById('s-pitch').innerHTML=fix1(d.pitch);
    document.getElementById('s-yaw').innerHTML=fix1(d.yaw);
    document.getElementById('s-ax').innerHTML=fix2(d.ax)+'g';
    document.getElementById('s-ay').innerHTML=fix2(d.ay)+'g';
    document.getElementById('s-az').innerHTML=fix2(d.az)+'g';
    document.getElementById('sb-imu').innerHTML='OK';
    if(curTab===3)drawOrient();
  });
}

function pollSensors(){
  if(curTab!==4)return;
  get('/sensors',function(d){
    if(!d.ok)return;
    document.getElementById('s-temp').innerHTML=fix1(d.temp_c);
    document.getElementById('s-hum').innerHTML=fix1(d.humidity_pct);
    document.getElementById('s-pres').innerHTML=fix1(d.pressure_hpa);
    document.getElementById('s-alt').innerHTML=fix1(d.baro_alt_m);
    document.getElementById('sb-bme').innerHTML='OK';
  });
}

function pollTxStatus(){
  if(gMode!=='TX')return;
  get('/txstatus',function(d){
    document.getElementById('tx-sn').innerHTML=d.sent||0;
    document.getElementById('tx-ak').innerHTML=d.acked||0;
    document.getElementById('tx-ls').innerHTML=(d.loss_pct||0)+'%';
    gTxCh=d.ch||gTxCh;
  });
}

function pollStatus(){
  get('/status',function(d){
    gStat=d;
    document.getElementById('sbl').innerHTML='UP:'+(d.uptime||0)+'s CY:'+(d.scan_cycles||0);
    if(gMode==='LOCK')document.getElementById('lk-pk').innerHTML=d.packets||0;
  });
}

function updateClock(){var d=new Date();document.getElementById('ht').innerHTML=p2(d.getHours())+':'+p2(d.getMinutes())+':'+p2(d.getSeconds());}

// Orient canvas continuously redraws when on tab 3
function orientLoop(){
  if(curTab===3)drawOrient();
  setTimeout(orientLoop,100);
}

function pollJoy(){
  get('/joystate',function(d){
    // Sync zoom window from joystick (joystick is authoritative)
    if(d.zs!==undefined){zS=d.zs;zE=d.ze;}
    // Show selected channel in channel input
    if(d.sel_ch!==undefined)document.getElementById('chi').value=d.sel_ch;
    // Joystick indicator in status bar
    var lxd=d.lx>10?'▶':d.lx<-10?'◀':'-';
    var lyd=d.ly<-10?'▲':d.ly>10?'▼':'-';
    var rxd=d.rx>10?'▶':d.rx<-10?'◀':'-';
    var ryd=d.ry>80?'LCK':d.ry<-80?'SCN':'-';
    document.getElementById('sb-joy').innerHTML=lxd+lyd+'|'+rxd+ryd;
  });
}

setInterval(pollScan,        300);
setInterval(
function()
{
  // drawStellar is scheduled via its own interval earlier; avoid duplicate calls here.
},
150
);
setInterval(pollTelemetry,   400);
setInterval(pollOrientation, 150);
setInterval(pollSensors,    2000);
setInterval(pollTxStatus,    500);
setInterval(pollStatus,     2500);
setInterval(pollJoy,         100);
setInterval(updateClock,    1000);

pollScan();pollStatus();updateClock();
refreshModeUI();
orientLoop();
</script>
</body>
</html>
)rawhtml";

