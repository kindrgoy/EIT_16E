/*
  EIT 16-Electrode — LIVE OPPOSITE EIT V2
  ESP32-S3 + AD5933 + 4x CD74HC4067

  COMMAND:
    c <R>       calibration resistor referensi, contoh: c 1000
    m a b c d   single measurement HC LC HP LP
    s           satu frame
    g           continuous frame
    x           stop
    scan        I2C scan
    p           print config
    diag        hardware/acquisition diagnostic
    help        daftar command

*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>

// ============================================================
// PIN / HARDWARE
// ============================================================

#define SDA_PIN 8
#define SCL_PIN 9

#define AD5933_ADDR 0x0D

#define MCLK_HZ 16776000UL
#define TARGET_FREQ 50000UL

#define N_EL 16
#define OFFSET 8
#define MEAS_PER_EXC 12
#define FRAME_MEAS (N_EL * MEAS_PER_EXC)

// CD74HC4067 address pins
const uint8_t MUX_HC[4] = {4, 5, 6, 7};
const uint8_t MUX_LC[4] = {10, 11, 12, 13};
const uint8_t MUX_HP[4] = {14, 15, 16, 17};
const uint8_t MUX_LP[4] = {18, 19, 20, 21};

// ============================================================
// PERFORMANCE / FILTERING
// ============================================================

#define SERIAL_BAUD 921600UL

// Average complex AD5933 result N times per measurement.
// 2 = faster, 3 = recommended, 5 = more stable but slower.
#define MEAS_AVERAGES 3

// Time after changing MUX before measurement.
#define MUX_SETTLE_MS 2

// Time after command repeat before polling.
#define DDS_SETTLE_MS 2

#define MEAS_TIMEOUT_MS 30

// Calibration / validation
#define CAL_SAMPLES 30
#define CAL_MIN_VALID 20
#define CAL_MIN_MAG 1.0f
#define CAL_MAX_CV_PCT 5.0

// Measurement sanity limits. These do NOT reject data based on expected
// impedance; they only reject impossible/invalid AD5933 results.
#define MIN_VALID_MAG 1.0f
#define MAX_VALID_MAG 30000.0f

// Small delay after calibration sample / diagnostic operation.
#define CAL_SAMPLE_GAP_MS 10

// ============================================================
// AD5933 REGISTERS
// ============================================================

#define REG_CTRL_H 0x80
#define REG_CTRL_L 0x81

#define REG_FREQ_H 0x82
#define REG_FREQ_M 0x83
#define REG_FREQ_L 0x84

#define REG_INC_H 0x85
#define REG_INC_M 0x86
#define REG_INC_L 0x87

#define REG_NINC_H 0x88
#define REG_NINC_L 0x89

#define REG_SETT_H 0x8A
#define REG_SETT_L 0x8B

#define REG_STATUS 0x8F

#define REG_REAL_H 0x94
#define REG_REAL_L 0x95

#define REG_IMAG_H 0x96
#define REG_IMAG_L 0x97

// AD5933 Control Register 1 (0x80)
// D10:D9 = output range, D8 = PGA gain.
// Range mapping used by this firmware:
//   00 = 2.0 Vp-p
//   01 = 200 mVp-p
//   10 = 400 mVp-p
//   11 = 1.0 Vp-p
#define CTRL_RANGE_2VPP 0x00
#define CTRL_RANGE_200MV 0x02
#define CTRL_RANGE_400MV 0x04
#define CTRL_RANGE_1VPP 0x06

#define CTRL_PGA_X1 0x01
#define CTRL_PGA_X5 0x00

#define AD_OUTPUT_RANGE CTRL_RANGE_200MV
#define AD_PGA_GAIN CTRL_PGA_X1

#define CMD_STANDBY 0xB0
#define CMD_INIT_FREQ 0x10
#define CMD_START_SWEEP 0x20
#define CMD_REPEAT_FREQ 0x40
#define CMD_RESET_BIT 0x10

// ============================================================
// STATE
// ============================================================

static bool ad_found = false;
static bool calibrated = false;
static bool continuous_mode = false;

static double gain_factor = 1.0; // legacy, tidak dipakai untuk EIT calibration
static uint32_t frame_id = 0;

// ============================================================
// MULTI-POINT CALIBRATION
// R = a*M^2 + b*M + c
// ============================================================

Preferences prefs;

#define CAL_NVS_NAMESPACE "eit_cal"

static double poly_a = 0.0;
static double poly_b = 1.0;
static double poly_c = 0.0;

static bool poly_calibrated = false;
static uint8_t cal_point_count = 0;

#define MAX_CAL_POINTS 6

static double cal_mag[MAX_CAL_POINTS];
static double cal_res[MAX_CAL_POINTS];

// Calibration record for reproducibility / Python metadata.
static double cal_r_ref = 0.0;
static double cal_avg_mag = 0.0;
static double cal_std_mag = 0.0;
static double cal_cv_pct = 0.0;
static uint16_t cal_valid_samples = 0;

String serial_buffer = "";

// ============================================================
// AD5933 LOW LEVEL
// ============================================================

bool ad_write(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(AD5933_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

uint8_t ad_read(uint8_t reg)
{
  Wire.beginTransmission(AD5933_ADDR);
  Wire.write(0xB0);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
    return 0xFF;

  if (Wire.requestFrom((uint8_t)AD5933_ADDR, (uint8_t)1) != 1)
  {
    return 0xFF;
  }

  return Wire.read();
}

void ad_ctrl(uint8_t cmd_h)
{
  const uint8_t ctrl =
      cmd_h |
      AD_OUTPUT_RANGE |
      AD_PGA_GAIN;

  ad_write(REG_CTRL_H, ctrl);
  ad_write(REG_CTRL_L, 0x00);
}

bool ad_detect()
{
  Wire.beginTransmission(AD5933_ADDR);
  return Wire.endTransmission() == 0;
}

// ============================================================
// I2C
// ============================================================

void i2c_scan()
{
  Serial.println("[I2C] Scanning...");

  int n = 0;

  for (uint8_t a = 1; a < 127; a++)
  {
    Wire.beginTransmission(a);

    if (Wire.endTransmission() == 0)
    {
      Serial.printf("  0x%02X%s\n",
                    a,
                    (a == AD5933_ADDR) ? " <- AD5933" : "");
      n++;
    }
  }

  if (!n)
  {
    Serial.println("  Tidak ada device.");
  }
}

// ============================================================
// DDS
// ============================================================

void write_freq(uint32_t hz)
{
  // AD5933 frequency tuning word:
  // FCODE = frequency * 2^29 / MCLK
  // Equivalent to the previous /4 * 2^27 expression, but explicit.
  uint32_t code =
      (uint32_t)lround(
          ((double)hz * 536870912.0) / (double)MCLK_HZ) &
      0xFFFFFFUL;

  ad_write(REG_FREQ_H, (code >> 16) & 0xFF);
  ad_write(REG_FREQ_M, (code >> 8) & 0xFF);
  ad_write(REG_FREQ_L, code & 0xFF);

  // Single frequency.
  ad_write(REG_INC_H, 0);
  ad_write(REG_INC_M, 0);
  ad_write(REG_INC_L, 0);

  ad_write(REG_NINC_H, 0);
  ad_write(REG_NINC_L, 0);

  // Settling cycles.
  ad_write(REG_SETT_H, 0x00);
  ad_write(REG_SETT_L, 0x64);
}

bool reset_and_start_dds()
{
  if (!ad_found)
    return false;

  // ------------------------------------------------------------
  // HARD RESET OF AD5933 STATE MACHINE
  // ------------------------------------------------------------
  // Reset bit berada di CONTROL LOW register (0x81).
  // Sweep registers tidak terhapus oleh reset.
  ad_write(REG_CTRL_L, CMD_RESET_BIT);
  delay(5);

  // De-assert reset.
  ad_write(REG_CTRL_L, 0x00);
  delay(5);

  // ------------------------------------------------------------
  // STANDBY
  // ------------------------------------------------------------
  ad_ctrl(CMD_STANDBY);
  delay(5);

  // ------------------------------------------------------------
  // PROGRAM SINGLE FREQUENCY
  // ------------------------------------------------------------
  write_freq(TARGET_FREQ);
  delay(2);

  // ------------------------------------------------------------
  // INITIALIZE WITH START FREQUENCY
  // ------------------------------------------------------------
  ad_ctrl(CMD_INIT_FREQ);
  delay(10);

  // ------------------------------------------------------------
  // START FREQUENCY SWEEP
  // Number of increments = 0, therefore only TARGET_FREQ
  // ------------------------------------------------------------
  ad_ctrl(CMD_START_SWEEP);
  delay(5);

  // ------------------------------------------------------------
  // Wait for VALID REAL/IMAG data.
  // D1 = 1
  // ------------------------------------------------------------
  uint32_t t0 = millis();

  while (millis() - t0 < 100)
  {
    uint8_t st = ad_read(REG_STATUS);

    if (st == 0xFF)
      return false;

    if (st & 0x02)
      return true;

    delay(1);
  }

  return false;
}

bool begin_fresh_measurement()
{
  if (!ad_found)
    return false;

  // Stop previous state.
  ad_ctrl(CMD_STANDBY);
  delay(2);

  // Re-initialize using the already programmed frequency registers.
  ad_ctrl(CMD_INIT_FREQ);
  delay(5);

  // Start a fresh conversion at TARGET_FREQ.
  ad_ctrl(CMD_START_SWEEP);
  delay(2);

  return true;
}

// ============================================================
// MUX
// ============================================================

void set_mux(const uint8_t pins[4], uint8_t ch)
{
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(pins[i], (ch >> i) & 1);
  }
}

// ============================================================
// MEASUREMENT
// ============================================================

struct Imp
{
  int16_t re;
  int16_t im;
  float mag;
  float z;
  bool ok;
};

bool read_complex_once(int16_t &re, int16_t &im)
{
  re = 0;
  im = 0;

  if (!ad_found)
    return false;

  // ------------------------------------------------------------
  // Start a completely fresh measurement.
  // This prevents stale DFT data after ESP reset / MUX switching.
  // ------------------------------------------------------------
  if (!begin_fresh_measurement())
    return false;

  uint32_t t0 = millis();

  while (millis() - t0 < MEAS_TIMEOUT_MS)
  {
    uint8_t st = ad_read(REG_STATUS);

    if (st == 0xFF)
      return false;

    // D1 = Real/Imaginary data valid
    if (st & 0x02)
    {
      uint8_t rh = ad_read(REG_REAL_H);
      uint8_t rl = ad_read(REG_REAL_L);

      uint8_t ih = ad_read(REG_IMAG_H);
      uint8_t il = ad_read(REG_IMAG_L);

      re = (int16_t)(((uint16_t)rh << 8) |
                     rl);

      im = (int16_t)(((uint16_t)ih << 8) |
                     il);

      return true;
    }

    delay(1);
  }

  return false;
}

bool magnitude_is_valid(float mag)
{
  return isfinite(mag) &&
         mag >= MIN_VALID_MAG &&
         mag <= MAX_VALID_MAG;
}

void print_imp_compact(const Imp &d)
{
  if (!d.ok)
  {
    Serial.println(
        "re=NaN,im=NaN,mag=NaN,z=NaN,ok=0");
    return;
  }

  Serial.printf(
      "re=%d,im=%d,mag=%.6f,R_est=%.3f,mode=%s,ok=1\n",
      d.re,
      d.im,
      d.mag,
      d.z,
      poly_calibrated
          ? "POLY2"
          : "RAW_MAG");
}

Imp measure()
{
  Imp d;
  d.re = 0;
  d.im = 0;
  d.mag = 0;
  d.z = 0;
  d.ok = false;

  if (!ad_found)
    return d;

  // Average COMPLEX values, then calculate magnitude.
  // This is better than averaging magnitudes because random phase/noise
  // can partially cancel before magnitude calculation.
  double sum_re = 0.0;
  double sum_im = 0.0;
  int valid = 0;

  for (int n = 0; n < MEAS_AVERAGES; n++)
  {
    int16_t re, im;

    if (read_complex_once(re, im))
    {
      sum_re += re;
      sum_im += im;
      valid++;
    }
  }

  if (valid == 0)
  {
    return d;
  }

  d.re = (int16_t)lround(sum_re / valid);
  d.im = (int16_t)lround(sum_im / valid);

  d.mag = sqrtf(
      (float)d.re * (float)d.re +
      (float)d.im * (float)d.im);

  if (d.mag > 0)
  {
    if (poly_calibrated)
    {
      d.z = (float)(poly_a * d.mag * d.mag +
                    poly_b * d.mag +
                    poly_c);
    }
    else
    {
      d.z = d.mag;
    }
  }
  else
  {
    d.z = 0.0f;
  }

  d.ok = magnitude_is_valid(d.mag);
  if (!d.ok)
  {
    d.z = 0.0f;
  }

  return d;
}

// ============================================================
// CALIBRATION
// ============================================================

struct CalStats
{
  int valid;
  double mean;
  double stddev;
  double cv_pct;
};

CalStats collect_calibration_stats()
{
  CalStats s;
  s.valid = 0;
  s.mean = 0.0;
  s.stddev = 0.0;
  s.cv_pct = 0.0;

  // First pass: collect magnitude values.
  double values[CAL_SAMPLES];

  for (int i = 0; i < CAL_SAMPLES; i++)
  {
    Imp d = measure();

    if (d.ok && d.mag >= CAL_MIN_MAG)
    {
      values[s.valid++] = d.mag;
    }

    delay(CAL_SAMPLE_GAP_MS);
  }

  if (s.valid <= 0)
  {
    return s;
  }

  // Mean.
  double sum = 0.0;
  for (int i = 0; i < s.valid; i++)
  {
    sum += values[i];
  }
  s.mean = sum / (double)s.valid;

  // Population standard deviation. We are describing the actual
  // repeatability of the calibration acquisition, not estimating a
  // larger external population.
  double sq = 0.0;
  for (int i = 0; i < s.valid; i++)
  {
    const double e = values[i] - s.mean;
    sq += e * e;
  }

  s.stddev = sqrt(sq / (double)s.valid);

  if (s.mean > 0.0)
  {
    s.cv_pct = (s.stddev / s.mean) * 100.0;
  }

  return s;
}

void clear_calibration()
{
  cal_point_count = 0;

  poly_a = 0.0;
  poly_b = 1.0;
  poly_c = 0.0;

  poly_calibrated = false;
  calibrated = false;

  cal_r_ref = 0.0;
  cal_avg_mag = 0.0;
  cal_std_mag = 0.0;
  cal_cv_pct = 0.0;
  cal_valid_samples = 0;

  prefs.begin(CAL_NVS_NAMESPACE, false);
  prefs.clear();
  prefs.end();

  Serial.println("### CAL_CLEARED");
}

void calibrate(double r_ref)
{
  if (!ad_found)
  {
    Serial.println("### CAL_ERROR,AD5933_NOT_FOUND");
    return;
  }

  if (!(r_ref > 0.0) || !isfinite(r_ref))
  {
    Serial.println("### CAL_ERROR,INVALID_RREF");
    return;
  }

  if (cal_point_count >= MAX_CAL_POINTS)
  {
    Serial.println("### CAL_ERROR,MAX_POINTS_REACHED");
    Serial.println("### CAL_HINT,use calclear before starting again");
    return;
  }

  // Fixed geometry for calibration.
  const uint8_t hc = 0;
  const uint8_t lc = 8;
  const uint8_t hp = 1;
  const uint8_t lp = 2;

  set_mux(MUX_HC, hc);
  set_mux(MUX_LC, lc);
  set_mux(MUX_HP, hp);
  set_mux(MUX_LP, lp);

  delay(100);

  Serial.printf(
      "### CAL_POINT_START,R=%.6f,HC=%d,LC=%d,HP=%d,LP=%d,POINT=%d/%d\n",
      r_ref,
      hc,
      lc,
      hp,
      lp,
      cal_point_count + 1,
      MAX_CAL_POINTS);

  CalStats s = collect_calibration_stats();

  if (s.valid < CAL_MIN_VALID || s.mean <= 0.0)
  {
    Serial.printf(
        "### CAL_POINT_ERROR,R=%.6f,VALID=%d,REQUIRED=%d,AVG_MAG=%.6f\n",
        r_ref,
        s.valid,
        CAL_MIN_VALID,
        s.mean);

    Serial.println("### CAL_POINT_END");
    return;
  }

  // Store this calibration point.
  cal_res[cal_point_count] = r_ref;
  cal_mag[cal_point_count] = s.mean;

  cal_point_count++;

  // Store latest statistics.
  cal_r_ref = r_ref;
  cal_avg_mag = s.mean;
  cal_std_mag = s.stddev;
  cal_cv_pct = s.cv_pct;
  cal_valid_samples = (uint16_t)s.valid;

  calibrated = false;
  poly_calibrated = false;

  Serial.printf(
      "CAL_POINT=%d,R=%.6f,AVG_MAG=%.6f,STD_MAG=%.6f,CV_PCT=%.4f\n",
      cal_point_count,
      r_ref,
      s.mean,
      s.stddev,
      s.cv_pct);

  Serial.printf(
      "CAL_POINT_STORED,%d/%d\n",
      cal_point_count,
      MAX_CAL_POINTS);

  if (cal_cv_pct > CAL_MAX_CV_PCT)
  {
    Serial.printf(
        "### CAL_WARNING,CV_GT_%.2f_PERCENT\n",
        CAL_MAX_CV_PCT);
  }

  Serial.println("### CAL_POINT_END");
}

void save_calibration_nvs()
{
  prefs.begin(CAL_NVS_NAMESPACE, false);

  prefs.putBool("valid", poly_calibrated);
  prefs.putDouble("a", poly_a);
  prefs.putDouble("b", poly_b);
  prefs.putDouble("c", poly_c);

  prefs.end();

  Serial.println("### CAL_NVS_SAVED");
}

bool fit_quadratic_calibration()
{
  if (cal_point_count < 3)
  {
    Serial.printf(
        "### CALFIT_ERROR,NEED_AT_LEAST_3_POINTS,HAVE=%d\n",
        cal_point_count);
    return false;
  }

  // Normalize magnitude to improve numerical conditioning.
  double m0 = 0.0;
  double ms = 0.0;

  for (uint8_t i = 0; i < cal_point_count; i++)
    m0 += cal_mag[i];

  m0 /= cal_point_count;

  for (uint8_t i = 0; i < cal_point_count; i++)
  {
    double x = cal_mag[i] - m0;
    ms += x * x;
  }

  ms = sqrt(ms / cal_point_count);

  if (ms < 1e-12)
  {
    Serial.println("### CALFIT_ERROR,MAG_RANGE_TOO_SMALL");
    return false;
  }

  // Normal equations for:
  // R = p2*x^2 + p1*x + p0
  double A[3][4] = {0};

  for (uint8_t i = 0; i < cal_point_count; i++)
  {
    double x = (cal_mag[i] - m0) / ms;
    double y = cal_res[i];

    double x2 = x * x;

    A[0][0] += x2 * x2;
    A[0][1] += x2 * x;
    A[0][2] += x2;

    A[1][0] += x * x2;
    A[1][1] += x * x;
    A[1][2] += x;

    A[2][0] += x2;
    A[2][1] += x;
    A[2][2] += 1.0;

    A[0][3] += x2 * y;
    A[1][3] += x * y;
    A[2][3] += y;
  }

  // Gaussian elimination with partial pivoting.
  for (int col = 0; col < 3; col++)
  {
    int pivot = col;

    for (int row = col + 1; row < 3; row++)
    {
      if (fabs(A[row][col]) > fabs(A[pivot][col]))
        pivot = row;
    }

    if (fabs(A[pivot][col]) < 1e-12)
    {
      Serial.println("### CALFIT_ERROR,SINGULAR_MATRIX");
      return false;
    }

    if (pivot != col)
    {
      for (int k = col; k < 4; k++)
      {
        double tmp = A[col][k];
        A[col][k] = A[pivot][k];
        A[pivot][k] = tmp;
      }
    }

    double div = A[col][col];

    for (int k = col; k < 4; k++)
      A[col][k] /= div;

    for (int row = 0; row < 3; row++)
    {
      if (row == col)
        continue;

      double factor = A[row][col];

      for (int k = col; k < 4; k++)
        A[row][k] -= factor * A[col][k];
    }
  }

  double p2 = A[0][3];
  double p1 = A[1][3];
  double p0 = A[2][3];

  // Convert normalized polynomial:
  //
  // R = p2*x^2 + p1*x + p0
  //
  // x = (M - m0)/ms
  //
  // into:
  //
  // R = a*M^2 + b*M + c

  poly_a = p2 / (ms * ms);

  poly_b =
      p1 / ms -
      (2.0 * p2 * m0) / (ms * ms);

  poly_c =
      p0 -
      (p1 * m0) / ms +
      (p2 * m0 * m0) / (ms * ms);

  poly_calibrated = true;
  calibrated = true;
  save_calibration_nvs();

  Serial.println("### CALFIT_OK");

  Serial.printf(
      "POINTS=%d\n",
      cal_point_count);

  Serial.printf(
      "POLY_A=%.15e\n",
      poly_a);

  Serial.printf(
      "POLY_B=%.15e\n",
      poly_b);

  Serial.printf(
      "POLY_C=%.15e\n",
      poly_c);

  // Print calibration points and fitted values.
  Serial.println("### CALFIT_POINTS");

  for (uint8_t i = 0; i < cal_point_count; i++)
  {
    double m = cal_mag[i];

    double r_fit =
        poly_a * m * m +
        poly_b * m +
        poly_c;

    double err_pct =
        100.0 *
        (r_fit - cal_res[i]) /
        cal_res[i];

    Serial.printf(
        "R=%.6f,MAG=%.6f,FIT=%.6f,ERR_PCT=%.4f\n",
        cal_res[i],
        m,
        r_fit,
        err_pct);
  }

  Serial.println("### CALFIT_END");

  return true;
}

void load_calibration_nvs()
{
  prefs.begin(CAL_NVS_NAMESPACE, true);

  poly_calibrated = prefs.getBool("valid", false);

  if (poly_calibrated)
  {
    poly_a = prefs.getDouble("a", 0.0);
    poly_b = prefs.getDouble("b", 1.0);
    poly_c = prefs.getDouble("c", 0.0);

    Serial.println("### CAL_NVS_LOADED");

    Serial.printf(
        "POLY_A=%.15e\n",
        poly_a);

    Serial.printf(
        "POLY_B=%.15e\n",
        poly_b);

    Serial.printf(
        "POLY_C=%.15e\n",
        poly_c);
  }
  else
  {
    poly_a = 0.0;
    poly_b = 1.0;
    poly_c = 0.0;

    Serial.println("### CAL_NVS_EMPTY");
  }

  prefs.end();
}

// ============================================================
// FRAME
// ============================================================

void print_frame_start()
{
  Serial.printf(
      "### SWEEP_START_OPPOSITE,%lu,%d,%d,%d\n",
      (unsigned long)frame_id,
      N_EL,
      OFFSET,
      FRAME_MEAS);

  Serial.println("idx,hc,lc,hp,lp,re,im,mag,z,ok");
}

void print_frame_end(uint32_t this_frame)
{
  Serial.printf(
      "### SWEEP_END_OPPOSITE,%lu\n",
      (unsigned long)this_frame);
}

void full_sweep_opposite()
{
  if (!ad_found)
  {
    Serial.println("[ERR] AD5933 tidak tersambung");
    return;
  }

  const uint32_t this_frame = frame_id++;

  print_frame_start();

  int meas_idx = 0;
  int valid_count = 0;

  for (uint8_t k = 0; k < N_EL; k++)
  {
    uint8_t hc = k;
    uint8_t lc = (k + OFFSET) % N_EL;

    // Set excitation first.
    set_mux(MUX_HC, hc);
    set_mux(MUX_LC, lc);

    delay(MUX_SETTLE_MS);

    for (uint8_t hp = 0; hp < N_EL; hp++)
    {
      uint8_t lp = (hp + 1) % N_EL;

      // Do not measure on current electrodes.
      if (
          hp == hc || hp == lc ||
          lp == hc || lp == lc)
      {
        continue;
      }

      set_mux(MUX_HP, hp);
      set_mux(MUX_LP, lp);

      delay(MUX_SETTLE_MS);

      Imp d = measure();

      if (d.ok)
      {
        valid_count++;

        Serial.printf(
            "%d,%d,%d,%d,%d,%d,%d,%.4f,%.8f,1\n",
            meas_idx,
            hc,
            lc,
            hp,
            lp,
            d.re,
            d.im,
            d.mag,
            d.z);
      }
      else
      {
        Serial.printf(
            "%d,%d,%d,%d,%d,NaN,NaN,NaN,NaN,0\n",
            meas_idx,
            hc,
            lc,
            hp,
            lp);
      }

      meas_idx++;
    }
  }

  print_frame_end(this_frame);

  // Diagnostic sent after the frame so the Python parser doesn't mistake
  // it for measurement data.
  Serial.printf(
      "### FRAME_INFO,%lu,%d,%d\n",
      (unsigned long)this_frame,
      valid_count,
      FRAME_MEAS);
}

// ============================================================
// SINGLE MEASUREMENT
// ============================================================

bool valid_electrode_pair(uint8_t a, uint8_t b)
{
  return a < N_EL && b < N_EL && a != b;
}

void single_measure(uint8_t hc, uint8_t lc, uint8_t hp, uint8_t lp)
{
  Serial.printf(
      "### SINGLE_START,HC=%d,LC=%d,HP=%d,LP=%d\n",
      hc, lc, hp, lp);

  if (!ad_found)
  {
    Serial.println("### SINGLE_ERROR,AD5933_NOT_FOUND");
    Serial.println("### SINGLE_END");
    return;
  }

  if (!valid_electrode_pair(hc, lc) ||
      !valid_electrode_pair(hp, lp))
  {
    Serial.println("### SINGLE_ERROR,BAD_ELECTRODE");
    Serial.println("### SINGLE_END");
    return;
  }

  // Do not allow sensing pair to use either current electrode.
  if (hp == hc || hp == lc || lp == hc || lp == lc)
  {
    Serial.println("### SINGLE_ERROR,SENSE_ON_CURRENT_ELECTRODE");
    Serial.println("### SINGLE_END");
    return;
  }

  set_mux(MUX_HC, hc);
  set_mux(MUX_LC, lc);
  set_mux(MUX_HP, hp);
  set_mux(MUX_LP, lp);

  delay(MUX_SETTLE_MS);

  Imp d = measure();

  Serial.printf(
      "### SINGLE_MEAS,hc=%d,lc=%d,hp=%d,lp=%d\n",
      hc, lc, hp, lp);

  print_imp_compact(d);

  Serial.printf(
      "poly_calibrated=%d,poly_a=%.12e,poly_b=%.12e,poly_c=%.12e\n",
      poly_calibrated ? 1 : 0,
      poly_a,
      poly_b,
      poly_c);

  Serial.println("### SINGLE_END");
}

// ============================================================
// DIAGNOSTIC
// ============================================================

void diagnostic()
{
  Serial.println("### DIAG_START");

  Serial.printf("AD_FOUND=%d\n", ad_found ? 1 : 0);
  Serial.printf("POLY_CALIBRATED=%d\n", poly_calibrated ? 1 : 0);
  Serial.printf("FRAME_ID=%lu\n", (unsigned long)frame_id);
  Serial.printf("TARGET_FREQ=%lu\n", TARGET_FREQ);
  Serial.printf("MCLK_HZ=%lu\n", MCLK_HZ);
  Serial.printf("AD_RANGE=200mVPP\n");
  Serial.printf("AD_PGA=X1\n");

  uint8_t status = ad_read(REG_STATUS);
  Serial.printf("AD_STATUS=0x%02X\n", status);

  Serial.printf(
      "POLY_A=%.12e\nPOLY_B=%.12e\nPOLY_C=%.12e\n",
      poly_a,
      poly_b,
      poly_c);
  Serial.printf(
      "CAL_RREF=%.6f\nCAL_AVG_MAG=%.6f\nCAL_STD_MAG=%.6f\nCAL_CV_PCT=%.4f\nCAL_VALID=%u\nGAIN_FACTOR=%.12e\n",
      cal_r_ref,
      cal_avg_mag,
      cal_std_mag,
      cal_cv_pct,
      cal_valid_samples,
      gain_factor);

  Serial.println("### DIAG_END");
}

// ============================================================
// HELP
// ============================================================

void print_help()
{
  Serial.println("### HELP");
  Serial.println("m hc lc hp lp   single measurement");
  Serial.println("s           acquire one 192-point frame");
  Serial.println("g           continuous frames");
  Serial.println("x           stop continuous mode");
  Serial.println("scan        I2C scan");
  Serial.println("p           print machine-readable config");
  Serial.println("diag        print acquisition/calibration diagnostics");
  Serial.println("help        print this help");
  Serial.println("rtest       30x repeatability test");
  Serial.println("cal         show saved calibration");
  Serial.println("calclear    erase saved calibration");
  Serial.println("calfit      fit and save calibration");
  Serial.println("c <R>       add calibration point");
  Serial.println("### HELP_END");
}

// ============================================================
// CONFIG
// ============================================================

// ============================================================

void print_config()
{
  Serial.println("### CONFIG");

  Serial.println("PROTOCOL=EIT16_OPPOSITE_ADJACENT_V4");
  Serial.printf("N_EL=%d\n", N_EL);
  Serial.printf("OFFSET=%d\n", OFFSET);
  Serial.printf("MEAS_PER_EXC=%d\n", MEAS_PER_EXC);
  Serial.printf("FRAME_MEAS=%d\n", FRAME_MEAS);

  Serial.printf("TARGET_FREQ=%lu\n", TARGET_FREQ);
  Serial.printf("MCLK_HZ=%lu\n", MCLK_HZ);
  Serial.println("AD_OUTPUT_RANGE=200mVPP");
  Serial.println("AD_PGA_GAIN=X1");

  Serial.printf("SERIAL_BAUD=%lu\n", SERIAL_BAUD);
  Serial.printf("MEAS_AVERAGES=%d\n", MEAS_AVERAGES);
  Serial.printf("MUX_SETTLE_MS=%d\n", MUX_SETTLE_MS);
  Serial.printf("DDS_SETTLE_MS=%d\n", DDS_SETTLE_MS);
  Serial.printf("MEAS_TIMEOUT_MS=%d\n", MEAS_TIMEOUT_MS);

  Serial.printf("CAL_SAMPLES=%d\n", CAL_SAMPLES);
  Serial.printf("CAL_MIN_VALID=%d\n", CAL_MIN_VALID);
  Serial.printf("CAL_MAX_CV_PCT=%.3f\n", CAL_MAX_CV_PCT);

  Serial.printf("AD5933_FOUND=%d\n", ad_found ? 1 : 0);
  Serial.printf("POLY_CALIBRATED=%d\n", poly_calibrated ? 1 : 0);
  Serial.printf("POLY_A=%.15e\n", poly_a);
  Serial.printf("POLY_B=%.15e\n", poly_b);
  Serial.printf("POLY_C=%.15e\n", poly_c);
  Serial.printf("GAIN_FACTOR=%.12e\n", gain_factor);
  Serial.printf("CAL_RREF=%.6f\n", cal_r_ref);
  Serial.printf("CAL_AVG_MAG=%.6f\n", cal_avg_mag);
  Serial.printf("CAL_STD_MAG=%.6f\n", cal_std_mag);
  Serial.printf("CAL_CV_PCT=%.4f\n", cal_cv_pct);
  Serial.printf("CAL_VALID=%u\n", cal_valid_samples);

  Serial.println("INJECTION=OPPOSITE");
  Serial.println("SENSING=ADJACENT");
  Serial.println("BASELINE=PYTHON");
  Serial.println("RECONSTRUCTION=PYTHON");

  Serial.println("### CONFIG_END");
}

// ============================================================
// 30x REPEATABILITY TEST
// ============================================================

void repeatability_test()
{
  const int N = 30;

  Serial.println("### REPEATABILITY_START");

  for (int i = 0; i < N; i++)
  {
    // Ambil satu measurement.
    // Tidak ada averaging antar 30 measurement.
    Imp d = measure();

    if (d.ok)
    {
      Serial.printf("%.6f\n", d.mag);
    }
    else
    {
      Serial.println("NaN");
    }

    // Jeda antar measurement
    delay(1000);
  }

  Serial.println("### REPEATABILITY_END");
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(1200);

  Serial.println();
  Serial.println("=== EIT 16E LIVE OPPOSITE");

  for (int i = 0; i < 4; i++)
  {
    pinMode(MUX_HC[i], OUTPUT);
    pinMode(MUX_LC[i], OUTPUT);
    pinMode(MUX_HP[i], OUTPUT);
    pinMode(MUX_LP[i], OUTPUT);
  }

  // Initial safe-ish known state.
  set_mux(MUX_HC, 0);
  set_mux(MUX_LC, 8);
  set_mux(MUX_HP, 1);
  set_mux(MUX_LP, 2);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000UL);

  delay(100);

  load_calibration_nvs();

  i2c_scan();

  Serial.print("[INIT] AD5933");

  uint32_t t0 = millis();

  while (!ad_detect() && millis() - t0 < 5000)
  {
    Serial.print(".");
    delay(300);
  }

  if (ad_detect())
  {
    ad_found = true;
    Serial.println(" DITEMUKAN");

    if (reset_and_start_dds())
    {
      Serial.printf(
          "[INIT] DDS aktif %lu Hz\n",
          TARGET_FREQ);
    }
    else
    {
      Serial.println("[WARN] DDS gagal start");
    }
  }
  else
  {
    Serial.println(" TIDAK DITEMUKAN");
  }

  print_config();

  Serial.println(
      "READY: c <R>| cal |calfit | calclear | m hc lc hp lp | s | g | x | scan | p | diag | help | rtest");
}

// ============================================================
// COMMAND LOOP
// ============================================================

void loop()
{
  // ============================================================
  // CONTINUOUS MODE
  // ============================================================
  if (continuous_mode)
  {
    full_sweep_opposite();

    if (Serial.available())
    {
      while (Serial.available())
      {
        char c = Serial.read();

        if (c == '\n' || c == '\r')
        {
          serial_buffer.trim();
          serial_buffer.toLowerCase();

          if (serial_buffer == "x")
          {
            continuous_mode = false;
            Serial.println("### STOPPED");
          }

          serial_buffer = "";
        }
        else
        {
          serial_buffer += c;
        }
      }
    }

    return;
  }

  // ============================================================
  // SERIAL COMMAND BUFFER
  // ============================================================
  while (Serial.available())
  {
    char c = Serial.read();

    // Command selesai saat Enter diterima
    if (c == '\n' || c == '\r')
    {
      if (serial_buffer.length() == 0)
        continue;

      String cmd = serial_buffer;
      serial_buffer = "";

      cmd.trim();
      cmd.toLowerCase();

      // --------------------------------------------------------
      // COMMANDS
      // --------------------------------------------------------

      if (cmd.startsWith("c "))
      {
        double r = cmd.substring(2).toDouble();

        if (r > 0)
        {
          calibrate(r);
        }
        else
        {
          Serial.println(
              "### COMMAND_ERROR,FORMAT=c 1000");
        }
      }

      else if (cmd.startsWith("m "))
      {
        int v[4] = {-1, -1, -1, -1};

        int parsed = sscanf(
            cmd.c_str(),
            "m %d %d %d %d",
            &v[0],
            &v[1],
            &v[2],
            &v[3]);

        if (parsed == 4)
        {
          single_measure(
              (uint8_t)v[0],
              (uint8_t)v[1],
              (uint8_t)v[2],
              (uint8_t)v[3]);
        }
        else
        {
          Serial.println(
              "### COMMAND_ERROR,FORMAT=m hc lc hp lp");
        }
      }

      else if (cmd == "s")
      {
        full_sweep_opposite();
      }

      else if (cmd == "g")
      {
        continuous_mode = true;
        Serial.println("### CONTINUOUS_START");
      }

      else if (cmd == "x")
      {
        continuous_mode = false;
        Serial.println("### STOPPED");
      }

      else if (cmd == "scan")
      {
        i2c_scan();
      }

      else if (cmd == "p")
      {
        print_config();
      }

      else if (cmd == "diag")
      {
        diagnostic();
      }

      else if (cmd == "help")
      {
        print_help();
      }

      else if (cmd == "rtest")
      {
        repeatability_test();
      }

      else if (cmd == "calfit")
      {
        fit_quadratic_calibration();
      }

      else if (cmd == "calclear")
      {
        clear_calibration();
      }

      else if (cmd == "cal")
      {
        Serial.println("### CAL_STATUS");

        Serial.printf(
            "VALID=%d\n",
            poly_calibrated ? 1 : 0);

        Serial.printf(
            "A=%.15e\n",
            poly_a);

        Serial.printf(
            "B=%.15e\n",
            poly_b);

        Serial.printf(
            "C=%.15e\n",
            poly_c);

        Serial.println("### CAL_STATUS_END");
      }

      else
      {
        Serial.println(
            "### COMMAND_ERROR,UNKNOWN_COMMAND");
      }
    }

    // Karakter biasa → simpan ke buffer
    else
    {
      serial_buffer += c;

      // Proteksi buffer
      if (serial_buffer.length() > 80)
      {
        serial_buffer = "";
        Serial.println("### COMMAND_ERROR,BUFFER_OVERFLOW");
      }
    }
  }
}
