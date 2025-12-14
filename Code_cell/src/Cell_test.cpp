#include <Wire.h>
#include <Adafruit_INA219.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// -------------------------------------------
// Cấu hình phần cứng
// -------------------------------------------
#define LCD_COLS 20
#define LCD_ROWS 4
#define LCD_I2C_ADDRESS 0x27

// Địa chỉ I2C INA219
Adafruit_INA219 ina219_1(0x40);  // A0:low, A1:low

// Relay đóng/ngắt cell (kích mức LOW)
int relayPins[] = {2};  // SIG1 cho cell 1
// MOSFET opto (kích mức HIGH để thông)
int mosfetPins[] = {3};  // PWM1 cho cell 1
// Relay chuyển chế độ charge/discharge nếu cần (kích mức LOW)
#define MODE_RELAY 8

// LOGIC relay cell
#define CLOSE false
#define OPEN true
// LOGIC relay switch mode
#define CHARGE false
#define DISCHARGE true
// LOGIC MOSFET opto
#define CUT_OFF false   // chặn dòng qua MOSFET
#define THROUGH true    // thông dòng qua MOSFET

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);

// -------------------------------------------
// Ngưỡng & hằng số hệ thống
// -------------------------------------------
const float VOLTAGE_MINIMUM_THRESHOLD = 3.7f;   // nếu xả < ngưỡng thì ngắt xả
const float VOLTAGE_STOP_THRESHOLD    = 3.95f;  // nếu OCV >= ngưỡng thì ngừng sạc
const float VOLTAGE_CHARGE_MAX        = 6.0f;   // nếu điện áp sạc > ngưỡng thì tạm ngừng sạc
const float CURRENT_MAX               = 500.0f; // mA, phòng ngừa quá dòng (có thể dùng nếu cần)
const float BAT_CAPACITY_MAH          = 1800.0f; // dung lượng danh định cell 18650

// Hiệu chỉnh sụt áp
const float OFFSET_OCV        = 0.03f; // khi đo OCV
const float OFFSET_VOL_CHARGE = 0.00f; // khi đo lúc sạc

// Thời gian/chu kỳ (ms)
const uint32_t OCV_PERIOD_MS          = 30UL * 1000; // chu kỳ đo OCV
const uint32_t OCV_SETTLE_MS          = 10UL * 1000; // thời gian chờ sau khi cô lập cell để đo OCV
const uint32_t CHARGE_SAMPLE_MS       = 1000;        // chu kỳ lấy mẫu khi đang sạc
const uint32_t RETRY_DELAY_MS         = 5UL * 1000;  // t (s) chờ trước khi thử sạc lại
const uint32_t RETRY_SAMPLE_WINDOW_MS = 2000;        // n (s) thời gian đóng sạc để kiểm tra lại
const uint32_t NO_INPUT_PAUSE_MS      = 20UL * 1000; // thời gian ngắt sạc khi dòng <= 0
const uint32_t NO_INPUT_DETECT_MS     = 3000;        // thời gian phải duy trì I <= 0 trước khi ngắt

// -------------------------------------------
// Biến trạng thái
// -------------------------------------------
float last_charge_v   = 0.0f;
float last_ocv        = 0.0f;
float coulomb_mAh     = 0.0f;  // lượng đã nạp
uint32_t lastCurrentSampleMs = 0;

bool chargePathClosed = true; // relay CLOSE, MOSFET CUT_OFF
bool state_mode_relay = CHARGE;

enum class BmsState {
  CHARGING,
  PAUSED_HIGH_VOLT,   // tạm dừng do quá áp sạc
  OCV_SETTLING,       // đang chờ để đo OCV
  OCV_SAMPLING,       // đo OCV
  FAULT_OVERCURRENT,  // quá dòng
  NO_INPUT_WAIT,      // tạm ngắt do nguồn không cấp dòng (I <= 0)
  NO_INPUT_SAMPLE     // đóng lại để kiểm tra lại dòng sau thời gian chờ
};
BmsState bmsState = BmsState::CHARGING;

uint32_t lastChargeSampleMs = 0;
uint32_t lastOcvRequestMs   = 0;
uint32_t ocvSettleStartMs   = 0;
uint32_t pauseStartMs       = 0;
uint32_t retryStartMs       = 0;
uint32_t noInputStartMs     = 0;
uint32_t noInputLowStartMs  = 0;

// -------------------------------------------
// Tiện ích đóng/ngắt đường sạc theo thứ tự yêu cầu
// -------------------------------------------
void openChargePath() {
  // Ngắt sạc: thông MOSFET trước, rồi mở relay
  digitalWrite(mosfetPins[0], THROUGH);
  delay(10);
  digitalWrite(relayPins[0], OPEN);
  delay(1000); // đợi xung quá độ dập tắt
  chargePathClosed = false;
}

void closeChargePath() {
  // Đóng sạc: cắt MOSFET trước, rồi đóng relay
  digitalWrite(mosfetPins[0], CUT_OFF);
  delay(10);
  digitalWrite(relayPins[0], CLOSE);
  delay(1000); // đợi xung quá độ dập tắt
  chargePathClosed = true;
}

// -------------------------------------------
// Đọc điện áp & dòng từ INA219
// -------------------------------------------
void readChargeVoltageCurrent(float &voltage, float &current_mA) {
  float shuntVoltage = ina219_1.getShuntVoltage_mV();
  float busVoltage   = ina219_1.getBusVoltage_V();
  current_mA         = ina219_1.getCurrent_mA();
  float loadVoltage  = busVoltage + (shuntVoltage / 1000.0f);

  voltage = loadVoltage - OFFSET_VOL_CHARGE;
}

// Đo OCV sau khi đã cô lập cell
float measureOCV() {
  float shuntVoltage = ina219_1.getShuntVoltage_mV();
  float busVoltage   = ina219_1.getBusVoltage_V();
  float loadVoltage  = busVoltage + (shuntVoltage / 1000.0f);
  return loadVoltage + OFFSET_OCV;
}

// -------------------------------------------
// Coulomb counting (tích phân dòng)
// -------------------------------------------
void integrateCoulomb(float current_mA) {
  uint32_t now = millis();
  if (lastCurrentSampleMs == 0) {
    lastCurrentSampleMs = now;
    return;
  }
  uint32_t dt_ms = now - lastCurrentSampleMs;
  lastCurrentSampleMs = now;

  // Chỉ cộng khi đang nạp (dòng dương)
  if (current_mA > 0) {
    coulomb_mAh += current_mA * (float)dt_ms / 3600000.0f;
    if (coulomb_mAh > BAT_CAPACITY_MAH) coulomb_mAh = BAT_CAPACITY_MAH;
  }
}

// -------------------------------------------
// LCD helper
// -------------------------------------------
void lcdShowStatus(const char* line1, float vCharge, float vOcv, float mah) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(line1);
  lcd.setCursor(0, 1); lcd.print("Vchg:");
  lcd.print(vCharge, 2); lcd.print(" V   ");
  lcd.setCursor(0, 2); lcd.print("OCV :");
  lcd.print(vOcv, 2); lcd.print(" V   ");
  lcd.setCursor(0, 3); lcd.print("Q   :");
  lcd.print(mah, 0); lcd.print(" mAh ");
}

// -------------------------------------------
// Setup
// -------------------------------------------
void setup() {
  Serial.begin(9600);
  while (!Serial) delay(1);

  // Relay mode (nếu dùng) - mặc định CHARGE
  pinMode(MODE_RELAY, OUTPUT);
  digitalWrite(MODE_RELAY, CHARGE);
  state_mode_relay = CHARGE;

  // Relay cell
  pinMode(relayPins[0], OUTPUT);
  digitalWrite(relayPins[0], CLOSE);
  chargePathClosed = true;

  // MOSFET opto
  pinMode(mosfetPins[0], OUTPUT);
  digitalWrite(mosfetPins[0], CUT_OFF);

  // INA219
  ina219_1.begin();

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.print("BMS Starting...");
  Serial.println("BMS Starting...");
  delay(1000);
  lcd.clear();

  // Bắt đầu đo OCV ngay khi khởi động để xác định trạng thái sạc
  ocvSettleStartMs = millis();
  bmsState = BmsState::OCV_SETTLING;
  Serial.println("Initial OCV measurement starting...");

  lastChargeSampleMs = millis();
  lastOcvRequestMs   = millis();
}

// -------------------------------------------
// Loop chính
// -------------------------------------------
void loop() {
  uint32_t now = millis();

  // 1) Chu kỳ đo OCV định kỳ
  if (bmsState == BmsState::CHARGING && (now - lastOcvRequestMs >= OCV_PERIOD_MS)) {
    // chuyển sang trạng thái chờ đo OCV: ngắt sạc đúng thứ tự
    openChargePath();
    ocvSettleStartMs = now;
    bmsState = BmsState::OCV_SETTLING;
    Serial.println("OCV settle start...");
  }

  // 2) Nếu đang chờ đo OCV
  if (bmsState == BmsState::OCV_SETTLING && (now - ocvSettleStartMs >= OCV_SETTLE_MS)) {
    // Đo OCV
    last_ocv = measureOCV();
    Serial.print("OCV measured: "); Serial.println(last_ocv, 3);

    // Kiểm tra ngưỡng OCV
    if (last_ocv >= VOLTAGE_STOP_THRESHOLD) {
      // giữ ngắt sạc
      bmsState = BmsState::PAUSED_HIGH_VOLT;
      pauseStartMs = now;
      retryStartMs = now + RETRY_DELAY_MS;
      Serial.println("OCV over stop threshold, keep paused.");
    } else {
      // Cho phép sạc lại
      closeChargePath();
      bmsState = BmsState::CHARGING;
      // Reset timer bằng millis() mới nhất (do closeChargePath có delay)
      lastChargeSampleMs = millis(); 
      Serial.println("OCV OK, resume charging.");
    }
    lastOcvRequestMs = now;
  }

  // 3) Đo khi đang sạc (hoặc trong cửa sổ thử lại)
  bool inRetryWindow = (bmsState == BmsState::PAUSED_HIGH_VOLT && chargePathClosed);
  bool inNoInputSample = (bmsState == BmsState::NO_INPUT_SAMPLE);
  if ((bmsState == BmsState::CHARGING || inRetryWindow || inNoInputSample) &&
      (now - lastChargeSampleMs >= CHARGE_SAMPLE_MS)) {

    lastChargeSampleMs = now;
    float current_mA = 0.0f;
    readChargeVoltageCurrent(last_charge_v, current_mA);
    integrateCoulomb(current_mA);

    // Quá dòng: ngắt ngay
    if (fabsf(current_mA) > CURRENT_MAX) {
      openChargePath();
      bmsState = BmsState::FAULT_OVERCURRENT;
      Serial.println("Overcurrent detected -> open relay.");
      lcdShowStatus("Overcurrent!", last_charge_v, last_ocv, coulomb_mAh);
      return;
    }

    // Nếu đang sạc nhưng nguồn không cấp dòng (I <= 0) duy trì 3s -> ngắt sạc 20s rồi thử lại
    if (bmsState == BmsState::CHARGING || bmsState == BmsState::NO_INPUT_SAMPLE) {
      if (current_mA <= 0.0f) {
        if (noInputLowStartMs == 0) noInputLowStartMs = now;
        if (now - noInputLowStartMs >= NO_INPUT_DETECT_MS) {
          openChargePath();
          bmsState = BmsState::NO_INPUT_WAIT;
          noInputStartMs = millis();
          noInputLowStartMs = 0;
          Serial.println("No input current >=3s -> open relay for 20s.");
          lcdShowStatus("No input I", last_charge_v, last_ocv, coulomb_mAh);
          return;
        }
      } else {
        noInputLowStartMs = 0; // reset khi dòng dương trở lại
      }
    } else {
      noInputLowStartMs = 0; // reset khi thoát trạng thái sạc/kiểm tra
    }

    Serial.print("Vchg:"); Serial.print(last_charge_v, 3);
    Serial.print(", I:");  Serial.print(current_mA, 1);
    Serial.print(" mA, Q:"); Serial.print(coulomb_mAh, 1);
    Serial.print(" mAh, OCV:"); Serial.println(last_ocv, 3);

    // Ngắt sạc nếu điện áp sạc vượt ngưỡng
    if (last_charge_v >= VOLTAGE_CHARGE_MAX) {
      openChargePath();
      bmsState = BmsState::PAUSED_HIGH_VOLT;
      pauseStartMs = now;
      retryStartMs = now + RETRY_DELAY_MS;
      Serial.println("Charge voltage high -> pause & wait to retry.");
    }
  }

  // 4) Cửa sổ thử đóng lại sau khi pause vì quá áp sạc
  if (bmsState == BmsState::PAUSED_HIGH_VOLT) {
    // Sau thời gian chờ t, thử đóng lại trong n giây để kiểm tra
    if (!chargePathClosed && (now >= retryStartMs)) {
      closeChargePath();
      retryStartMs = millis() + RETRY_SAMPLE_WINDOW_MS; // tính lại thời điểm kết thúc cửa sổ thử
      // Reset timer bằng millis() mới nhất
      lastChargeSampleMs = millis();
      Serial.println("Retry: re-enable charge path for sampling.");
    }

    // Sau n giây cửa sổ thử -> đánh giá lại
    if (chargePathClosed && (now >= retryStartMs)) {
      float current_mA = 0.0f;
      readChargeVoltageCurrent(last_charge_v, current_mA);

      // Quá dòng trong cửa sổ thử lại: ngắt ngay
      if (fabsf(current_mA) > CURRENT_MAX) {
        openChargePath();
        bmsState = BmsState::FAULT_OVERCURRENT;
        Serial.println("Overcurrent detected in retry -> open relay.");
        lcdShowStatus("Overcurrent!", last_charge_v, last_ocv, coulomb_mAh);
        return;
      }

      // nếu vẫn quá áp -> lại pause và chờ t giây nữa
      if (last_charge_v >= VOLTAGE_CHARGE_MAX || last_ocv >= VOLTAGE_STOP_THRESHOLD) {
        openChargePath();
        pauseStartMs = now;
        retryStartMs = now + RETRY_DELAY_MS;
        Serial.println("Retry still high -> keep paused.");
      } else {
        // áp đã hạ, tiếp tục sạc
        bmsState = BmsState::CHARGING;
        lastChargeSampleMs = now;
        Serial.println("Retry OK -> resume charging.");
      }
    }
  }

  // 5) Xử lý chu trình mất dòng vào (I <= 0): ngắt 20s rồi đóng lại kiểm tra
  if (bmsState == BmsState::NO_INPUT_WAIT) {
    if (now - noInputStartMs >= NO_INPUT_PAUSE_MS) {
      closeChargePath();
      // Sau khi đóng lại, chờ một chu kỳ đo để kiểm tra lại
      lastChargeSampleMs = millis();
      bmsState = BmsState::NO_INPUT_SAMPLE;
      Serial.println("No input wait done -> close relay to re-check.");
    }
  }

  // 6) Cập nhật LCD đơn giản
  if ((now % 2000) < 50) { // refresh khoảng 2s/lần (nhẹ)
    const char* line1 =
      (bmsState == BmsState::CHARGING)        ? "Charging" :
      (bmsState == BmsState::PAUSED_HIGH_VOLT) ? "Paused Hi-Volt" :
      (bmsState == BmsState::OCV_SETTLING)     ? "OCV settling" :
      (bmsState == BmsState::FAULT_OVERCURRENT)? "Overcurrent!" :
      (bmsState == BmsState::NO_INPUT_WAIT)    ? "No input wait" :
      (bmsState == BmsState::NO_INPUT_SAMPLE)  ? "Recheck input" :
                                                "OCV sampling";
    lcdShowStatus(line1, last_charge_v, last_ocv, coulomb_mAh);
  }
}