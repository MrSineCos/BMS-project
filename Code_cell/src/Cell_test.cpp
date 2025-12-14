// BMS single-cell controller implemented per plan.txt
// Two cooperative tasks are modeled: manage_cell (state machine) and monitor_cell (sensing & coulomb count).

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
Adafruit_INA219 ina219_1(0x40); // A0:low, A1:low

// Relay đóng/ngắt cell (kích mức LOW)
int relayPins[] = {2}; // SIG1 cho cell 1
// MOSFET opto (kích mức HIGH để thông)
int mosfetPins[] = {3}; // PWM1 cho cell 1
// Relay chuyển chế độ charge/discharge nếu cần (kích mức LOW)
#define MODE_RELAY 8

// LOGIC relay cell
#define CLOSE false
#define OPEN true
// LOGIC relay switch mode
#define CHARGE false
#define DISCHARGE true
// LOGIC MOSFET opto
#define CUT_OFF false // chặn dòng qua MOSFET
#define THROUGH true  // thông dòng qua MOSFET

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);

// -------------------------------------------
// Ngưỡng & hằng số hệ thống
// -------------------------------------------
const float VOLTAGE_MINIMUM_THRESHOLD = 3.7f; // nếu xả < ngưỡng thì ngắt xả
const float VOLTAGE_STOP_THRESHOLD = 3.95f;   // nếu OCV >= ngưỡng thì ngừng sạc
const float VOLTAGE_CHARGE_MAX = 6.0f;        // nếu điện áp sạc > ngưỡng thì tạm ngừng sạc
const float CURRENT_MAX = 500.0f;             // mA, phòng ngừa quá dòng
const float BAT_CAPACITY_MAH = 1800.0f;       // dung lượng danh định cell 18650

// Hiệu chỉnh sụt áp
const float OFFSET_OCV = 0.03f;        // khi đo OCV
const float OFFSET_VOL_CHARGE = 0.00f; // khi đo lúc sạc

// Thời gian/chu kỳ (ms)
const uint32_t CHECK_PERIOD_MS = 1000;      // kiểm tra định kỳ
const uint32_t OCV_PERIOD_MS = 20UL * 1000; // chu kỳ đo OCV
const uint32_t OCV_SETTLE_MS = 5UL * 1000;  // chờ sau khi cô lập cell để đo OCV
const uint32_t RETRY_DELAY_MS = 5UL * 1000; // thời gian chờ trước khi thử lại
const uint32_t RETRY_SAMPLE_MS = 1500;      // thời gian chờ sau khi đóng để đọc lại
const uint32_t PAUSE_HIGH_VOLT_MS = 3000;   // thời gian pause ban đầu khi quá áp

// Dung lượng tối thiểu để thoát FULLY_CHARGED
const float CAPACITY_MINIMUM_THRESHOLD = 0.40f; // 40% dung lượng
const float DISCHARGE_VOLT_FLOOR = 0.75f;       // nhân với VOLTAGE_MINIMUM_THRESHOLD
const float CAPACITY_RESUME_DISCHARGE = 0.95f;  // ngưỡng quay lại sạc khi xả

// -------------------------------------------
// Biến trạng thái
// -------------------------------------------
float last_charge_v = 0.0f;
float last_current_mA = 0.0f;
float last_ocv = 0.0f;
float coulomb_mAh = 0.0f;
uint32_t lastCurrentSampleMs = 0;

bool chargePathClosed = false; // relay CLOSE, MOSFET CUT_OFF
bool state_mode_relay = CHARGE;

enum class BmsState
{
  INIT,
  CHARGING,
  DISCHARGING,
  FULLY_CHARGED,
  PAUSED_HIGH_VOLT,
  FAULT_OVERCURRENT,
  SOURCE_UNAVAILABLE
};
BmsState bmsState = BmsState::INIT;

uint32_t stateEntryMs = 0;
uint32_t lastCheckMs = 0;
uint32_t lastOcvRequestMs = 0;
uint32_t ocvSettleStartMs = 0;
bool ocvInProgress = false;
// uint32_t ocvUpdatedAtMs = 0;
bool ocvUpdated = 0;
uint32_t lastRetryAttemptMs = 0;
uint32_t retrySettleStartMs = 0;
bool retrySettling = false;
uint32_t monitorHoldUntilMs = 0; // chặn monitor lấy mẫu sau thao tác relay/mode

// Bộ nhớ đệm 20 giá trị dung lượng đã hiệu chỉnh từ OCV để lấy trung bình
static float capacityHistory[20];
static uint8_t capacityCount = 0;
static uint8_t capacityHead = 0;

// -------------------------------------------
// Tiện ích đóng/ngắt dòng theo thứ tự yêu cầu
// -------------------------------------------
void notifyMonitorHold()
{
  monitorHoldUntilMs = millis() + 1000; // 1s không lấy mẫu sau thao tác cơ khí
}

void openCurrentCell()
{
  // Thông MOSFET trước, sau đó mở relay để ngắt dòng an toàn
  digitalWrite(mosfetPins[0], THROUGH);
  delay(10);
  digitalWrite(relayPins[0], OPEN);
  chargePathClosed = false;
  notifyMonitorHold();
}

void closeCurrentCell()
{
  // Đóng relay trước, sau đó chặn MOSFET để nối cứng mạch
  digitalWrite(relayPins[0], CLOSE);
  delay(10);
  digitalWrite(mosfetPins[0], CUT_OFF);
  chargePathClosed = true;
  notifyMonitorHold();
}

void setModeCharge()
{
  digitalWrite(MODE_RELAY, CHARGE);
  state_mode_relay = CHARGE;
  notifyMonitorHold();
}

void setModeDischarge()
{
  digitalWrite(MODE_RELAY, DISCHARGE);
  state_mode_relay = DISCHARGE;
  notifyMonitorHold();
}

// -------------------------------------------
// Đọc điện áp & dòng từ INA219
// -------------------------------------------
void readChargeVoltageCurrent(float &voltage, float &current_mA)
{
  float shuntVoltage = ina219_1.getShuntVoltage_mV();
  float busVoltage = ina219_1.getBusVoltage_V();
  current_mA = ina219_1.getCurrent_mA();
  float loadVoltage = busVoltage + (shuntVoltage / 1000.0f);

  voltage = loadVoltage - OFFSET_VOL_CHARGE;
}

// Đo OCV sau khi đã cô lập cell
float measureOCV()
{
  float shuntVoltage = ina219_1.getShuntVoltage_mV();
  float busVoltage = ina219_1.getBusVoltage_V();
  float loadVoltage = busVoltage + (shuntVoltage / 1000.0f);
  return loadVoltage + OFFSET_OCV;
}

// Ước lượng dung lượng dựa trên OCV bằng nội suy tuyến tính
// Bảng ánh xạ OCV (V) -> phần trăm dung lượng
float estimateCapacityFromOcv(float ocv)
{
  static const float ocvVoltages[] = {
      3.00f, 3.45f, 3.68f, 3.74f, 3.77f, 3.79f,
      3.82f, 3.87f, 3.92f, 3.98f, 4.06f, 4.20f};
  static const float ocvPercents[] = {
      0.00f, 0.05f, 0.10f, 0.20f, 0.30f, 0.40f,
      0.50f, 0.60f, 0.70f, 0.80f, 0.90f, 1.00f};
  const int N = sizeof(ocvVoltages) / sizeof(ocvVoltages[0]);

  if (ocv <= ocvVoltages[0]) {
    return BAT_CAPACITY_MAH * ocvPercents[0];
  }
  if (ocv >= ocvVoltages[N - 1]) {
    return BAT_CAPACITY_MAH * ocvPercents[N - 1];
  }

  // Tìm khoảng [v_i, v_{i+1}] chứa ocv và nội suy phần trăm
  for (int i = 0; i < N - 1; ++i) {
    float v0 = ocvVoltages[i];
    float v1 = ocvVoltages[i + 1];
    if (ocv >= v0 && ocv <= v1) {
      float p0 = ocvPercents[i];
      float p1 = ocvPercents[i + 1];
      float t = (ocv - v0) / (v1 - v0);
      float percent = p0 + t * (p1 - p0);
      return BAT_CAPACITY_MAH * percent;
    }
  }

  // Fallback (không nên xảy ra): trả về dung lượng tối thiểu
  return BAT_CAPACITY_MAH * ocvPercents[0];
}

// -------------------------------------------
// Coulomb counting (tích phân dòng)
// -------------------------------------------
void integrateCoulomb(float current_mA)
{
  uint32_t now = millis();
  if (lastCurrentSampleMs == 0)
  {
    lastCurrentSampleMs = now;
    return;
  }

  uint32_t dt_ms = now - lastCurrentSampleMs;
  lastCurrentSampleMs = now;

  float delta_mAh = current_mA * (float)dt_ms / 3600000.0f;
  coulomb_mAh += delta_mAh;
  if (coulomb_mAh < 0)
    coulomb_mAh = 0;
  if (coulomb_mAh > BAT_CAPACITY_MAH)
    coulomb_mAh = BAT_CAPACITY_MAH;
}

// Đọc cảm biến và cập nhật coulomb count; hiệu chỉnh khi có OCV mới
void monitorCell()
{
  uint32_t now = millis();
  if (monitorHoldUntilMs != 0 && (int32_t)(monitorHoldUntilMs - now) > 0)
  {
    // Trong 1s sau thao tác relay/mode: trả về I=0, V=OCV và bỏ qua đo
    // last_current_mA = 0.0f;
    // Serial.println("Monitor hold active");
    // last_charge_v = last_charge_v;
    lastCurrentSampleMs = now; // reset mốc thời gian để tránh tích phân lệch
    return;
  }
  if (ocvInProgress)
  {
    // Khi đang đo OCV thì trả về I=0, V=OCV và bỏ qua đo
    last_current_mA = 0.0f;
    lastCurrentSampleMs = now; // reset mốc thời gian để tránh tích phân lệch
    return;
  }
  monitorHoldUntilMs = 0;

  readChargeVoltageCurrent(last_charge_v, last_current_mA);
  integrateCoulomb(last_current_mA);

  // Hiệu chỉnh dung lượng khi vừa cập nhật OCV
  if (ocvUpdated)
  {
    Serial.println("Adjust capacity from OCV");
    float cap = estimateCapacityFromOcv(last_ocv);
    // đưa vào bộ nhớ đệm 20 mẫu và lấy trung bình
    capacityHistory[capacityHead] = cap;
    capacityHead = (capacityHead + 1) % 20;
    if (capacityCount < 20) capacityCount++;
    float sum = 0.0f;
    for (uint8_t i = 0; i < capacityCount; ++i) sum += capacityHistory[i];
    float avg = (capacityCount > 0) ? (sum / capacityCount) : cap;
    coulomb_mAh = avg;
    ocvUpdated = false;
  }
}

// -------------------------------------------
// LCD helper
// -------------------------------------------
void lcdShowStatus(const char *line1, float vCharge, float vOcv, float mah)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(vCharge, 2);
  lcd.print(" V   ");
  lcd.setCursor(9, 1);
  lcd.print("C:");
  lcd.print(mah * 100 / 1800, 1);
  lcd.print("% ");
}

// -------------------------------------------
// State helpers
// -------------------------------------------
void changeState(BmsState next)
{
  if (bmsState == next)
    return;
  bmsState = next;
  stateEntryMs = millis();
  lastCheckMs = stateEntryMs; // đảm bảo chờ tối thiểu ~1s trước lần kiểm tra kế tiếp
  retrySettling = false;
  lastRetryAttemptMs = 0;
  retrySettleStartMs = 0;
  // Nếu chuyển trạng thái thì dừng chu trình OCV đang đợi
  ocvInProgress = false;
}

bool startOcvIfNeeded(uint32_t now)
{
  if (ocvInProgress)
    return true;
  ocvInProgress = true;
  ocvSettleStartMs = now;
  openCurrentCell();
  Serial.println("OCV settle start...");
  return true;
}

bool finishOcvIfReady(uint32_t now)
{
  if (!ocvInProgress)
    return false;
  if (now - ocvSettleStartMs < OCV_SETTLE_MS)
    return false;
  last_ocv = measureOCV();
  ocvInProgress = false;
  ocvUpdated = true;
  // // Hiệu chỉnh dung lượng ngay khi có OCV mới để tránh bỏ lỡ cửa sổ 200ms của monitor
  // coulomb_mAh = estimateCapacityFromOcv(last_ocv);
  Serial.print("OCV measured: ");
  Serial.println(last_ocv, 3);
  return true;
}

// -------------------------------------------
// Setup
// -------------------------------------------
void setup()
{
  Serial.begin(9600);
  while (!Serial)
    delay(1);

  pinMode(MODE_RELAY, OUTPUT);
  pinMode(relayPins[0], OUTPUT);
  pinMode(mosfetPins[0], OUTPUT);

  setModeCharge();
  closeCurrentCell();

  ina219_1.begin();

  lcd.init();
  lcd.backlight();
  lcd.print("BMS Starting...");
  Serial.println("BMS Starting...");
  delay(800);
  lcd.clear();

  bmsState = BmsState::INIT;
  stateEntryMs = millis();
  lastOcvRequestMs = millis();
}

// -------------------------------------------
// Manage task: state machine
// -------------------------------------------
uint8_t num_try = 0; // Số lần thực hiện tại trạng thái INIT

void manageCell()
{
  uint32_t now = millis();

  // Hoàn tất OCV nếu đang chờ
  if (finishOcvIfReady(now))
  {
    // Sau khi đo OCV xong, nếu trạng thái cần đóng mạch thì đóng lại
    if (bmsState == BmsState::CHARGING || bmsState == BmsState::DISCHARGING)
    {
      closeCurrentCell();
    }
  }

  switch (bmsState)
  {
  case BmsState::INIT:
  {
    // Khởi động: đo OCV trước khi đi vào CHARGING
    if (state_mode_relay != CHARGE)
      setModeCharge();
    if (!ocvInProgress && num_try < 1)
    {
      startOcvIfNeeded(now);
      lastOcvRequestMs = now;
      num_try++;
    }
    // Chỉ chuyển sang CHARGING sau khi đã đo xong OCV đầu tiên
    if (!ocvInProgress && last_ocv > 0.0f)
    {
      Serial.println("Initial OCV done, go to CHARGING");
      closeCurrentCell();
      changeState(BmsState::CHARGING);
    }
    break;
  }

  case BmsState::CHARGING:
  {
    if (state_mode_relay != CHARGE)
      setModeCharge();
    if (!chargePathClosed && !ocvInProgress)
      closeCurrentCell();

    // OCV định kỳ
    if (!ocvInProgress && (now - lastOcvRequestMs >= OCV_PERIOD_MS))
    {
      lastOcvRequestMs = now;
      startOcvIfNeeded(now);
    }

    // Kiểm tra định kỳ
    if (lastCheckMs == 0 || now - lastCheckMs >= CHECK_PERIOD_MS)
    {
      lastCheckMs = now;
      if (fabsf(last_current_mA) > CURRENT_MAX)
      {
        openCurrentCell();
        changeState(BmsState::FAULT_OVERCURRENT);
        Serial.println("Overcurrent -> FAULT");
        break;
      }
      if (last_charge_v > VOLTAGE_CHARGE_MAX)
      {
        openCurrentCell();
        changeState(BmsState::PAUSED_HIGH_VOLT);
        Serial.println("High voltage -> PAUSE");
        break;
      }
      if (coulomb_mAh >= BAT_CAPACITY_MAH)
      {
        openCurrentCell();
        changeState(BmsState::FULLY_CHARGED);
        Serial.println("Capacity full -> FULLY_CHARGED");
        break;
      }
      if ((last_current_mA < 0 || last_charge_v < last_ocv) && !ocvInProgress)
      {
        openCurrentCell();
        Serial.print("ocvInProgress=");
        Serial.println(ocvInProgress);
        changeState(BmsState::SOURCE_UNAVAILABLE);
        Serial.println("Source unavailable -> wait");
        break;
      }
    }
    break;
  }

  case BmsState::DISCHARGING:
  {
    if (state_mode_relay != DISCHARGE)
      setModeDischarge();
    if (!chargePathClosed && !ocvInProgress)
      closeCurrentCell();

    if (lastCheckMs == 0 || now - lastCheckMs >= CHECK_PERIOD_MS)
    {
      lastCheckMs = now;
      if (fabsf(last_current_mA) > CURRENT_MAX)
      {
        openCurrentCell();
        changeState(BmsState::FAULT_OVERCURRENT);
        Serial.println("Overcurrent (discharge) -> FAULT");
        break;
      }
      if (last_charge_v < DISCHARGE_VOLT_FLOOR * VOLTAGE_MINIMUM_THRESHOLD ||
          coulomb_mAh < BAT_CAPACITY_MAH * CAPACITY_RESUME_DISCHARGE)
      {
        setModeCharge();
        changeState(BmsState::CHARGING);
        Serial.println("Low voltage/capacity -> CHARGING");
        break;
      }
    }
    break;
  }

  case BmsState::FULLY_CHARGED:
  {
    if (chargePathClosed)
      openCurrentCell();

    if (!ocvInProgress && (now - lastOcvRequestMs >= OCV_PERIOD_MS))
    {
      lastOcvRequestMs = now;
      startOcvIfNeeded(now);
    }

    if (lastCheckMs == 0 || now - lastCheckMs >= OCV_PERIOD_MS)
    {
      lastCheckMs = now;
      if (last_ocv < VOLTAGE_MINIMUM_THRESHOLD ||
          coulomb_mAh < BAT_CAPACITY_MAH * CAPACITY_MINIMUM_THRESHOLD)
      {
        setModeCharge();
        changeState(BmsState::CHARGING);
        Serial.println("Capacity/OCV dropped -> CHARGING");
      }
    }
    break;
  }

  case BmsState::PAUSED_HIGH_VOLT:
  {
    if (chargePathClosed && !retrySettling)
      openCurrentCell();
    if (now - stateEntryMs < PAUSE_HIGH_VOLT_MS)
      break; // chờ hạ áp ban đầu

    if (!retrySettling && (now - lastRetryAttemptMs >= RETRY_DELAY_MS))
    {
      closeCurrentCell();
      retrySettling = true;
      retrySettleStartMs = now;
      Serial.println("Retry closing to sample...");
    }

    if (retrySettling && (now - retrySettleStartMs >= RETRY_SAMPLE_MS))
    {
      retrySettling = false;
      lastRetryAttemptMs = now;
      if (fabsf(last_current_mA) > CURRENT_MAX)
      {
        openCurrentCell();
        changeState(BmsState::FAULT_OVERCURRENT);
        Serial.println("Overcurrent on retry -> FAULT");
        break;
      }
      if (last_charge_v < VOLTAGE_CHARGE_MAX)
      {
        changeState(BmsState::CHARGING);
        Serial.println("Voltage ok -> resume CHARGING");
      }
      else
      {
        openCurrentCell();
        Serial.println("Still high -> keep paused");
      }
    }
    break;
  }

  case BmsState::SOURCE_UNAVAILABLE:
  {
    if (chargePathClosed && !retrySettling)
      openCurrentCell();
    if (!retrySettling && (now - stateEntryMs >= RETRY_DELAY_MS))
    {
      closeCurrentCell();
      retrySettling = true;
      retrySettleStartMs = now;
      Serial.println("Retry source check...");
    }

    if (retrySettling && (now - retrySettleStartMs >= RETRY_SAMPLE_MS))
    {
      retrySettling = false;
      lastRetryAttemptMs = now;
      if (last_charge_v >= last_ocv || last_current_mA >= 0)
      {
        changeState(BmsState::CHARGING);
        Serial.println("Source ok -> CHARGING");
      }
      else
      {
        openCurrentCell();
        stateEntryMs = now; // restart wait
        Serial.println("Source still low -> wait more");
      }
    }
    break;
  }

  case BmsState::FAULT_OVERCURRENT:
  {
    if (chargePathClosed)
      openCurrentCell();
    // Dừng mãi mãi
    break;
  }
  }
}

// -------------------------------------------
// Loop chính: chạy monitor + manage
// -------------------------------------------
uint32_t lastPrintMs = 0;
uint32_t lastMonitorMs = 0;
uint32_t lastManageMs = 0;

void loop()
{
  uint32_t now = millis();

  // Giới hạn monitorCell mỗi 0.5s để lấy mẫu ổn định
  if (lastMonitorMs == 0 || (now - lastMonitorMs) >= 250)
  {
    monitorCell();
    lastMonitorMs = now;
  }

  if (lastManageMs == 0 || (now - lastManageMs) >= 500)
  {
    manageCell();
    lastManageMs = now;
  }

  // Cập nhật LCD định kỳ (~2s)
  if ((now % 2000) < 50)
  {
    const char *line1 =
        (bmsState == BmsState::CHARGING) ? "Charging" : (bmsState == BmsState::DISCHARGING)      ? "Discharging"
                                                    : (bmsState == BmsState::FULLY_CHARGED)      ? "Full"
                                                    : (bmsState == BmsState::PAUSED_HIGH_VOLT)   ? "Paused Hi-V"
                                                    : (bmsState == BmsState::SOURCE_UNAVAILABLE) ? "Src unavailable"
                                                    : (bmsState == BmsState::FAULT_OVERCURRENT)  ? "Overcurrent!"
                                                                                                 : "Init";
    lcdShowStatus(line1, last_charge_v, last_ocv, coulomb_mAh);
  }
  if (lastPrintMs == 0 || now - lastPrintMs >= 2000)
  {
    Serial.print("State: ");
    Serial.print((uint8_t)bmsState);
    Serial.print(", V: ");
    Serial.print(last_charge_v, 3);
    Serial.print(" V, OCV: ");
    Serial.print(last_ocv, 3);
    Serial.print(" V, I: ");
    Serial.print(last_current_mA, 1);
    Serial.print(" mA, Q: ");
    Serial.print(coulomb_mAh, 1);
    Serial.println(" mAh");
    lastPrintMs = now;
  }
}