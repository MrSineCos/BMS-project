# Single-Cell BMS Controller

Hệ thống điều khiển BMS cho một cell pin, triển khai hai tác vụ chạy song song: `manage_cell` và `monitor_cell`. Mục tiêu: kiểm soát sạc/xả, bảo vệ quá dòng/quá áp, đo OCV định kỳ và hiệu chỉnh dung lượng pin.

## Kiến trúc tác vụ

- manage_cell
  - Máy trạng thái điều khiển BMS.
  - Thực thi theo timer mềm (chu kỳ quản lý ~500 ms).
  - Điều phối đóng/ngắt relay cell, chuyển chế độ CHARGE/DISCHARGE, và chu trình đo OCV.

- monitor_cell
  - Đọc cảm biến INA219: điện áp V và dòng I.
  - Tích phân coulomb cho cả sạc/xả để tính dung lượng C (mAh).
  - Hiệu chỉnh dung lượng theo OCV khi có OCV mới (nội suy tuyến tính theo bảng).
  - Chu kỳ ~250 ms và được “hold” 1 s sau thao tác relay/mode hoặc trong quá trình đo OCV:
    - Trong thời gian hold, bỏ qua đo tức thời, đặt I=0 (không tích phân thêm), V giữ nguyên/OCV.

## Phần cứng và tín hiệu

- Relay cell: đóng/ngắt dòng của cell (logic: CLOSE/OPEN).
- Relay switch mode: chuyển chế độ hệ thống CHARGE/DISCHARGE.
- MOSFET opto: điều phối đường dòng phụ (logic: CUT_OFF/THROUGH).

Trình tự thao tác:
- openCurrentCell: MOSFET THROUGH → relay OPEN.
- closeCurrentCell: relay CLOSE → MOSFET CUT_OFF.
- Chuyển mode: `setModeCharge` / `setModeDischarge` kích relay mode và báo hold 1 s cho `monitor_cell`.

## Đo OCV

Được điều phối bởi `manage_cell`:
1. Ngắt dòng (openCurrentCell).
2. Chờ settle 5 s (`OCV_SETTLE_MS`).
3. Đo điện áp mạch hở, gán OCV.

- OCV định kỳ mỗi 20 s (`OCV_PERIOD_MS`) trong CHARGING và các trạng thái cho phép.
- OCV đầu tiên được đo tại INIT trước khi vào CHARGING.

## Máy trạng thái manage_cell

Các trạng thái: `{ INIT, CHARGING, DISCHARGING, FULLY_CHARGED, PAUSED_HIGH_VOLT, FAULT_OVERCURRENT, SOURCE_UNAVAILABLE }`.

- INIT
  - Đặt mode CHARGE, thực hiện đo OCV đầu tiên.
  - Khi OCV có giá trị, đóng lại đường cell và chuyển CHARGING.

- CHARGING
  - Đóng relay cell, mode CHARGE.
  - Kiểm tra mỗi `CHECK_PERIOD_MS` (1 s):
    - |I| > `CURRENT_MAX` → FAULT_OVERCURRENT.
    - V > `VOLTAGE_CHARGE_MAX` → PAUSED_HIGH_VOLT.
    - C ≥ `BAT_CAPACITY_MAH` → FULLY_CHARGED.
    - Nếu I < 0 hoặc V < OCV (khi không đo OCV) → SOURCE_UNAVAILABLE.
  - OCV định kỳ mỗi 20 s (ngắt đo 5 s rồi đọc).

- DISCHARGING
  - Đóng relay cell, mode DISCHARGE.
  - Kiểm tra mỗi 1 s:
    - |I| > `CURRENT_MAX` → FAULT_OVERCURRENT.
    - Nếu V < 0.75 × `VOLTAGE_MINIMUM_THRESHOLD` hoặc C < 0.95 × `BAT_CAPACITY_MAH` → CHARGING.

- FULLY_CHARGED
  - Mở relay cell.
  - OCV định kỳ mỗi 20 s.
  - Mỗi 20 s: nếu OCV < `VOLTAGE_MINIMUM_THRESHOLD` hoặc C < 40% dung lượng → CHARGING.

- PAUSED_HIGH_VOLT
  - Mở relay cell, chờ 3 s (`PAUSE_HIGH_VOLT_MS`).
  - Chu kỳ retry:
    - Mỗi `RETRY_DELAY_MS` (5 s) đóng lại, chờ `RETRY_SAMPLE_MS` (1.5 s) để đo.
    - Nếu V < `VOLTAGE_CHARGE_MAX` → CHARGING; nếu vẫn cao → mở lại và chờ tiếp.
    - Quá dòng trong retry → FAULT_OVERCURRENT.

- SOURCE_UNAVAILABLE
  - Mở relay cell, chờ `RETRY_DELAY_MS` (5 s).
  - Đóng lại và chờ `RETRY_SAMPLE_MS` (1.5 s); nếu V ≥ OCV hoặc I ≥ 0 → CHARGING, ngược lại mở ra và chờ thêm.

- FAULT_OVERCURRENT
  - Mở relay cell, dừng vĩnh viễn và báo LCD.

## Hiệu chỉnh dung lượng theo OCV

- Bảng nội suy tuyến tính OCV → % dung lượng:
  - 3.00 V = 0%, 3.45 V = 5%, 3.68 V = 10%, 3.74 V = 20%, 3.77 V = 30%,
    3.79 V = 40%, 3.82 V = 50%, 3.87 V = 60%, 3.92 V = 70%,
    3.98 V = 80%, 4.06 V = 90%, 4.20 V = 100%.
- Dung lượng sau hiệu chỉnh = (% × `BAT_CAPACITY_MAH`).
- Giá trị dung lượng pin sử dụng trung bình của 20 mẫu hiệu chỉnh gần nhất (bộ đệm vòng).

## Hằng số chính (tham khảo)

- `CHECK_PERIOD_MS` = 1000 ms
- `OCV_PERIOD_MS` = 20000 ms
- `OCV_SETTLE_MS` = 5000 ms
- `RETRY_DELAY_MS` = 5000 ms
- `RETRY_SAMPLE_MS` = 1500 ms
- `PAUSE_HIGH_VOLT_MS` = 3000 ms
- `CURRENT_MAX` = 500 mA
- `VOLTAGE_MINIMUM_THRESHOLD` = 3.7 V
- `VOLTAGE_STOP_THRESHOLD` = 3.95 V
- `VOLTAGE_CHARGE_MAX` = 6.0 V
- `BAT_CAPACITY_MAH` = 1800 mAh
- `CAPACITY_MINIMUM_THRESHOLD` = 40%

## Ghi chú triển khai

- Tác vụ `monitor_cell` chạy ~250 ms với cơ chế hold 1 s sau mọi thao tác cơ khí (open/close/mode) và trong lúc đang đo OCV.
- Tất cả thao tác thời gian dùng `millis()` (không dùng `delay`) để giữ tính phi chặn.
- LCD hiển thị các thông số chính và trạng thái để theo dõi vận hành.
