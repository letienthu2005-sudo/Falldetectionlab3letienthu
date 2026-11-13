/*
 * MẠCH PHÁT HIỆN TÉ NGÃ (FALL DETECTION)
 * ESP32 + MPU6050 (I2C) + LED + Buzzer, FreeRTOS 3 task, MQTT (broker EMQX)
 *
 * Kiến trúc:
 *   Task đọc data    (Core 1, ưu tiên 2): đọc MPU6050 mỗi 50 ms, gửi qAlert mỗi mẫu;
 *                                         mỗi 0,5 s in Serial và gửi gói đầy đủ vào qData
 *   Task cảnh báo    (Core 1, ưu tiên 3): nhận qAlert + qLenh, là task DUY NHẤT điều khiển LED/còi
 *   Task truyền data (Core 0, ưu tiên 1): kết nối WiFi, chờ cờ semWiFi, kết nối MQTT, gửi dữ liệu, nhận lệnh từ web
 *
 * Đối tượng giao tiếp:
 *   qData      (Queue 5 ô)  : Task đọc     -> Task truyền   ("data Queue": ax, ay, az, a, nghiêng, pitch, roll)
 *   qAlert     (Queue 10 ô) : Task đọc     -> Task cảnh báo ("data Alert": a, nghiêng)
 *   qLenh      (Queue 3 ô)  : Task truyền  -> Task cảnh báo (lệnh nút bấm web)
 *   qTrangThai (Queue 1 ô)  : Task cảnh báo-> Task truyền   (trạng thái đèn/còi, hộp thư ghi đè)
 *   semWiFi    (Binary sem) : sự kiện WiFi -> Task truyền   ("kết nối WiFi thành công")
 *   mSerial    (Mutex)      : khóa Serial dùng chung giữa Task đọc (Core 1) và Task truyền (Core 0)
 *
 * Phát hiện té ngã (a là độ lớn gia tốc tổng, tính bằng g; đứng yên a ≈ 1 g):
 *   - Gia tốc đột ngột: a > 2,5 g (va đập) hoặc a < 0,4 g (rơi tự do)
 *   - Góc nghiêng đột ngột: góc nghiêng đổi quá 45° so với 1 giây trước
 *   - TÉ NGÃ = gia tốc đột ngột VÀ góc nghiêng đột ngột trong vòng 2 giây
 * Luật đèn/còi (chỉ một trong hai):
 *   - Té ngã                                  -> CÒI (đèn tắt), giữ FALL_HOLD_MS
 *   - Chỉ gia tốc đột ngột / chỉ nghiêng đột ngột -> ĐÈN, giữ WARN_HOLD_MS
 *   - Nằm/nghiêng quá 60° kéo dài              -> ĐÈN
 *   - Chế độ thủ công (nút web): bật thiết bị nào thì tắt thiết bị kia,
 *     tự quay về tự động sau MANUAL_TIMEOUT_MS, hoặc ngay khi phát hiện té ngã mới.
 *
 * Thư viện: Adafruit MPU6050, Adafruit Unified Sensor, Adafruit BusIO, PubSubClient
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PubSubClient.h>

// ---------------- Cấu hình phần cứng ----------------
#define LED_PIN     12
#define BUZZER_PIN  5
#define BUZZER_FREQ 2000

// ---------------- Cấu hình mạng ----------------
// Tên mạng WiFi ảo của trình mô phỏng: Wokwi phát "Wokwi-GUEST", Cirkit Designer phát "CirkitWifi".
// Task truyền quét mạng lúc khởi động và tự chọn tên có thật (xem TaskTruyenData).
const char* WIFI_SSID_WOKWI  = "Wokwi-GUEST";
const char* WIFI_SSID_CIRKIT = "CirkitWifi";
const char* WIFI_PASS = "";
const char* MQTT_HOST = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
// Topic của đề té ngã (nếu trùng người khác trên broker công cộng, đổi "lab3" thành tên riêng của bạn)
const char* TOPIC_DATA = "FallMonitor/data";
const char* TOPIC_CMD  = "FallMonitor/cmd";

// ---------------- Ngưỡng phát hiện ----------------
const float IMPACT_G   = 2.5;          // g, gia tốc tổng lớn hơn => va đập
const float FREEFALL_G = 0.4;          // g, gia tốc tổng nhỏ hơn => đang rơi tự do
const float TILT_JUMP  = 45.0;         // độ, góc nghiêng đổi hơn chừng này trong 1 s => đột ngột
const float TILT_LYING = 60.0;         // độ, nghiêng hơn chừng này => đang nằm / nghiêng bất thường
const uint32_t FALL_WINDOW_MS = 2000;  // va đập và nghiêng đột ngột cách nhau tối đa 2 s => té ngã
const uint32_t FALL_HOLD_MS   = 5000; // 
const uint32_t WARN_HOLD_MS   = 3000;  // giữ đèn 3 s sau một dấu hiệu bất thường đơn lẻ
const uint32_t MANUAL_TIMEOUT_MS = 10000; // 

// ---------------- Nhịp đọc ----------------
const uint32_t SAMPLE_PERIOD_MS = 50;  // đọc cảm biến 20 lần/giây để bắt kịp cú va đập ngắn
const uint8_t  REPORT_EVERY     = 10;  // cứ 10 mẫu (0,5 s) mới in Serial và gửi lên MQTT một lần
const int      TILT_HIST        = 20;  // số mẫu nhớ lại = 1 giây (20 x 50 ms)
const float    G_MS2            = 9.80665; // 1 g = 9,80665 m/s²

// ---------------- Kiểu dữ liệu ----------------
typedef struct { float ax, ay, az, a, tilt, pitch, roll; } DuLieu;  // gói đầy đủ (qData)
typedef struct { float a, tilt; } CanhBao;                           // gói cảnh báo (qAlert)
typedef enum { CMD_LED_ON, CMD_LED_OFF, CMD_BUZ_ON, CMD_BUZ_OFF, CMD_AUTO } Lenh;
typedef enum { R_NONE, R_IMPACT, R_TILT, R_FALL, R_MANUAL } LyDo;
typedef struct { bool led; bool buzzer; bool manual; uint8_t reason; } TrangThai;

// ---------------- Đối tượng FreeRTOS ----------------
QueueHandle_t qData, qAlert, qLenh, qTrangThai;
SemaphoreHandle_t semWiFi, mSerial;

Adafruit_MPU6050 mpu;
WiFiClient netClient;
PubSubClient mqtt(netClient);

// ---------------- Serial dùng chung (mutex) ----------------
void logLine(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (xSemaphoreTake(mSerial, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.println(buf);
    xSemaphoreGive(mSerial);
  }
}

// ---------------- Sự kiện WiFi -> giơ cờ semWiFi ----------------
// Hàm này chạy trong task sự kiện của hệ thống (không phải ngắt), nên dùng xSemaphoreGive thường.
void onWiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    logLine("[WIFI] Su kien GOT_IP, IP = %s", WiFi.localIP().toString().c_str());
    xSemaphoreGive(semWiFi);
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    logLine("[WIFI] Su kien mat ket noi WiFi");
  }
}

// ---------------- Nhận lệnh MQTT (chạy bên trong Task truyền) ----------------
void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  char msg[24];
  unsigned int n = len < sizeof(msg) - 1 ? len : sizeof(msg) - 1;
  memcpy(msg, payload, n);
  msg[n] = '\0';

  Lenh cmd;
  if      (strcmp(msg, "led_on") == 0)     cmd = CMD_LED_ON;
  else if (strcmp(msg, "led_off") == 0)    cmd = CMD_LED_OFF;
  else if (strcmp(msg, "buzzer_on") == 0)  cmd = CMD_BUZ_ON;
  else if (strcmp(msg, "buzzer_off") == 0) cmd = CMD_BUZ_OFF;
  else if (strcmp(msg, "auto") == 0)       cmd = CMD_AUTO;
  else { logLine("[TRUYEN] Lenh khong hop le: %s", msg); return; }

  // Không chạm vào LED/còi ở đây: chỉ chuyển lệnh cho Task cảnh báo
  xQueueSend(qLenh, &cmd, 0);
  logLine("[TRUYEN] Nhan lenh: %s", msg);
}

// =====================================================================
// TASK ĐỌC DATA - Core 1, ưu tiên 2
// =====================================================================
void TaskDocData(void* pv) {
  TickType_t lastWake = xTaskGetTickCount();
  uint8_t dem = 0;
  float aDinh = 1.0f;          // gia tốc "bất thường nhất" (xa 1 g nhất) trong 0,5 s gần đây
  for (;;) {
    sensors_event_t acc, gyr, nhiet;
    mpu.getEvent(&acc, &gyr, &nhiet);

    // Đổi m/s² sang g
    float ax = acc.acceleration.x / G_MS2;
    float ay = acc.acceleration.y / G_MS2;
    float az = acc.acceleration.z / G_MS2;
    float a  = sqrtf(ax * ax + ay * ay + az * az);           // độ lớn gia tốc tổng

    // Góc nghiêng so với tư thế đứng thẳng (trục z hướng lên): 0° là thẳng, 90° là nằm ngang
    float cosTilt = (a > 0.05f) ? fmaxf(-1.0f, fminf(1.0f, az / a)) : 1.0f;
    float tilt  = acosf(cosTilt) * RAD_TO_DEG;
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;   // nghiêng trước/sau
    float roll  = atan2f(ay, az) * RAD_TO_DEG;                          // nghiêng trái/phải

    // 1. Gửi dữ liệu cảnh báo NGAY mỗi mẫu: Task cảnh báo (ưu tiên 3) chen ngang xử lý liền
    CanhBao c = { a, tilt };
    xQueueSend(qAlert, &c, 0);

    // Ghi nhớ giá trị gia tốc bất thường nhất trong khoảng báo cáo, để web không bỏ sót cú va đập ngắn
    if (fabsf(a - 1.0f) > fabsf(aDinh - 1.0f)) aDinh = a;

    if (++dem >= REPORT_EVERY) {
      DuLieu d = { ax, ay, az, aDinh, tilt, pitch, roll };

      // 2. In Serial (có khóa mSerial)
      logLine("[DOC] a=%.2f g | ax=%.2f ay=%.2f az=%.2f | nghieng=%.1f | pitch=%.1f roll=%.1f",
              d.a, d.ax, d.ay, d.az, d.tilt, d.pitch, d.roll);

      // 3. Gửi gói đầy đủ cho Task truyền; đầy (mất mạng) thì bỏ gói cũ nhất, giữ gói mới
      if (xQueueSend(qData, &d, 0) != pdTRUE) {
        DuLieu cu;
        xQueueReceive(qData, &cu, 0);
        xQueueSend(qData, &d, 0);
      }
      dem = 0;
      aDinh = 1.0f;
    }

    // 4. Ngủ đến đúng nhịp 50 ms tiếp theo (không trôi chu kỳ)
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
  }
}

// =====================================================================
// TASK CẢNH BÁO - Core 1, ưu tiên 3 (chủ duy nhất của LED và còi)
// =====================================================================
static bool ledOn = false, buzOn = false;

void apDungDauRa(bool led, bool buz) {
  if (led && buz) led = false;                     // luật: chỉ một trong hai, còi ưu tiên
  if (led != ledOn) { digitalWrite(LED_PIN, led ? HIGH : LOW); ledOn = led; }
  if (buz != buzOn) { if (buz) tone(BUZZER_PIN, BUZZER_FREQ); else noTone(BUZZER_PIN); buzOn = buz; }
}

void TaskCanhBao(void* pv) {
  float tiltHist[TILT_HIST];   // góc nghiêng của 1 giây gần nhất (vòng tròn)
  int nHist = 0, iHist = 0;
  bool coSoc = false, coNghieng = false;   // đã thấy gia tốc đột ngột / nghiêng đột ngột chưa
  uint32_t socLuc = 0, nghiengLuc = 0;     // thời điểm thấy
  uint32_t fallUntil = 0, warnUntil = 0;   // còi / đèn giữ đến thời điểm này (ms)
  uint8_t warnReason = R_NONE;
  bool manual = false;
  bool manLed = false, manBuz = false;
  uint32_t manualUntil = 0;
  float lastTilt = 0;

  TrangThai st = { false, false, false, R_NONE };
  xQueueOverwrite(qTrangThai, &st);               // hộp thư luôn có giá trị ban đầu

  for (;;) {
    CanhBao c;
    bool coDuLieu = xQueueReceive(qAlert, &c, pdMS_TO_TICKS(200)) == pdTRUE;
    uint32_t now = millis();

    if (coDuLieu) {
      lastTilt = c.tilt;

      // Dấu hiệu 1: gia tốc đột ngột (va đập mạnh hoặc rơi tự do)
      if (c.a > IMPACT_G || c.a < FREEFALL_G) {
        coSoc = true; socLuc = now;
        warnUntil = now + WARN_HOLD_MS; warnReason = R_IMPACT;
      }

      // Dấu hiệu 2: góc nghiêng đổi đột ngột so với 1 giây trước
      if (nHist == TILT_HIST && fabsf(c.tilt - tiltHist[iHist]) > TILT_JUMP) {
        coNghieng = true; nghiengLuc = now;
        warnUntil = now + WARN_HOLD_MS; warnReason = R_TILT;
      }
      tiltHist[iHist] = c.tilt;
      iHist = (iHist + 1) % TILT_HIST;
      if (nHist < TILT_HIST) nHist++;

      // Quên dấu hiệu đã quá cũ
      if (coSoc && (now - socLuc) > FALL_WINDOW_MS) coSoc = false;
      if (coNghieng && (now - nghiengLuc) > FALL_WINDOW_MS) coNghieng = false;

      // TÉ NGÃ: có cả hai dấu hiệu trong cùng khoảng 2 giây
      if (coSoc && coNghieng) {
        fallUntil = now + FALL_HOLD_MS;
        manual = false;                                  // té ngã luôn giành lại quyền tự động
        coSoc = coNghieng = false;
        for (int i = 0; i < TILT_HIST; i++) tiltHist[i] = c.tilt;   // làm mới lịch sử, tránh báo lặp
        logLine("[CANH BAO] PHAT HIEN TE NGA! a=%.2f g, nghieng=%.1f do", c.a, c.tilt);
      }
    }

    // Lệnh từ web: xem qLenh, không chờ
    Lenh cmd;
    while (xQueueReceive(qLenh, &cmd, 0) == pdTRUE) {
      if (cmd == CMD_AUTO) { manual = false; continue; }
      if (!manual) { manLed = ledOn; manBuz = buzOn; }   // vừa vào thủ công: bắt đầu từ trạng thái hiện tại
      manual = true;
      manualUntil = now + MANUAL_TIMEOUT_MS;
      if (cmd == CMD_LED_ON)  { manLed = true;  manBuz = false; }
      if (cmd == CMD_LED_OFF) { manLed = false; }
      if (cmd == CMD_BUZ_ON)  { manBuz = true;  manLed = false; }
      if (cmd == CMD_BUZ_OFF) { manBuz = false; }
    }
    if (manual && (int32_t)(now - manualUntil) >= 0) manual = false;

    // Quyết định đầu ra
    bool led, buz;
    uint8_t reason;
    if (manual) {
      led = manLed; buz = manBuz; reason = R_MANUAL;
    } else {
      bool fall = (int32_t)(fallUntil - now) > 0;
      bool warn = (int32_t)(warnUntil - now) > 0;
      bool lying = lastTilt > TILT_LYING;
      if (fall)        { led = false; buz = true;  reason = R_FALL; }
      else if (warn)   { led = true;  buz = false; reason = warnReason; }
      else if (lying)  { led = true;  buz = false; reason = R_TILT; }
      else             { led = false; buz = false; reason = R_NONE; }
    }
    apDungDauRa(led, buz);

    // Ghi đè trạng thái thật vào hộp thư cho Task truyền
    TrangThai moi = { ledOn, buzOn, manual, reason };
    if (moi.led != st.led || moi.buzzer != st.buzzer || moi.manual != st.manual || moi.reason != st.reason) {
      st = moi;
      xQueueOverwrite(qTrangThai, &st);
    }
  }
}

// =====================================================================
// TASK TRUYỀN DATA - Core 0, ưu tiên 1
// =====================================================================
const char* tenLyDo(uint8_t r) {
  switch (r) {
    case R_IMPACT: return "impact";
    case R_TILT:   return "tilt";
    case R_FALL:   return "fall";
    case R_MANUAL: return "manual";
    default:       return "none";
  }
}

bool ketNoiMQTT() {
  char clientId[32];
  snprintf(clientId, sizeof(clientId), "fall-esp32-%04X", (unsigned)(esp_random() & 0xFFFF));
  if (mqtt.connect(clientId)) {
    mqtt.subscribe(TOPIC_CMD);
    logLine("[TRUYEN] Da ket noi MQTT broker %s", MQTT_HOST);
    return true;
  }
  IPAddress ip;
  bool dns = WiFi.hostByName(MQTT_HOST, ip);
  logLine("[TRUYEN] Ket noi MQTT loi (rc=%d), DNS %s %s, RSSI %d, thu lai sau 2 s",
          mqtt.state(), dns ? "ok" : "loi", dns ? ip.toString().c_str() : "", WiFi.RSSI());
  return false;
}

void TaskTruyenData(void* pv) {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.onEvent(onWiFiEvent);

  // Quét mạng rồi chọn đúng tên mạng mà trình mô phỏng đang phát
  const char* ssid = WIFI_SSID_WOKWI;
  int n = WiFi.scanNetworks();
  logLine("[WIFI] Quet thay %d mang", n);
  for (int i = 0; i < n; i++) {
    logLine("[WIFI]  - %s (kenh %d)", WiFi.SSID(i).c_str(), WiFi.channel(i));
    if (WiFi.SSID(i) == WIFI_SSID_CIRKIT) ssid = WIFI_SSID_CIRKIT;
  }
  WiFi.scanDelete();
  logLine("[WIFI] Ket noi vao mang: %s", ssid);
  WiFi.begin(ssid, WIFI_PASS);                    // không ép kênh: tự tìm mạng trên mọi kênh

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(512);                        // JSON té ngã dài hơn bài trước, nới bộ đệm cho chắc
  mqtt.setSocketTimeout(5);                       // mỗi lần thử kết nối broker tối đa 5 s
  uint8_t loiMqtt = 0;                            // số lần vào broker thất bại liên tiếp

  for (;;) {
    // "Kết nối WiFi, nếu thành công..." : chưa có WiFi thì ngủ chờ cờ semWiFi
    if (WiFi.status() != WL_CONNECTED) {
      logLine("[TRUYEN] Cho WiFi... (status=%d)", WiFi.status());
      xSemaphoreTake(semWiFi, pdMS_TO_TICKS(2000));  // chờ cờ tối đa 2 s rồi tự kiểm tra lại
      continue;
    }
    if (!mqtt.connected()) {
      if (!ketNoiMQTT()) {
        if (++loiMqtt >= 3) {                     // 3 lần liền không vào được broker: làm mới WiFi
          logLine("[TRUYEN] Lam moi ket noi WiFi");
          WiFi.disconnect();
          vTaskDelay(pdMS_TO_TICKS(500));
          WiFi.begin(ssid, WIFI_PASS);
          loiMqtt = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
      loiMqtt = 0;
    }

    mqtt.loop();                                  // giữ kết nối và nhận lệnh (gọi onMqttMessage)

    // Chờ dữ liệu tối đa 100 ms để còn quay lại gọi mqtt.loop() đều đặn
    DuLieu d;
    if (xQueueReceive(qData, &d, pdMS_TO_TICKS(100)) == pdTRUE) {
      TrangThai st;
      xQueuePeek(qTrangThai, &st, 0);             // xem trạng thái, không lấy ra
      char json[256];
      snprintf(json, sizeof(json),
               "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"a\":%.2f,\"tilt\":%.1f,\"pitch\":%.1f,\"roll\":%.1f,"
               "\"led\":%d,\"buzzer\":%d,\"mode\":\"%s\",\"alert\":\"%s\"}",
               d.ax, d.ay, d.az, d.a, d.tilt, d.pitch, d.roll,
               st.led, st.buzzer, st.manual ? "manual" : "auto", tenLyDo(st.reason));
      if (mqtt.publish(TOPIC_DATA, json)) logLine("[TRUYEN] Gui: %s", json);
    }
  }
}

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);

  // Tạo đối tượng giao tiếp TRƯỚC khi tạo task
  qData      = xQueueCreate(5, sizeof(DuLieu));
  qAlert     = xQueueCreate(10, sizeof(CanhBao));
  qLenh      = xQueueCreate(3, sizeof(Lenh));
  qTrangThai = xQueueCreate(1, sizeof(TrangThai));
  semWiFi    = xSemaphoreCreateBinary();
  mSerial    = xSemaphoreCreateMutex();

  if (!mpu.begin()) {
    Serial.println("Khong tim thay MPU6050, kiem tra day SDA/SCL!");
    while (true) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   // đo được tới 8 g, đủ bắt cú va đập > 2,5 g
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);     // lọc nhiễu rung nhẹ
  Serial.println("He thong khoi dong...");

  //                       hàm             tên          stack  tham số  ưu tiên handle  core
  xTaskCreatePinnedToCore(TaskCanhBao,    "CanhBao",   4096,  NULL,    3,      NULL,   1);
  xTaskCreatePinnedToCore(TaskDocData,    "DocData",   4096,  NULL,    2,      NULL,   1);
  xTaskCreatePinnedToCore(TaskTruyenData, "TruyenData",8192,  NULL,    1,      NULL,   0);
}

void loop() {
  vTaskDelete(NULL);   // loopTask không dùng đến, tự xóa để nhường CPU
}
