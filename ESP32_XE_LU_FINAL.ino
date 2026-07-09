#include <WiFi.h>

const char* ssid = "ESP32_XE_LU";
const char* password = "12345678";
WiFiServer server(80);

// ================== CẢM BIẾN ==================
const int POT_PIN   = 32;   // biến trở góc lái
const int GEAR_N_SW = 23;   // cảm biến/công tắc N: N = HIGH, ngoài N = LOW

const float STRAIGHT_MIN = 1.50;
const float STRAIGHT_MAX = 1.80;
bool pendingRunAfterN = false;
bool steerDisplayActive = false;
char lastSteer = 'S';

// Khóa dừng khẩn cấp: khi bật, ESP32 bỏ qua RUN/tiến/lùi/lái cho tới khi mở khóa.
bool emergencyLock = false;

// Khóa lệnh RUN/R từ Jetson khi người dùng ưu tiên điều khiển tay.
// Bật khóa khi bấm VỀ N, LÙI hoặc STOP AI để Jetson không kéo cần số lên F lại.
// Mở khóa khi bấm START AI, CLEAR hoặc TIẾN để cho phép Jetson điều khiển RUN trở lại.
bool aiStopLock = false;

// ================== MOTOR 1: LÁI ==================
const int RPWM1 = 18;
const int LPWM1 = 19;
const int R_EN1 = 26;
const int L_EN1 = 25;

// ================== MOTOR 2: CHUYỂN SỐ ==================
const int RPWM2 = 16;
const int LPWM2 = 17;
const int R_EN2 = 12;
const int L_EN2 = 14;

// ================== PWM ==================
const int freq = 10000;
const int resolution = 8;

// Tốc độ lái cố định
int speedLai = 220;
const int START_PWM_LAI = 180;
// Tốc độ motor chuyển số:
// - speedChay dùng cho lúc sang F/R: để cần số đi đủ mạnh.
// - speedReturnN dùng riêng cho lúc quay về N: để chậm hơn, cảm biến N bắt kịp.
// Tốc độ chuyển số cố định
int speedChay = 180;
int speedReturnN = 250;

// PWM khởi động riêng cho chuyển số.
// Sang F cần lực ban đầu lớn hơn; về N phải chạy chậm, không giật để tránh lố qua R.
const int START_PWM_SHIFT_F = 180;
const int START_PWM_SHIFT_R = 180;
const int START_PWM_RETURN_N = 250;

// PWM riêng cho lệnh R từ Jetson.
// Thấp hơn START_PWM_SHIFT_F để motor chuyển số chỉ đẩy cần lên F nhẹ.
const int START_PWM_AI_SHIFT_F = 160;
const int SPEED_AI_SHIFT_F = 160;

int pwmLai = 0;
int pwmChay = 0;
int targetLai = 0;
int targetChay = 0;

int dirLai = 0;     // -1 trái, 1 phải, 0 dừng
int dirChay = 0;    // 1 tiến/F, -1 lùi/R, 0 dừng

unsigned long lastRamp = 0;
const int rampStep = 10;
const int rampDelay = 30;

// ================== THỜI GIAN CHUYỂN SỐ ==================
// Thời gian gạt cần số khi Jetson gửi R.
// Giá trị này cố tình nhỏ để cần số chỉ lên TIẾN nhẹ,
// tránh đẩy F quá sâu làm xe còn trôi do quán tính dầu khi STOP.
const unsigned long AI_SHIFT_TO_F_MS = 250;

// Khi về N, nếu quá thời gian này mà chưa thấy N thì dừng bảo vệ.
const unsigned long RETURN_N_TIMEOUT = 3000;

// Không dùng debounce dài khi motor đang quay về N, vì cần số đi ngang N rất nhanh.
// Khi đã rời khỏi vùng N, chỉ cần cảm biến N báo HIGH là dừng motor ngay.
const unsigned long N_DEBOUNCE_MS = 20;

// Nếu quay về N bị ngược chiều thì đổi 2 dòng này.
const int FROM_F_TO_N_DIR = -1;
const int FROM_R_TO_N_DIR = 1;

// ================== TRẠNG THÁI CẦN SỐ ==================
enum GearState {
  GEAR_N,
  GEAR_F,
  GEAR_R,
  GEAR_UNKNOWN
};

enum GearMoveState {
  MOVE_IDLE,
  MOVE_TO_F_AI,     // Tiến nhẹ do Jetson gửi R
  MOVE_TO_N
};

GearState currentGear = GEAR_UNKNOWN;
GearMoveState gearMove = MOVE_IDLE;

unsigned long gearMoveStart = 0;
bool seenOutOfN = false;

// ================== KHAI BÁO HÀM ==================
void checkWiFi();
void updatePWM();
void handleCmd(String cmd);
void sendPage(WiFiClient client);

void quayThuanMotor2(int pwm);
void quayNguocMotor2(int pwm);
void quayTraiMotor1(int pwm);
void quayPhaiMotor1(int pwm);

void dungMotor1();
void dungMotor2();
void dungMotor2Ngay();
void dungTatCaNgay();

void startMotor2Dir(int dir, int targetSpeed, int startPwm);
void startAIMoveToF();
void startReturnToN();
void handleGearMove();

bool isNeutralRaw();
bool isNeutralStable();


void updateSteeringDisplayIfAllowed();

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(R_EN1, OUTPUT);
  pinMode(L_EN1, OUTPUT);
  pinMode(R_EN2, OUTPUT);
  pinMode(L_EN2, OUTPUT);

  digitalWrite(R_EN1, HIGH);
  digitalWrite(L_EN1, HIGH);
  digitalWrite(R_EN2, HIGH);
  digitalWrite(L_EN2, HIGH);

  pinMode(POT_PIN, INPUT);
  pinMode(GEAR_N_SW, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);

  ledcAttach(RPWM1, freq, resolution);
  ledcAttach(LPWM1, freq, resolution);
  ledcAttach(RPWM2, freq, resolution);
  ledcAttach(LPWM2, freq, resolution);

  dungTatCaNgay();

  // Xe khởi động ở N. Lấy N làm mốc ban đầu.
  if (isNeutralRaw()) {
    currentGear = GEAR_N;
    Serial.println("[GEAR] KHOI DONG TAI VI TRI N");
  } else {
    currentGear = GEAR_UNKNOWN;
    Serial.println("[GEAR] CAN SO KHONG O N - KIEM TRA LAI");
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  server.begin();

  Serial.println("ESP32 READY");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

// ================== LOOP ==================
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCmd(cmd);
  }

  checkWiFi();
  updatePWM();
  handleGearMove();
  updateSteeringDisplayIfAllowed();
}

// ================== MOTOR CONTROL ==================
void quayThuanMotor2(int pwm) {
  ledcWrite(LPWM2, 0);
  ledcWrite(RPWM2, pwm);
}

void quayNguocMotor2(int pwm) {
  ledcWrite(RPWM2, 0);
  ledcWrite(LPWM2, pwm);
}

void quayTraiMotor1(int pwm) {
  ledcWrite(RPWM1, 0);
  ledcWrite(LPWM1, pwm);
}

void quayPhaiMotor1(int pwm) {
  ledcWrite(LPWM1, 0);
  ledcWrite(RPWM1, pwm);
}

void dungMotor1() {
  ledcWrite(RPWM1, 0);
  ledcWrite(LPWM1, 0);
}

void dungMotor2() {
  ledcWrite(RPWM2, 0);
  ledcWrite(LPWM2, 0);
}

void dungMotor2Ngay() {
  pwmChay = 0;
  targetChay = 0;
  dirChay = 0;
  dungMotor2();
}

void dungTatCaNgay() {
  pwmLai = 0;
  pwmChay = 0;
  targetLai = 0;
  targetChay = 0;
  dirLai = 0;
  dirChay = 0;

  gearMove = MOVE_IDLE;
  seenOutOfN = false;

  dungMotor1();
  dungMotor2();
}

void startMotor2Dir(int dir, int targetSpeed, int startPwm) {
  dirChay = dir;
  pwmChay = constrain(startPwm, 0, 255);
  targetChay = constrain(targetSpeed, 0, 255);

  // Xuất PWM ngay ở vòng lệnh đầu để motor không bị trễ, đặc biệt khi nhận R từ Jetson.
  if (dirChay == 1) quayThuanMotor2(pwmChay);
  else if (dirChay == -1) quayNguocMotor2(pwmChay);
}

// ================== PWM MỀM ==================
void updatePWM() {
  if (millis() - lastRamp < rampDelay) return;
  lastRamp = millis();

  // Motor chuyển số
  if (pwmChay < targetChay) pwmChay += rampStep;
  else if (pwmChay > targetChay) pwmChay -= rampStep;
  pwmChay = constrain(pwmChay, 0, 255);

  if (pwmChay == 0 && targetChay == 0) {
    dirChay = 0;
    dungMotor2();
  } else {
    if (dirChay == 1) quayThuanMotor2(pwmChay);
    else if (dirChay == -1) quayNguocMotor2(pwmChay);
  }

  // Motor lái
  if (pwmLai < targetLai) pwmLai += rampStep;
  else if (pwmLai > targetLai) pwmLai -= rampStep;
  pwmLai = constrain(pwmLai, 0, 255);

  if (pwmLai == 0 && targetLai == 0) {
    dirLai = 0;
    dungMotor1();
  } else {
    if (dirLai == -1) quayTraiMotor1(pwmLai);
    else if (dirLai == 1) quayPhaiMotor1(pwmLai);
  }
}

// ================== CẢM BIẾN N ==================
bool isNeutralRaw() {
  return digitalRead(GEAR_N_SW) == HIGH;   // đo thực tế: N = 1, ngoài N = 0
}

bool isNeutralStable() {
  static bool lastRaw = false;
  static unsigned long lastChange = 0;

  bool raw = isNeutralRaw();

  if (raw != lastRaw) {
    lastRaw = raw;
    lastChange = millis();
  }

  return raw && (millis() - lastChange >= N_DEBOUNCE_MS);
}

// ================== CHUYỂN SỐ TỰ ĐỘNG ==================
void startAIMoveToF() {
  if (gearMove != MOVE_IDLE) {
    Serial.println("[GEAR] DANG CHUYEN SO - BO QUA RUN AI");
    return;
  }

  if (currentGear == GEAR_F) {
    Serial.println("[GEAR] DA O F - BO QUA RUN AI");
    return;
  }

  if (!isNeutralRaw() && currentGear == GEAR_UNKNOWN) {
    Serial.println("[GEAR] KHONG BIET VI TRI - KHONG TU CHAY AI");
    return;
  }

  gearMove = MOVE_TO_F_AI;
  gearMoveStart = millis();
  currentGear = GEAR_UNKNOWN;
  seenOutOfN = !isNeutralRaw();

  startMotor2Dir(1, SPEED_AI_SHIFT_F, START_PWM_AI_SHIFT_F);

  Serial.println("[GEAR] RUN AI: N -> F NHE");
}

void startReturnToN() {
  if (gearMove == MOVE_TO_N) {
    Serial.println("[GEAR] DANG VE N - BO QUA LENH MOI");
    return;
  }

  // Nếu đang ở N thật thì không cần quay nữa.
  if (currentGear == GEAR_N && isNeutralRaw()) {
    dungMotor2Ngay();
    gearMove = MOVE_IDLE;
    Serial.println("[GEAR] DA O N - BO QUA STOP");
    return;
  }

  GearState from = currentGear;

  // Nếu đang chuyển số mà bị STOP chen vào thì xác định hướng quay về N theo hướng đang chạy.
  if (gearMove == MOVE_TO_F_AI || dirChay == 1) {
    from = GEAR_F;
  }
  else if (dirChay == -1) {
    from = GEAR_R;
  }

  // Nếu vẫn còn nằm ngay vùng N thì chỉ cần dừng lại.
  if (isNeutralRaw() && from == GEAR_UNKNOWN) {
    dungMotor2Ngay();
    currentGear = GEAR_N;
    gearMove = MOVE_IDLE;
    Serial.println("[GEAR] DANG O N");
    return;
  }

  int returnDir = 0;

  if (from == GEAR_F) {
    returnDir = FROM_F_TO_N_DIR;
    Serial.println("[GEAR] STOP: F -> N");
  }
  else if (from == GEAR_R) {
    returnDir = FROM_R_TO_N_DIR;
    Serial.println("[GEAR] STOP: R -> N");
  }
  else {
    dungMotor2Ngay();
    currentGear = GEAR_UNKNOWN;
    gearMove = MOVE_IDLE;
    Serial.println("[GEAR] KHONG BIET VI TRI - DUNG BAO VE");
    return;
  }

  gearMove = MOVE_TO_N;
  gearMoveStart = millis();
  seenOutOfN = !isNeutralRaw();
  currentGear = GEAR_UNKNOWN;

  startMotor2Dir(returnDir, speedReturnN, START_PWM_RETURN_N);
}

void handleGearMove() {
  if (gearMove == MOVE_IDLE) return;

  unsigned long elapsed = millis() - gearMoveStart;

  // Jetson gửi R: tự gạt từ N sang F nhẹ theo thời gian ngắn.
  if (gearMove == MOVE_TO_F_AI) {
    if (elapsed >= AI_SHIFT_TO_F_MS) {
      dungMotor2Ngay();
      currentGear = GEAR_F;
      gearMove = MOVE_IDLE;
      Serial.println("[GEAR] DA SANG F NHE - DUNG MOTOR");
    }
    return;
  }

  // Quay về N bằng cảm biến N.
  if (gearMove == MOVE_TO_N) {
    if (!seenOutOfN && !isNeutralRaw()) {
      seenOutOfN = true;
      Serial.println("[GEAR] DA ROI KHOI VUNG N");
    }

    // Đã rời khỏi vùng N rồi mà cảm biến vừa báo N lại -> dừng ngay, không đợi 120 ms nữa.
    // Mục tiêu là tránh chạy lố từ F qua N sang R.
    if (seenOutOfN && isNeutralRaw()) {
      dungMotor2Ngay();
      currentGear = GEAR_N;
      gearMove = MOVE_IDLE;
          Serial.println("[GEAR] BAT GAP N - DUNG MOTOR NGAY");

      if (pendingRunAfterN && !aiStopLock && !emergencyLock) {
        pendingRunAfterN = false;
        Serial.println("[AI] VE N XONG - TU DONG RUN LAI NHE");
        startAIMoveToF();
      }

      return;
    }

    if (elapsed > RETURN_N_TIMEOUT) {
      dungMotor2Ngay();

      if (isNeutralRaw()) currentGear = GEAR_N;
      else currentGear = GEAR_UNKNOWN;

      gearMove = MOVE_IDLE;
          Serial.println("[GEAR] LOI: QUA THOI GIAN CHUA VE N");
    }
  }
}

// ================== HIỂN THỊ GÓC LÁI ==================
void updateSteeringDisplayIfAllowed() {
  if (!steerDisplayActive) return;

  int adc = analogRead(POT_PIN);
  float v = adc * (3.3 / 4095.0);

  char next = 'S';

  if (v < STRAIGHT_MIN) next = 'L';
  else if (v > STRAIGHT_MAX) next = 'R';

  if (next != lastSteer) {
    lastSteer = next;

    if (next == 'L') Serial.println("[STEER] XE RE TRAI");
    else if (next == 'R') Serial.println("[STEER] XE RE PHAI");
    else Serial.println("[STEER] XE DI THANG");
  }
}

// ================== COMMAND ==================
void handleCmd(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  Serial.print("[RX] ");
  Serial.println(cmd);

  // START AI: mở khóa lệnh R/RUN từ Jetson và cho phép AI điều khiển tiến/dừng lại.
  if (cmd == "AISTART") {
    if (emergencyLock) {
      Serial.println("[AI] DANG DUNG KHAN - RESET HE THONG TRUOC");
      return;
    }

    aiStopLock = false;
    Serial.println("[AI] START AI - MO KHOA LENH R");
    return;
  }

  // STOP AI: khóa R/RUN ngay lập tức, đưa cần số về N, sau đó web sẽ gọi /stop trên Jetson.
  if (cmd == "AISTOP") {
    pendingRunAfterN = false;
    aiStopLock = true;

    targetLai = 0;
    steerDisplayActive = false;

    Serial.println("[AI] STOP AI - KHOA LENH R VA VE N");
    startReturnToN();
    return;
  }

  if (cmd == "CLEAR" || cmd == "RESET" || cmd == "UNLOCK") {
    Serial.println("[EMERGENCY] KHONG MO KHOA BANG PHAN MEM - RESET HE THONG TRUOC");
    return;
}

  // Khi đang dừng khẩn cấp thì chỉ nhận X hoặc lệnh mở khóa.
  // Mọi lệnh RUN/tiến/lùi/lái từ Jetson/web đều bị bỏ qua để tránh xe tự chạy lại.
  if (emergencyLock && cmd != "X") {
    Serial.print("[EMERGENCY] DANG KHOA CUNG - BO QUA LENH: ");
    Serial.println(cmd);
    return;
  }

  // R từ Jetson = RUN. Chỉ cho chạy khi không bị khóa bởi N/B/STOP AI.
  if (cmd == "R") {
    if (aiStopLock) {
     Serial.println("[AI] DANG KHOA RUN - BO QUA LENH R TU JETSON");
     return;
  }

  if (gearMove == MOVE_TO_N) {
    pendingRunAfterN = true;
    Serial.println("[AI] DANG VE N - LUU LENH RUN");
    return;
  }

  startAIMoveToF();
}

  // F từ web = người dùng muốn cho xe chạy tiến lại.
  // Khi bấm F thì mở khóa R, để nếu AI vẫn đang chạy thì Jetson được quyền gửi RUN tiếp.
  else if (cmd == "F") {
    if (gearMove == MOVE_TO_N) {
      Serial.println("[CMD] DANG VE N - BO QUA F");
      return;
    }

    aiStopLock = false;
    gearMove = MOVE_IDLE;
    startMotor2Dir(1, speedChay, START_PWM_SHIFT_F);
    currentGear = GEAR_F;

    Serial.println("[CMD] TIEN TAY - MO KHOA R - GHI NHO F");
  }

  // B từ web = điều khiển lùi bằng tay.
  // Khi lùi bằng tay, phải khóa R từ Jetson để AI không kéo cần số lên F gây xung đột.
  else if (cmd == "B") {
    if (gearMove == MOVE_TO_N) {
      Serial.println("[CMD] DANG VE N - BO QUA B");
      return;
    }

    aiStopLock = true;
    gearMove = MOVE_IDLE;
    startMotor2Dir(-1, speedChay, START_PWM_SHIFT_R);
    currentGear = GEAR_R;

    Serial.println("[CMD] LUI TAY - KHOA LENH R TU JETSON - GHI NHO R");
  }

  // D = dừng mềm/manual release. Không dùng cho STOP an toàn.
  else if (cmd == "D") {
    if (gearMove == MOVE_TO_N) {
      Serial.println("[CMD] DANG VE N - BO QUA D");
      return;
    }

    dungMotor2Ngay();
    gearMove = MOVE_IDLE;

    Serial.println("[CMD] DUNG CHAY");
  }

  // S từ Jetson = STOP do AI phát hiện nguy hiểm.
  // Lệnh này chỉ đưa xe về N, KHÔNG khóa aiStopLock để khi hết nguy hiểm Jetson còn có thể gửi R chạy lại.
  else if (cmd == "S") {
    pendingRunAfterN = false;

    targetLai = 0;
    steerDisplayActive = false;

    startReturnToN();

    Serial.println("[CMD] STOP TU JETSON - VE N");
  }

  // N/STOP từ web = người dùng muốn dừng giữ ở N nhưng AI vẫn có thể đang nhận diện.
  // Phải khóa R từ Jetson, nếu không AI còn chạy sẽ kéo cần số lên F lại.
  else if (cmd == "N" || cmd == "STOP") {
    pendingRunAfterN = false;
    aiStopLock = true;

    targetLai = 0;
    steerDisplayActive = false;

    startReturnToN();

    Serial.println("[CMD] VE N TAY - KHOA LENH R");
  }

  // Dừng khẩn cấp: khóa RUN từ Jetson, dừng lái ngay và bắt buộc đưa cần số về N.
  else if (cmd == "X") {
    pendingRunAfterN = false;
    emergencyLock = true;
    aiStopLock = true;

    // Dừng motor lái ngay.
    pwmLai = 0;
    targetLai = 0;
    dirLai = 0;
    steerDisplayActive = false;
    dungMotor1();

    Serial.println("[EMERGENCY] DUNG KHAN - KHOA LENH RUN");

    // Nếu đã ở N thật sự thì chỉ ngắt motor chuyển số.
    if (currentGear == GEAR_N && isNeutralRaw()) {
      dungMotor2Ngay();
      gearMove = MOVE_IDLE;
      currentGear = GEAR_N;
      Serial.println("[EMERGENCY] DANG O N - DA DUNG");
    }
    // Nếu đang ở F/R hoặc đang chuyển số thì quay về N.
    else {
      startReturnToN();
      Serial.println("[EMERGENCY] BAT BUOC VE N");
    }
  }

  else if (cmd == "L") {
    dirLai = -1;
    pwmLai = START_PWM_LAI;
    targetLai = speedLai;
    steerDisplayActive = true;

    quayTraiMotor1(pwmLai);

    Serial.println("[CMD] TRAI - KHOI DONG NHANH");
  }

  else if (cmd == "P") {
    dirLai = 1;
    pwmLai = START_PWM_LAI;
    targetLai = speedLai;
    steerDisplayActive = true;

    quayPhaiMotor1(pwmLai);

    Serial.println("[CMD] PHAI - KHOI DONG NHANH");
  }

  else if (cmd == "C") {
    targetLai = 0;
    steerDisplayActive = true;

    Serial.println("[CMD] THANG LAI");
  }

}

// ================== WEB ==================
void sendPage(WiFiClient client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html; charset=UTF-8");
  client.println();

  client.println("<!DOCTYPE html>");
  client.println("<html>");
  client.println("<head>");
  client.println("<meta charset='UTF-8'>");
  client.println("<meta name='viewport' content='width=device-width, initial-scale=1'>");

  client.println("<style>");
  client.println("body{margin:0;background:#f2f4f7;font-family:Arial;text-align:center;}");
  client.println(".card{max-width:430px;margin:20px auto;background:white;padding:20px;border-radius:20px;box-shadow:0 4px 15px #0003;}");
  client.println("h2{color:#ff8c00;}");
  client.println("button{width:120px;height:60px;margin:6px;border:0;border-radius:15px;color:white;font-size:21px;font-weight:bold;touch-action:none;user-select:none;}");
  client.println(".run{background:#28a745;}");
  client.println(".back{background:#ff8c00;}");
  client.println(".stop{background:#dc3545;}");
  client.println(".steer{background:#007bff;}");
  client.println(".center{background:#6c757d;}");
  client.println(".ai{background:#0096c7;width:150px;height:55px;font-size:18px;}");
  client.println("</style>");

  client.println("</head>");
  client.println("<body>");
  client.println("<div class='card'>");

  client.println("<h2>HE THONG DIEU KHIEN XE LU</h2>");
  client.println("<img id='roi' style='width:100%;border-radius:15px;' onerror='setTimeout(loadStream,1000)'>");
  client.println("<br><br>");

  client.println("<button class='ai' onclick='startAI()'>START AI</button>");
  client.println("<button class='stop' onclick='stopAI()'>STOP AI</button>");
  client.println("<br><br>");

  client.println("<button class='run' onpointerdown=\"press(event,'F')\" onpointerup=\"release(event,'D')\" onpointercancel=\"release(event,'D')\">TIEN</button><br>");

  client.println("<button class='steer' onpointerdown=\"press(event,'L')\" onpointerup=\"release(event,'C')\" onpointercancel=\"release(event,'C')\">TRAI</button>");
  client.println("<button class='center' onclick=\"cmd('N')\">VE N</button>");
  client.println("<button class='steer' onpointerdown=\"press(event,'P')\" onpointerup=\"release(event,'C')\" onpointercancel=\"release(event,'C')\">PHAI</button><br>");

  client.println("<button class='center' onclick=\"cmd('C')\">THANG LAI</button>");
  client.println("<button class='back' onpointerdown=\"press(event,'B')\" onpointerup=\"release(event,'D')\" onpointercancel=\"release(event,'D')\">LUI</button>");
  client.println("<button class='stop' onclick=\"emergencyStop()\">DUNG KHAN</button>");

  client.println("</div>");

  client.println("<script>");
  client.println("function loadStream(){document.getElementById('roi').src='http://192.168.4.4:5000/video?t='+Date.now();}");
  client.println("function clearStream(){document.getElementById('roi').removeAttribute('src');}");
  client.println("function cmd(c){return fetch('/'+c).then(r=>r.text()).catch(e=>{console.log(e);return 'ERR';});}");
  client.println("function press(e,c){try{e.currentTarget.setPointerCapture(e.pointerId);}catch(err){} cmd(c);}");
  client.println("function release(e,c){try{e.currentTarget.releasePointerCapture(e.pointerId);}catch(err){} cmd(c);}");
  client.println("function startAI(){cmd('AISTART').then(t=>{if(t.indexOf('LOCKED')>=0){alert('DANG DUNG KHAN - RESET HE THONG TRUOC');return;}setTimeout(()=>{fetch('http://192.168.4.4:8080/start',{mode:'no-cors'}).then(()=>{setTimeout(loadStream,3000);}).catch(e=>console.log(e));},300);});}");
  client.println("function stopAI(){cmd('AISTOP');setTimeout(()=>{fetch('http://192.168.4.4:8080/stop',{mode:'no-cors'}).then(()=>{clearStream();}).catch(e=>console.log(e));},300);}");
  client.println("function emergencyStop(){cmd('X');setTimeout(()=>{fetch('http://192.168.4.4:8080/stop',{mode:'no-cors'}).catch(e=>console.log(e));clearStream();alert('DA DUNG KHAN - AI DA TAT - RESET HE THONG DE CHAY LAI');},300);}");
  client.println("document.addEventListener('contextmenu',e=>e.preventDefault());");
  client.println("window.addEventListener('load',()=>{setTimeout(loadStream,500);});");
  client.println("</script>");

  client.println("</body>");
  client.println("</html>");
}

// ================== WIFI REQUEST ==================
void checkWiFi() {
  WiFiClient client = server.available();
  if (!client) return;

  String req = client.readStringUntil('\r');
  client.flush();
  bool isCommand = true;
  String reply = "OK";

  if (req.indexOf("GET /AISTART") >= 0) {
    if (emergencyLock) reply = "LOCKED";
    handleCmd("AISTART");
  }
  else if (req.indexOf("GET /AISTOP") >= 0) handleCmd("AISTOP");
  else if (req.indexOf("GET /STOP") >= 0) handleCmd("STOP");
  else if (req.indexOf("GET /X") >= 0) handleCmd("X");
  else if (req.indexOf("GET /N") >= 0) handleCmd("N");

  else if (req.indexOf("GET /F") >= 0) handleCmd("F");
  else if (req.indexOf("GET /B") >= 0) handleCmd("B");
  else if (req.indexOf("GET /D") >= 0) handleCmd("D");
  else if (req.indexOf("GET /S ") >= 0) handleCmd("S");
  else if (req.indexOf("GET /L") >= 0) handleCmd("L");
  else if (req.indexOf("GET /P") >= 0) handleCmd("P");
  else if (req.indexOf("GET /C") >= 0) handleCmd("C");
  else {
    isCommand = false;
  }

  if (isCommand) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println(reply);
  } else {
    sendPage(client);
  }
  client.stop();
}
