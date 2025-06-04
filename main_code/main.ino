#include <Arduino.h>
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

// Định nghĩa chân L298N
#define IN1 26  // Động cơ trái
#define IN2 27
#define IN3 14  // Động cơ phải
#define IN4 12
#define ENA 25  // PWM tốc độ trái
#define ENB 33  // PWM tốc độ phải

// Định nghĩa chân cảm biến TCRT5000 (AO)
#define TCRT_LEFT_AO 34   // AO của cảm biến trái (ADC1_6)
#define TCRT_RIGHT_AO 35  // AO của cảm biến phải (ADC1_7)

// Định nghĩa chân cảm biến HY-SRF05
#define TRIG_PIN 18  // Chân TRIG
#define ECHO_PIN 19  // Chân ECHO

// Biến tốc độ (0-255)
int speed = 255;
// Ngưỡng TCRT5000 để phát hiện vạch trắng
const int TCRT_THRESHOLD = 200; // Sửa lỗi từ TCPT_THRESHOLD
// Khoảng cách tối đa để phát hiện đối thủ (cm)
const float MAX_DISTANCE = 80.0;
// Thời gian lùi lại khi phát hiện vạch trắng (ms)
const unsigned long BACKWARD_DURATION = 500;
// Khoảng thời gian đọc TCRT5000 và in giá trị (ms)
const unsigned long TCRT_READ_INTERVAL = 10;
// Số lần lấy mẫu TCRT5000 trong 10ms
const int TCRT_SAMPLES = 5;
const int TCRT_SAMPLE_DELAY = 2; // 2ms mỗi mẫu, tổng 10ms cho 5 mẫu
// Khoảng thời gian đọc HY-SRF05 khi nhấn Triangle (ms)
const unsigned long SENSOR_READ_INTERVAL = 50; // Đọc HY-SRF05 mỗi 50ms để giảm trễ

// Biến trạng thái
unsigned long lastTCRTReadTime = 0; // Thời gian đọc TCRT5000 cuối cùng
unsigned long lastSensorReadTime = 0; // Thời gian đọc HY-SRF05 cuối cùng
int tcrtLeftValue = 0; // Giá trị TCRT5000 trái
int tcrtRightValue = 0; // Giá trị TCRT5000 phải
float lastDistance = 999.0; // Giá trị HY-SRF05 gần nhất
bool isReadingSensors = false; // Trạng thái đọc/in giá trị cảm biến
bool isAutoMode = false; // Trạng thái chế độ tự động
unsigned long backwardStartTime = 0; // Thời gian bắt đầu lùi
bool isBacking = false; // Trạng thái đang lùi

void setup() {
  // Thiết lập chân L298N
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Thiết lập chân cảm biến TCRT5000 (AO)
  pinMode(TCRT_LEFT_AO, INPUT);
  pinMode(TCRT_RIGHT_AO, INPUT);

  // Thiết lập chân cảm biến HY-SRF05
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(115200);
  Serial.println("ESP32 Dabble Gamepad Car with TCRT5000 and HY-SRF05 Starting...");

  Dabble.begin("MyEsp32"); // Tênmem Bluetooth
  Serial.println("Bluetooth Started! Connect to MyEsp32 via Dabble");
}

void driveMotor(int leftSpeed, bool leftForward, int rightSpeed, bool rightForward) {
  digitalWrite(IN1, leftForward ? HIGH : LOW);
  digitalWrite(IN2, leftForward ? LOW : HIGH);
  analogWrite(ENA, leftSpeed);
  digitalWrite(IN3, rightForward ? HIGH : LOW);
  digitalWrite(IN4, rightForward ? LOW : HIGH);
  analogWrite(ENB, rightSpeed);
}

void forward() {
  driveMotor(speed, true, speed, true);
  if (!isReadingSensors) Serial.println("Xe tiến");
}

void backward() {
  driveMotor(speed, false, speed, false);
  if (!isReadingSensors) Serial.println("Xe lùi");
}

void turnLeft() {
  driveMotor(speed, true, speed, false); // Đảo logic: trái xuôi, phải ngược
  if (!isReadingSensors) Serial.println("Rẽ trái");
}

void turnRight() {
  driveMotor(speed, false, speed, true); // Đảo logic: trái ngược, phải xuôi
  if (!isReadingSensors) Serial.println("Rẽ phải");
}

void stop() {
  driveMotor(0, true, 0, true);
  // Không in "Xe dừng" để Serial Monitor thông thoáng
} 

// Hàm đo khoảng cách bằng HY-SRF05 (trả về cm)
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout 30ms (~5m)
  if (duration == 0) return 999.0; // Không nhận sóng
  return duration * 0.034 / 2; // Tính khoảng cách (cm)
}

// Hàm đọc giá trị TCRT5000 với lấy mẫu trung bình trong 10ms
int readTCRT5000(int pin) {
  long sum = 0;
  for (int i = 0; i < TCRT_SAMPLES; i++) {
    sum += analogRead(pin);
    delay(TCRT_SAMPLE_DELAY); // Đợi 2ms giữa các mẫu
  }
  return sum / TCRT_SAMPLES; // Trả về giá trị trung bình
}

void loop() {
  Dabble.processInput();

  // Đọc TCRT5000 mỗi 10ms
  unsigned long currentTime = millis();
  if (currentTime - lastTCRTReadTime >= TCRT_READ_INTERVAL) {
    tcrtLeftValue = readTCRT5000(TCRT_LEFT_AO);
    tcrtRightValue = readTCRT5000(TCRT_RIGHT_AO);
    lastTCRTReadTime = currentTime;
  }

  // Kiểm tra chế độ lùi lại
  if (isBacking) {
    if (millis() - backwardStartTime < BACKWARD_DURATION) {
      backward();
    } else {
      isBacking = false; // Kết thúc lùi, tiếp tục chế độ tự động
    }
    return; // Không xử lý các lệnh khác khi đang lùi
  }

  // Xử lý các nút điều khiển
  if (!isReadingSensors) {
    Serial.print("KeyPressed: ");
    if (GamePad.isUpPressed()) {
      isAutoMode = false; // Tắt chế độ tự động
      forward();
      Serial.print("Up");
    }
    if (GamePad.isDownPressed()) {
      isAutoMode = false; // Tắt chế độ tự động
      backward();
      Serial.print("Down");
    }
    if (GamePad.isLeftPressed()) {
      isAutoMode = false; // Tắt chế độ tự động
      turnLeft();
      Serial.print("Left");
    }
    if (GamePad.isRightPressed()) {
      isAutoMode = false; // Tắt chế độ tự động
      turnRight();
      Serial.print("Right");
    }
    if (GamePad.isSelectPressed()) {
      isAutoMode = false; // Tắt chế độ tự động
      stop();
      Serial.print("Select");
    }
    // In dữ liệu GamePad qua Serial
    Serial.print('\t');
    int a = GamePad.getAngle();
    Serial.print("Angle: ");
    Serial.print(a);
    Serial.print('\t');
    int b = GamePad.getRadius();
    Serial.print("Radius: ");
    Serial.print(b);
    Serial.print('\t');
    float c = GamePad.getXaxisData();
    Serial.print("x_axis: ");
    Serial.print(c);
    Serial.print('\t');
    float d = GamePad.getYaxisData();
    Serial.print("y_axis: ");
    Serial.print(d);
    Serial.println();
    Serial.println();
  }

  // Xử lý nút Triangle: In giá trị TCRT5000 và HY-SRF05 qua Serial mỗi 10ms
  static unsigned long lastTrianglePrintTime = 0;
  if (GamePad.isTrianglePressed()) {
    stop();
    isAutoMode = false; // Tắt chế độ tự động
    isReadingSensors = true;
    if (currentTime - lastTrianglePrintTime >= TCRT_READ_INTERVAL) {
      // Đọc HY-SRF05 mỗi 50ms để giảm trễ
      if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
        lastDistance = getDistance();
        lastSensorReadTime = currentTime;
      }
      Serial.print("TCRT Left: ");
      Serial.print(tcrtLeftValue);
      Serial.print(" | TCRT Right: ");
      Serial.print(tcrtRightValue);
      Serial.print(" | HY-SRF05 Distance: ");
      Serial.print(lastDistance);
      Serial.println(" cm");
      lastTrianglePrintTime = currentTime;
    }
  }

  // Xử lý nút Circle: Thoát chế độ tự động và dừng xe
  if (GamePad.isCirclePressed()) {
    isAutoMode = false; // Thoát chế độ tự động
    stop();
    if (!isReadingSensors) Serial.println("Circle - Chế độ tự động dừng");
  }

  // Xử lý nút Cross: Dừng in giá trị cảm biến
  if (GamePad.isCrossPressed()) {
    isReadingSensors = false;
    Serial.println("Cross - Dừng đọc cảm biến");
  }

  // Xử lý nút Start: Kích hoạt chế độ tự động và in giá trị cảm biến
  if (GamePad.isStartPressed()) {
    isAutoMode = true;
    // In giá trị cảm biến
    float distance = getDistance();
    Serial.print("Start - TCRT Left: ");
    Serial.print(tcrtLeftValue);
    Serial.print(" | TCRT Right: ");
    Serial.print(tcrtRightValue);
    Serial.print(" | HY-SRF05 Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  // Nút Square để debug
  if (GamePad.isSquarePressed() && !isReadingSensors) {
    Serial.println("Square");
  }
 
  // Chế độ tự động
  if (isAutoMode) {
    // Kiểm tra vạch trắng (TCRT5000)
    if (tcrtLeftValue < TCRT_THRESHOLD || tcrtRightValue < TCRT_THRESHOLD) {
      isBacking = true;
      backwardStartTime = millis();
      backward();
      Serial.println("Phát hiện vạch trắng, lùi lại");
      return;
    }
    // Kiểm tra khoảng cách (HY-SRF05)
    float distance = getDistance();
    if (distance <= MAX_DISTANCE && distance > 0) {
      forward(); // Phát hiện đối thủ trong 100cm, lao tới
      Serial.println("Phát hiện đối thủ, lao tới");
    } else {
      turnRight(); // Không thấy đối thủ, xoay trái
      Serial.println("Xoay trái tìm đối thủ");
    }
  }

  // Dừng xe nếu không nhấn nút và không ở chế độ tự động
  if (!GamePad.isUpPressed() && !GamePad.isDownPressed() &&
      !GamePad.isLeftPressed() && !GamePad.isRightPressed() &&
      !GamePad.isSelectPressed() && !GamePad.isTrianglePressed() &&
      !GamePad.isCirclePressed() && !isAutoMode) {
    stop();
  }
}