// 필요한 라이브러리 포함
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiSSLClient.h>
#include <ArduinoHttpClient.h>
#include <ArduinoMqttClient.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include "secrets.h"  // Wi-Fi 및 API 키 등을 관리하는 secrets.h 파일을 포함합니다.

// WiFi 설정 (secrets.h에서 관리)
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// MQTT 브로커 설정
const char* mqttServer = "test.mosquitto.org";
const int mqttPort = 1883;
const char* mqttClientID = "berryblind";
const char* controlTopic = "blinds/control";
const char* settingsTopic = "blinds/settings";
const char* reservationUpdateTopic = "blinds/reservation/update";

// Firestore 설정
const char* projectID = FIREBASE_PROJECT_ID;

// WiFi 및 클라이언트 객체 생성
WiFiClient mqttWifiClient;
WiFiSSLClient httpsWifiClient;
MqttClient mqttClient(mqttWifiClient);

// 서보 모터 설정
Servo continuousServo;
const int servoPin = 9;

// 핀 설정
const int trigPin = 10;
const int echoPin = 11;
const int ldrPin = A0;

// 설정 변수 (Firestore에서 가져올 예정)
struct Config {
  String lowerOperation;
  String upperOperation;
  float motorLowerLimit;
  float motorUpperLimit;
  int sensorLowerLimit;  // Firestore에서 1~100 범위로 관리
  int sensorUpperLimit;  // Firestore에서 1~100 범위로 관리
  bool autoMode;
  
  // 스케일된 한계값 (원본 값)
  int sensorLowerLimitRaw;
  int sensorUpperLimitRaw;
};
Config config;

// 모터 상태
enum MotorState { STOPPED, MOVING_UP, MOVING_DOWN };
MotorState motorState = STOPPED;

// 작동 명령 및 모드 저장 변수
String lastOperationCommand = "";
String lastOperationMode = "";

// 초기 시작 거리 변수
float initialDistance = -1.0; // -1.0은 초기화되지 않음을 의미

// NTP 클라이언트 설정 (한국 시간대 적용)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 9 * 3600, 60000);

// 주기적인 센서 데이터 업로드
unsigned long lastPeriodicUploadTime = 0;
const unsigned long periodicUploadInterval = 900000; // 15분

// 예약 정보 구조체
struct Reservation {
  bool active;
  String operation;
  int hour;
  int minute;
  int second;
};
#define MAX_RESERVATIONS 10
Reservation reservations[MAX_RESERVATIONS];
int reservationCount = 0;

// 비동기 데이터 업로드를 위한 큐
struct UploadData {
  String operationMode;
  String operationCommand;
  float movementDistance;
  float distance;
  int ldrValue;      // 스케일된 값 (1~100)
  String state;      // 블라인드 상태 ("bottom", "middle", "top")
};
#define UPLOAD_QUEUE_SIZE 10
UploadData uploadQueue[UPLOAD_QUEUE_SIZE];
int uploadQueueHead = 0;
int uploadQueueTail = 0;

// 네트워크 작업 타입 정의
enum NetworkTaskType { UPLOAD_DATA, CHECK_RESERVATION, UPDATE_STATE };

// 네트워크 작업 구조체
struct NetworkTask {
  NetworkTaskType type;
  UploadData data;
  String state; // UPDATE_STATE 작업을 위한 상태 값
};
#define NETWORK_TASK_QUEUE_SIZE 5
int networkTaskQueueHead = 0;
int networkTaskQueueTail = 0;
NetworkTask networkTaskQueue[NETWORK_TASK_QUEUE_SIZE];

// 마지막 예약 확인 시간
time_t lastReservationCheck = 0;

// 전역 변수 추가: 블라인드 작동 전 LDR 값 저장
int preMovementLdrValue = 0;

// 함수 프로토타입 선언
void initializeHardware();
void initializeNetwork();
void fetchSettings();
void fetchAutoMode();
void connectMQTT();
void setup_wifi();
void loopTasks();
void handleMQTT();
void processSerialCommand();
void updateMotor();
void handleAutoMode();
float getDistance();
int getLDRRawValue();
int getScaledLDRValue(); // LDR 값을 1~100으로 스케일링
String calculateBlindState(float distance);
void moveBlindUp();
void moveBlindDown();
void moveBlindToPosition(float targetDistance);
void stopMotor();
void moveMotor(int angle);
void enqueueUploadData(String operationMode, String operationCommand, float movementDistance, float distance, int ldrValue, String state);
void handleUploadQueue();
void enqueueNetworkTask(NetworkTaskType type, UploadData data = UploadData(), String state = "");
void handleNetworkTasks();
void uploadSensorData(UploadData data);
void updateBlindLocationState(String state);
void handleControlMessage(String cmd);
void handleSettingsMessage(String json);
void checkAndExecuteReservation();
void fetchReservationsFromFirestore();
void mqttCallback(int messageSize);
void fetchSettingsFromFirestore();
void fetchAutoModeFromFirestore();
void parseTimeFromTimestamp(String timestampStr, int &hour, int &minute, int &second);
void assignDefaultSettings(); // 기본 설정 함수 재추가
void updateAutoModeInFirestore(bool autoMode); // 함수 프로토타입 추가

// secrets.h에서 가져온 API 키
extern const char* apiKey;

void setup() {
  Serial.begin(115200);
  initializeHardware();
  setup_wifi();
  initializeNetwork();
  
  // Firestore에서 설정을 가져오기 전에 WiFi가 연결되어 있어야 합니다.
  if (WiFi.status() == WL_CONNECTED) {
    fetchSettings();
    fetchAutoMode();
    fetchReservationsFromFirestore();  // 예약 정보 가져오기 (초기 시작 시)
  } else {
    Serial.println("WiFi가 연결되지 않았습니다. 기본 설정을 사용합니다.");
    assignDefaultSettings(); // WiFi가 연결되지 않았을 때 기본 설정 적용
  }
  
  timeClient.begin();
}

void loop() {
  loopTasks();
}

void initializeHardware() {
  continuousServo.attach(servoPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  delay(1000); // 센서 안정화
  initialDistance = getDistance();
  if (initialDistance != -1.0) {
    Serial.print("초기 시작 거리: ");
    Serial.println(initialDistance);
  } else {
    Serial.println("초기 거리 설정 실패.");
  }

  stopMotor();
  Serial.println("시리얼 명령어: 'U' - 올리기, 'D' - 내리기, 'S' - 정지, 'AF' - 자동모드 켜기, 'AN' - 자동모드 끄기");
}

void initializeNetwork() {
  mqttClient.setId(mqttClientID);
  mqttClient.onMessage(mqttCallback);
  connectMQTT();
}

void fetchSettings() {
  fetchSettingsFromFirestore();
}

void fetchAutoMode() {
  fetchAutoModeFromFirestore();
}

void loopTasks() {
  updateMotor();
  handleMQTT();
  timeClient.update();
  setTime(timeClient.getEpochTime());

  if (config.autoMode) {
    handleAutoMode();
  }

  processSerialCommand();

  unsigned long currentMillis = millis();
  if (currentMillis - lastPeriodicUploadTime >= periodicUploadInterval) {
    lastPeriodicUploadTime = currentMillis;
    int ldrValueScaled = getScaledLDRValue(); // 스케일된 값 사용
    float distance = getDistance();
    String blindState = calculateBlindState(distance);
    enqueueUploadData("Periodic", "NONE", 0.0, distance, ldrValueScaled, blindState);
    enqueueNetworkTask(UPDATE_STATE, UploadData(), blindState);
  }

  // 예약 확인 주기 설정 (예: 1초)
  if (now() - lastReservationCheck >= 1) {
    lastReservationCheck = now();
    checkAndExecuteReservation();
  }

  handleUploadQueue();
  handleNetworkTasks();
}

void handleMQTT() {
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.poll();
}

void setup_wifi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 30) {
    delay(1000);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi 연결 성공");
    Serial.print("IP 주소: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi 연결 실패");
    Serial.println("기본 설정을 사용합니다.");
    assignDefaultSettings(); // WiFi 연결 실패 시 기본 설정 적용
  }
}

void connectMQTT() {
  Serial.print("MQTT 서버에 연결 중...");
  while (!mqttClient.connect(mqttServer, mqttPort)) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nMQTT 연결 성공");
  mqttClient.subscribe(controlTopic);
  mqttClient.subscribe(settingsTopic);
  mqttClient.subscribe(reservationUpdateTopic);
}

void processSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "U") {
      lastOperationMode = "수동모드";
      lastOperationCommand = "올림";
      moveBlindUp();
    } else if (command == "D") {
      lastOperationMode = "수동모드";
      lastOperationCommand = "내림";
      moveBlindDown();
    } else if (command == "S") {
      stopMotor();
    } else if (command == "AF") {
      config.autoMode = true;
      Serial.println("자동 모드 활성화");
      enqueueNetworkTask(UPDATE_STATE, UploadData(), "Auto Mode Enabled");
      updateAutoModeInFirestore(true); // 함수 호출
    } else if (command == "AN") {
      config.autoMode = false;
      stopMotor();
      Serial.println("자동 모드 비활성화");
      enqueueNetworkTask(UPDATE_STATE, UploadData(), "Auto Mode Disabled");
      updateAutoModeInFirestore(false); // 함수 호출
    } else {
      Serial.println("잘못된 명령어입니다.");
    }
  }
}

void updateMotor() {
  float distance = getDistance();
  if (distance == -1.0) {
    return;
  }

  if ((motorState == MOVING_UP && distance >= config.motorUpperLimit) ||
      (motorState == MOVING_DOWN && distance <= config.motorLowerLimit)) {
    stopMotor();
  }
}

void handleAutoMode() {
  static unsigned long lastCheckTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastCheckTime >= 1000) { // 1초마다 체크
    lastCheckTime = currentTime;
    float distance = getDistance();
    if (distance == -1.0) return;

    int ldrValueRaw = getLDRRawValue();         // 원본 값 사용

    // Firestore에서 가져온 설정을 원본 스케일로 변환
    int sensorLowerLimitRaw = config.sensorLowerLimitRaw;
    int sensorUpperLimitRaw = config.sensorUpperLimitRaw;

    if (ldrValueRaw <= sensorLowerLimitRaw && distance < config.motorUpperLimit) {
      if (motorState != MOVING_UP) {
        lastOperationMode = "Auto";
        lastOperationCommand = config.upperOperation;
        preMovementLdrValue = getScaledLDRValue(); // 블라인드 작동 전 LDR 값 저장
        moveBlindUp();
      }
    } else if (ldrValueRaw >= sensorUpperLimitRaw && distance > config.motorLowerLimit) {
      if (motorState != MOVING_DOWN) {
        lastOperationMode = "Auto";
        lastOperationCommand = config.lowerOperation;
        preMovementLdrValue = getScaledLDRValue(); // 블라인드 작동 전 LDR 값 저장
        moveBlindDown();
      }
    } else {
      stopMotor();
    }
  }
}

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 5000);
  if (duration == 0) {
    Serial.println("센서 타임아웃");
    return -1.0;
  }
  float distance = duration * 0.034 / 2.0;

  Serial.print("측정된 거리: ");
  Serial.println(distance);

  // 2.2cm ~ 2.5cm 사이의 값 무시
  if (distance >= 2.2 && distance <= 2.5) {
    Serial.println("잘못된 거리 측정값 (2.2cm ~ 2.5cm). 무시합니다.");
    return -1.0;
  }

  return distance;
}

int getLDRRawValue() {
  int ldrValue = analogRead(ldrPin);
  Serial.print("LDR 원본 값: ");
  Serial.println(ldrValue);
  return ldrValue;
}

int getScaledLDRValue() {
  int rawValue = analogRead(ldrPin);
  int scaledValue = map(constrain(rawValue, 1, 1023), 1, 1023, 1, 100);
  Serial.print("스케일된 LDR 값: ");
  Serial.println(scaledValue);
  return scaledValue;
}

String calculateBlindState(float distance) {
  if (config.motorLowerLimit >= config.motorUpperLimit) {
    Serial.println("motorLowerLimit가 motorUpperLimit보다 크거나 같습니다.");
    return "unknown";
  }

  float range = config.motorUpperLimit - config.motorLowerLimit;
  float lowerThird = config.motorLowerLimit + range / 3.0;
  float upperThird = config.motorLowerLimit + 2 * (range / 3.0);

  if (distance <= lowerThird) {
    return "bottom";
  } else if (distance <= upperThird) {
    return "middle";
  } else {
    return "top";
  }
}

void moveBlindUp() {
  if (motorState != MOVING_UP) {
    motorState = MOVING_UP;
    preMovementLdrValue = getScaledLDRValue(); // 블라인드 작동 전 LDR 값 저장
    moveMotor(0);
    Serial.println("블라인드 올리기...");
  }
}

void moveBlindDown() {
  if (motorState != MOVING_DOWN) {
    motorState = MOVING_DOWN;
    preMovementLdrValue = getScaledLDRValue(); // 블라인드 작동 전 LDR 값 저장
    moveMotor(180);
    Serial.println("블라인드 내리기...");
  }
}

void moveBlindToPosition(float targetDistance) {
  float currentDistance = getDistance();
  if (currentDistance == -1.0) return;

  if (currentDistance < targetDistance) {
    lastOperationMode = "Auto";
    lastOperationCommand = config.upperOperation;
    preMovementLdrValue = getScaledLDRValue(); // 블라인드 작동 전 LDR 값 저장
    moveBlindUp();
  } else if (currentDistance > targetDistance) {
    lastOperationMode = "Auto";
    lastOperationCommand = config.lowerOperation;
    preMovementLdrValue = getScaledLDRValue(); // 블라인드 작동 전 LDR 값 저장
    moveBlindDown();
  } else {
    stopMotor();
  }
}

void stopMotor() {
  if (motorState != STOPPED) {
    Serial.println("모터 정지.");
    motorState = STOPPED;
    moveMotor(90);
    delay(500);

    float currentDistance = getDistance();

    float movementDistance = -1.0;
    if (initialDistance != -1.0 && currentDistance != -1.0) {
      movementDistance = abs(initialDistance - currentDistance);
      Serial.print("이동 거리: ");
      Serial.println(movementDistance);
    } else {
      Serial.println("이동 거리 계산 불가.");
    }

    Serial.print("업로드할 LDR 값: ");
    Serial.print(preMovementLdrValue); // 저장된 LDR 값 사용
    Serial.print(", 현재 거리: ");
    Serial.println(currentDistance);

    if (lastOperationMode == "") {
      lastOperationMode = config.autoMode ? "Auto" : "Manual";
    }

    String blindState = calculateBlindState(currentDistance);
    Serial.print("블라인드 상태: ");
    Serial.println(blindState);

    enqueueUploadData(lastOperationMode, lastOperationCommand, movementDistance, currentDistance, preMovementLdrValue, blindState);
    enqueueNetworkTask(UPDATE_STATE, UploadData(), blindState);

    initialDistance = currentDistance;

    lastOperationCommand = "";
    lastOperationMode = "";
  }
}

void moveMotor(int angle) {
  continuousServo.write(angle);
}

void enqueueUploadData(String operationMode, String operationCommand, float movementDistance, float distance, int ldrValue, String state) {
  int nextTail = (uploadQueueTail + 1) % UPLOAD_QUEUE_SIZE;
  if (nextTail != uploadQueueHead) {
    uploadQueue[uploadQueueTail] = {operationMode, operationCommand, movementDistance, distance, ldrValue, state};
    uploadQueueTail = nextTail;
    Serial.println("데이터 업로드 큐에 추가됨.");
  } else {
    Serial.println("업로드 큐가 가득 찼습니다. 데이터를 버립니다.");
  }
}

void handleUploadQueue() {
  static bool isUploading = false;
  static unsigned long lastUploadTime = 0;
  const unsigned long uploadInterval = 1000;

  if (!isUploading && uploadQueueHead != uploadQueueTail) {
    unsigned long currentTime = millis();
    if (currentTime - lastUploadTime >= uploadInterval) {
      lastUploadTime = currentTime;
      isUploading = true;

      UploadData data = uploadQueue[uploadQueueHead];
      uploadQueueHead = (uploadQueueHead + 1) % UPLOAD_QUEUE_SIZE;

      enqueueNetworkTask(UPLOAD_DATA, data);

      isUploading = false;
    }
  }
}

void enqueueNetworkTask(NetworkTaskType type, UploadData data, String state) {
  int nextTail = (networkTaskQueueTail + 1) % NETWORK_TASK_QUEUE_SIZE;
  if (nextTail != networkTaskQueueHead) {
    networkTaskQueue[networkTaskQueueTail] = {type, data, state};
    networkTaskQueueTail = nextTail;
    Serial.println("네트워크 작업 큐에 추가됨.");
  } else {
    Serial.println("네트워크 작업 큐가 가득 찼습니다. 작업을 버립니다.");
  }
}

void handleNetworkTasks() {
  static bool isProcessingNetworkTask = false;

  if (!isProcessingNetworkTask && networkTaskQueueHead != networkTaskQueueTail) {
    isProcessingNetworkTask = true;

    NetworkTask task = networkTaskQueue[networkTaskQueueHead];
    networkTaskQueueHead = (networkTaskQueueHead + 1) % NETWORK_TASK_QUEUE_SIZE;

    if (task.type == UPLOAD_DATA) {
      uploadSensorData(task.data);
    } else if (task.type == CHECK_RESERVATION) {
      fetchReservationsFromFirestore();
    } else if (task.type == UPDATE_STATE) {
      updateBlindLocationState(task.state);
    }

    isProcessingNetworkTask = false;
  }
}

void uploadSensorData(UploadData data) {
  HttpClient httpClient = HttpClient(httpsWifiClient, "firestore.googleapis.com", 443);
  httpClient.setTimeout(5000);

  time_t nowTime = now();
  char timestamp[30];
  sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02d+09:00", year(nowTime), month(nowTime), day(nowTime), hour(nowTime), minute(nowTime), second(nowTime));
  String operationTimeStr = String(timestamp);

  char documentId[50];
  sprintf(documentId, "blind_data_%04d%02d%02d_%02d%02d%02d", year(nowTime), month(nowTime), day(nowTime), hour(nowTime), minute(nowTime), second(nowTime));

  String path = "/v1/projects/" + String(projectID) + "/databases/(default)/documents/data/blind/logs?documentId=" + String(documentId) + "&key=" + String(apiKey);

  DynamicJsonDocument jsonDoc(1024);
  JsonObject fields = jsonDoc.createNestedObject("fields");

  fields["operationTime"]["timestampValue"] = operationTimeStr;
  fields["operationMode"]["stringValue"] = data.operationMode;
  fields["operationCommand"]["stringValue"] = data.operationCommand;
  fields["movementDistance"]["doubleValue"] = data.movementDistance;
  fields["distance"]["doubleValue"] = data.distance;
  fields["ldrValue"]["integerValue"] = data.ldrValue; // 스케일된 값 사용
  fields["state"]["stringValue"] = data.state;

  String jsonData;
  serializeJson(jsonDoc, jsonData);

  httpClient.beginRequest();
  httpClient.post(path);
  httpClient.sendHeader("Content-Type", "application/json");
  httpClient.sendHeader("Content-Length", jsonData.length());
  httpClient.beginBody();
  httpClient.print(jsonData);
  httpClient.endRequest();

  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();

  if (statusCode == 200 || statusCode == 201) {
    Serial.print("Firestore 업로드 성공, 응답 코드: ");
    Serial.println(statusCode);
    Serial.println("응답 본문: " + response);
  } else {
    Serial.print("Firestore 업로드 실패, 응답 코드: ");
    Serial.println(statusCode);
    Serial.println("응답 본문: " + response);
  }
}

void updateBlindLocationState(String state) {
  HttpClient httpClient = HttpClient(httpsWifiClient, "firestore.googleapis.com", 443);
  httpClient.setTimeout(5000);

  String path = "/v1/projects/" + String(projectID) + "/databases/(default)/documents/config/blindlocation?key=" + String(apiKey);

  DynamicJsonDocument jsonDoc(512);
  JsonObject fields = jsonDoc.createNestedObject("fields");
  fields["state"]["stringValue"] = state;

  String updateMask = "updateMask.fieldPaths=state";

  String jsonData;
  serializeJson(jsonDoc, jsonData);

  httpClient.beginRequest();
  httpClient.patch(path + "&" + updateMask);
  httpClient.sendHeader("Content-Type", "application/json");
  httpClient.sendHeader("Content-Length", jsonData.length());
  httpClient.beginBody();
  httpClient.print(jsonData);
  httpClient.endRequest();

  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();

  if (statusCode == 200) {
    Serial.print("블라인드 위치 상태 업데이트 성공, 응답 코드: ");
    Serial.println(statusCode);
  } else {
    Serial.print("블라인드 위치 상태 업데이트 실패, 응답 코드: ");
    Serial.println(statusCode);
    Serial.println("응답 본문: " + response);
  }
}

void handleControlMessage(String cmd) {
  if (cmd == "UP_PRESS") {
    lastOperationMode = "수동모드";
    lastOperationCommand = "올림";
    preMovementLdrValue = getScaledLDRValue(); // 블라인드 작동 전 LDR 값 저장
    moveBlindUp();
  } else if (cmd == "UP_RELEASE") {
    stopMotor();
  } else if (cmd == "DOWN_PRESS") {
    lastOperationMode = "수동모드";
    lastOperationCommand = "내림";
    preMovementLdrValue = getScaledLDRValue(); // 블라인드 작동 전 LDR 값 저장
    moveBlindDown();
  } else if (cmd == "DOWN_RELEASE") {
    stopMotor();
  } else if (cmd == "FULL_UP") {
    lastOperationMode = "수동모드";
    lastOperationCommand = "완전올림";
    preMovementLdrValue = getScaledLDRValue(); // 블라인드 작동 전 LDR 값 저장
    moveBlindToPosition(config.motorUpperLimit);
  } else if (cmd == "FULL_DOWN") {
    lastOperationMode = "수동모드";
    lastOperationCommand = "완전내림";
    preMovementLdrValue = getScaledLDRValue(); // 블라인드 작동 전 LDR 값 저장
    moveBlindToPosition(config.motorLowerLimit);
  } else if (cmd == "STOP") {
    stopMotor();
  } else if (cmd == "AUTO_ON") {
    config.autoMode = true;
    Serial.println("자동 모드 활성화");
    enqueueNetworkTask(UPDATE_STATE, UploadData(), "Auto Mode Enabled");
    updateAutoModeInFirestore(true); // 함수 호출
  } else if (cmd == "AUTO_OFF") {
    config.autoMode = false;
    stopMotor();
    Serial.println("자동 모드 비활성화");
    enqueueNetworkTask(UPDATE_STATE, UploadData(), "Auto Mode Disabled");
    updateAutoModeInFirestore(false); // 함수 호출
  } else {
    Serial.println("알 수 없는 명령어입니다.");
  }
}

void handleSettingsMessage(String json) {
  Serial.print("설정 메시지 수신: ");
  Serial.println(json);

  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    Serial.print("JSON 파싱 실패: ");
    Serial.println(error.c_str());
    return;
  }

  if (doc.containsKey("sensorLowerLimit")) {
    config.sensorLowerLimit = doc["sensorLowerLimit"];
    config.sensorLowerLimitRaw = map(config.sensorLowerLimit, 1, 100, 1, 1023); // 스케일링 수정
    Serial.print("sensorLowerLimit 업데이트: ");
    Serial.println(config.sensorLowerLimit);
    Serial.print("sensorLowerLimitRaw 업데이트: ");
    Serial.println(config.sensorLowerLimitRaw);
  }

  if (doc.containsKey("sensorUpperLimit")) {
    config.sensorUpperLimit = doc["sensorUpperLimit"];
    config.sensorUpperLimitRaw = map(config.sensorUpperLimit, 1, 100, 1, 1023); // 스케일링 수정
    Serial.print("sensorUpperLimit 업데이트: ");
    Serial.println(config.sensorUpperLimit);
    Serial.print("sensorUpperLimitRaw 업데이트: ");
    Serial.println(config.sensorUpperLimitRaw);
  }

  if (doc.containsKey("motorLowerLimit")) {
    config.motorLowerLimit = doc["motorLowerLimit"];
    Serial.print("motorLowerLimit 업데이트: ");
    Serial.println(config.motorLowerLimit);
  }

  if (doc.containsKey("motorUpperLimit")) {
    config.motorUpperLimit = doc["motorUpperLimit"];
    Serial.print("motorUpperLimit 업데이트: ");
    Serial.println(config.motorUpperLimit);
  }

  if (doc.containsKey("lowerOperation")) {
    config.lowerOperation = doc["lowerOperation"].as<String>();
    Serial.print("lowerOperation 업데이트: ");
    Serial.println(config.lowerOperation);
  }

  if (doc.containsKey("upperOperation")) {
    config.upperOperation = doc["upperOperation"].as<String>();
    Serial.print("upperOperation 업데이트: ");
    Serial.println(config.upperOperation);
  }
}

void checkAndExecuteReservation() {
  time_t currentTime = now();
  int currentHour = hour(currentTime);
  int currentMinute = minute(currentTime);
  int currentSecond = second(currentTime);

  for (int i = 0; i < reservationCount; i++) {
    if (reservations[i].active) {
      if (reservations[i].hour == currentHour &&
          reservations[i].minute == currentMinute &&
          reservations[i].second == currentSecond) {
        Serial.println("예약된 작업 실행 중...");

        if (reservations[i].operation == "하강") {
          lastOperationMode = "예약모드";
          lastOperationCommand = "내림";
          preMovementLdrValue = getScaledLDRValue(); // 블라인드 작동 전 LDR 값 저장
          moveBlindDown();
        } else if (reservations[i].operation == "상승") {
          lastOperationMode = "예약모드";
          lastOperationCommand = "올림";
          preMovementLdrValue = getScaledLDRValue(); // 블라인드 작동 전 LDR 값 저장
          moveBlindUp();
        } else {
          Serial.println("알 수 없는 예약 작업입니다.");
        }
      }
    }
  }
}

void fetchReservationsFromFirestore() {
  Serial.println("예약 정보 가져오는 중...");
  HttpClient httpClient = HttpClient(httpsWifiClient, "firestore.googleapis.com", 443);
  httpClient.setTimeout(10000);

  // 요청 경로 수정: 예약 정보가 있는 경로로 설정
  String path = "/v1/projects/" + String(projectID) + "/databases/(default)/documents/data/blind/schedules?key=" + String(apiKey);

  Serial.print("Firestore 예약 정보 요청 경로: ");
  Serial.println(path);

  httpClient.beginRequest();
  httpClient.get(path);
  httpClient.sendHeader("Content-Type", "application/json");
  httpClient.endRequest();

  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();

  Serial.print("Firestore 응답 코드 (예약 정보): ");
  Serial.println(statusCode);
  Serial.println("Firestore 응답 본문 (예약 정보): ");
  Serial.println(response);

  if (statusCode == 200) {
    DynamicJsonDocument doc(16384); // 필요한 경우 메모리 크기 조정
    DeserializationError error = deserializeJson(doc, response);
    if (!error) {
      reservationCount = 0;
      if (doc.containsKey("documents")) {
        JsonArray documents = doc["documents"].as<JsonArray>();
        for (JsonObject document : documents) {
          if (reservationCount >= MAX_RESERVATIONS) {
            Serial.println("최대 예약 수에 도달했습니다.");
            break;
          }

          JsonObject fields = document["fields"];

          Reservation res;
          res.active = false;
          res.operation = "";
          res.hour = 0;
          res.minute = 0;
          res.second = 0;

          if (fields.containsKey("active")) {
            res.active = fields["active"]["booleanValue"].as<bool>();
          }

          if (fields.containsKey("operation")) {
            res.operation = fields["operation"]["stringValue"].as<String>();
          }

          if (fields.containsKey("time")) {
            if (fields["time"].containsKey("timestampValue")) {
              String timestampStr = fields["time"]["timestampValue"].as<String>();
              parseTimeFromTimestamp(timestampStr, res.hour, res.minute, res.second);
            } else {
              Serial.println("time 필드에 timestampValue가 없습니다.");
            }
          } else {
            Serial.println("time 필드가 없습니다.");
          }

          // active가 true인 예약만 추가
          if (res.active) {
            reservations[reservationCount++] = res;
            Serial.print("예약 추가됨 - ");
            Serial.print("Active: ");
            Serial.print(res.active);
            Serial.print(", Operation: ");
            Serial.print(res.operation);
            Serial.print(", Time: ");
            Serial.print(res.hour);
            Serial.print(":");
            Serial.print(res.minute);
            Serial.print(":");
            Serial.println(res.second);
          }
        }
      } else {
        Serial.println("예약 문서가 없습니다.");
      }
    } else {
      Serial.print("JSON 파싱 실패 (예약 정보): ");
      Serial.println(error.c_str());
    }
  } else {
    Serial.print("예약 정보 가져오기 실패, 응답 코드: ");
    Serial.println(statusCode);
    Serial.println("응답 본문: ");
    Serial.println(response);
  }
}

void parseTimeFromTimestamp(String timestampStr, int &hour, int &minute, int &second) {
  // "YYYY-MM-DDThh:mm:ss.sssZ" 또는 "YYYY-MM-DDThh:mm:ssZ" 형식에서 시간 파싱
  int tIndex = timestampStr.indexOf('T');
  int colonIndex1 = timestampStr.indexOf(':', tIndex);
  int colonIndex2 = timestampStr.indexOf(':', colonIndex1 + 1);
  int endIndex = timestampStr.indexOf('Z', colonIndex2);
  if (endIndex == -1) endIndex = timestampStr.length();

  if (tIndex != -1 && colonIndex1 != -1 && colonIndex2 != -1) {
    hour = timestampStr.substring(tIndex + 1, colonIndex1).toInt();
    minute = timestampStr.substring(colonIndex1 + 1, colonIndex2).toInt();
    second = timestampStr.substring(colonIndex2 + 1, endIndex).toInt();

    // UTC 시간을 로컬 시간대로 변환 (한국 시간대 기준 +9시간)
    hour = (hour + 9) % 24;
  } else {
    Serial.println("타임스탬프 파싱 실패");
    hour = 0;
    minute = 0;
    second = 0;
  }
}

void fetchSettingsFromFirestore() {
  HttpClient httpClient = HttpClient(httpsWifiClient, "firestore.googleapis.com", 443);
  httpClient.setTimeout(5000);

  String path = "/v1/projects/" + String(projectID) + "/databases/(default)/documents/config/blind?key=" + String(apiKey);

  Serial.print("Firestore 요청 경로: ");
  Serial.println(path);

  httpClient.beginRequest();
  httpClient.get(path);
  httpClient.sendHeader("Content-Type", "application/json");
  httpClient.endRequest();

  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();

  Serial.print("Firestore 응답 코드: ");
  Serial.println(statusCode);
  Serial.println("Firestore 응답 본문: ");
  Serial.println(response);

  if (statusCode == 200) {
    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, response);
    if (!error) {
      JsonObject fields = doc["fields"];

      if (fields.containsKey("lowerOperation")) {
        config.lowerOperation = fields["lowerOperation"]["stringValue"].as<String>();
        Serial.print("lowerOperation: ");
        Serial.println(config.lowerOperation);
      } else {
        Serial.println("lowerOperation 필드가 없습니다.");
      }

      if (fields.containsKey("upperOperation")) {
        config.upperOperation = fields["upperOperation"]["stringValue"].as<String>();
        Serial.print("upperOperation: ");
        Serial.println(config.upperOperation);
      } else {
        Serial.println("upperOperation 필드가 없습니다.");
      }

      if (fields.containsKey("motorLowerLimit")) {
        if (fields["motorLowerLimit"].containsKey("doubleValue")) {
          config.motorLowerLimit = fields["motorLowerLimit"]["doubleValue"].as<float>();
          Serial.print("motorLowerLimit: ");
          Serial.println(config.motorLowerLimit);
        } else if (fields["motorLowerLimit"].containsKey("integerValue")) {
          config.motorLowerLimit = fields["motorLowerLimit"]["integerValue"].as<float>();
          Serial.print("motorLowerLimit: ");
          Serial.println(config.motorLowerLimit);
        } else {
          Serial.println("motorLowerLimit 필드에 숫자 값이 없습니다.");
        }
      } else {
        Serial.println("motorLowerLimit 필드가 없습니다.");
      }

      if (fields.containsKey("motorUpperLimit")) {
        if (fields["motorUpperLimit"].containsKey("doubleValue")) {
          config.motorUpperLimit = fields["motorUpperLimit"]["doubleValue"].as<float>();
          Serial.print("motorUpperLimit: ");
          Serial.println(config.motorUpperLimit);
        } else if (fields["motorUpperLimit"].containsKey("integerValue")) {
          config.motorUpperLimit = fields["motorUpperLimit"]["integerValue"].as<float>();
          Serial.print("motorUpperLimit: ");
          Serial.println(config.motorUpperLimit);
        } else {
          Serial.println("motorUpperLimit 필드에 숫자 값이 없습니다.");
        }
      } else {
        Serial.println("motorUpperLimit 필드가 없습니다.");
      }

      if (fields.containsKey("sensorLowerLimit")) {
        if (fields["sensorLowerLimit"].containsKey("doubleValue")) {
          config.sensorLowerLimit = fields["sensorLowerLimit"]["doubleValue"].as<int>();
          Serial.print("sensorLowerLimit: ");
          Serial.println(config.sensorLowerLimit);
        } else if (fields["sensorLowerLimit"].containsKey("integerValue")) {
          config.sensorLowerLimit = fields["sensorLowerLimit"]["integerValue"].as<int>();
          Serial.print("sensorLowerLimit: ");
          Serial.println(config.sensorLowerLimit);
        } else {
          Serial.println("sensorLowerLimit 필드에 숫자 값이 없습니다.");
        }
      } else {
        Serial.println("sensorLowerLimit 필드가 없습니다.");
      }

      if (fields.containsKey("sensorUpperLimit")) {
        if (fields["sensorUpperLimit"].containsKey("doubleValue")) {
          config.sensorUpperLimit = fields["sensorUpperLimit"]["doubleValue"].as<int>();
          Serial.print("sensorUpperLimit: ");
          Serial.println(config.sensorUpperLimit);
        } else if (fields["sensorUpperLimit"].containsKey("integerValue")) {
          config.sensorUpperLimit = fields["sensorUpperLimit"]["integerValue"].as<int>();
          Serial.print("sensorUpperLimit: ");
          Serial.println(config.sensorUpperLimit);
        } else {
          Serial.println("sensorUpperLimit 필드에 숫자 값이 없습니다.");
        }
      } else {
        Serial.println("sensorUpperLimit 필드가 없습니다.");
      }

      // 스케일된 한계값 계산 (1~100을 1~1023으로 변환)
      config.sensorLowerLimitRaw = map(config.sensorLowerLimit, 1, 100, 1, 1023);
      config.sensorUpperLimitRaw = map(config.sensorUpperLimit, 1, 100, 1, 1023);
      Serial.print("sensorLowerLimitRaw: ");
      Serial.println(config.sensorLowerLimitRaw);
      Serial.print("sensorUpperLimitRaw: ");
      Serial.println(config.sensorUpperLimitRaw);
    } else {
      Serial.print("JSON 파싱 실패: ");
      Serial.println(error.c_str());
      Serial.println("기본 설정을 사용합니다.");
      assignDefaultSettings(); // JSON 파싱 실패 시 기본 설정 적용
    }
  } else {
    Serial.print("설정값 가져오기 실패, 응답 코드: ");
    Serial.println(statusCode);
    Serial.println("기본 설정을 사용합니다.");
    assignDefaultSettings(); // Firestore에서 설정을 가져오지 못했을 때 기본 설정 적용
  }
}

void fetchAutoModeFromFirestore() {
  HttpClient httpClient = HttpClient(httpsWifiClient, "firestore.googleapis.com", 443);
  httpClient.setTimeout(5000);

  String path = "/v1/projects/" + String(projectID) + "/databases/(default)/documents/config/blindlocation?key=" + String(apiKey);

  Serial.print("Firestore 요청 경로 (autoMode): ");
  Serial.println(path);

  httpClient.beginRequest();
  httpClient.get(path);
  httpClient.sendHeader("Content-Type", "application/json");
  httpClient.endRequest();

  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();

  Serial.print("Firestore 응답 코드 (autoMode): ");
  Serial.println(statusCode);
  Serial.println("Firestore 응답 본문 (autoMode): ");
  Serial.println(response);

  if (statusCode == 200) {
    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, response);
    if (!error) {
      JsonObject fields = doc["fields"];

      if (fields.containsKey("auto")) {
        if (fields["auto"].containsKey("booleanValue")) {
          config.autoMode = fields["auto"]["booleanValue"].as<bool>();
          Serial.print("autoMode 설정: ");
          Serial.println(config.autoMode ? "true" : "false");
        } else {
          Serial.println("auto 필드에 booleanValue가 없습니다.");
        }
      } else {
        Serial.println("auto 필드가 없습니다.");
      }
    } else {
      Serial.print("JSON 파싱 실패 (autoMode): ");
      Serial.println(error.c_str());
      Serial.println("기본 설정을 사용합니다.");
      assignDefaultSettings(); // JSON 파싱 실패 시 기본 설정 적용
    }
  } else {
    Serial.print("autoMode 가져오기 실패, 응답 코드: ");
    Serial.println(statusCode);
    Serial.println("기본 설정을 사용합니다.");
    assignDefaultSettings(); // Firestore에서 autoMode를 가져오지 못했을 때 기본 설정 적용
  }
}

void assignDefaultSettings() {
  config.lowerOperation = "DOWN";
  config.upperOperation = "UP";
  config.motorLowerLimit = 5.0;
  config.motorUpperLimit = 40.0;
  config.sensorLowerLimit = 49; // 1~100 범위
  config.sensorUpperLimit = 76; // 1~100 범위
  config.autoMode = false;

  // 스케일된 한계값 계산 (1~100을 1~1023으로 변환)
  config.sensorLowerLimitRaw = map(config.sensorLowerLimit, 1, 100, 1, 1023);
  config.sensorUpperLimitRaw = map(config.sensorUpperLimit, 1, 100, 1, 1023);

  Serial.println("기본 설정값이 적용되었습니다.");
}

void mqttCallback(int messageSize) {
  String topic = mqttClient.messageTopic();
  String messageTemp;
  while (mqttClient.available()) messageTemp += (char)mqttClient.read();

  Serial.print("메시지 수신 [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(messageTemp);

  if (topic == controlTopic) handleControlMessage(messageTemp);
  else if (topic == settingsTopic) handleSettingsMessage(messageTemp);
  else if (topic == reservationUpdateTopic) {
    Serial.println("예약 정보 업데이트 요청 수신.");
    enqueueNetworkTask(CHECK_RESERVATION);
  }
}

// **중요:** 아래에 `updateAutoModeInFirestore` 함수 정의를 추가하세요.
void updateAutoModeInFirestore(bool autoMode) {
  HttpClient httpClient = HttpClient(httpsWifiClient, "firestore.googleapis.com", 443);
  httpClient.setTimeout(5000);

  String path = "/v1/projects/" + String(projectID) + "/databases/(default)/documents/config/blindlocation?key=" + String(apiKey);

  DynamicJsonDocument jsonDoc(512);
  JsonObject fields = jsonDoc.createNestedObject("fields");
  fields["auto"]["booleanValue"] = autoMode;

  String updateMask = "updateMask.fieldPaths=auto";

  String jsonData;
  serializeJson(jsonDoc, jsonData);

  httpClient.beginRequest();
  httpClient.patch(path + "&" + updateMask);
  httpClient.sendHeader("Content-Type", "application/json");
  httpClient.sendHeader("Content-Length", jsonData.length());
  httpClient.beginBody();
  httpClient.print(jsonData);
  httpClient.endRequest();

  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();

  if (statusCode == 200) {
    Serial.print("자동 모드 상태 업데이트 성공, 응답 코드: ");
    Serial.println(statusCode);
  } else {
    Serial.print("자동 모드 상태 업데이트 실패, 응답 코드: ");
    Serial.println(statusCode);
    Serial.println("응답 본문: " + response);
  }
}



