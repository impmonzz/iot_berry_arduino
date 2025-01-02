#include <WiFiS3.h>
#include <WiFiClient.h>
#include <WiFiSSLClient.h>
#include <ArduinoHttpClient.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include "secrets.h"  // Wi-Fi 및 API 키 등을 관리하는 secrets.h 파일을 포함합니다.

// WiFi 설정 (secrets.h에서 관리)
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// MQTT 브로커 설정
const char* mqttServer = MQTT_SERVER;
const int mqttPort = MQTT_PORT;
const char* mqttClientID = "lantern_control";
const char* controlTopic = "lantern/control";
const char* statusTopic = "lantern/status";

// Firestore 설정
const char* projectID = FIREBASE_PROJECT_ID;
// API 키는 secrets.h 파일에서 관리합니다.

// WiFi 및 클라이언트 객체 생성
WiFiClient mqttWifiClient;
WiFiSSLClient httpsWifiClient;
MqttClient mqttClient(mqttWifiClient);

// NTP 클라이언트 설정 (한국 시간대 적용)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 9 * 3600, 60000);

// 하드웨어 핀 설정
const int soundSensorPin = 2;  // 사운드 센서 핀 (인터럽트 지원 핀)
const int relayPin = 7;        // 릴레이 제어 핀
const int ldrPin = A0;         // 조도 센서 핀 (A0에 연결)

// 설정 변수 (Firestore에서 가져올 예정)
struct Config {
  String lowerOperation;       // "꺼짐"
  String upperOperation;       // "켜짐"
  int sensorLowerLimit;        // Firestore에서 1~100 범위로 관리
  int sensorUpperLimit;        // Firestore에서 1~100 범위로 관리
  bool autoMode;               // 자동 모드 활성화 여부
  int mappedSensorLowerLimit;  // 1~1024 범위로 변환된 값
  int mappedSensorUpperLimit;  // 1~1024 범위로 변환된 값
};

Config config;

// 변수 선언
volatile bool clapDetected = false;   // 박수 감지 플래그 (인터럽트에서 사용)
bool lightState = false;              // 조명 상태 (켜짐/꺼짐)
int clapCount = 0;                    // 박수 횟수
unsigned long lastClapTime = 0;      // 마지막 박수 시간
const unsigned long clapInterval = 1000; // 박수 간 최대 간격 (밀리초)

// 작동 명령 및 모드 저장 변수
String lastOperationCommand = "";
String lastOperationMode = "";

// 예약 정보 구조체 및 변수 선언
struct Reservation {
  bool active;
  String operation;
  int hour;
  int minute;
  int second;
  String documentId;  // 예약 문서의 ID
};

#define MAX_RESERVATIONS 10
Reservation reservations[MAX_RESERVATIONS];
int reservationCount = 0;
time_t lastReservationCheck = 0;

// 함수 프로토타입 선언
void initializeHardware();
void initializeNetwork();
void fetchSettings();
void connectMQTT();
void setup_wifi();
void loopTasks();
void handleMQTT();
void handleAutoMode();
int getLDRValue();
int getScaledLDRValue(); // 추가: LDR 값을 1~100으로 스케일링
void turnLightOn(int ldrValue);  // 변경: 인자 추가
void turnLightOff(int ldrValue); // 변경: 인자 추가
void handleClapDetection();
void mqttCallback(int messageSize);
void assignDefaultSettings();
void fetchSettingsFromFirestore();
void uploadLightData(int ldrValue); // 변경: 인자 추가
void updateLightStatusInFirestore(String state);
void updateAutoModeInFirestore(bool autoMode);
void handleControlMessage(String cmd);
void fetchReservationsFromFirestore();
void checkAndExecuteReservation();
void deactivateReservationInFirestore(int index);
void parseTimeFromTimestamp(String timestampStr, int &hour, int &minute, int &second);

// 스케일 업 함수: 1~100을 1~1024로 변환
int scaleUp(int dbValue) {
  return map(constrain(dbValue, 1, 100), 1, 100, 1, 1024);
}

// 스케일 다운 함수: 1~1024을 1~100으로 변환
int scaleDown(int rawValue) {
  return map(constrain(rawValue, 1, 1024), 1, 1024, 1, 100);
}

// secrets.h에서 가져온 API 키
extern const char* apiKey;

void setup() {
  Serial.begin(115200);
  initializeHardware();
  setup_wifi();
  initializeNetwork();
  fetchSettings();
  timeClient.begin();
  fetchReservationsFromFirestore();  // 예약 정보 가져오기
}

void loop() {
  loopTasks();
}

void initializeHardware() {
  pinMode(soundSensorPin, INPUT_PULLUP);  // 사운드 센서 핀 설정 (풀업 저항 사용)
  pinMode(relayPin, OUTPUT);              // 릴레이 핀 출력 설정
  digitalWrite(relayPin, LOW);            // 초기 조명 상태: 꺼짐

  attachInterrupt(digitalPinToInterrupt(soundSensorPin), handleClapDetection, RISING);  // 박수 감지 인터럽트 설정
}

void initializeNetwork() {
  mqttClient.setId(mqttClientID);
  mqttClient.onMessage(mqttCallback);
  connectMQTT();
}

void fetchSettings() {
  fetchSettingsFromFirestore();
}

void loopTasks() {
  handleMQTT();
  timeClient.update();
  setTime(timeClient.getEpochTime());

  if (config.autoMode) {
    handleAutoMode();
  }

  checkAndExecuteReservation();  // 예약 확인 및 실행

  if (clapDetected) {
    clapDetected = false;
    unsigned long currentMillis = millis();
    if (currentMillis - lastClapTime <= clapInterval) {
      clapCount++;
    } else {
      clapCount = 1;
    }
    lastClapTime = currentMillis;

    Serial.print("박수 감지! 박수 횟수: ");
    Serial.println(clapCount);

    if (clapCount == 3) {
      int currentLdrValue = getLDRValue(); // 변경: 조명 상태 변경 전에 LDR 값 읽기
      if (lightState) {
        lastOperationMode = "박수감지";
        lastOperationCommand = config.upperOperation;
        turnLightOff(currentLdrValue); // 변경: 인자 전달
      } else {
        lastOperationMode = "박수감지";
        lastOperationCommand = config.lowerOperation;
        turnLightOn(currentLdrValue); // 변경: 인자 전달
      }
      clapCount = 0;
    }
  }
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
}

void handleAutoMode() {
  static unsigned long lastCheckTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastCheckTime >= 1000) { // 1초마다 체크
    lastCheckTime = currentTime;
    int ldrValue = getLDRValue();

    // 조도 센서 값을 기반으로 조명 제어
    if (ldrValue <= config.mappedSensorLowerLimit && lightState) { // 조도 낮음 & 조명 켜져 있을 때
      lastOperationMode = "자동모드";
      lastOperationCommand = config.lowerOperation; // "꺼짐"
      turnLightOff(ldrValue); // 변경: 인자 전달
    } else if (ldrValue >= config.mappedSensorUpperLimit && !lightState) { // 조도 높음 & 조명 꺼져 있을 때
      lastOperationMode = "자동모드";
      lastOperationCommand = config.upperOperation; // "켜짐"
      turnLightOn(ldrValue); // 변경: 인자 전달
    }
  }
}

int getLDRValue() {
  int ldrValue = analogRead(ldrPin);
  Serial.print("조도 센서 값: ");
  Serial.println(ldrValue);
  return ldrValue;
}

int getScaledLDRValue() {
  int rawValue = getLDRValue();
  int scaledValue = scaleDown(rawValue); // 스케일 다운 함수 사용
  Serial.print("스케일된 조도 센서 값: ");
  Serial.println(scaledValue);
  return scaledValue;
}

void turnLightOn(int ldrValue) { // 변경: 인자 추가
  if (!lightState) {
    lightState = true;
    digitalWrite(relayPin, HIGH);
    Serial.println("조명 켜짐");

    // MQTT로 상태 전송
    mqttClient.beginMessage(statusTopic);
    mqttClient.print("ON");
    mqttClient.endMessage();

    updateLightStatusInFirestore("on");
    uploadLightData(ldrValue); // 변경: 인자 전달
  }
}

void turnLightOff(int ldrValue) { // 변경: 인자 추가
  if (lightState) {
    lightState = false;
    digitalWrite(relayPin, LOW);
    Serial.println("조명 꺼짐");

    // MQTT로 상태 전송
    mqttClient.beginMessage(statusTopic);
    mqttClient.print("OFF");
    mqttClient.endMessage();

    updateLightStatusInFirestore("off");
    uploadLightData(ldrValue); // 변경: 인자 전달
  }
}

void handleClapDetection() {
  clapDetected = true;
}

void mqttCallback(int messageSize) {
  String topic = mqttClient.messageTopic();
  String message = "";

  while (mqttClient.available()) {
    char c = mqttClient.read();
    message += c;
  }

  message.trim();

  Serial.print("메시지 수신 [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if (topic == controlTopic) {
    handleControlMessage(message);
  } else {
    Serial.print("알 수 없는 토픽에서 메시지 수신: ");
    Serial.println(topic);
  }
}

void handleControlMessage(String cmd) {
  if (cmd == "ON") {
    lastOperationMode = "어플제어";
    lastOperationCommand = config.upperOperation; // "켜짐"
    int currentLdrValue = getLDRValue(); // 변경: 조명 상태 변경 전에 LDR 값 읽기
    turnLightOn(currentLdrValue); // 변경: 인자 전달
  } else if (cmd == "OFF") {
    lastOperationMode = "어플제어";
    lastOperationCommand = config.lowerOperation; // "꺼짐"
    int currentLdrValue = getLDRValue(); // 변경: 조명 상태 변경 전에 LDR 값 읽기
    turnLightOff(currentLdrValue); // 변경: 인자 전달
  } else if (cmd == "AUTO_ON") {
    config.autoMode = true;
    Serial.println("자동 모드 활성화");
    updateAutoModeInFirestore(true);
  } else if (cmd == "AUTO_OFF") {
    config.autoMode = false;
    Serial.println("자동 모드 비활성화");
    updateAutoModeInFirestore(false);
  } else {
    Serial.println("알 수 없는 명령어입니다.");
  }
}

void assignDefaultSettings() {
  config.lowerOperation = "꺼짐"; // 변경: "켜짐" -> "꺼짐"
  config.upperOperation = "켜짐"; // 변경: "꺼짐" -> "켜짐"
  config.sensorLowerLimit = 28;    // Firestore에서 1~100 범위
  config.sensorUpperLimit = 50;    // Firestore에서 1~100 범위
  config.autoMode = false;

  // Firestore에서 가져온 1~100 범위의 값을 1~1024 범위로 변환
  config.mappedSensorLowerLimit = scaleUp(config.sensorLowerLimit);
  config.mappedSensorUpperLimit = scaleUp(config.sensorUpperLimit);

  Serial.println("기본 설정값이 적용되었습니다.");
  Serial.print("mappedSensorLowerLimit: ");
  Serial.println(config.mappedSensorLowerLimit);
  Serial.print("mappedSensorUpperLimit: ");
  Serial.println(config.mappedSensorUpperLimit);
}

void fetchSettingsFromFirestore() {
  HttpClient httpClient = HttpClient(httpsWifiClient, "firestore.googleapis.com", 443);
  httpClient.setTimeout(5000);

  String path = "/v1/projects/";
  path += projectID;
  path += "/databases/(default)/documents/config/lantern";
  path += "?key=";
  path += apiKey;

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
    DynamicJsonDocument doc(4096);
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

      if (fields.containsKey("sensorLowerLimit")) {
        if (fields["sensorLowerLimit"].containsKey("integerValue")) {
          config.sensorLowerLimit = fields["sensorLowerLimit"]["integerValue"].as<int>();
          Serial.print("sensorLowerLimit: ");
          Serial.println(config.sensorLowerLimit);
        } else {
          Serial.println("sensorLowerLimit 필드에 integerValue가 없습니다.");
        }
      } else {
        Serial.println("sensorLowerLimit 필드가 없습니다.");
      }

      if (fields.containsKey("sensorUpperLimit")) {
        if (fields["sensorUpperLimit"].containsKey("integerValue")) {
          config.sensorUpperLimit = fields["sensorUpperLimit"]["integerValue"].as<int>();
          Serial.print("sensorUpperLimit: ");
          Serial.println(config.sensorUpperLimit);
        } else {
          Serial.println("sensorUpperLimit 필드에 integerValue가 없습니다.");
        }
      } else {
        Serial.println("sensorUpperLimit 필드가 없습니다.");
      }

      // Firestore에서 가져온 1~100 범위의 값을 1~1024 범위로 변환
      config.mappedSensorLowerLimit = scaleUp(config.sensorLowerLimit);
      config.mappedSensorUpperLimit = scaleUp(config.sensorUpperLimit);
      Serial.print("mappedSensorLowerLimit: ");
      Serial.println(config.mappedSensorLowerLimit);
      Serial.print("mappedSensorUpperLimit: ");
      Serial.println(config.mappedSensorUpperLimit);

    } else {
      Serial.print("JSON 파싱 실패: ");
      Serial.println(error.c_str());
      assignDefaultSettings();
    }
  } else {
    Serial.print("설정값 가져오기 실패, 응답 코드: ");
    Serial.println(statusCode);
    Serial.println("기본 설정값을 사용합니다.");
    assignDefaultSettings();
  }

  // autoMode 가져오기
  fetchAutoModeFromFirestore();
}

void fetchAutoModeFromFirestore() {
  HttpClient httpClient = HttpClient(httpsWifiClient, "firestore.googleapis.com", 443);
  httpClient.setTimeout(5000);

  String path = "/v1/projects/";
  path += projectID;
  path += "/databases/(default)/documents/config/lanternstate";
  path += "?key=";
  path += apiKey;

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
      assignDefaultSettings(); // JSON 파싱 실패 시 기본 설정 적용
    }
  } else {
    Serial.print("autoMode 가져오기 실패, 응답 코드: ");
    Serial.println(statusCode);
    Serial.println("기본 설정을 사용합니다.");
    assignDefaultSettings(); // Firestore에서 autoMode를 가져오지 못했을 때 기본 설정 적용
  }
}

void uploadLightData(int ldrValue) { // 변경: 인자 추가
  HttpClient httpClient = HttpClient(httpsWifiClient, "firestore.googleapis.com", 443);
  httpClient.setTimeout(5000);

  time_t nowTime = now();
  char timestamp[30];
  sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02d+09:00",
          year(nowTime), month(nowTime), day(nowTime),
          hour(nowTime), minute(nowTime), second(nowTime));
  String operationTimeStr = String(timestamp);

  char documentId[50];
  sprintf(documentId, "lantern_data_%04d%02d%02d_%02d%02d%02d",
          year(nowTime), month(nowTime), day(nowTime),
          hour(nowTime), minute(nowTime), second(nowTime));

  String path = "/v1/projects/";
  path += projectID;
  path += "/databases/(default)/documents/data/lantern/logs";
  path += "?documentId=";
  path += documentId;
  path += "&key=";
  path += apiKey;

  DynamicJsonDocument jsonDoc(1024);
  JsonObject fields = jsonDoc.createNestedObject("fields");

  fields["operationTime"]["timestampValue"] = operationTimeStr;
  fields["operationMode"]["stringValue"] = lastOperationMode;
  fields["operationCommand"]["stringValue"] = lastOperationCommand;
  fields["ldrValue"]["integerValue"] = scaleDown(ldrValue);  // 변경: 전달된 ldrValue 사용
  fields["state"]["stringValue"] = lightState ? "on" : "off";

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
  } else {
    Serial.print("Firestore 업로드 실패, 응답 코드: ");
    Serial.println(statusCode);
    Serial.println("응답 본문: " + response);
  }

  // 마지막 명령 초기화
  lastOperationCommand = "";
  lastOperationMode = "";
}

void updateLightStatusInFirestore(String state) {
  HttpClient httpClient = HttpClient(httpsWifiClient, "firestore.googleapis.com", 443);
  httpClient.setTimeout(5000);

  String path = "/v1/projects/";
  path += projectID;
  path += "/databases/(default)/documents/config/lanternstate?key=";
  path += apiKey;

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
    Serial.print("조명 상태 업데이트 성공, 응답 코드: ");
    Serial.println(statusCode);
  } else {
    Serial.print("조명 상태 업데이트 실패, 응답 코드: ");
    Serial.println(statusCode);
    Serial.println("응답 본문: " + response);
  }
}

void updateAutoModeInFirestore(bool autoMode) {
  HttpClient httpClient = HttpClient(httpsWifiClient, "firestore.googleapis.com", 443);
  httpClient.setTimeout(5000);

  String path = "/v1/projects/";
  path += projectID;
  path += "/databases/(default)/documents/config/lanternstate?key=";
  path += apiKey;

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

void fetchReservationsFromFirestore() {
  Serial.println("예약 정보 가져오는 중...");
  HttpClient httpClient = HttpClient(httpsWifiClient, "firestore.googleapis.com", 443);
  httpClient.setTimeout(10000);

  // 요청 경로: /data/lantern/schedules
  String path = "/v1/projects/";
  path += projectID;
  path += "/databases/(default)/documents/data/lantern/schedules?key=";
  path += apiKey;

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
          res.documentId = "";

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

          // 문서 ID 추출
          String fullDocumentName = document["name"].as<String>();
          int lastSlashIndex = fullDocumentName.lastIndexOf('/');
          res.documentId = fullDocumentName.substring(lastSlashIndex + 1);

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
            Serial.print("Document ID: ");
            Serial.println(res.documentId);
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

void checkAndExecuteReservation() {
  time_t nowTime = now();
  int currentHour = hour(nowTime);
  int currentMinute = minute(nowTime);
  int currentSecond = second(nowTime);

  // 예약 정보를 주기적으로 갱신 (여기서는 1시간마다 갱신)
  if (nowTime - lastReservationCheck >= 3600) {
    lastReservationCheck = nowTime;
    fetchReservationsFromFirestore();
  }

  for (int i = 0; i < reservationCount; i++) {
    Reservation res = reservations[i];
    if (res.active) {
      if (res.hour == currentHour && res.minute == currentMinute && res.second == currentSecond) {
        Serial.println("예약 실행 중...");
        lastOperationMode = "Reservation";
        lastOperationCommand = res.operation;

        int currentLdrValue = getLDRValue(); // 변경: 조명 상태 변경 전에 LDR 값 읽기
        if (res.operation == config.lowerOperation) {
          turnLightOff(currentLdrValue); // 변경: 인자 전달
        } else if (res.operation == config.upperOperation) {
          turnLightOn(currentLdrValue); // 변경: 인자 전달
        } else {
          Serial.println("알 수 없는 예약 명령어입니다.");
        }

        // 예약을 비활성화합니다
        deactivateReservationInFirestore(i);
      }
    }
  }
}

void deactivateReservationInFirestore(int index) {
  if (index < 0 || index >= reservationCount) return;

  String documentId = reservations[index].documentId;
  String path = "/v1/projects/";
  path += projectID;
  path += "/databases/(default)/documents/data/lantern/schedules/";
  path += documentId;
  path += "?key=";
  path += apiKey;

  HttpClient httpClient = HttpClient(httpsWifiClient, "firestore.googleapis.com", 443);
  httpClient.setTimeout(5000);

  DynamicJsonDocument jsonDoc(256);
  JsonObject fields = jsonDoc.createNestedObject("fields");
  fields["active"]["booleanValue"] = false;

  String updateMask = "updateMask.fieldPaths=active";

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
    Serial.print("예약 비활성화 성공, 응답 코드: ");
    Serial.println(statusCode);
    // 로컬에서도 active를 false로 설정
    reservations[index].active = false;
  } else {
    Serial.print("예약 비활성화 실패, 응답 코드: ");
    Serial.println(statusCode);
    Serial.println("응답 본문: " + response);
  }
}

void parseTimeFromTimestamp(String timestampStr, int &hour, int &minute, int &second) {
  // 타임스탬프 형식: "YYYY-MM-DDTHH:MM:SSZ" 또는 "YYYY-MM-DDTHH:MM:SS+09:00"
  int tIndex = timestampStr.indexOf('T');
  int colonIndex1 = timestampStr.indexOf(':', tIndex);
  int colonIndex2 = timestampStr.indexOf(':', colonIndex1 + 1);

  if (tIndex != -1 && colonIndex1 != -1 && colonIndex2 != -1) {
    hour = timestampStr.substring(tIndex + 1, colonIndex1).toInt();
    minute = timestampStr.substring(colonIndex1 + 1, colonIndex2).toInt();
    // 초 부분은 2자리만 추출 (밀리초는 무시)
    second = timestampStr.substring(colonIndex2 + 1, colonIndex2 + 3).toInt();

    // 시간대 보정 (UTC 시간으로 가정하고 +9시간 추가)
    hour = (hour + 9) % 24;
  } else {
    Serial.println("타임스탬프 파싱 실패");
    hour = 0;
    minute = 0;
    second = 0;
  }
}

