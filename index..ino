#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <DHT.h>
#include <math.h>
#include <stdarg.h>
#include "esp_task_wdt.h"
#include "esp_system.h"

// তোমার wiring অনুযায়ী
#define RGB_PIN          48
#define BUZZER_PIN       2
#define DHT_PIN          14
#define DHT_TYPE         DHT11
#define FLAME_PIN        21
#define FLAME_ACTIVE_LOW 1

#define RFID_SCK         41
#define RFID_MISO        42
#define RFID_MOSI        40
#define RFID_SS          1
#define RFID_RST         3

#define SERIAL_BAUD      115200
#define FW_VERSION       "1.0.4-L1"
#define WDT_TIMEOUT_MS   8000
#define LED_BRIGHTNESS   40

DHT dht(DHT_PIN, DHT_TYPE);
MFRC522 rfid(RFID_SS, RFID_RST);

static SemaphoreHandle_t logMux = nullptr;

static volatile bool flameNow = false;
static volatile bool doorUnlocked = false;
static volatile bool dhtValid = false;
static volatile float gTemp = 0;
static volatile float gHum = 0;
static volatile bool errDht = false;
static volatile bool errRfid = false;

static const char *bootReason = "POWER_ON";

struct TaskInfo {
  const char *name;
  const char *period;
  TaskHandle_t handle;
  int core;
};

static TaskInfo taskList[8];
static int taskCount = 0;

static void plog(const char *format, ...) {
  char buffer[256];

  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (logMux != nullptr &&
      xSemaphoreTake(logMux, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.println(buffer);
    xSemaphoreGive(logMux);
  } else {
    Serial.println(buffer);
  }
}

static const char *getBootReason() {
  switch (esp_reset_reason()) {
    case ESP_RST_SW:
      return "SOFTWARE";
    case ESP_RST_PANIC:
      return "CRASH";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      return "WATCHDOG";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT";
    default:
      return "POWER_ON";
  }
}

static bool readFlame() {
  int value = digitalRead(FLAME_PIN);
#if FLAME_ACTIVE_LOW
  return value == LOW;
#else
  return value == HIGH;
#endif
}

static void setupWatchdog() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t config = {};
  config.timeout_ms = WDT_TIMEOUT_MS;
  config.idle_core_mask = 0;
  config.trigger_panic = true;

  esp_err_t result = esp_task_wdt_reconfigure(&config);
  if (result != ESP_OK) {
    result = esp_task_wdt_init(&config);
  }

  if (result != ESP_OK) {
    plog("[ERROR] WATCHDOG setup failed: %d", (int)result);
  }
#else
  esp_err_t result = esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, true);

  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
    plog("[ERROR] WATCHDOG setup failed: %d", (int)result);
  }
#endif
}

static void startTask(
  TaskFunction_t function,
  const char *name,
  const char *period,
  uint32_t stackSize,
  UBaseType_t priority,
  int core
) {
  if (taskCount >= 8) {
    plog("[ERROR] Too many tasks");
    return;
  }

  TaskHandle_t handle = nullptr;
  BaseType_t result = xTaskCreatePinnedToCore(
    function, name, stackSize, nullptr, priority, &handle, core
  );

  if (result == pdPASS) {
    taskList[taskCount++] = {name, period, handle, core};
    plog("[INFO] task=%s core=%d period=%s created",
         name, core, period);
  } else {
    plog("[ERROR] task=%s creation failed", name);
  }
}

// Safe: green 2 Hz. Sensor error: magenta 2 Hz.
// Fire: red fast flashing.
static void ledTask(void *) {
  esp_task_wdt_add(nullptr);
  TickType_t lastWake = xTaskGetTickCount();
  bool ledOn = false;

  for (;;) {
    ledOn = !ledOn;

    if (!ledOn) {
      neopixelWrite(RGB_PIN, 0, 0, 0);
    } else if (flameNow) {
      neopixelWrite(RGB_PIN, LED_BRIGHTNESS, 0, 0);
    } else if (errDht || errRfid) {
      neopixelWrite(
        RGB_PIN, LED_BRIGHTNESS, 0, LED_BRIGHTNESS
      );
    } else {
      neopixelWrite(RGB_PIN, 0, LED_BRIGHTNESS, 0);
    }

    esp_task_wdt_reset();

    TickType_t interval =
      flameNow ? pdMS_TO_TICKS(100) : pdMS_TO_TICKS(250);

    vTaskDelayUntil(&lastWake, interval);
  }
}

// Fire buzzer: 200 ms on, 200 ms off until flame clears.
static void buzzerTask(void *) {
  esp_task_wdt_add(nullptr);
  TickType_t lastWake = xTaskGetTickCount();
  bool buzzerOn = false;

  for (;;) {
    if (flameNow) {
      buzzerOn = !buzzerOn;
      digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);
    } else {
      buzzerOn = false;
      digitalWrite(BUZZER_PIN, LOW);
    }

    esp_task_wdt_reset();
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(200));
  }
}

// Flame sensor: every 100 ms on Core 0
static void flameTask(void *) {
  esp_task_wdt_add(nullptr);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    flameNow = readFlame();
    doorUnlocked = flameNow;

    esp_task_wdt_reset();
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));
  }
}

// RFID: every 200 ms on Core 0
static void rfidTask(void *) {
  esp_task_wdt_add(nullptr);

  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  rfid.PCD_Init();

  TickType_t lastWake = xTaskGetTickCount();
  int readFailures = 0;
  int missedReads = 0;
  bool cardReported = false;

  for (;;) {
    byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
    bool readerOK = (version != 0x00 && version != 0xFF);

    if (!readerOK) {
      readFailures++;

      if (readFailures >= 3 && !errRfid) {
        errRfid = true;
        cardReported = false;
        plog("[ERROR] SENSOR ERROR: RFID");
      }
    } else {
      if (readFailures > 0) {
        rfid.PCD_Init();
      }
      readFailures = 0;

      if (errRfid) {
        errRfid = false;
        plog("[ERROR] SENSOR ERROR CLEARED: RFID");
      }

      byte atqa[2];
      byte atqaLength = sizeof(atqa);
      MFRC522::StatusCode result =
        rfid.PICC_WakeupA(atqa, &atqaLength);

      if (result == MFRC522::STATUS_OK ||
          result == MFRC522::STATUS_COLLISION) {
        if (rfid.PICC_ReadCardSerial()) {
          if (!cardReported) {
            cardReported = true;

            char uid[24] = {0};
            for (byte i = 0; i < rfid.uid.size && i < 10; i++) {
              sprintf(uid + i * 2, "%02X", rfid.uid.uidByte[i]);
            }

            plog("[CARD] uid=%s", uid);
          }

          rfid.PICC_HaltA();
        }

        missedReads = 0;
      } else {
        missedReads++;

        // Card must be absent for about 2 seconds before another tap.
        if (missedReads >= 10) {
          cardReported = false;
        }
      }
    }

    esp_task_wdt_reset();
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(200));
  }
}

// DHT11: read every 5 seconds on Core 1
static void dhtTask(void *) {
  esp_task_wdt_add(nullptr);
  dht.begin();

  uint32_t nextReadMs = 0;
  int failedReads = 0;

  for (;;) {
    uint32_t now = millis();

    if ((int32_t)(now - nextReadMs) >= 0) {
      float temperature = dht.readTemperature(false, true);
      float humidity = dht.readHumidity();

      if (!isnan(temperature) && !isnan(humidity)) {
        gTemp = temperature;
        gHum = humidity;
        dhtValid = true;
        failedReads = 0;

        if (errDht) {
          errDht = false;
          plog("[ERROR] SENSOR ERROR CLEARED: DHT11");
        }

        nextReadMs = millis() + 5000;
      } else {
        failedReads++;
        dhtValid = false;

        if (failedReads >= 3 && !errDht) {
          errDht = true;
          plog("[ERROR] SENSOR ERROR: DHT11");
        }

        nextReadMs = millis() + 1000;
      }
    }

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Status: every second on Core 1
static void printStatus() {
  char temperature[16] = "--";
  char humidity[16] = "--";

  if (dhtValid) {
    snprintf(temperature, sizeof(temperature), "%.1fC", (float)gTemp);
    snprintf(humidity, sizeof(humidity), "%.0f%%", (float)gHum);
  }

  plog(
    "[STATUS] up=%lus T=%s RH=%s HI=- flame=%d door=%s "
    "state=%s trend=STABLE rssi=- heap=%luk psram=%luk "
    "chip=%.1fC fw=%s",
    (unsigned long)(millis() / 1000),
    temperature,
    humidity,
    flameNow ? 1 : 0,
    doorUnlocked ? "UNLOCKED" : "LOCKED",
    flameNow ? "FIRE" : "SAFE",
    (unsigned long)(ESP.getFreeHeap() / 1024),
    (unsigned long)(ESP.getFreePsram() / 1024),
    temperatureRead(),
    FW_VERSION
  );
}

static void statusTask(void *) {
  esp_task_wdt_add(nullptr);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    printStatus();
    esp_task_wdt_reset();
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
  }
}

// ---------- Serial commands ----------
static void commandHelp() {
  plog("[CMD] commands: help, status, health, tasks, ver, hang");
}

static void commandHealth() {
  plog("[CMD] uptime=%lus", (unsigned long)(millis() / 1000));
  plog("[CMD] chip_temp=%.1fC", temperatureRead());
  plog("[CMD] free_heap=%luk",
       (unsigned long)(ESP.getFreeHeap() / 1024));
  plog("[CMD] free_psram=%luk",
       (unsigned long)(ESP.getFreePsram() / 1024));
  plog("[CMD] flash=%luMB",
       (unsigned long)(ESP.getFlashChipSize() / (1024 * 1024)));
  plog("[CMD] cpu=%luMHz",
       (unsigned long)ESP.getCpuFreqMHz());
  plog("[CMD] wifi_rssi=-");
  plog("[CMD] reset_reason=%s", bootReason);
  plog("[CMD] fw=%s", FW_VERSION);
}

static void commandTasks() {
  for (int i = 0; i < taskCount; i++) {
    plog("[CMD] task=%s core=%d period=%s free_stack=%lu",
         taskList[i].name,
         taskList[i].core,
         taskList[i].period,
         (unsigned long)uxTaskGetStackHighWaterMark(
           taskList[i].handle
         ));
  }
}

static void handleCommand(String command) {
  command.trim();
  command.toLowerCase();

  if (command == "help") {
    commandHelp();
  } else if (command == "status") {
    printStatus();
  } else if (command == "health") {
    commandHealth();
  } else if (command == "tasks") {
    commandTasks();
  } else if (command == "ver") {
    plog("[CMD] fw=%s build=%s %s",
         FW_VERSION, __DATE__, __TIME__);
  } else if (command == "hang") {
    plog("[CMD] watchdog test: board should restart within 10 seconds");
    Serial.flush();

    // Deliberately stop feeding this task's watchdog.
    for (;;) {
      __asm__ __volatile__("nop");
    }
  } else if (command.length() > 0) {
    plog("[CMD] unknown command: %s (type help)", command.c_str());
  }
}

// Command task: accepts newline, CR, or no line ending
static void commandTask(void *) {
  esp_task_wdt_add(nullptr);
  String line;
  uint32_t lastInputMs = 0;

  for (;;) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();

      if (c == '\r' || c == '\n') {
        if (line.length() > 0) {
          handleCommand(line);
          line = "";
        }
      } else if (line.length() < 64) {
        line += c;
        lastInputMs = millis();
      } else {
        line = "";
        plog("[CMD] input too long");
      }
    }

    // If Serial Monitor is set to "No line ending", run after typing stops.
    if (line.length() > 0 && millis() - lastInputMs >= 150) {
      handleCommand(line);
      line = "";
    }

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  // Check flame first, before creating tasks.
  pinMode(FLAME_PIN, INPUT);
  flameNow = readFlame();
  doorUnlocked = flameNow;

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.begin(SERIAL_BAUD);
  logMux = xSemaphoreCreateMutex();

  bootReason = getBootReason();
  plog("[EVENT] BOOT reason=%s", bootReason);
  plog("[INFO] FastRobox Level 1 starting");
  plog("[INFO] RGB=%d BUZZER=%d", RGB_PIN, BUZZER_PIN);
  plog("[INFO] DHT=%d FLAME=%d RFID SS=%d SCK=%d MOSI=%d MISO=%d RST=%d",
       DHT_PIN, FLAME_PIN, RFID_SS, RFID_SCK,
       RFID_MOSI, RFID_MISO, RFID_RST);

  setupWatchdog();

  // Core 0: LED, buzzer, flame sensor, RFID
  startTask(ledTask,    "LED",    "250ms", 3072, 2, 0);
  startTask(buzzerTask, "BUZZER", "200ms", 3072, 2, 0);
  startTask(flameTask,  "FLAME",  "100ms", 3072, 3, 0);
  startTask(rfidTask,   "RFID",   "200ms", 4096, 2, 0);

  // Core 1: DHT11, status, commands
  startTask(dhtTask,     "DHT11",  "5s",   4096, 2, 1);
  startTask(statusTask,  "STATUS", "1s",   4096, 2, 1);
  startTask(commandTask, "CMD",   "10ms",  4096, 2, 1);

  vTaskDelete(nullptr);
}

void loop() {
  // setup() deletes the Arduino loop task.
}