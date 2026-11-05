#include "debug_log.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "config.h"
#include "log_queue_policy.h"

namespace {

struct LogLine {
  char text[LOG_LINE_CAPACITY + 1];
};

QueueHandle_t logQueue = nullptr;
// logf() runs on loop() (core 1), the upload task and the MQTT task (core 0):
// one mutex keeps their Serial lines from interleaving mid-line.
SemaphoreHandle_t serialMutex = nullptr;
volatile uint32_t queuedCount = 0;
volatile uint32_t droppedCount = 0;

void printLine(const char *text) {
  if (!Config::SERIAL_DEBUG) return;
  bool locked = serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE;
  Serial.println(text);
  if (locked) xSemaphoreGive(serialMutex);
}

}  // namespace

void beginDebugLog() {
  if (!serialMutex) serialMutex = xSemaphoreCreateMutex();
  if (!logQueue) logQueue = xQueueCreate(LOG_QUEUE_DEPTH, sizeof(LogLine));
}

void serialDebugf(const char *format, ...) {
  if (!Config::SERIAL_DEBUG) return;
  char text[LOG_LINE_CAPACITY + 1];
  va_list args;
  va_start(args, format);
  vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  printLine(text);
}

void logf(const char *format, ...) {
  LogLine line;
  va_list args;
  va_start(args, format);
  vsnprintf(line.text, sizeof(line.text), format, args);
  va_end(args);

  printLine(line.text);

  if (!logQueue) return;  // beginDebugLog() not called yet (very early boot)

  // The queued copy is embedded verbatim in a JSON string by
  // mqtt_client.cpp, so anything that would need escaping is neutralised.
  for (char *c = line.text; *c; ++c) {
    if (*c == '"' || *c == '\\') *c = '\'';
    else if (static_cast<unsigned char>(*c) < 0x20 || static_cast<unsigned char>(*c) >= 0x7F) *c = ' ';
  }

  UBaseType_t occupied = uxQueueMessagesWaiting(logQueue);
  if (LogQueuePolicy::decideEnqueueAction(occupied, LOG_QUEUE_DEPTH) ==
      LogQueuePolicy::Action::DROP_OLDEST_THEN_ENQUEUE) {
    LogLine discarded;
    if (xQueueReceive(logQueue, &discarded, 0) == pdTRUE) ++droppedCount;
  }
  if (xQueueSend(logQueue, &line, 0) == pdPASS) {
    ++queuedCount;
  } else {
    ++droppedCount;  // another task filled the slot between the check and here
  }
}

size_t drainLogLines(char out[][LOG_LINE_CAPACITY + 1], size_t maxLines) {
  if (!logQueue) return 0;
  size_t copied = 0;
  LogLine line;
  while (copied < maxLines && xQueueReceive(logQueue, &line, 0) == pdTRUE) {
    memcpy(out[copied], line.text, sizeof(line.text));
    ++copied;
  }
  return copied;
}

uint32_t logLinesQueued() { return queuedCount; }
uint32_t logLinesDropped() { return droppedCount; }
