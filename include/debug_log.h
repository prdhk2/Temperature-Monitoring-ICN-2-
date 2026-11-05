#pragma once

#include <Arduino.h>

// Bounded, non-blocking debug log. logf() queues each line for
// mqtt_client.cpp to batch-publish to <site>/<id>/log - the dashboard's live
// log tail - and, on the bench build only (Config::SERIAL_DEBUG), also prints
// it on Serial.
//
// Serial (UART0) is the LFS flow-meter bus on this board, so in the release
// build nothing but the LFS protocol is ever written to it.
void beginDebugLog();

// Safe before beginDebugLog() and while MQTT is down: the line is printed
// (bench build) and, once the queue exists, queued (oldest dropped when full).
void logf(const char *format, ...) __attribute__((format(printf, 1, 2)));

// Serial-only debug output (bench build only), for places where the MQTT log
// cannot be used yet, e.g. before the boot-rollback decision.
void serialDebugf(const char *format, ...) __attribute__((format(printf, 1, 2)));

// One formatted line, capped so a single line can never eat the whole
// MQTT_PACKET_SIZE budget once batched with others.
constexpr size_t LOG_LINE_CAPACITY = 127;
constexpr size_t LOG_QUEUE_DEPTH = 32;

// Drains up to maxLines queued lines into out[], returning how many were
// copied. Never blocks.
size_t drainLogLines(char out[][LOG_LINE_CAPACITY + 1], size_t maxLines);

uint32_t logLinesQueued();
uint32_t logLinesDropped();
