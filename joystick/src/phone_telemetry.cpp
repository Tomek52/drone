// SPDX-License-Identifier: MIT
//
// [POSHOLD] See phone_telemetry.h.

#include "phone_telemetry.h"

#include <Arduino.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "phone_page.h"

namespace {

constexpr const char *kSsid     = "StampFly-Pilot";
constexpr const char *kPassword = "stampfly1";  // WPA2 needs at least 8 characters
constexpr int kMaxClients       = 2;
constexpr size_t kMaxFrame      = 250;  // ESP-NOW payload limit
constexpr int kQueueLen         = 16;   // ~0.6 s of telemetry at 25 Hz

struct Item {
    uint16_t len;
    uint8_t data[kMaxFrame];
};

QueueHandle_t g_queue      = nullptr;
volatile bool g_active     = false;
WebServer *g_server        = nullptr;
WebSocketsServer *g_socket = nullptr;

// Runs on core 0 at low priority, away from the control loop (core 1). A slow phone can only stall this task.
void web_task(void *) {
    for (;;) {
        g_server->handleClient();
        g_socket->loop();
        Item it;
        int sent = 0;
        while (sent < 8 && xQueueReceive(g_queue, &it, 0) == pdTRUE) {
            if (g_socket->connectedClients() > 0) g_socket->broadcastBIN(it.data, it.len);
            sent++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

}  // namespace

bool phone_telemetry_start(uint8_t channel) {
    if (g_active) return true;
    g_queue = xQueueCreate(kQueueLen, sizeof(Item));
    if (g_queue == nullptr) return false;

    // The radio is shared with ESP-NOW: the access point must sit on the channel of the drone link.
    WiFi.mode(WIFI_AP_STA);
    if (!WiFi.softAP(kSsid, kPassword, channel, 0, kMaxClients)) return false;
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    g_server = new WebServer(80);
    g_server->on("/", []() { g_server->send_P(200, "text/html; charset=utf-8", kPhonePage); });
    g_server->onNotFound([]() { g_server->send(404, "text/plain", "404"); });
    g_server->begin();
    g_socket = new WebSocketsServer(81);
    g_socket->begin();

    g_active = true;
    xTaskCreatePinnedToCore(web_task, "phone_telem", 6144, nullptr, 1, nullptr, 0);
    USBSerial.printf("Phone telemetry: Wi-Fi %s (password %s), channel %u, http://%s/\n\r", kSsid, kPassword, channel,
                     WiFi.softAPIP().toString().c_str());
    return true;
}

bool phone_telemetry_active(void) {
    return g_active;
}

void phone_telemetry_push(const uint8_t *data, size_t len) {
    if (!g_active || len == 0 || len > kMaxFrame) return;
    Item it;
    it.len = (uint16_t)len;
    memcpy(it.data, data, len);
    xQueueSend(g_queue, &it, 0);
}
