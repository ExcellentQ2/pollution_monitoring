#include "mqtt.h"
#include <string.h>
#include "esp_log.h"
#include "mqtt_client.h"
#include "ca_cert.h"   // <-- ADD: TLS CA certificate

static const char *TAG = "MQTT";

// ===== SECURE MQTT SETTINGS =====
#define MQTT_BROKER_URI "mqtts://055848714165464c85b6d80c181feb17.s1.eu.hivemq.cloud:8883"
#define MQTT_USERNAME   "esp32_city_smog"
#define MQTT_PASSWORD   "ACS8Cloud"

// Topic stays unchanged
#define MQTT_TOPIC     "acs/group8/airquality/data"
// ================================

static esp_mqtt_client_handle_t client = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected (TLS)");
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error");
            break;

        default:
            break;
    }
}

void mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,

        // --- Authentication ---
        .credentials.username = MQTT_USERNAME,
        .credentials.authentication.password = MQTT_PASSWORD,

        // --- TLS ---
        .broker.verification.certificate = ca_cert_pem,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    ESP_LOGI(TAG, "MQTT client started (TLS)");
}

void mqtt_publish(const char *payload)
{
    if (client == NULL) {
        ESP_LOGE(TAG, "MQTT client not initialized");
        return;
    }

    int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, payload, 0, 1, 0);
    ESP_LOGI(TAG, "Published to %s, msg_id=%d", MQTT_TOPIC, msg_id);
}
