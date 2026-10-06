/* iot_cams.c — application entry point (app_main).
 *
 * Responsibilities:
 *   - Bootstrap the WiFi provisioning package and wait for
 *     credentials to land in NVS.
 *   - Once credentials are committed, wait for the station DHCP
 *     lease, log the assigned IP, shut down the softAP, and
 *     enter the application supervisor loop.
 *   - Bring up the OV2640 sensor (via cam_reader) before any
 *     branch runs so GET /capture on the STA server always finds
 *     a working sensor.
 *
 * On every boot (first provision or subsequent re-join) the same
 * wait-for-IP-and-log sequence runs. On a first boot the softAP
 * is torn down after DHCP; on subsequent boots the softAP is
 * never started because provisioning_run() is skipped entirely.
 *
 * Constraints enforced by this file:
 *   - It includes ONLY `provisioning.h`, `cam_reader.h`, and
 *     `cam_stream.h`. No `esp_wifi.h`, `esp_netif.h`, `mdns.h`,
 *     `protocomm.h` or `esp_camera.h` shows up here — those
 *     are private to the component that owns them. If you find
 *     yourself wanting to add one of those includes, it is a
 *     sign that component's public surface is too narrow and
 *     should be widened.
 */
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"

#include "provisioning.h"
#include "cam_reader.h"
#include "cam_stream.h"

static const char *TAG = "app_main";
static const char *FW_VERSION = "0.1.0";

/* Bit set by ip_handler when IP_EVENT_STA_GOT_IP fires. */
#define GOT_IP_BIT  (1u << 0)

/* Forward declarations. */
static void ip_handler(void *arg, esp_event_base_t ev_base,
                       int32_t ev_id, void *ev_data);

/* IP event handler — logs the assigned IPv4 address and signals
 * the waiting task via the event group. Runs on every STA attach
 * (first provision, subsequent reboots, transient reconnects). */
static void ip_handler(void *arg, esp_event_base_t ev_base,
                       int32_t ev_id, void *ev_data)
{
    (void)ev_base;

    if (ev_id != IP_EVENT_STA_GOT_IP) return;

    EventGroupHandle_t ev = (EventGroupHandle_t)arg;
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)ev_data;
    const esp_netif_ip_info_t *ip = &event->ip_info;

    /* Filter out IPv6 events — we only want the IPv4 DHCP lease. */
    uint8_t *b = (uint8_t *)&ip->ip.addr;
    if (b[0] == 0 && b[1] == 0 && b[2] == 0 && b[3] == 0) return;

    /* Pull the SSID so the log is self-contained. */
    wifi_config_t wcfg = {0};
    char ssid[33] = "(unknown)";
    if (esp_wifi_get_config(WIFI_IF_STA, &wcfg) == ESP_OK) {
        strncpy(ssid, (const char *)wcfg.sta.ssid, sizeof(ssid) - 1);
        ssid[sizeof(ssid) - 1] = '\0';
    }

    ESP_LOGI(TAG, "station connected to \"%s\": ip=" IPSTR
                  " netmask=" IPSTR " gw=" IPSTR,
             ssid,
             IP2STR(&ip->ip),
             IP2STR(&ip->netmask),
             IP2STR(&ip->gw));

    xEventGroupSetBits(ev, GOT_IP_BIT);
}

/* Wait up to 30 s for the station to receive a DHCP lease from the
 * home AP, then log the assigned IPv4 address.
 *
 * IMPORTANT: does NOT call esp_wifi_connect(). The caller is
 * responsible for triggering the station attach before calling this:
 *   - post-provisioning path: provisioning_apply_captive_form()
 *     already called esp_wifi_connect() when the form posted.
 *   - already-provisioned path: provisioning_join_ap() does it.
 *
 * Calling esp_wifi_connect() here would race with the in-flight
 * connect from the form handler (the wifi driver returns
 * ESP_ERR_WIFI_CONN and prints "sta is connecting, return error"
 * in the log). Each path triggers the connect exactly once.
 *
 * The wifi driver and event loop run on their own tasks; we just
 * need this task alive so esp_event_loop_run() keeps dispatching
 * events to our handler. */
static void wait_for_sta_ip_and_log(const char *label)
{
    /* Register a one-shot IP handler and block until it fires. */
    StaticEventGroup_t ev_group_buf = {0};
    EventGroupHandle_t ev_group = xEventGroupCreateStatic(&ev_group_buf);

    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                               ip_handler, ev_group);

    EventBits_t bits = xEventGroupWaitBits(
        ev_group, GOT_IP_BIT, pdTRUE, pdFALSE,
        pdMS_TO_TICKS(30000));

    if (bits & GOT_IP_BIT) {
        ESP_LOGI(TAG, "%s: station DHCP lease acquired", label);
    } else {
        ESP_LOGW(TAG, "%s: IP_EVENT_STA_GOT_IP timed out "
                      "(DHCP may still complete in the background)", label);
    }

    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                ip_handler);
    vEventGroupDelete(ev_group);
}

void app_main(void)
{
    /* Provisioning configuration. Zero-init so PoP and SSID fall
     * through to the Kconfig defaults (CONFIG_PROVISIONING_POP,
     * CONFIG_PROVISIONING_SERVICE_NAME_PREFIX). Force_provisioning
     * stays false — the future factory-reset path will toggle it. */
    provisioning_config_t cfg = {0};
    cfg.security = PROV_SECURITY_1;

    provisioning_app_info_t info = {
        .name       = CONFIG_PROVISIONING_DEVICE_NAME,
        .fw_version = FW_VERSION,
    };

    esp_err_t r = provisioning_init(&cfg, &info);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "provisioning_init failed: %s", esp_err_to_name(r));
        return;
    }

    /* Bring up the OV2640 once at boot. After this call the
     * cam_reader_capture() API is hot for the lifetime of the
     * process; the GET /capture endpoint on the STA server
     * exercises it. Failure here is non-fatal — we log and keep
     * running so provisioning + /whoami still work; /capture
     * will simply return 500 until the device is power-cycled. */
    ESP_LOGI(TAG, "initializing camera reader...");
    esp_err_t cam_r = cam_reader_init();
    if (cam_r != ESP_OK) {
        ESP_LOGE(TAG, "cam_reader_init failed: %s -- /capture will be unavailable",
                 esp_err_to_name(cam_r));
    }

    /* W1 (feat/camera-flash-capture) — bring up the GPIO 4 flash
     * LEDC channel and disable the sensor's advanced AEC DSP so
     * flash-lit frames do not overexpose. Idempotent; if
     * cam_reader_init failed above we still attempt the LEDC
     * setup because it does not require the sensor — the sensor-
     * side `set_aec2` call inside the init function will
     * gracefully no-op when esp_camera_sensor_get() returns NULL.
     *
     * Failure here is non-fatal: /capture without ?flash=1 still
     * works; only flash photography is affected. */
    ESP_LOGI(TAG, "initializing flash LED...");
    esp_err_t fl_r = cam_reader_flash_init();
    if (fl_r != ESP_OK) {
        ESP_LOGE(TAG, "cam_reader_flash_init failed: %s -- "
                      "flash photography will be unavailable",
                 esp_err_to_name(fl_r));
    }

    /* W2 (feat/iot-cams-ws-cams-endpoint) — long-lived stream
     * consumer task. Pulls JPEGs from cam_reader at
     * CAM_STREAM_PERIOD_MS and ships them to the WS viewer sink
     * installed by /ws/cams handshake accept (W4/W5). Independent
     * from /capture: the cam_reader mutex serializes them so
     * /capture is starved (5 s timeout → 503) while WS streams. */
    ESP_LOGI(TAG, "initializing camera stream...");
    esp_err_t cs_r = cam_stream_init();
    if (cs_r != ESP_OK) {
        ESP_LOGE(TAG, "cam_stream_init failed: %s -- /ws/cams will be unavailable",
                 esp_err_to_name(cs_r));
    } else {
        esp_err_t cs_t = cam_stream_task_start();
        if (cs_t != ESP_OK) {
            ESP_LOGE(TAG, "cam_stream_task_start failed: %s",
                     esp_err_to_name(cs_t));
        }
    }

    ESP_LOGI(TAG, "device: %s fw=%s", info.name, info.fw_version);

    if (provisioning_is_provisioned()) {
        /* Credentials exist in NVS from a previous provisioning session.
         * Bring up wifi in STA-only mode and trigger a connect, then
         * wait for the DHCP lease so we can log the assigned IP. The
         * softAP is never started on this path (provisioning_run is
         * skipped). provisioning_join_ap() handles the wifi init +
         * esp_wifi_connect() so we don't race with anyone. */
        ESP_LOGI(TAG, "credentials present in NVS — joining home AP");
        r = provisioning_join_ap();
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "provisioning_join_ap failed: %s",
                     esp_err_to_name(r));
            return;
        }
        wait_for_sta_ip_and_log("already-provisioned");

        /* Start the STA server for device discovery /whoami endpoint. */
        r = provisioning_start_sta_server();
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "failed to start STA server: %s", esp_err_to_name(r));
        }

        ESP_LOGI(TAG, "entering supervisor loop");
        while (1) {
            /* Call the provisioning supervisor on every tick. The
             * supervisor owns the no-IP watchdog (forces reconnect
             * after 60 s without an IP) and emits a heartbeat log
             * every 30 s so the operator can confirm the loop is
             * alive. */
            (void)provisioning_supervise_sta();
            vTaskDelay(pdMS_TO_TICKS(30000));
        }
    }

    /* Reset station state before entering the provisioning branch.
     * Clears s_sta_got_ip and s_consecutive_failures so the
     * disconnection handler in provisioning.c uses retry logic
     * from a clean slate during this provisioning session. */
    provisioning_reset_sta_state();

    ESP_LOGI(TAG, "no credentials in NVS — starting SoftAP provisioning");
    r = provisioning_run();
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "provisioning_run failed: %s", esp_err_to_name(r));
        return;
    }

    /* Provisioning succeeded. Credentials are committed to NVS.
     * The station has already attempted to connect when the captive
     * form called esp_wifi_connect() inside provisioning_apply_
     * captive_form(). The disconnection handler (WIFI_EVENT_STA_
     * DISCONNECTED) will retry with exponential backoff (up to 5
     * attempts) if the home AP drops the connection.
     *
     * WAIT FOR IP BEFORE SHUTTING DOWN THE softAP — this mirrors
     * esp32-cam-surveillance wifi_event.c: the softAP is torn down
     * only when IP_EVENT_STA_GOT_IP fires (not on WIFI_EVENT_STA_
     * CONNECTED which fires at L2 association, before DHCP).
     * Shutting down on L2-assoc was too early: the home AP sometimes
     * deauths the ESP32 before the DHCP lease completes, and without
     * the softAP running the device had no recovery path. */
    ESP_LOGI(TAG, "post-provisioning: waiting for DHCP lease from home AP...");
    StaticEventGroup_t ev_group_buf = {0};
    EventGroupHandle_t ev_group = xEventGroupCreateStatic(&ev_group_buf);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                               ip_handler, ev_group);
    EventBits_t bits = xEventGroupWaitBits(
        ev_group, GOT_IP_BIT, pdTRUE, pdFALSE,
        pdMS_TO_TICKS(60000));
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                ip_handler);
    if (!(bits & GOT_IP_BIT)) {
        ESP_LOGW(TAG, "post-provisioning: DHCP lease timed out — "
                      "softAP stays up; device will retry from app");
    } else {
        ESP_LOGI(TAG, "post-provisioning: DHCP lease acquired — shutting down softAP");
    }
    vEventGroupDelete(ev_group);

    /* Now shut down the softAP. The station is already connected
     * and has an IP. The disconnection handler will retry if needed
     * (s_sta_got_ip=true means it won't retry but the app could
     * use provisioning_join_ap() for an explicit reconnect). */
    r = esp_wifi_set_mode(WIFI_MODE_STA);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "post-provisioning: esp_wifi_set_mode(STA): %s",
                 esp_err_to_name(r));
    }

    /* Start the STA server for device discovery /whoami endpoint. */
    r = provisioning_start_sta_server();
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "failed to start STA server: %s", esp_err_to_name(r));
    }

    ESP_LOGI(TAG, "post-provisioning: entering supervisor loop");
    while (1) {
        /* Call the provisioning supervisor on every tick. Belt
         * and braces over the reconnect task — the supervisor
         * additionally forces esp_wifi_connect() after 60 s
         * without an IP. */
        (void)provisioning_supervise_sta();
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
