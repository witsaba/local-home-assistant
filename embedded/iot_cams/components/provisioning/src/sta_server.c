/* sta_server.c — STA-bound HTTP server for post-provisioning device
 * identity and capture endpoints.
 *
 * PURPOSE
 *   After provisioning completes and the device has joined the home
 *   AP (STA mode), this module starts a lightweight httpd on the
 *   station interface. It exposes /whoami for device discovery and
 *   registration by the home-assistant backend, and /capture for
 *   taking single JPEG frames from the OV2640 sensor.
 *
 * LIFECYCLE
 *   - NOT started during softAP provisioning (the captive portal
 *     serves provisioning UX; /whoami + /capture are intentionally
 *     absent so unprovisioned devices don't appear in discovery
 *     and operators on the softAP can't pull frames before WiFi
 *     is configured).
 *   - Started by provisioning_join_ap() / provisioning_run() once
 *     the station has an IP.
 *   - Runs for the lifetime of the device.
 *
 * PORT
 *   Fixed port 80. The ESP32 station only serves local LAN clients
 *   (no inbound internet routing), so port 80 is safe and convenient.
 *
 * ENDPOINTS
 *   - GET /whoami  — device identity JSON (mac, name, description,
 *     fw, chip). Same contract as esp32-cam-surveillance /whoami
 *     for cross-device compatibility.
 *   - GET /capture — single JPEG frame from the OV2640. Mirrors the
 *     esp32-cam-surveillance /capture contract: image/jpeg body,
 *     Content-Disposition: inline; filename=capture.jpg, no chunking,
 *     no base64. Concurrency is guarded by a binary semaphore in
 *     cam_reader — concurrent callers past the 5 s wait budget get
 *     HTTP 503.
 *
 * SECURITY
 *   The device is on a trusted private LAN by design. No
 *   authentication on /whoami — the MAC is the canonical identity.
 *   /capture is similarly open because the LAN is trusted. Future
 *   endpoints (config, control) will add auth if needed.
 */
#include "sta_server.h"

#include <string.h>

#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_mac.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"

#include "cam_reader.h"
#include "ws_cams.h"

/* Forward declarations for provisioning diagnostics accessors.
 * These live in provisioning.c behind provisioning.h's public
 * seam so they are accessible to /health without pulling esp_wifi
 * into the sta_server TU. */
extern const char  *prov_fw_version(void);
extern uint32_t    provisioning_reconnect_count(void);
extern uint8_t     provisioning_last_disconnect_reason(void);

/* Forward declaration for the ws_cams viewer fd accessor. */
extern int ws_cams_viewer_fd_get(void);

static const char *TAG = "sta_srv";

/* The httpd handle. NULL when not running. */
static httpd_handle_t s_sta_httpd = NULL;

/* ---------- /health ---------- */

/* GET /health — diagnostics JSON.
 *
 * Returns 200 with a JSON body describing the device's network and
 * memory health. Never returns 500 — if WiFi is down the endpoint
 * still answers with RSSI -1 and IP 0.0.0.0 so an off-network or
 * wedged device can still be diagnosed.
 *
 * Fields:
 *   uptime_ms        — milliseconds since boot (monotonic)
 *   free_heap        — bytes of free internal RAM right now
 *   min_free_heap    — lowest-ever free internal RAM since boot
 *   free_psram       — bytes of free PSRAM (0 if not present)
 *   ip               — current IPv4 address, "0.0.0.0" if not connected
 *   rssi             — current WiFi RSSI in dBm, -1 if not connected
 *   reconnect_count  — number of esp_wifi_connect() calls issued
 *                      by the reconnect task since boot
 *   last_disconn_rc  — wifi_event_sta_disconnected_t.reason from
 *                      the most recent disconnect, 0 if none
 *   ws_viewer        — 1 if a WS viewer is currently attached, 0 otherwise
 *
 * The off-network / wedged / healthy discriminator:
 *   off-network — ip == "0.0.0.0", rssi == -1
 *   wedged     — ip != "0.0.0.0", heap collapsing
 *   healthy    — ip present, RSSI reasonable, reconnect_count stable
 */
static esp_err_t health_get_handler(httpd_req_t *req)
{
    if (!req) return ESP_FAIL;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    /* Uptime: use xTaskGetTickCount() × portTICK_PERIOD_MS.
     * Both are always available without any new includes.
     * xTaskGetTickCount is safe to call from an ISR or task. */
    uint32_t uptime_ms = (uint32_t)(
        (uint64_t)xTaskGetTickCount() * (uint64_t)portTICK_PERIOD_MS);

    uint32_t free_heap     = esp_get_free_heap_size();
    uint32_t min_free_heap = esp_get_minimum_free_heap_size();
    uint32_t free_psram    = 0;
#if CONFIG_ESP32_SPIRAM_SUPPORT || CONFIG_ESP_SPIRAM
    free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
#endif

    /* IP address from the station netif. Graceful fallback if
     * the station is not connected: report 0.0.0.0 rather than
     * returning 500 — /health must still answer when the device
     * is off-network. */
    char ip_str[16] = "0.0.0.0";
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK) {
            snprintf(ip_str, sizeof(ip_str), IPSTR,
                     IP2STR(&ip_info.ip));
        }
    }

    /* RSSI from the AP record. Graceful fallback: report -1 when
     * the station is not connected (esp_wifi_sta_get_ap_info returns
     * ESP_ERR_WIFI_NOT_CONNECT). */
    int rssi = -1;
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        rssi = (int)ap_info.rssi;
    }

    uint32_t reconn    = provisioning_reconnect_count();
    uint8_t  disconn  = provisioning_last_disconnect_reason();
    int      ws_fd     = ws_cams_viewer_fd_get();
    int      ws_viewer = (ws_fd >= 0) ? 1 : 0;

    char buf[256];
    int len = snprintf(buf, sizeof(buf),
        "{\"uptime_ms\":%lu,\"free_heap\":%lu,\"min_free_heap\":%lu,"
        "\"free_psram\":%lu,\"ip\":\"%s\",\"rssi\":%d,"
        "\"reconnect_count\":%lu,\"last_disconn_rc\":%u,\"ws_viewer\":%d}",
        (unsigned long)uptime_ms,
        (unsigned long)free_heap,
        (unsigned long)min_free_heap,
        (unsigned long)free_psram,
        ip_str,
        rssi,
        (unsigned long)reconn,
        (unsigned)disconn,
        ws_viewer);

    if (len < 0 || (size_t)len >= sizeof(buf)) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"error\":\"buf_overflow\"}");
        return ESP_OK;
    }

    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

/* ---------- /whoami ---------- */

/* Convert 6-byte MAC to 12-char lowercase hex string (no separators).
 * out must be at least 13 bytes. Returns ESP_OK on success. */
static esp_err_t mac_to_hex_lower(const uint8_t mac[6], char *out, size_t out_len)
{
    if (!mac || !out || out_len < 13) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < 6; i++) {
        uint8_t b = mac[i];
        uint8_t hi = (b >> 4) & 0x0F;
        out[i * 2] = (hi < 10) ? ('0' + hi) : ('a' + hi - 10);
        uint8_t lo = b & 0x0F;
        out[i * 2 + 1] = (lo < 10) ? ('0' + lo) : ('a' + lo - 10);
    }
    out[12] = '\0';
    return ESP_OK;
}

/* GET /whoami — JSON device identity.
 * Returns: mac, name, description (omitted if empty), fw, chip
 * Follows the esp32-cam-surveillance /whoami contract.
 * Optimized: static buffer, no heap, no per-request logging. */
static esp_err_t whoami_get_handler(httpd_req_t *req)
{
    if (!req) return ESP_FAIL;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    /* Read MAC from eFuse. */
    uint8_t mac[6] = {0};
    esp_err_t r = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "whoami: esp_read_mac failed: %s", esp_err_to_name(r));
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"error\":\"mac_unavailable\"}");
        return ESP_OK;
    }

    /* Convert MAC to lowercase hex string (12 chars, no separators). */
    char mac_hex[13];
    mac_to_hex_lower(mac, mac_hex, sizeof(mac_hex));

    /* Read chip info. */
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    /* chip model -> human-readable string. */
    const char *chip_str = "ESP32-UNKNOWN";
    switch (chip.model) {
        case 1:  chip_str = "ESP32-D0WDQ6"; break;
        case 2:  chip_str = "ESP32-S2";     break;
        case 5:  chip_str = "ESP32-S3";     break;
        case 12: chip_str = "ESP32-C3";     break;
        default: chip_str = "ESP32-UNKNOWN"; break;
    }

    /* Device name and firmware version from provisioning. */
    extern const char *prov_device_name(void);
    const char *name = prov_device_name();
    const char *description = "";  /* no identity NVS yet */
    const char *fw_ver = prov_fw_version();

    /* Uptime in seconds (monotonic, integer). */
    uint32_t uptime_s = ((uint32_t)xTaskGetTickCount() * (uint32_t)portTICK_PERIOD_MS) / 1000U;

    /* Live IP and RSSI. Graceful fallback: "0.0.0.0" and -1 when
     * the station is not connected. */
    char ip_str[16] = "0.0.0.0";
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK) {
            snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
        }
    }

    int rssi = -1;
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        rssi = (int)ap_info.rssi;
    }

    /* Build JSON response using static buffer (no heap allocation).
     * Format: {"mac":"...","name":"...","fw":"...","chip":"...",
     *          "idf":"...","ip":"...","rssi":N,"uptime_s":N}
     *
     * fw: application firmware version (e.g. "0.1.0"), NOT esp_get_idf_version().
     *     The backend uses fw to identify which deployed image is running;
     *     esp_get_idf_version() (e.g. "v5.5.3") is meaningless for that purpose.
     *
     * idf: ESP-IDF version string. Additive field — Go's encoding/json
     *      ignores unknown fields, so this is safe for existing backends.
     *
     * ip, rssi, uptime_s: live state. Fallback values when the station
     *     is not connected (off-network / wedged) so the endpoint still
     *     answers 200 instead of 500. */
    char buf[256];
    int len;
    if (description[0] == '\0') {
        len = snprintf(buf, sizeof(buf),
            "{\"mac\":\"%s\",\"name\":\"%s\",\"fw\":\"%s\",\"chip\":\"%s\","
            "\"idf\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"uptime_s\":%u}",
            mac_hex, name,
            fw_ver[0] ? fw_ver : "unknown",
            chip_str,
            esp_get_idf_version(),
            ip_str, rssi, (unsigned)uptime_s);
    } else {
        len = snprintf(buf, sizeof(buf),
            "{\"mac\":\"%s\",\"name\":\"%s\",\"description\":\"%s\",\"fw\":\"%s\",\"chip\":\"%s\","
            "\"idf\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"uptime_s\":%u}",
            mac_hex, name, description,
            fw_ver[0] ? fw_ver : "unknown",
            chip_str,
            esp_get_idf_version(),
            ip_str, rssi, (unsigned)uptime_s);
    }

    if (len < 0 || (size_t)len >= sizeof(buf)) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"error\":\"buf_overflow\"}");
        return ESP_OK;
    }

    httpd_resp_set_hdr(req, "X-Witsaba-Device", "true");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

/* ---------- /capture ---------- */

/* GET /capture — single JPEG frame from the OV2640.
 *
 * Optional query parameter: `?flash=1` enables the GPIO 4 flash
 * LED for 200 ms before the sensor integrates the frame (see
 * cam_reader_capture_with_flash). Any other value, or absent,
 * means no flash — existing behavior. The clock-window decision
 * lives on the caller (the surveillance worker); this endpoint
 * honors the flag unconditionally.
 *
 * Concurrency contract: cam_reader owns the camera mutex; this
 * handler maps the three documented outcomes to HTTP status codes:
 *
 *   - ESP_ERR_TIMEOUT   → 503 (mutex not acquired; do NOT call
 *                         cam_reader_release on this path)
 *   - sensor failure    → 500 + cam_reader_release(NULL) so the
 *                         mutex is given back to the driver pool
 *   - success           → 200 + httpd_resp_send + cam_reader_release(fb)
 *
 * The mutex MUST be released on every path that took it. The helper
 * makes that explicit: cam_reader_release(NULL) is permitted and
 * only gives the mutex (the buffer return becomes a no-op). */
static esp_err_t capture_get_handler(httpd_req_t *req)
{
    if (req == NULL) {
        return ESP_FAIL;
    }

    /* Parse ?flash=1 from the URI. httpd_query_key_value expects
     * a *pure* query string (e.g. "flash=1"), NOT the full URI
     * ("/capture?flash=1"). The matcher strips the query string
     * before invoking us (see httpd_uri.c's use of UF_PATH
     * field_data), so req->uri still contains the full string
     * and we must extract the query substring via the dedicated
     * helper. httpd_req_get_url_query_str writes the query
     * string into our buffer without the leading '?'.
     *
     * httpd_query_key_value returns ESP_OK on a hit, ESP_ERR_NOT_FOUND
     * if the key is absent (i.e. no flash). Any other return is
     * treated conservatively as no flash. We only enable flash
     * when the value is exactly the ASCII string "1". */
    bool flash = false;
    char query_buf[32] = {0};
    esp_err_t qry_err = httpd_req_get_url_query_str(req, query_buf,
                                                    sizeof(query_buf));
    ESP_LOGD(TAG, "capture: req->uri='%s' query='%s' (err=%s)",
             req->uri, query_buf, esp_err_to_name(qry_err));
    if (qry_err == ESP_OK) {
        char flash_val[8] = {0};
        esp_err_t kv_err = httpd_query_key_value(query_buf, "flash",
                                                 flash_val, sizeof(flash_val));
        ESP_LOGD(TAG, "capture: query_key_value flash err=%s val='%s'",
                 esp_err_to_name(kv_err), flash_val);
        if (kv_err == ESP_OK) {
            flash = (flash_val[0] == '1' && flash_val[1] == '\0');
        }
    }

    camera_fb_t *fb = NULL;
    esp_err_t r = cam_reader_capture_with_flash(&fb, flash);

    if (r == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "capture: mutex timeout; another caller busy");
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "Camera busy, please try again");
        return ESP_OK;
    }
    if (r != ESP_OK || fb == NULL) {
        ESP_LOGE(TAG, "capture: sensor returned no frame (flash=%d)",
                 (int)flash);
        httpd_resp_send_500(req);
        cam_reader_release(NULL);  /* drop the mutex */
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Cache-Control",
                       "no-store, no-cache, must-revalidate, max-age=0");
    if (flash) {
        /* Echo the flag for clients that want to verify the server
         * honored it (especially helpful while debugging the
         * 200ms pre-charge timing on hardware). */
        httpd_resp_set_hdr(req, "X-Witsaba-Flash", "1");
    }

    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    cam_reader_release(fb);

    if (res == ESP_OK) {
        ESP_LOGD(TAG, "capture: served frame flash=%d", (int)flash);
    } else {
        ESP_LOGE(TAG, "capture: httpd_resp_send: %s", esp_err_to_name(res));
    }
    return res;
}

/* ---------- lifecycle ---------- */

esp_err_t sta_server_start(void)
{
    if (s_sta_httpd) {
        ESP_LOGW(TAG, "sta_server already running");
        return ESP_OK;
    }

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.stack_size = 4096;
    cfg.max_uri_handlers = 8;
    cfg.max_req_hdr_len = 512;

    /* Defect B fix, part 1 — size the socket pool.
     * Three /ws/cams viewers (one per camera, single-viewer
     * enforcement exists) hold 3 sockets permanently. A CCTV
     * grid with all three cameras open holds 3 WS + 3 capture
     * sessions + discovery polling + slack. HTTPD_DEFAULT_CONFIG
     * leaves max_open_sockets=7 with lru_purge_enable=false,
     * so the pool never self-reclaims and new connections
     * stall in backlog_conn=5. */
    cfg.max_open_sockets = 12;
    cfg.lru_purge_enable = true;

    /* Defect B fix, part 2 — bound how long a half-dead client
     * pins a thread + socket. Request parsing needs no longer
     * than 3 s. send_wait_timeout stays at the 5 s default —
     * a ~30 KB JPEG on a weak link genuinely needs it. */
    cfg.recv_wait_timeout = 3;

    /* Defect B fix, part 3 — raise httpd worker priority above
     * the camera stream producer. Priority invariant:
     * esp_event (20) > httpd (5) > cam_stream (3).
     * The old priority (tskIDLE_PRIORITY+5 = 1) let the
     * cam_stream producer at priority 5 starve the httpd
     * workers that all discovery, surveillance, and the UI
     * depend on. */
    cfg.task_priority = 5;

    esp_err_t err = httpd_start(&s_sta_httpd, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        s_sta_httpd = NULL;
        return err;
    }

    /* Register /whoami. */
    httpd_uri_t whoami_uri = {
        .uri       = "/whoami",
        .method    = HTTP_GET,
        .handler   = whoami_get_handler,
        .user_ctx  = NULL,
    };
    err = httpd_register_uri_handler(s_sta_httpd, &whoami_uri);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register /whoami failed: %s", esp_err_to_name(err));
        httpd_stop(s_sta_httpd);
        s_sta_httpd = NULL;
        return err;
    }

    /* Register /capture. Same handle, same audience (post-
     * provisioning LAN clients). Concurrency guard owned by
     * cam_reader; the handler here is a thin wrapper that maps
     * the cam_reader_capture outcomes to HTTP status codes. */
    httpd_uri_t capture_uri = {
        .uri       = "/capture",
        .method    = HTTP_GET,
        .handler   = capture_get_handler,
        .user_ctx  = NULL,
    };
    err = httpd_register_uri_handler(s_sta_httpd, &capture_uri);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register /capture failed: %s", esp_err_to_name(err));
        httpd_stop(s_sta_httpd);
        s_sta_httpd = NULL;
        return err;
    }

    /* Register /health — diagnostics endpoint for network and memory
     * observability. Same handle, same audience (post-provisioning
     * LAN clients and the Pi monitoring agent). */
    httpd_uri_t health_uri = {
        .uri       = "/health",
        .method    = HTTP_GET,
        .handler   = health_get_handler,
        .user_ctx  = NULL,
    };
    err = httpd_register_uri_handler(s_sta_httpd, &health_uri);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register /health failed: %s", esp_err_to_name(err));
        httpd_stop(s_sta_httpd);
        s_sta_httpd = NULL;
        return err;
    }

    /* W4 (feat/iot-cams-ws-cams-endpoint) — register the
     * WebSocket endpoint for live JPEG streaming. Same httpd
     * handle, same audience (post-provisioning LAN clients).
     * The handler currently accepts handshakes and captures
     * the fd; the live sink install lands in W5. */
    err = ws_cams_register_uri(s_sta_httpd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register /ws/cams failed: %s",
                 esp_err_to_name(err));
        httpd_stop(s_sta_httpd);
        s_sta_httpd = NULL;
        return err;
    }

    /* W7 — subscribe to IP_EVENT_STA_GOT_IP and
     * WIFI_EVENT_STA_DISCONNECTED so the /ws/cams URI
     * survives transient WiFi blips (the got-IP subscriber
     * is defensive today; the disconnected subscriber
     * clears the viewer slot so a vanished viewer can't
     * deadlock the slot across the reconnect). */
    err = ws_cams_install();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "ws_cams_install failed: %s — reconnects "
                      "may not clear the viewer slot cleanly",
                 esp_err_to_name(err));
    }

    ESP_LOGI(TAG,
             "STA server running on port 80 "
             "(/whoami, /capture, /ws/cams registered; "
             "wifi lifecycle subscribers live)");
    return ESP_OK;
}

void sta_server_stop(void)
{
    if (!s_sta_httpd) {
        return;
    }
    esp_err_t err = httpd_stop(s_sta_httpd);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "httpd_stop: %s", esp_err_to_name(err));
    }
    s_sta_httpd = NULL;
    ESP_LOGI(TAG, "STA server stopped");
}

bool sta_server_is_running(void)
{
    return s_sta_httpd != NULL;
}
