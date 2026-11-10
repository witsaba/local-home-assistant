/* provisioning.c — implementation home of the iot_cams WiFi
 * provisioning component.
 *
 * SCOPE — clean captive-portal HTML form for offline /
 * private-network deployments:
 *
 *   - Bring up the softAP on first boot (or whenever no
 *     credentials exist in NVS).
 *   - Run an httpd server with GET / (HTML form), POST
 *     /provision (apply Wi-Fi credentials), GET /whoami
 *     (JSON identity).
 *   - Block in provisioning_run() until the form posts
 *     valid credentials or the caller calls provisioning_stop().
 *   - Persist via esp_wifi_set_config(); the credentials
 *     land in esp_wifi's NVS storage automatically.
 *
 * We deliberately do NOT use IDF's wifi_provisioning /
 * protocomm manager for the captive-portal build. The
 * manager requires a separate phone-app install (which is
 * not available on offline deployments) and the
 * `set_httpd_handle` shared-server pattern crashed with
 * LoadProhibited in httpd_find_uri_handler on the device.
 * Direct esp_wifi + httpd is smaller, faster to verify, and
 * matches the operator flow.
 */
#include "provisioning.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "mdns.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "provisioning.h"
#include "captive_portal.h"
#include "sta_server.h"
#include "wifi_cred.h"
#include "cam_reader.h"

#include "nvs.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

/* Needed for heap_caps_get_free_size(MALLOC_CAP_SPIRAM) in the
 * periodic heartbeat. The heap component is a fundamental ESP-IDF
 * component available transitively without an explicit REQUIRES entry. */
#include "esp_heap_caps.h"

static const char *TAG = "prov";

/* Module-private state. Lives in .bss; initialization is
 * serialized through the boot thread. */
static struct {
    bool      initialized;
    bool      run_in_progress;
    char      service_name[PROV_NAME_MAX_LEN + 1];
    char      fw_version[PROV_VERSION_MAX + 1];
} s_prov;

/* Station lifecycle state.
 * - s_sta_got_ip: set true when IP_EVENT_STA_GOT_IP fires.
 *   Used by app_main to know when it's safe to shut down the softAP.
 *   Cleared on every WIFI_EVENT_STA_DISCONNECTED so the retry task
 *   stays armed across AP-side events (Defect A fix).
 * - s_sta_disconnected_pending: true when the disconnect handler has
 *   fired but reconnect has not yet succeeded. The supervisor task
 *   watches this to detect a stuck device.
 * - s_last_disconnect_reason: wifi_event_sta_disconnected_t.reason
 *   from the most recent disconnect, stored for the /health endpoint.
 * - s_reconnect_count: monotonic count of esp_wifi_connect() calls
 *   issued by the reconnect task since boot. */
static volatile bool     s_sta_got_ip               = false;
static volatile bool     s_sta_disconnected_pending = false;
static volatile uint8_t   s_last_disconnect_reason   = 0;
static volatile uint32_t  s_reconnect_count          = 0;

/* Supervisor tick counter: incremented each time provisioning_supervise_sta
 * is called with no IP. Reset to 0 when the station gets an IP.
 * Promoted to module scope so emit_heartbeat (a separate static function)
 * can access it without a static variable inside provisioning_supervise_sta. */
static volatile uint32_t s_no_ip_ticks = 0;

/* Handle for the station-reconnect task created by ensure_sta_event_handlers.
 * Valid only after the handlers have been registered at least once. */
static TaskHandle_t s_reconnect_task_handle = NULL;

/* ---------- self-recovery ----------
 *
 * The device must survive a power cut on its own: after a power cut
 * the chip may boot and run while the router is still off, so the
 * response has to be INCREMENTAL — cheapest action first, and a
 * reboot only when the cheaper steps are exhausted AND the fault is
 * provably on our side of the wire.
 *
 * The escalation ladder:
 *
 *   step 1  reconnect     owned by sta_reconnect_task (existing)
 *   step 2  re-associate  esp_wifi_disconnect() + esp_wifi_connect()
 *   step 3  reboot        esp_restart()
 *
 * The ladder is NOT entered for the ROUTER_ABSENT class. A reboot
 * cannot conjure a router, and rebooting while the router is still
 * coming up would turn a long power event into a reboot storm —
 * indistinguishable from the fault we are trying to fix. In that
 * class the device simply keeps retrying, which is exactly what
 * the reconnect task already does, forever, with no attempt ceiling.
 *
 * Every duration below is derived from the supervisor tick or from
 * uptime. The firmware has no wall-clock time source (no SNTP, no
 * RTC), so nothing here may depend on one. */

/* NVS namespace for the self-recovery state. Kept separate from
 * "wifi_cred" and from the driver's "nvs.net80211" so the reboot
 * bookkeeping can never disturb the credentials. */
#define SELFREC_NS              "selfrec"
#define SELFREC_KEY_BOOT_COUNT  "boot_count"
#define SELFREC_KEY_REBOOT_PEND "reboot_pending"
#define SELFREC_KEY_REBOOT_RSN  "reboot_reason"

/* Reason codes recorded before a requested reboot. */
#define REBOOT_RSN_LINK_FAILED 1
#define REBOOT_RSN_WEDGED      2

/* Dwell before the re-associate step (step 2). */
#define REASSOCIATE_AFTER_MS    (2U * 60U * 1000U)
/* Dwell before the reboot step (step 3) may fire. */
#define REBOOT_AFTER_MS         (15U * 60U * 1000U)
/* Wait enforced before the FIRST unhealthy reboot. Doubles per
 * consecutive unhealthy boot and is capped: this is the
 * reboot-storm guard. */
#define REBOOT_BACKOFF_BASE_MS  (15U * 60U * 1000U)
#define REBOOT_BACKOFF_MAX_MS   (4U * 60U * 60U * 1000U)
/* A liveness signal that has not moved for this long is stale. */
#define LIVENESS_STALE_MS       (10U * 60U * 1000U)
/* How often the HTTP self-probe runs. */
#define SELF_PROBE_INTERVAL_MS  (2U * 60U * 1000U)
#define SELF_PROBE_TIMEOUT_MS   3000U
/* Consecutive probe failures before the server counts as wedged. */
#define SELF_PROBE_FAIL_LIMIT   3U
/* Healthy for this long -> forget the previous unhealthy boots. */
#define HEALTHY_RESET_AFTER_MS  (10U * 60U * 1000U)

/* Boot bookkeeping and the reboot-storm guard. */
static volatile uint32_t s_boot_count        = 0;
static volatile bool     s_reboot_pending    = false;
static volatile uint8_t  s_reboot_reason     = 0;
static volatile uint8_t  s_last_reset_reason = 0;

/* Current outage classification and its age. */
static volatile uint8_t s_outage_class     = PROV_OUTAGE_NONE;
static bool     s_outage_active   = false;
static uint32_t s_outage_started_ms = 0;
static uint32_t s_outage_elapsed_ms = 0;

/* Uptime at boot. The reboot backoff is measured from here, so a
 * device that rebooted recently cannot immediately reboot again. */
static uint32_t s_boot_started_ms = 0;

/* Liveness signals. */
static uint32_t s_last_frame_count   = 0;
static uint32_t s_last_frame_seen_ms = 0;
static uint32_t s_fb_drops_seen      = 0;
static bool     s_self_probe_ok      = true;
static bool     s_probe_started      = false;
static uint32_t s_probe_fail_streak  = 0;
static uint32_t s_probe_last_ms      = 0;

/* Healthy tracking and the escalation step. */
static uint32_t s_healthy_since_ms   = 0;
static bool     s_healthy_tracking   = false;
static bool     s_healthy_reset_done = false;
static bool     s_escalated_reassoc  = false;

/* Milliseconds since boot. Derived from the FreeRTOS tick counter,
 * which wraps after ~49 days; all arithmetic below is unsigned so
 * the wrap is handled correctly. */
static uint32_t now_ms(void)
{
    return (uint32_t)((uint64_t)xTaskGetTickCount() *
                      (uint64_t)portTICK_PERIOD_MS);
}

/* Persist the self-recovery bookkeeping. Failure is logged, not
 * fatal: the worst case is a reboot-storm guard that does not
 * survive a power cut. */
static esp_err_t selfrec_save(void)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(SELFREC_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "selfrec: nvs_open: %s", esp_err_to_name(r));
        return r;
    }
    (void)nvs_set_u32(h, SELFREC_KEY_BOOT_COUNT, s_boot_count);
    (void)nvs_set_u8(h, SELFREC_KEY_REBOOT_PEND, s_reboot_pending ? 1 : 0);
    (void)nvs_set_u8(h, SELFREC_KEY_REBOOT_RSN, s_reboot_reason);
    r = nvs_commit(h);
    nvs_close(h);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "selfrec: nvs_commit: %s", esp_err_to_name(r));
    }
    return r;
}

/* Read the self-recovery bookkeeping at boot. If the previous run
 * asked for a reboot it never recovered, so the consecutive-unhealthy
 * boot counter grows and the backoff widens. A boot that was not
 * preceded by a requested reboot (a power cycle) starts the counter
 * over: the device is not in a reboot loop. */
static void selfrec_load(void)
{
    s_boot_count = 0;
    s_reboot_pending = false;
    s_reboot_reason = 0;

    nvs_handle_t h;
    esp_err_t r = nvs_open(SELFREC_NS, NVS_READONLY, &h);
    if (r != ESP_OK) {
        /* First boot, or the namespace was erased on purpose (an
         * unusable NVS is erased at boot — see provisioning_init). */
        return;
    }

    uint32_t bc = 0;
    uint8_t pend = 0;
    uint8_t rsn = 0;
    if (nvs_get_u32(h, SELFREC_KEY_BOOT_COUNT, &bc) != ESP_OK) bc = 0;
    if (nvs_get_u8(h, SELFREC_KEY_REBOOT_PEND, &pend) != ESP_OK) pend = 0;
    if (nvs_get_u8(h, SELFREC_KEY_REBOOT_RSN, &rsn) != ESP_OK) rsn = 0;
    nvs_close(h);

    if (pend) {
        s_boot_count = bc + 1;
        s_reboot_reason = rsn;
        ESP_LOGW(TAG, "selfrec: previous boot asked for a reboot and did not "
                      "recover — unhealthy boot #%u",
                 (unsigned)s_boot_count);
    }
    s_reboot_pending = false;
}

/* The wait enforced before the next unhealthy reboot. Doubles per
 * consecutive unhealthy boot, capped at four hours. */
static uint32_t reboot_backoff_ms(void)
{
    uint32_t shift = s_boot_count;
    if (shift > 4U) {
        shift = 4U;
    }
    uint32_t ms = REBOOT_BACKOFF_BASE_MS << shift;
    if (ms > REBOOT_BACKOFF_MAX_MS) {
        ms = REBOOT_BACKOFF_MAX_MS;
    }
    return ms;
}

/* Loopback GET /health. Uses a raw lwIP socket rather than
 * esp_http_client so the firmware gains no new managed dependency.
 * The socket carries send/receive timeouts, so a wedged server
 * cannot hang the supervisor task indefinitely. */
static bool http_self_probe(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return false;
    }

    struct timeval tv;
    tv.tv_sec  = (long)(SELF_PROBE_TIMEOUT_MS / 1000);
    tv.tv_usec = (long)((SELF_PROBE_TIMEOUT_MS % 1000) * 1000);
    (void)setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    (void)setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(80);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    bool ok = false;
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0) {
        static const char kReq[] =
            "GET /health HTTP/1.0\r\n"
            "Host: 127.0.0.1\r\n"
            "Connection: close\r\n\r\n";
        if (send(fd, kReq, sizeof(kReq) - 1, 0) > 0) {
            char buf[192];
            int n = recv(fd, buf, sizeof(buf) - 1, 0);
            if (n > 0) {
                buf[n] = '\0';
                ok = (strstr(buf, " 200 ") != NULL);
            }
        }
    }
    close(fd);
    return ok;
}

/* The camera path is stuck when NEITHER the captured-frame counter
 * NOR the dropped-frame counter moves. A dead sensor keeps raising
 * the drop counter, so a broken sensor is reported but never
 * triggers a reboot — a reboot cannot fix a dead sensor, and
 * rebooting for it would loop forever. A stuck capture path (the
 * task wedged, the camera mutex stuck) moves neither counter, and a
 * reboot is exactly what that needs. */
static bool camera_path_stuck(void)
{
    uint32_t frames = cam_reader_frames_captured_get();
    uint32_t drops  = cam_reader_fb_drops_get();
    uint32_t now    = now_ms();

    if (frames != s_last_frame_count) {
        s_last_frame_count = frames;
        s_last_frame_seen_ms = now;
        s_fb_drops_seen = drops;
        return false;
    }
    if (drops != s_fb_drops_seen) {
        /* Captures are being attempted and every one of them fails:
         * the sensor is broken. Reported, not rebooted. */
        s_fb_drops_seen = drops;
        s_last_frame_seen_ms = now;
        return false;
    }
    return (now - s_last_frame_seen_ms) >= LIVENESS_STALE_MS;
}

/* Run the HTTP self-probe, rate-limited. Skipped entirely while the
 * HTTP server is not listening, so a server that has not started yet
 * is never mistaken for a wedged one. */
static void run_self_probe(uint32_t now)
{
    if (!sta_server_is_running()) {
        return;
    }
    if (s_probe_started && (now - s_probe_last_ms) < SELF_PROBE_INTERVAL_MS) {
        return;
    }
    s_probe_started = true;
    s_probe_last_ms = now;

    bool ok = http_self_probe();
    if (ok) {
        s_probe_fail_streak = 0;
        s_self_probe_ok = true;
        return;
    }
    s_probe_fail_streak++;
    if (s_probe_fail_streak >= SELF_PROBE_FAIL_LIMIT) {
        if (s_self_probe_ok) {
            ESP_LOGW(TAG, "selfrec: HTTP self-probe failed %u times in a row "
                          "— the server is not answering on loopback",
                     (unsigned)s_probe_fail_streak);
        }
        s_self_probe_ok = false;
    }
}

/* Classify the current outage. This is the gate that keeps a missing
 * router from ever reaching a reboot. */
static prov_outage_class_t classify_outage(bool has_ip)
{
    if (has_ip) {
        /* The link is fine — we hold an IP. The only failure left is
         * our own process not serving. */
        if (camera_path_stuck() || !s_self_probe_ok) {
            return PROV_OUTAGE_WEDGED;
        }
        return PROV_OUTAGE_NONE;
    }

    /* No IP. The disconnect reason decides whose fault it is.
     * WIFI_REASON_NO_AP_FOUND (201) means the scan found no matching
     * AP: the router is not broadcasting. Reason 0 means no
     * disconnect event has arrived yet, which is treated the same
     * way — the safe default is not to reboot on a guess. */
    uint8_t reason = s_last_disconnect_reason;
    if (reason == 0 || reason == WIFI_REASON_NO_AP_FOUND) {
        return PROV_OUTAGE_ROUTER_ABSENT;
    }
    return PROV_OUTAGE_LINK_FAILED;
}

/* Record that the device has been healthy, and clear the
 * unhealthy-boot counter once it has stayed healthy long enough. A
 * device that had one bad week must not carry a four-hour backoff
 * for the rest of its life. */
static void note_healthy(uint32_t now)
{
    if (!s_healthy_tracking) {
        s_healthy_tracking = true;
        s_healthy_since_ms = now;
        return;
    }
    if (s_healthy_reset_done) {
        return;
    }
    if ((now - s_healthy_since_ms) < HEALTHY_RESET_AFTER_MS) {
        return;
    }
    s_healthy_reset_done = true;
    if (s_boot_count != 0 || s_reboot_pending) {
        ESP_LOGI(TAG, "selfrec: healthy for %u s — clearing the "
                      "unhealthy-boot counter and the reboot backoff",
                 (unsigned)(HEALTHY_RESET_AFTER_MS / 1000));
        s_boot_count = 0;
        s_reboot_pending = false;
        (void)selfrec_save();
    }
}

/* Step 3 of the ladder. Records the reason, persists it, then
 * resets. esp_restart() does not return. */
static void request_reboot(uint8_t reason, const char *why)
{
    ESP_LOGE(TAG, "selfrec: step 3 — REBOOT (reason=%u, class=%s, "
                  "outage=%u s, unhealthy_boots=%u). The cheaper recovery "
                  "steps are exhausted and the fault is on our side.",
             (unsigned)reason, why,
             (unsigned)(s_outage_elapsed_ms / 1000),
             (unsigned)s_boot_count);
    s_reboot_pending = true;
    s_reboot_reason = reason;
    (void)selfrec_save();
    /* Let the log reach the console before the reset. */
    vTaskDelay(pdMS_TO_TICKS(250));
    esp_restart();
}

/* Idempotent handler registration helper. Registers both
 * sta_got_ip_event_handler and sta_disconnected_event_handler
 * on the default event loop, then spawns the dedicated reconnect
 * task. Tolerates ESP_ERR_INVALID_STATE from duplicate registrations
 * so it is safe to call from both provisioning_join_ap() (the
 * already-provisioned boot path) and softap_bring_up() (the first-
 * provision path). Exactly one task and one task handle are created
 * across all calls. */
static void ensure_sta_event_handlers(void);

/* Entry point for the station-reconnect FreeRTOS task.
 * Waits on s_sta_disconnected_pending, then retries esp_wifi_connect()
 * forever with exponential backoff (2/4/8/16/30 s, capped at 30 s).
 * Resets the backoff window when s_sta_disconnected_pending clears
 * (i.e., when the station re-attaches to the home AP). The esp_event
 * task is never blocked by this task. */
static void sta_reconnect_task(void *arg);

/* FreeRTOS semaphore that provisioning_run() blocks on.
 * Signalled by the captive POST /provision handler when
 * credentials have been applied (success), or by
 * provisioning_stop() (failure). Created on demand at the
 * start of provisioning_run(). */
static SemaphoreHandle_t s_done_sema = NULL;

/* Last outcome of the captive form submission. Flipped to
 * 'true' by the handler on a successful esp_wifi_set_config,
 * left as 'false' by provisioning_stop(). */
static volatile bool s_last_prov_success = false;

static bool copy_bounded(char *dst, size_t dst_size, const char *src,
                         size_t src_len)
{
    if (dst_size == 0) return false;
    if (src_len >= dst_size) return false;
    memcpy(dst, src, src_len);
    dst[src_len] = '\0';
    return true;
}

/* Build the runtime SoftAP SSID. If service_name is empty we
 * compose "{prefix}_{MAC3}" so two devices side by side don't
 * collide. The MAC suffix is the last three bytes of the base
 * MAC. */
static void derive_service_name(char *out, size_t out_size)
{
    if (out_size == 0) return;
    if (s_prov.service_name[0] != '\0') {
        copy_bounded(out, out_size, s_prov.service_name,
                     strlen(s_prov.service_name));
        return;
    }
    const char *prefix = CONFIG_PROVISIONING_SERVICE_NAME_PREFIX;
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        memset(mac, 0, sizeof(mac));
    }
    snprintf(out, out_size, "%s_%02X%02X%02X",
             prefix, mac[3], mac[4], mac[5]);
}

/* Internal accessor for cross-module use (the captive
 * portal's /whoami handler). */
const char *internal_prov_ssid(void)
{
    static char buf[PROV_NAME_MAX_LEN + 1];
    derive_service_name(buf, sizeof(buf));
    return buf;
}

/* Internal accessor for the device name (exported for sta_server.c). */
const char *prov_device_name(void)
{
    return CONFIG_PROVISIONING_DEVICE_NAME;
}

/* On the bring-up path we currently use esp_netif + esp_wifi
 * directly without a manager; we still need the netif + mDNS
 * idioms for the SoftAP service advertising. */

/* Forward decl for the station IP-acquired event handler.
 * Registered in softap_bring_up() so it's armed before any
 * wifi connect attempt; never unregistered (the operator wants
 * to see every home-Wi-Fi join in the log). */
static void sta_got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data);

static void sta_disconnected_event_handler(void *arg, esp_event_base_t event_base,
                                            int32_t event_id, void *event_data);

/* Operator-visible "we joined the home AP" log line. Fires on
 * every successful station DHCP lease — the post-provisioning
 * attach, every subsequent reboot that re-joins, and any
 * reconnect after Wi-Fi transient drop. */
static void sta_got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base != IP_EVENT || event_id != IP_EVENT_STA_GOT_IP) return;

    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    /* Filter out IPv6 events — IP_EVENT_STA_GOT_IP fires for
     * both families on dual-stack systems; we only log IPv4
     * so the operator sees a familiar 192.168.x.x address.
     * In ESP-IDF v5.5 esp_netif_ip_info_t stores the IPv4 address
     * as esp_ip4_addr_t (4 bytes); IPv6 variants have the IPv4
     * bytes zeroed. We identify IPv4 by checking that the
     * first byte is non-zero (valid unicast IPv4 always has
     * first byte > 0; all-0 means IPv6-only). */
    uint8_t *b = (uint8_t *)&ip_info->ip.addr;
    if (b[0] == 0 && b[1] == 0 && b[2] == 0 && b[3] == 0) {
        ESP_LOGD(TAG, "sta_got_ip: ignoring IPv6/link-local event");
        return;
    }

    /* Pull the SSID the station just joined so the operator
     * doesn't have to cross-reference with the form they typed.
     * esp_wifi_get_config returns the same NVS-backed storage
     * the form POST wrote through esp_wifi_set_config. */
    wifi_config_t wifi_cfg = {0};
    char ssid[33] = "(unset)";
    if (esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg) == ESP_OK) {
        strncpy(ssid, (const char *)wifi_cfg.sta.ssid, sizeof(ssid));
        ssid[sizeof(ssid) - 1] = '\0';
    }

    /* Mark that we got an IP — used by app_main to know
     * it's safe to shut down the softAP. Also clears the
     * pending flag so the reconnect task's backoff resets. */
    s_sta_got_ip = true;
    s_sta_disconnected_pending = false;

    ESP_LOGI(TAG, "station connected to \"%s\": ip=" IPSTR
                  " netmask=" IPSTR " gw=" IPSTR,
             ssid,
             IP2STR(&ip_info->ip),
             IP2STR(&ip_info->netmask),
             IP2STR(&ip_info->gw));
}

/* Fires on WIFI_EVENT_STA_DISCONNECTED. O(1): records the
 * reason, clears s_sta_got_ip, sets the pending flag, and returns.
 * The esp_event task is never blocked. All retrying is owned by
 * sta_reconnect_task, which runs on its own FreeRTOS task at
 * higher priority than idle.
 *
 * Defect A fix: the old handler early-returned when s_sta_got_ip
 * was already true (a one-way latch never cleared on boot), so
 * transient disconnects after the first DHCP lease were never
 * retried. Clearing s_sta_got_ip here removes that latch. */
static void sta_disconnected_event_handler(void *arg, esp_event_base_t event_base,
                                            int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base != WIFI_EVENT || event_id != WIFI_EVENT_STA_DISCONNECTED) {
        return;
    }

    wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
    s_last_disconnect_reason = disc->reason;

    ESP_LOGW(TAG, "station disconnected from home AP: reason=%d (%s)",
             disc->reason,
             esp_err_to_name(disc->reason));

    /* Clear the IP-acquired flag so both the reconnect task and
     * the supervisor's no-IP watchdog stay armed. */
    s_sta_got_ip = false;
    s_sta_disconnected_pending = true;

    /* Poke the reconnect task so it wakes immediately rather than
     * waiting for its next backoff sleep to expire. NULL handle is
     * tolerated (race: disconnect fires before the task was created
     * in ensure_sta_event_handlers — the task checks the pending flag
     * before blocking so the missed poke is harmless). */
    if (s_reconnect_task_handle != NULL) {
        (void)xTaskNotifyGive(s_reconnect_task_handle);
    }
}

/* Idempotent helper: registers sta_got_ip_event_handler and
 * sta_disconnected_event_handler on the default event loop,
 * then spawns the dedicated reconnect task (exactly once).
 * Called from both softap_bring_up() (first-provision path) and
 * provisioning_join_ap() (already-provisioned boot path), so
 * both paths arm the handlers and both get reconnect coverage.
 * Duplicate esp_event_handler_register calls return
 * ESP_ERR_INVALID_STATE and are tolerated. */
static void ensure_sta_event_handlers(void)
{
    /* Register the IP-acquired handler. Tolerates ESP_ERR_INVALID_STATE
     * from re-registration on already-provisioned boots. */
    (void)esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      sta_got_ip_event_handler, NULL);
    (void)esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                                      sta_disconnected_event_handler, NULL);

    /* Spawn the reconnect task exactly once. The static handle
     * stays NULL until the first call, and stays non-NULL
     * on all subsequent calls (xTaskCreate is idempotent in
     * effect — only one task is created). */
    if (s_reconnect_task_handle == NULL) {
        BaseType_t ok = xTaskCreate(
            sta_reconnect_task,         /* entry */
            "sta_reconnect",             /* name */
            4096,                        /* stack bytes */
            NULL,                        /* args */
            (tskIDLE_PRIORITY + 4),      /* priority: above idle, below event */
            &s_reconnect_task_handle);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "ensure_sta_event_handlers: xTaskCreate(sta_reconnect) failed");
            s_reconnect_task_handle = NULL;
        } else {
            ESP_LOGI(TAG, "ensure_sta_event_handlers: sta_reconnect task spawned");
        }
    }
}

/* sta_reconnect_task — dedicated FreeRTOS task owning all
 * esp_wifi_connect() retrying. Not registered as an esp_event
 * handler so it runs on its own stack and never blocks the
 * esp_event dispatcher.
 *
 * Control flow:
 *   1. If the pending flag is already set (race: disconnect fired
 *      before this task was created), handle it immediately — no
 *      notification exists to clear so the flag alone is the signal.
 *   2. Block on ulTaskNotifyTake(portMAX_DELAY) — zero CPU cost
 *      while healthy. The disconnect handler pokes this task via
 *      xTaskNotifyGive(), which unblocks it and clears the notification
 *      value in one step.
 *   3. While the pending flag is set, retry esp_wifi_connect() and
 *      sleep the exponential backoff between each attempt:
 *      2 → 4 → 8 → 16 → 30 s, capped at 30 s, repeating forever.
 *   4. When the pending flag clears (sta_got_ip_event_handler set
 *      it false and poked us), reset the backoff and go back to
 *      step 2.
 *
 * The retry cadence is owned exclusively by this task. The
 * supervisor (provisioning_supervise_sta) is a backstop watchdog
 * only — it does NOT call esp_wifi_connect() for retry pacing. */
static void sta_reconnect_task(void *arg)
{
    (void)arg;
    uint32_t backoff_ms = 2000;

    while (true) {
        /* Defect 1+2 fix: check the pending flag BEFORE blocking.
         * Handles the spawn race (disconnect fires before the task
         * exists) and the case where the flag was set between the
         * last retry and this loop iteration. */
        if (!s_sta_disconnected_pending) {
            /* Block until the disconnect handler pokes us.
             * portMAX_DELAY means infinite block — zero CPU cost
             * while the station is healthy. */
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            /* Woken: re-check the flag. If it was already cleared
             * by sta_got_ip_event_handler (re-attached between
             * disconnect and our wake), go back to blocking. */
            if (!s_sta_disconnected_pending) {
                continue;
            }
        }

        /* Pending flag is set — the station has not recovered.
         * Attempt a reconnect immediately (no extra delay on the
         * first attempt after a fresh poke). */
        /* No backoff label here on purpose. This attempt did not
         * sleep — it fired immediately on the wake — and backoff_ms
         * is about to double, so printing it would misreport the
         * interval to the NEXT attempt (4 s, not 2 s). The log below
         * is consistent: "backoff=Xms" always means "slept X ms
         * before this attempt". */
        ESP_LOGI(TAG, "reconnect: esp_wifi_connect (attempt after disconnect)");
        s_reconnect_count++;

        esp_err_t r = esp_wifi_connect();
        if (r != ESP_OK && r != ESP_ERR_WIFI_CONN) {
            ESP_LOGE(TAG, "reconnect: esp_wifi_connect: %s",
                     esp_err_to_name(r));
        }

        /* Advance backoff: 2→4→8→16→30, capped at 30 s. */
        backoff_ms = (backoff_ms < 30000U) ? (backoff_ms * 2U) : 30000U;

        /* Sleep the backoff interval, re-checking the pending flag
         * after each sleep so we exit immediately when the station
         * re-attaches rather than waiting for the full backoff. */
        while (s_sta_disconnected_pending) {
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            if (!s_sta_disconnected_pending) {
                break;
            }

            /* Still pending — attempt again and double the backoff. */
            ESP_LOGI(TAG, "reconnect: esp_wifi_connect (backoff=%ums)",
                     (unsigned)backoff_ms);
            s_reconnect_count++;

            r = esp_wifi_connect();
            if (r != ESP_OK && r != ESP_ERR_WIFI_CONN) {
                ESP_LOGE(TAG, "reconnect: esp_wifi_connect: %s",
                         esp_err_to_name(r));
            }

            backoff_ms = (backoff_ms < 30000U) ? (backoff_ms * 2U) : 30000U;
        }

        /* Pending flag cleared by sta_got_ip_event_handler.
         * Reset backoff and return to the blocking wait above. */
        backoff_ms = 2000;
    }
}

static esp_err_t softap_bring_up(void)
{
    esp_err_t r;

    r = esp_netif_init();
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "softap_bring_up: esp_netif_init: %s",
                 esp_err_to_name(r));
        return r;
    }

    r = esp_event_loop_create_default();
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "softap_bring_up: esp_event_loop_create_default: %s",
                 esp_err_to_name(r));
        return r;
    }

    /* Register the permanent event handlers (IP acquired + station
     * disconnected) and spawn the reconnect task. Idempotent —
     * safe on repeat softap_bring_up() calls. */
    ensure_sta_event_handlers();

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    if (ap_netif == NULL) {
        ESP_LOGE(TAG, "softap_bring_up: default wifi ap netif NULL");
        return ESP_FAIL;
    }

    /* CRITICAL: also create the station netif. Without it, the LwIP
     * stack has no DHCP client for the station interface, so
     * IP_EVENT_STA_GOT_IP never fires after the device joins the
     * home AP — confirmed on device (wifi:connected with <ssid>
     * logged but no IP event for 30 s, then timeout). APSTA mode
     * still works at the wifi-driver level for L2 association,
     * but the L3 / DHCP path requires this netif. */
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == NULL) {
        ESP_LOGE(TAG, "softap_bring_up: default wifi sta netif NULL");
        return ESP_FAIL;
    }

    /* Set the station netif as default — once the softAP is torn
     * down (post-provisioning) the station is the only interface
     * and default routing/hosted services should land there. */
    esp_netif_set_default_netif(sta_netif);

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    r = esp_wifi_init(&wifi_init_cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "softap_bring_up: esp_wifi_init: %s",
                 esp_err_to_name(r));
        return r;
    }

    /* Mode = APSTA so we can complete the softAP advertise
     * and (later) attach to the user's station. The captive
     * portal flow drops the AP after a successful POST. */
    r = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "softap_bring_up: esp_wifi_set_mode(APSTA): %s",
                 esp_err_to_name(r));
        return r;
    }
    r = esp_wifi_start();
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "softap_bring_up: esp_wifi_start: %s",
                 esp_err_to_name(r));
        return r;
    }

    /* Disable STA power save so ARP/ICMP/DHCP frames are not
     * AP-buffered during DTIM sleep. An always-reachable LAN
     * device must not sleep. */
    (void)esp_wifi_set_ps(WIFI_PS_NONE);

    r = mdns_init();
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "softap_bring_up: mdns_init: %s",
                 esp_err_to_name(r));
        return r;
    }
    char hostname[PROV_NAME_MAX_LEN + 1] = {0};
    derive_service_name(hostname, sizeof(hostname));
    mdns_hostname_set(hostname);

    return ESP_OK;
}

esp_err_t provisioning_init(const provisioning_config_t *cfg,
                            const provisioning_app_info_t *app_info)
{
    if (!cfg) {
        ESP_LOGE(TAG, "init: cfg NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (s_prov.initialized) {
        ESP_LOGE(TAG, "init: already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Reset reason: lets an operator tell a self-inflicted software
     * reset (our own recovery reboot) from a power-loss or watchdog
     * reset, without a serial console being optional. */
    s_last_reset_reason = (uint8_t)esp_reset_reason();
    s_boot_started_ms = now_ms();
    s_last_frame_seen_ms = s_boot_started_ms;
    ESP_LOGI(TAG, "init: reset reason = %d", (int)s_last_reset_reason);

    /* Idempotent NVS init — esp_wifi_set_config() requires
     * the wifi NVS storage to be ready.
     *
     * An unusable NVS must NEVER leave the device inert. The old
     * behaviour returned the error here, which made app_main bail:
     * no camera, no SoftAP, no HTTP server — a powered but dead
     * chip that only a power cycle could revive, and a reboot
     * cannot fix it either, because the corruption is in flash.
     *
     * Instead: erase and re-initialise, and carry on. The device
     * then comes up with no credentials and starts its SoftAP
     * provisioning portal, where the operator can re-provision it.
     * A device that visibly asks to be provisioned is recoverable;
     * a device that is silently inert is not. */
    {
        esp_err_t nv = nvs_flash_init();
        if (nv != ESP_OK && nv != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "init: nvs_flash_init: %s", esp_err_to_name(nv));
            ESP_LOGW(TAG, "init: NVS is unusable — erasing it. The device "
                          "will come up in SoftAP provisioning mode so it "
                          "can be re-provisioned.");
            esp_err_t er = nvs_flash_erase();
            if (er != ESP_OK) {
                ESP_LOGE(TAG, "init: nvs_flash_erase: %s",
                         esp_err_to_name(er));
            } else {
                nv = nvs_flash_init();
                if (nv != ESP_OK && nv != ESP_ERR_INVALID_STATE) {
                    ESP_LOGE(TAG, "init: nvs_flash_init (after erase): %s",
                             esp_err_to_name(nv));
                }
            }
        }
        if (nv != ESP_OK && nv != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "init: NVS still unusable — continuing anyway. "
                          "The device must not be left inert; it will come "
                          "up in SoftAP provisioning mode.");
        }
    }

    /* Load the self-recovery bookkeeping: the consecutive unhealthy
     * boot counter and the pending-reboot marker. Both must survive a
     * power cycle, otherwise the reboot-storm guard resets every time
     * the power flickers. */
    selfrec_load();
    (void)selfrec_save();

    memset(&s_prov, 0, sizeof(s_prov));

    if (cfg->pop_len > 0) {
        /* pop is accepted for API compatibility but the
         * captive flow does not surface a PoP prompt; the WPA2
         * passphrase is the operator-visible security gate. */
        ESP_LOGW(TAG, "init: captive flow ignores PoP arg (use softAP pass)");
    }

    if (cfg->service_name[0] != '\0') {
        if (!copy_bounded(s_prov.service_name,
                          sizeof(s_prov.service_name),
                          cfg->service_name,
                          strlen(cfg->service_name))) {
            ESP_LOGE(TAG, "init: service_name exceeds %d bytes",
                     (int)sizeof(s_prov.service_name) - 1);
            return ESP_ERR_INVALID_ARG;
        }
    }
    (void)cfg->security;

    /* Store the application firmware version for later exposure on
     * /whoami. NULL app_info is tolerated (fw_version stays empty
     * string, which is safe for snprintf). */
    if (app_info && app_info->fw_version) {
        copy_bounded(s_prov.fw_version, sizeof(s_prov.fw_version),
                     app_info->fw_version, strlen(app_info->fw_version));
    }
    s_prov.initialized = true;
    ESP_LOGI(TAG, "init ok (captive-portal mode)");
    return ESP_OK;
}

bool provisioning_is_provisioned(void)
{
    if (!s_prov.initialized) {
        return false;
    }

    /* Read from our own NVS namespace "wifi_cred" (wifi_cred.c).
     * This mirrors the esp32-cam-surveillance config.c pattern:
     * we manage credential persistence explicitly rather than
     * relying on the wifi driver's internal "nvs.net80211" storage.
     * The wifi driver path was not surviving power cycles reliably.
     * wifi_cred_is_saved() returns true iff "ssid" key is present
     * and non-empty in the "wifi_cred" namespace. */
    return wifi_cred_is_saved();
}

esp_err_t provisioning_run(void)
{
    if (!s_prov.initialized) {
        ESP_LOGE(TAG, "run: not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_prov.run_in_progress) {
        ESP_LOGW(TAG, "run: already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    /* Fast-path: credentials already present, just confirm and
     * let the application continue. The actual station attach
     * happens in the post-provisioning application body, not
     * here. */
    if (provisioning_is_provisioned()) {
        ESP_LOGI(TAG, "run: credentials present in NVS — no softAP");
        return ESP_OK;
    }

    s_prov.run_in_progress = true;
    s_last_prov_success = false;

    esp_err_t r = softap_bring_up();
    if (r != ESP_OK) {
        s_prov.run_in_progress = false;
        return r;
    }

    /* Bring up the captive httpd server (port 80). */
    r = captive_portal_bring_up();
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "run: captive_portal_bring_up: %s",
                 esp_err_to_name(r));
        s_prov.run_in_progress = false;
        return r;
    }

    char ssid[PROV_NAME_MAX_LEN + 1] = {0};
    derive_service_name(ssid, sizeof(ssid));
    ESP_LOGI(TAG, "captive portal live on %s (WPA2 passphrase from Kconfig)",
             ssid);

    /* Block on the semaphore until the captive form posts
     * credentials (signals s_done_sema with success) or the
     * caller signals via provisioning_stop(). */
    s_done_sema = xSemaphoreCreateBinary();
    if (s_done_sema == NULL) {
        captive_portal_tear_down();
        s_prov.run_in_progress = false;
        return ESP_ERR_NO_MEM;
    }

    if (xSemaphoreTake(s_done_sema, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "run: semaphore wait failed");
        captive_portal_tear_down();
        vSemaphoreDelete(s_done_sema);
        s_done_sema = NULL;
        s_prov.run_in_progress = false;
        return ESP_FAIL;
    }

    vSemaphoreDelete(s_done_sema);
    s_done_sema = NULL;

    /* Tear down the httpd. After this, the device keeps the
     * softAP interface alive for a brief moment so the
     * operator's browser still has time to receive the JSON
     * success response before disconnecting. The post-
     * provisioning application body drops the AP if/when it
     * wants to. */
    captive_portal_tear_down();

    s_prov.run_in_progress = false;

    if (!s_last_prov_success) {
        ESP_LOGW(TAG, "run: provisioning ended without success");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "run: provisioning complete — credentials persisted to NVS");
    return ESP_OK;
}

void provisioning_stop(void)
{
    ESP_LOGI(TAG, "stop: signal");
    if (s_done_sema) {
        /* success flag stays false → run() returns ESP_FAIL
         * so the caller knows it was force-stopped, not
         * completed via the form. */
        xSemaphoreGive(s_done_sema);
    }
}

esp_err_t provisioning_reset_credentials(void)
{
    ESP_LOGI(TAG, "reset: clearing station config");
    /* esp_wifi_restore() wipes the persisted station config
     * keys in NVS. After this call provisioning_is_provisioned()
     * returns false and the next provisioning_run() brings up
     * the softAP. */
    esp_err_t r = esp_wifi_restore();
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "reset: esp_wifi_restore: %s", esp_err_to_name(r));
        return r;
    }
    ESP_LOGI(TAG, "reset: credentials cleared");
    return ESP_OK;
}

esp_err_t provisioning_join_ap(void)
{
    if (!s_prov.initialized) {
        ESP_LOGE(TAG, "join_ap: not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (!provisioning_is_provisioned()) {
        ESP_LOGE(TAG, "join_ap: no credentials in NVS");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t r;

    /* Idempotent netif + event loop init. ESP_ERR_INVALID_STATE
     * is acceptable — means already initialized. */
    r = esp_netif_init();
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "join_ap: esp_netif_init: %s", esp_err_to_name(r));
        return r;
    }
    r = esp_event_loop_create_default();
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "join_ap: esp_event_loop_create_default: %s",
                 esp_err_to_name(r));
        return r;
    }

    /* Station netif (not AP — we are STA-only here). */
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == NULL) {
        ESP_LOGE(TAG, "join_ap: create_default_wifi_sta returned NULL");
        return ESP_FAIL;
    }
    esp_netif_set_default_netif(sta_netif);

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    r = esp_wifi_init(&wifi_init_cfg);
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "join_ap: esp_wifi_init: %s", esp_err_to_name(r));
        return r;
    }

    /* STA-only mode — no softAP. Used on already-provisioned
     * boots where the operator never interacts with the device. */
    r = esp_wifi_set_mode(WIFI_MODE_STA);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "join_ap: esp_wifi_set_mode(STA): %s",
                 esp_err_to_name(r));
        return r;
    }
    r = esp_wifi_start();
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "join_ap: esp_wifi_start: %s", esp_err_to_name(r));
        return r;
    }

    /* Disable STA power save so ARP/ICMP/DHCP frames are not
     * AP-buffered during DTIM sleep. An always-reachable LAN
     * device must not sleep. */
    (void)esp_wifi_set_ps(WIFI_PS_NONE);

    /* Register handlers and spawn the reconnect task.
     * Idempotent — safe on repeat join_ap() calls. */
    ensure_sta_event_handlers();

    /* Load credentials from our own NVS namespace "wifi_cred" and
     * pass them to esp_wifi_set_config(). This mirrors the
     * esp32-cam-surveillance wifi_init(cfg) pattern:
     * wifi_init reads from its config_t (backed by NVS "config")
     * and explicitly calls esp_wifi_set_config(). The driver does NOT
     * auto-load stored credentials on esp_wifi_start() — it needs
     * an explicit esp_wifi_set_config() call to know what to use. */
    {
        char ssid[33] = {0};
        char password[65] = {0};
        esp_err_t lr = wifi_cred_load(ssid, sizeof(ssid),
                                      password, sizeof(password));
        if (lr != ESP_OK) {
            ESP_LOGE(TAG, "join_ap: wifi_cred_load: %s",
                     esp_err_to_name(lr));
            return lr;
        }

        wifi_config_t wifi_cfg = {0};
        strncpy((char *)wifi_cfg.sta.ssid, ssid, sizeof(wifi_cfg.sta.ssid) - 1);
        strncpy((char *)wifi_cfg.sta.password, password,
                sizeof(wifi_cfg.sta.password) - 1);
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        r = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "join_ap: esp_wifi_set_config: %s",
                     esp_err_to_name(r));
            return r;
        }
        ESP_LOGI(TAG, "join_ap: loaded ssid='%s' from NVS", ssid);
    }

    /* Trigger the station attach. */
    r = esp_wifi_connect();
    if (r != ESP_OK && r != ESP_ERR_WIFI_CONN) {
        ESP_LOGE(TAG, "join_ap: esp_wifi_connect: %s",
                 esp_err_to_name(r));
        return r;
    }

    ESP_LOGI(TAG, "join_ap: station connecting (credentials from NVS)");
    return ESP_OK;
}

/* Forward declaration for the periodic heartbeat emitter.
 * Defined after provisioning_supervise_sta to keep the public API
 * function first in the translation unit. */
static void emit_heartbeat(bool has_ip);

esp_err_t provisioning_supervise_sta(void)
{
    uint32_t now = now_ms();

    /* Liveness self-probe. Rate-limited internally and skipped while
     * the HTTP server is not listening. */
    run_self_probe(now);

    /* Classify the outage BEFORE deciding anything. This is the gate
     * that keeps a missing router from ever reaching a reboot. */
    prov_outage_class_t cls = classify_outage(s_sta_got_ip);
    s_outage_class = (uint8_t)cls;

    if (cls == PROV_OUTAGE_NONE) {
        s_no_ip_ticks = 0;
        s_outage_active = false;
        s_outage_elapsed_ms = 0;
        s_escalated_reassoc = false;
        note_healthy(now);
        emit_heartbeat(s_sta_got_ip);
        return ESP_OK;
    }

    if (!s_outage_active) {
        s_outage_active = true;
        s_outage_started_ms = now;
        s_outage_elapsed_ms = 0;
        ESP_LOGW(TAG, "supervise_sta: outage detected: class=%s",
                 provisioning_outage_class_name(cls));
    }
    s_outage_elapsed_ms = now - s_outage_started_ms;
    s_healthy_tracking = false;
    s_healthy_reset_done = false;

    if (!s_sta_got_ip) {
        s_no_ip_ticks++;
    }

    emit_heartbeat(s_sta_got_ip);

    /* Backstop: force one reconnect after 4 consecutive no-IP ticks
     * (120 s), in case the reconnect task itself is stuck. The retry
     * cadence is owned by sta_reconnect_task. */
    if (!s_sta_got_ip && s_no_ip_ticks >= 4) {
        ESP_LOGW(TAG, "supervise_sta: no IP for %u ticks — forcing esp_wifi_connect()",
                 (unsigned)s_no_ip_ticks);
        (void)esp_wifi_connect();
    }

    /* --- the escalation ladder --- */

    if (cls == PROV_OUTAGE_ROUTER_ABSENT) {
        /* The router is not broadcasting. A reboot cannot make a
         * router come alive: rebooting here would waste a boot
         * cycle and, under a long power event, produce a reboot
         * storm that looks exactly like the fault we are fixing.
         * Step 1 of the ladder is all that applies — the reconnect
         * task keeps trying, without an attempt ceiling, and the
         * device attaches the moment the router is back. */
        return ESP_OK;
    }

    if (cls == PROV_OUTAGE_LINK_FAILED) {
        /* Step 2 — re-associate. Clears stale driver/BSSID state
         * without losing the process. Only for the link class: a
         * wedged server cannot be repaired by dropping the IP, and
         * doing so would trade a wedged device for an offline one. */
        if (!s_escalated_reassoc &&
            s_outage_elapsed_ms >= REASSOCIATE_AFTER_MS) {
            s_escalated_reassoc = true;
            ESP_LOGW(TAG, "selfrec: step 2 — re-associating after %u s "
                          "(the reconnect task has not recovered the link)",
                     (unsigned)(s_outage_elapsed_ms / 1000));
            (void)esp_wifi_disconnect();
            vTaskDelay(pdMS_TO_TICKS(1000));
            (void)esp_wifi_connect();
        }
    }

    /* Step 3 — reboot. Reached only when the outage has lasted long
     * enough AND this boot has already lasted at least the persisted
     * backoff, so consecutive unhealthy boots space themselves out
     * instead of looping. */
    uint32_t backoff = reboot_backoff_ms();
    if (s_outage_elapsed_ms >= REBOOT_AFTER_MS &&
        (now - s_boot_started_ms) >= backoff) {
        request_reboot(cls == PROV_OUTAGE_WEDGED ? REBOOT_RSN_WEDGED
                                                 : REBOOT_RSN_LINK_FAILED,
                       provisioning_outage_class_name(cls));
    }

    return ESP_OK;
}

/* Periodic heartbeat emitter. When the station has an IP, logs at
 * most once every 10 ticks (~5 min). When it has no IP, logs
 * every tick so the operator can track the watchdog cadence.
 * Includes the same live diagnostics as /health (uptime, IP, RSSI,
 * free heap, min-free heap, free PSRAM, ws_viewer) so the two
 * are comparable by eye. Graceful fallbacks (0.0.0.0, -1) are
 * used when the STA is not connected rather than omitting fields. */
static void emit_heartbeat(bool has_ip)
{
    /* Rate-limit the healthy heartbeat: emit at most once every
     * 10 ticks (10 × 30 s = 5 min) to avoid log flooding while
     * still providing a clear liveness signal. */
    static uint32_t s_last_healthy_tick = 0;
    if (has_ip) {
        if (s_no_ip_ticks - s_last_healthy_tick < 10) {
            return;
        }
        s_last_healthy_tick = s_no_ip_ticks;
    }

    /* Uptime in ms. */
    uint32_t uptime_ms = (uint32_t)(
        (uint64_t)xTaskGetTickCount() * (uint64_t)portTICK_PERIOD_MS);

    /* IP address. */
    char ip_str[16] = "0.0.0.0";
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK) {
            snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
        }
    }

    /* RSSI. */
    int rssi = -1;
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        rssi = (int)ap_info.rssi;
    }

    uint32_t free_heap      = esp_get_free_heap_size();
    uint32_t min_free_heap  = esp_get_minimum_free_heap_size();
    uint32_t free_psram     = 0;
#if CONFIG_ESP32_SPIRAM_SUPPORT || CONFIG_ESP_SPIRAM
    free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
#endif

    extern int ws_cams_viewer_fd_get(void);
    int ws_fd = ws_cams_viewer_fd_get();

    ESP_LOGI(TAG,
             "supervise_sta: heartbeat: uptime_ms=%lu ip=%s rssi=%d "
             "free_heap=%lu min_free_heap=%lu free_psram=%lu "
             "reconnect_count=%lu ws_viewer=%s "
             "outage=%s outage_ms=%lu unhealthy_boots=%lu backoff_ms=%lu "
             "self_probe=%s frames=%lu drops=%lu reset_reason=%u",
             (unsigned long)uptime_ms,
             ip_str,
             rssi,
             (unsigned long)free_heap,
             (unsigned long)min_free_heap,
             (unsigned long)free_psram,
             (unsigned long)s_reconnect_count,
             (ws_fd >= 0) ? "yes" : "no",
             provisioning_outage_class_name(
                 (prov_outage_class_t)s_outage_class),
             (unsigned long)s_outage_elapsed_ms,
             (unsigned long)s_boot_count,
             (unsigned long)reboot_backoff_ms(),
             s_self_probe_ok ? "ok" : "fail",
             (unsigned long)cam_reader_frames_captured_get(),
             (unsigned long)cam_reader_fb_drops_get(),
             (unsigned)s_last_reset_reason);
}

uint32_t provisioning_reconnect_count(void)
{
    return s_reconnect_count;
}

uint8_t provisioning_last_disconnect_reason(void)
{
    return s_last_disconnect_reason;
}

const char *prov_fw_version(void)
{
    return s_prov.fw_version;
}

esp_err_t provisioning_start_sta_server(void)
{
    return sta_server_start();
}

/* Expose the IP-acquired state to app_main.
 * s_sta_got_ip is set true by sta_got_ip_event_handler()
 * when IP_EVENT_STA_GOT_IP fires, and reset by
 * provisioning_reset_sta_state(). Used by app_main to know
 * when it is safe to shut down the softAP. */
bool provisioning_sta_has_ip(void)
{
    return s_sta_got_ip;
}

/* Reset the station state. Called before starting a fresh
 * provisioning session so the retry/disconnect state from a
 * previous session does not bleed into the new one. */
void provisioning_reset_sta_state(void)
{
    s_sta_got_ip = false;
    s_sta_disconnected_pending = false;
    s_last_disconnect_reason = 0;
    s_reconnect_count = 0;
}

/* Called by the captive /provision handler when the operator's
 * form submission is accepted. Does the wifi config write and
 * signals the semaphore so provisioning_run() resumes. */
esp_err_t provisioning_apply_captive_form(const char *ssid, const char *password)
{
    if (!ssid || !password) return ESP_ERR_INVALID_ARG;
    if (ssid[0] == '\0') return ESP_ERR_INVALID_ARG;

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid, ssid, sizeof(wifi_cfg.sta.ssid));
    strncpy((char *)wifi_cfg.sta.password, password,
            sizeof(wifi_cfg.sta.password));
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    /* NVS-SAVE BOUNDARY: IDF's wifi driver owns the actual nvs_set_blob
     * under namespace "nvs.net80211" once we hand it a wifi_config_t.
     * The keys it writes are sta.ssid[0], sta.pswd[0], sta.bssid[0], etc.
     * (multi-blob indexed) — verified in espressif/esp-idf #14554.
     * Log the inputs and the driver's return so the persist path is
     * observable from monitor. Do NOT log the password in plaintext. */
    ESP_LOGI(TAG, "apply_form: NVS-save start: ssid='%s' ssid_len=%u "
                   "password_len=%u authmode=WPA2_PSK",
             ssid, (unsigned)strlen(ssid), (unsigned)strlen(password));

    esp_err_t r = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "apply_form: esp_wifi_set_config: %s",
                 esp_err_to_name(r));
        return r;
    }
    /* CRITICAL: also save to our own NVS namespace "wifi_cred" (wifi_cred.c).
     * This mirrors the esp32-cam-surveillance config_save() pattern.
     * The wifi driver's internal "nvs.net80211" storage was NOT surviving
     * power cycles reliably on this device. Explicit manual persistence is
     * the only path that works. */
    {
        esp_err_t sv = wifi_cred_save(ssid, password);
        if (sv != ESP_OK) {
            ESP_LOGE(TAG, "apply_form: wifi_cred_save failed: %s",
                     esp_err_to_name(sv));
            return sv;
        }
        ESP_LOGI(TAG, "apply_form: wifi_cred_save OK");
    }

    ESP_LOGI(TAG, "apply_form: NVS-save OK: esp_wifi_set_config returned ESP_OK");

    /* Kick off the station connect. In APSTA mode this only
     * affects the STA side; the softAP stays up so the
     * operator's browser gets the JSON success response before
     * they disconnect. The IP_EVENT_STA_GOT_IP handler logs the
     * assigned address. */
    esp_err_t cr = esp_wifi_connect();
    if (cr != ESP_OK && cr != ESP_ERR_WIFI_CONN) {
        ESP_LOGE(TAG, "apply_form: esp_wifi_connect: %s",
                 esp_err_to_name(cr));
        return cr;
    }

    s_last_prov_success = true;
    ESP_LOGI(TAG, "apply_form: ssid=%s password=(redacted) — persisted",
             ssid);

    /* Match esp32-cam-surveillance softap_handlers.c pattern:
     * restart immediately so the wifi driver reads the persisted
     * credentials from NVS in a clean boot. The alternative
     * (stay running, switch to STA-only) was causing credentials
     * to not survive the power cycle — the wifi driver's NVS
     * write from esp_wifi_set_config() was not surviving the
     * next boot reliably when the device continued running. */
    ESP_LOGI(TAG, "apply_form: restarting in 100ms...");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();

    /* Unreachable — esp_restart() does not return on device. */
    return ESP_OK;
}

/* ---------- self-recovery public accessors ---------- */

prov_outage_class_t provisioning_outage_class(void)
{
    return (prov_outage_class_t)s_outage_class;
}

const char *provisioning_outage_class_name(prov_outage_class_t cls)
{
    switch (cls) {
        case PROV_OUTAGE_NONE:          return "none";
        case PROV_OUTAGE_ROUTER_ABSENT: return "router_absent";
        case PROV_OUTAGE_LINK_FAILED:   return "link_failed";
        case PROV_OUTAGE_WEDGED:        return "wedged";
        default:                        return "unknown";
    }
}

uint32_t provisioning_outage_elapsed_ms(void)
{
    return s_outage_elapsed_ms;
}

uint32_t provisioning_boot_count(void)
{
    return s_boot_count;
}

uint32_t provisioning_reboot_backoff_ms(void)
{
    return reboot_backoff_ms();
}

uint8_t provisioning_last_reset_reason(void)
{
    return s_last_reset_reason;
}

uint8_t provisioning_reboot_reason(void)
{
    return s_reboot_reason;
}

bool provisioning_self_probe_ok(void)
{
    return s_self_probe_ok;
}

uint32_t provisioning_frames_captured(void)
{
    return cam_reader_frames_captured_get();
}

uint32_t provisioning_frames_dropped(void)
{
    return cam_reader_fb_drops_get();
}
