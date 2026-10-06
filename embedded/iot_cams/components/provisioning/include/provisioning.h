/* provisioning.h — public surface for the iot_cams WiFi provisioning
 * component.
 *
 * The component wraps ESP-IDF v5.5 `wifi_provisioning` (SoftAP scheme,
 * security 1 by default) and hides every IDF-specific include from
 * the rest of the firmware. Application code keeps three lines:
 *
 *     provisioning_init(&cfg, &info);   // once per boot
 *     if (!provisioning_is_provisioned()) provisioning_run();
 *
 * Internals — protocomm, mDNS, esp_wifi, the SoftAP HTTP server — are
 * not part of the public surface. If you find yourself wanting to
 * include them from app_main, you are reaching past the package.
 *
 * PERSISTENCE — the package does NOT carry a parallel NVS
 * namespace. The IDF `wifi_provisioning` manager writes the user-
 * submitted ssid + password to NVS via `esp_wifi_set_config()` /
 * esp_wifi's own NVS storage layer during the apply_config
 * flow. We rely on that built-in persistence for the credentials
 * the user cares about (so the device joins the same AP after a
 * reboot without re-provisioning). `provisioning_is_provisioned()`
 * reads back through `wifi_prov_mgr_is_provisioned()` against
 * the same storage.
 *
 * PoP and service_name are input to the provisioning session,
 * not output. They are Kconfig-fixed at build time (or supplied
 * per-boot via `provisioning_config_t`); runtime NVS persistence
 * is not required for either. `provisioning_reset_credentials()`
 * is the single reset path and routes through
 * `wifi_prov_mgr_reset_provisioning()`.
 *
 * THREADING — provisioning_init() must be called from app_main
 * (single thread) before provisioning_run(). provisioning_run()
 * blocks. provisioning_stop() may be called from any task to
 * trigger teardown; wifi_prov_mgr_stop_provisioning schedules the
 * stop on its own task and emits WIFI_PROV_END.
 *
 * LIFTING — to publish this component as its own upstream / repo
 * later, move the component directory, register the new URL via
 * the consumer's `idf_component.yml`, and replace the consumer's
 * `path:` dep with the URL. Nothing in this header changes.
 */
#ifndef IOT_CAMS_PROVISIONING_H
#define IOT_CAMS_PROVISIONING_H

#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROV_POP_MAX_LEN    32   /* max bytes of proof-of-possession */
#define PROV_NAME_MAX_LEN   32   /* max bytes of softAP SSID */
#define PROV_VERSION_MAX    16   /* max bytes exposed as fw_version */

typedef enum {
    PROV_SECURITY_0 = 0,           /* plain text — debug only */
    PROV_SECURITY_1 = 1,           /* X25519 + PoP + AES-CTR — shipping default */
} provisioning_security_t;

typedef struct {
    /* Skip the is_provisioned() early-out and force provisioning
     * to run. Use this from the factory-reset flow. */
    bool     force_provisioning;

    /* Security level. PROV_SECURITY_1 strongly recommended. */
    provisioning_security_t security;

    /* Proof-of-possession (PoP), used with SECURITY_1. Must be
     * null-terminated. Leave empty to fall back to
     * CONFIG_PROVISIONING_POP at runtime. */
    char     pop[PROV_POP_MAX_LEN + 1];
    size_t   pop_len;

    /* SoftAP SSID. Must be null-terminated. If empty, the
     * component constructs "{CONFIG_PROVISIONING_SERVICE_NAME_PREFIX}_{MAC6}". */
    char     service_name[PROV_NAME_MAX_LEN + 1];
} provisioning_config_t;

/* Application-level identity exposed on the optional
 * `iot-cam-info` custom protocomm endpoint. NULL fields are
 * serialized as empty strings.
 *
 * Lifetime: the package stores the pointers verbatim. They must
 * outlive the underlying provisioning session. String literals
 * are safe; stack-locals from app_main are also safe because
 * app_main stays blocked in provisioning_run() for the lifetime
 * of the session. Heap strings require the caller to keep them
 * alive for as long as provisioning could be re-entered. */
typedef struct {
    const char *name;
    const char *fw_version;
} provisioning_app_info_t;

/* One-shot initializer. Safe to call once per boot before
 * provisioning_run(). cfg must not be NULL. app_info may be
 * NULL — the iot-cam-info endpoint simply returns an empty body
 * if not provided.
 *
 * Returns ESP_OK on success, ESP_ERR_INVALID_ARG if cfg is NULL
 * or a string field is longer than its budget, or ESP_ERR_INVALID_STATE
 * if the package was already initialized. */
esp_err_t provisioning_init(const provisioning_config_t *cfg,
                            const provisioning_app_info_t *app_info);

/* True iff Wi-Fi credentials are present in NVS (managed by
 * esp_wifi_set_storage(WIFI_STORAGE_FLASH)). This is the fast
 * path; the manager reads from the same store during the next
 * provisioning init. */
bool provisioning_is_provisioned(void);

/* Blocking bring-up. Initializes the default event loop, NVS,
 * esp_netif, esp_wifi (APSTA so the station can connect to the
 * AP the user just provisioned), and mDNS; then calls
 * wifi_prov_mgr_start_provisioning() and
 * wifi_prov_mgr_wait(). Returns when the manager emits
 * WIFI_PROV_END.
 *
 * Returns ESP_OK on a successful provisioning session, or the
 * underlying esp_err_t on bring-up failure (logged with the
 * failing step + esp_err_to_name). Returns ESP_ERR_INVALID_STATE
 * if the package is not initialized or
 * force_provisioning is not set and credentials already exist. */
esp_err_t provisioning_run(void);

/* Trigger an in-progress stop on the manager's own task. Safe
 * to call from a task other than the one blocked in
 * provisioning_run(). Idempotent while the manager is idle. */
void provisioning_stop(void);

/* Wipe the wifi-managed NVS credentials (ssid + password) via
 * `wifi_prov_mgr_reset_provisioning()`. After this call
 * provisioning_is_provisioned() returns false and the next
 * provisioning_run() will start the SoftAP. Used by the
 * factory-reset flow. */
esp_err_t provisioning_reset_credentials(void);

/* Bring up the wifi subsystem in STA-only mode (no softAP) and
 * trigger a station connect using the credentials already in NVS.
 * Used on already-provisioned boots where provisioning_run() is
 * skipped and the wifi stack has never been started.
 *
 * Idempotent: safe to call more than once per boot. The IP event
 * handler (logs station connected to "<ssid>": ip=...) is registered
 * here so the same log line fires on every attach — first provision,
 * every subsequent reboot, and any transient reconnect.
 *
 * Returns ESP_OK on success, or the underlying esp_err_t on
 * bring-up failure. ESP_ERR_WIFI_CONN is treated as success
 * (the driver is already mid-connect — the caller can still
 * wait on IP_EVENT_STA_GOT_IP). */
esp_err_t provisioning_join_ap(void);

/* Inter-module helper that the captive portal's POST handler
 * calls with the operator's form input. Persists the credentials
 * via esp_wifi_set_config() and signals provisioning_run() to
 * unblock. Caller in captive_portal.c is the httpd worker thread;
 * provisioning_run() is on the boot thread, synchronized
 * through a FreeRTOS semaphore. */
esp_err_t provisioning_apply_captive_form(const char *ssid,
                                         const char *password);

/* Returns true when the station has received a DHCP lease from the
 * home AP (IP_EVENT_STA_GOT_IP fired). Used by app_main to know
 * it is safe to shut down the softAP. The flag is set by the IP
 * event handler and reset by provisioning_reset_sta_state(). */
bool provisioning_sta_has_ip(void);

/* Resets the station state (s_sta_got_ip, s_consecutive_failures)
 * before starting a fresh provisioning session. Prevents retry state
 * from a previous session bleeding into the new one. */
void provisioning_reset_sta_state(void);

/* Start the STA-bound HTTP server that exposes /whoami for device
 * discovery. Call this after the device has joined the home AP
 * and received an IP (after provisioning_join_ap() succeeds and
 * IP_EVENT_STA_GOT_IP fires). The server runs on port 80 for the
 * lifetime of the device.
 *
 * Idempotent: safe to call more than once.
 *
 * Returns ESP_OK on success, or the underlying httpd_start error. */
esp_err_t provisioning_start_sta_server(void);

/* Called from app_main's supervisor loop every tick. Owns the
 * no-IP watchdog and emits the heartbeat log. */
esp_err_t provisioning_supervise_sta(void);

/* Monotonic count of reconnect attempts issued since boot.
 * Never reset during the device lifetime.
 * Safe to call from any task; reads are atomic.
 * Exposed so the /health endpoint can report it to the operator. */
uint32_t provisioning_reconnect_count(void);

/* wifi_event_sta_disconnected_t.reason from the last disconnect,
 * or 0 if none since boot.
 * Safe to call from any task; reads are atomic. */
uint8_t provisioning_last_disconnect_reason(void);

/* Application firmware version recorded at provisioning_init()
 * from provisioning_app_info_t.fw_version. Never NULL.
 * Mirrors the existing prov_device_name() seam. */
const char *prov_fw_version(void);

#ifdef __cplusplus
}
#endif
#endif /* IOT_CAMS_PROVISIONING_H */
