/* cam_stream.c — W2 implementation.
 *
 * Owns the long-lived FreeRTOS stream task. The loop body
 * (cam_stream_loop_iteration) consumes one JPEG from
 * cam_reader, pushes it through the sink seam
 * (cam_stream_sink_send_bin) and releases the buffer on every
 * exit path — same leak-free contract cam_reader relies on
 * (REQ-ST-005, engram 4330).
 *
 * Module layout:
 *   - module-static state (s_init_done, s_sink, s_frames_*,
 *     s_tx_mtx in W5)
 *   - disconnected sink stubs (default install; sink_install
 *     uses NULL to mean "reinstall these")
 *   - public API (init / task_start / sink install /
 *     connected / sink_get / loop_iteration / counters)
 *   - FreeRTOS task wrapper (cam_stream_task_entry) — wraps
 *     cam_stream_loop_iteration in an infinite for-loop
 *     paced at CAM_STREAM_PERIOD_MS via vTaskDelayUntil.
 *
 * What is NOT in this file (lands in later work-units):
 *   - xSemaphoreCreateMutex + s_tx_mtx (W5) — sent through
 *     cam_stream_sink_send_bin so only that TU needs to
 *     include the mutex primitives.
 *   - JSON hello/status builders                           (W3)
 *   - Real server-side sink vtable with
 *     httpd_ws_send_frame_async                             (W5)
 *   - IP_EVENT_STA_GOT_IP re-attach / STA_DISCONNECTED
 *     clear                                                 (W7)
 *     (those live in provisioning/src/ws_cams.c, not here)
 */

#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_camera.h"
#include "esp_log.h"

#include "cam_reader.h"
#include "cam_stream.h"

static const char *TAG = "cam_stream";

/* ---------- module-static state ---------- */

static bool s_init_done = false;

/* TX mutex (W5). Serializes every wire write so the stream
 * loop, the handshake hello frame, and any future status-timer
 * pushes never interleave bytes on the same fd. Created in
 * cam_stream_init() before the sink install so no producer can
 * reach the seam through an un-serialized path; failure to
 * create is fatal (callers cannot proceed without serialized
 * sends). */
static SemaphoreHandle_t s_tx_mtx = NULL;

/* Cross-task counters, lock-free reads on Xtensa LX6 per the
 * reference's stream.h:38-43 pattern. */
static volatile uint32_t s_frames_sent    = 0;
static volatile uint32_t s_frames_dropped = 0;

/* Disconnected sink stubs (default install). Mirrors
 * esp32-cam-surveillance's ws.c:60-86 pattern. The stream
 * task's no-viewer path returns ESP_ERR_INVALID_STATE which
 * the loop body maps to the drop-counter path. */
static esp_err_t sink_disconnected_send_bin(const uint8_t *buf,
                                             size_t len)
{
    (void)buf;
    (void)len;
    return ESP_ERR_INVALID_STATE;
}

static esp_err_t sink_disconnected_send_text(const char *buf,
                                              size_t len)
{
    (void)buf;
    (void)len;
    return ESP_ERR_INVALID_STATE;
}

static bool sink_disconnected_is_connected(void)
{
    return false;
}

static const cam_stream_sink_t s_sink_disconnected = {
    .send_bin     = sink_disconnected_send_bin,
    .send_text    = sink_disconnected_send_text,
    .is_connected = sink_disconnected_is_connected,
};

/* Active sink pointer. Never NULL after init; always points
 * to a fully-formed `cam_stream_sink_t` (default = disconnected
 * stubs; live = server-side sink installed at /ws/cams
 * handshake accept in W5). */
static const cam_stream_sink_t *s_sink = &s_sink_disconnected;

/* ---------- sink seam accessors (cross-TU share) ---------- */

void cam_stream_sink_install(const cam_stream_sink_t *sink)
{
    s_sink = (sink != NULL) ? sink : &s_sink_disconnected;
}

bool cam_stream_sink_connected(void)
{
    return s_sink->is_connected();
}

const cam_stream_sink_t *cam_stream_sink_get(void)
{
    /* Used by cam_stream_sender.c to reach into the active
     * sink without exporting more state into the header. */
    return s_sink;
}

/* TX-mutex accessor for cam_stream_sender.c. FreeRTOS handle
 * is owned by this TU; the sender grabs it before every send
 * and never stores it. Keeping the handle module-static here
 * keeps `SemaphoreHandle_t` out of the public header. */
SemaphoreHandle_t cam_stream_tx_mtx_get(void)
{
    return s_tx_mtx;
}

/* ---------- public counters ---------- */

uint32_t cam_stream_frames_sent_get(void)
{
    return s_frames_sent;
}

uint32_t cam_stream_frames_dropped_get(void)
{
    return s_frames_dropped;
}

/* ---------- the loop iteration ----------
 *
 * Honors cam_reader.h's release contract precisely:
 *   ESP_ERR_TIMEOUT  → mutex not taken, no release, return false
 *   ESP_FAIL         → mutex taken but fb_get failed, must
 *                      release(NULL) to drop the mutex,
 *                      increment drop counter, return false
 *   ESP_OK           → fb non-NULL, must send + release(fb),
 *                      increment the right counter
 *
 * Send failure (sink returns ESP_ERR_INVALID_STATE because no
 * viewer is installed, OR a transport error after install):
 * the loop increments the drop counter, releases the buffer,
 * and returns false. The stream task keeps looping.
 */
bool cam_stream_loop_iteration(void)
{
    camera_fb_t *fb = NULL;
    esp_err_t r = cam_reader_capture(&fb);

    if (r == ESP_ERR_TIMEOUT || r == ESP_ERR_INVALID_STATE ||
        r == ESP_ERR_INVALID_ARG) {
        /* No release needed; the mutex was never taken
         * (timeout) or the caller's invariants broke
         * (invalid_arg / invalid_state). Stay idle and try
         * again next tick. */
        return false;
    }
    if (r == ESP_FAIL) {
        /* Mutex acquired, fb_get failed. MUST release(NULL)
         * to drop the mutex per cam_reader.h. */
        cam_reader_release(NULL);
        s_frames_dropped++;
        return false;
    }

    /* r == ESP_OK, fb is non-NULL. Send then release. */
    esp_err_t send_r = cam_stream_sink_send_bin(fb->buf, fb->len);

    /* cam_reader_release is safe even on send failure — we
     * owned the buffer from cam_reader_capture onwards. */
    cam_reader_release(fb);

    if (send_r != ESP_OK) {
        s_frames_dropped++;
        return false;
    }
    s_frames_sent++;
    return true;
}

/* ---------- FreeRTOS task wrapper (device-only) ---------- */
#ifndef UNITY_HOST_BUILD

static TaskHandle_t s_stream_task_handle = NULL;

/* vTaskDelayUntil pacing: CAM_STREAM_PERIOD_MS between
 * iterations. Each iteration owns the bus for (cam_reader
 * budget + send cost); on a busy camera cycle the next
 * vTaskDelayUntil fires immediately, which is acceptable —
 * the queue discipline lives inside cam_reader. */
static void cam_stream_task_entry(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        (void)cam_stream_loop_iteration();
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(CAM_STREAM_PERIOD_MS));
    }
}

#endif  /* UNITY_HOST_BUILD */

/* ---------- public init / task lifecycle ---------- */

esp_err_t cam_stream_init(void)
{
    if (s_init_done) {
        ESP_LOGW(TAG, "init: already initialized — ignoring");
        return ESP_ERR_INVALID_STATE;
    }

    /* W5 — create the TX mutex BEFORE any sink install so no
     * producer can dispatch through an un-serialized seam. */
    if (s_tx_mtx == NULL) {
        s_tx_mtx = xSemaphoreCreateMutex();
        if (s_tx_mtx == NULL) {
            ESP_LOGE(TAG, "init: xSemaphoreCreateMutex failed");
            return ESP_ERR_NO_MEM;
        }
    }

    /* Default sink = disconnected stubs, so the loop body in
     * cam_stream_sink_send_bin can route every send through
     * s_sink without a NULL check. */
    cam_stream_sink_install(NULL);

    s_init_done = true;
    ESP_LOGI(TAG,
             "init: ready "
             "(period=%d ms, %.1f fps; sink=disconnected; tx_mtx=ok)",
             (int)CAM_STREAM_PERIOD_MS,
             1000.0f / (float)CAM_STREAM_PERIOD_MS);
    return ESP_OK;
}

esp_err_t cam_stream_task_start(void)
{
#ifdef UNITY_HOST_BUILD
    ESP_LOGI(TAG,
             "task_start: host build — no FreeRTOS task spawned");
    return ESP_OK;
#else
    if (s_stream_task_handle != NULL) {
        ESP_LOGW(TAG, "task_start: already running — ignoring");
        return ESP_ERR_INVALID_STATE;
    }

    /* Priority invariant: wifi_task (23) > esp_event (20) >
     * httpd (5) > cam_stream (3). The producer must never
     * outrank the HTTP server that the UI and all discovery
     * depend on. The old priority of 5 let cam_stream starve
     * httpd workers under load (Defect B fix). */
    enum {
        CAM_STREAM_TASK_STACK = 4096,
        CAM_STREAM_TASK_PRIO  = 3,
    };
    BaseType_t ok = xTaskCreate(
        cam_stream_task_entry, "cam_stream",
        CAM_STREAM_TASK_STACK, NULL, CAM_STREAM_TASK_PRIO,
        &s_stream_task_handle);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "task_start: xTaskCreate failed (ret=%ld)",
                 (long)ok);
        s_stream_task_handle = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG,
             "task_start: spawned FreeRTOS task "
             "(stack=%d, prio=%d, period=%d ms)",
             (int)CAM_STREAM_TASK_STACK, (int)CAM_STREAM_TASK_PRIO,
             (int)CAM_STREAM_PERIOD_MS);
    return ESP_OK;
#endif  /* UNITY_HOST_BUILD */
}
