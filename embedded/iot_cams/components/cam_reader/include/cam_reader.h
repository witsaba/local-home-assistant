/* cam_reader.h — public API for the iot_cams camera reader.
 *
 * Owns the AI-Thinker ESP32-CAM OV2640 bring-up (lifted from
 * `../rural_home_assistant/backend/iot-camera/components/cam_reader/`)
 * and a binary semaphore that serializes access to the camera
 * hardware across every consumer — httpd handlers in this firmware,
 * and any future FreeRTOS task / WebSocket / MQTT path.
 *
 * The mutex is non-recursive. At most one `esp_camera_fb_get` runs
 * system-wide at any time; concurrent callers see `ESP_ERR_TIMEOUT`
 * from `cam_reader_capture()` and should map that to HTTP 503.
 *
 * LIFECYCLE:
 *
 *     cam_reader_init()        one-shot at app_main; boots the sensor
 *                              and creates the binary semaphore in
 *                              the unlocked state.
 *
 *     cam_reader_capture(&fb)  take the next JPEG frame (5 s wait
 *                              budget on the mutex; ESP_ERR_TIMEOUT
 *                              beyond that).
 *
 *     cam_reader_release(fb)   return the frame buffer to the driver
 *                              AND give the mutex. NULL fb is
 *                              permitted — the buffer return is a
 *                              no-op and only the mutex is given.
 *                              The caller MUST call this on every
 *                              `cam_reader_capture` outcome except
 *                              the `ESP_ERR_TIMEOUT` path (where the
 *                              mutex was never taken).
 */

#ifndef IOT_CAMS_CAM_READER_H
#define IOT_CAMS_CAM_READER_H

#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Size of the JPEG frame buffer, in bytes.
 *
 *  The camera driver is configured with exactly this value
 *  (`camera_config_t.jpeg_buffer_size`), and every copy buffer in
 *  this firmware is sized to match, so a frame can never be larger
 *  than the buffer it is copied into. The driver's own default for
 *  SVGA (800x600) is `width * height / 5` = 96 000; this is that
 *  value rounded up to whole 4 KB pages.
 */
#define CAM_READER_JPEG_BUFFER_SIZE 98304

/** Boot the OV2640 sensor and create the camera mutex.
 *
 *  Idempotent: returns `ESP_ERR_INVALID_STATE` on a second call.
 *  On success the sensor is ready and the mutex is unlocked.
 *
 *  @return ESP_OK on success, otherwise an esp_err_t propagated
 *          from `esp_camera_init` or `xSemaphoreCreateBinary`.
 */
esp_err_t cam_reader_init(void);

/** Take the camera mutex and return the next JPEG frame.
 *
 *  Blocks up to 5 s on the mutex.
 *
 *  @param[out] fb On success, *fb is non-NULL. On either failure
 *                 outcome (timeout or sensor failure), *fb is NULL.
 *
 *  @return
 *    - `ESP_OK`            mutex acquired and `esp_camera_fb_get`
 *                          returned a valid frame; *fb is non-NULL.
 *    - `ESP_ERR_TIMEOUT`   mutex contention; the 5 s budget elapsed.
 *                          *fb is NULL. Caller should NOT call
 *                          `cam_reader_release` (the mutex was
 *                          never taken).
 *    - `ESP_FAIL`          mutex acquired but `esp_camera_fb_get`
 *                          returned NULL. *fb is NULL. Caller MUST
 *                          call `cam_reader_release(NULL)` to drop
 *                          the mutex.
 *    - `ESP_ERR_INVALID_ARG` *fb was NULL on the way in.
 *    - `ESP_ERR_INVALID_STATE` `cam_reader_init` not called.
 */
esp_err_t cam_reader_capture(camera_fb_t **fb);

/** Return the frame buffer to the driver pool and give the mutex.
 *
 *  Safe to call with `fb == NULL`; the buffer return is a no-op and
 *  only the mutex is given. This is the right call to pair with the
 *  `ESP_FAIL` outcome of `cam_reader_capture()`.
 */
void cam_reader_release(camera_fb_t *fb);

/** Producer-side counters. Lock-free u32 reads on Xtensa LX6.
 *  Bumped once per `cam_reader_capture` outcome:
 *    - `frames_captured` — successful `esp_camera_fb_get`.
 *    - `fb_drops`        — `esp_camera_fb_get` returned NULL
 *                          (after the mutex was taken); the
 *                          mutex has been released automatically.
 *  Called by the cam_stream status-frame builder in W5+.
 *
 *  @return monotonic counter values. Reset to zero only at boot. */
uint32_t cam_reader_frames_captured_get(void);
uint32_t cam_reader_fb_drops_get(void);

/** Initialize the GPIO 4 flash LED PWM and adjust the sensor for
 *  flash photography.
 *
 *  Boots a single LEDC channel (Timer 1, Channel 1 — chosen to
 *  avoid the Timer 0 / Channel 0 already used by the camera driver
 *  for XCLK) at 8-bit / 2 kHz / ~80 % duty on GPIO 4. The LED
 *  starts off (duty = 0). On the sensor side, disables
 *  `aec2` (advanced AEC digital signal processing) so the OV2640's
 *  auto-exposure can adapt properly to the flash-lit scene —
 *  otherwise AECDSP overrides AELevel and the frame is overexposed
 *  per arendst/Tasmota#23222.
 *
 *  Idempotent: returns `ESP_ERR_INVALID_STATE` on a second call.
 *  Returns `ESP_ERR_INVALID_STATE` if `cam_reader_init` has not
 *  been called yet (the sensor must exist before we can disable
 *  aec2 on it).
 *
 *  Called once from `app_main` after `cam_reader_init`.
 *
 *  @return ESP_OK on success, otherwise an esp_err_t propagated
 *          from `ledc_timer_config` / `ledc_channel_config`.
 */
esp_err_t cam_reader_flash_init(void);

/** Capture one JPEG frame with the flash LED illuminated.
 *  Flash control is owned by this function:
 *    - LED on at ~80 % duty
 *    - 200 ms pre-charge (lets the OV2640 auto white balance
 *      stabilize before the sensor integrates the frame; 50 ms
 *      is empirically too short and produces off-color frames —
 *      see community teardowns like Mi-Bee Studio's
 *      ai-thinker-esp32-cam)
 *    - existing `cam_reader_capture` (sema-take + fb_get)
 *    - LED off (UNCONDITIONALLY on every exit path, including
 *      the 503 / 500 paths; a missed LEDC reset leaves the LED
 *      pinned on, which is both a power drain and a misleading
 *      user signal)
 *
 *  When `flash == false`, this is a thin wrapper around
 *  `cam_reader_capture(&fb)` with byte-for-byte identical
 *  behavior. Pre-charge and post-charge never run.
 *
 *  The mutex budget (5 s) is unchanged; the 200 ms pre-charge
 *  happens INSIDE the mutex so a concurrent caller cannot slip
 *  in and observe the LED on while no capture is happening.
 *
 *  @param[out] fb see `cam_reader_capture`. Same semantics.
 *  @param[in]  flash true to enable flash, false to skip.
 *
 *  @return same outcomes as `cam_reader_capture`.
 */
esp_err_t cam_reader_capture_with_flash(camera_fb_t **fb, bool flash);

/** Capture one JPEG frame and COPY it into a caller-owned buffer,
 *  releasing the camera before returning.
 *
 *  This is the API every network consumer should use. The camera
 *  mutex must NEVER be held across a socket write: a blocking
 *  `httpd_resp_send` or `httpd_ws_send_frame_async` can stall for
 *  the whole `send_wait_timeout` (5 s by default), and holding the
 *  sensor for that long starves every other consumer of it — the
 *  picture request and the stream fight over the camera instead of
 *  over the network.
 *
 *  The copy costs one memcpy of the frame (~22 KB, about 1 ms) and
 *  the caller owns the buffer afterwards, so it may hold it for as
 *  long as it likes without blocking the sensor.
 *
 *  @param[out] dst      destination buffer.
 *  @param[in]  dst_size size of `dst`; must be at least
 *                       `CAM_READER_JPEG_BUFFER_SIZE` to be safe
 *                       for any frame the driver can produce.
 *  @param[out] out_len  receives the number of bytes written.
 *  @param[in]  flash    true to illuminate the flash LED for the
 *                       capture, exactly as
 *                       `cam_reader_capture_with_flash` does.
 *
 *  @return
 *    - `ESP_OK`            the frame was copied; *out_len > 0. The
 *                          camera has already been released.
 *    - `ESP_ERR_TIMEOUT`   mutex contention; the 5 s budget elapsed.
 *                          The mutex was never taken — do NOT
 *                          release it.
 *    - `ESP_FAIL`          the mutex was taken but the sensor
 *                          produced no frame. The mutex has already
 *                          been released.
 *    - `ESP_ERR_INVALID_SIZE` the frame did not fit in `dst`. The
 *                          camera has already been released.
 *    - `ESP_ERR_INVALID_ARG` a NULL pointer was passed.
 *    - `ESP_ERR_INVALID_STATE` `cam_reader_init` not called, or the
 *                          flash was requested before
 *                          `cam_reader_flash_init`.
 */
esp_err_t cam_reader_capture_copy(uint8_t *dst, size_t dst_size,
                                  size_t *out_len, bool flash);

#ifdef __cplusplus
}
#endif

#endif /* IOT_CAMS_CAM_READER_H */
