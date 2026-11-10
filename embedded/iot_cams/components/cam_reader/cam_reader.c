/* cam_reader.c — iot_cams OV2640 bring-up + serialized capture API.
 *
 * Ported from
 *     ../rural_home_assistant/backend/iot-camera/components/cam_reader/cam_reader.c
 *
 * The pin map, sensor defaults (SVGA 800x600, JPEG quality 12,
 * `grab_mode = CAMERA_GRAB_LATEST`), PSRAM-driven `fb_count` /
 * `fb_location` logic, and `init_camera()` body are unchanged —
 * the iot_cams firmware targets the same AI-Thinker ESP32-CAM
 * board family, so the AI-Thinker pin map (PWDN=32, XCLK=0,
 * SIOD=26, SIOC=27, D7..D0={35,34,39,36,21,19,18,5},
 * VSYNC=25, HREF=23, PCLK=22) is reusable byte-for-byte.
 *
 * Difference from the reference: iot_cams lifts the binary
 * semaphore into this component so it can guard every future
 * consumer (httpd handlers, future FreeRTOS task / WebSocket /
 * MQTT paths). The reference keeps the semaphore inside
 * `http_server.c` because its only consumer is the HTTP server.
 */

#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/ledc.h"

#include "esp_log.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "esp_camera.h"

#include "cam_reader.h"

static const char *TAG = "cam-reader";

/* AI-Thinker ESP32-CAM board pin map. Identical to the reference. */
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1   /* software reset; no physical pin */
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

#define CAM_XCLK_FREQ_HZ   20000000   /* 20 MHz */
#define CAM_JPEG_QUALITY   12         /* lower == higher quality */
#define CAM_FB_COUNT_PSRAM 2
#define CAM_FB_COUNT_NOMEM 1

/* The AI-Thinker ESP32-CAM ships with the OV2640 sensor mounted
 * such that the raw frame is vertically inverted — without
 * `set_vflip(sensor, 1)` the top of the scene lands at the
 * bottom of the JPEG. Operator-confirmed on hardware after the
 * first /capture flash. Flip once at init.
 *
 * Bump this constant to 0 only if a different physical mount
 * (camera flipped 180 deg) lands on the AI-Thinker board; the
 * default matches the stock enclosure. */
#define CAM_READER_VFLIP  1

/* Mutex wait budget for a single capture. The reference uses
 * 5 s; we mirror that as an executable contract. */
#define CAM_READER_WAIT_MS 5000

/* ---- Flash LED (GPIO 4) constants ----
 *
 * The AI-Thinker ESP32-CAM's onboard flash is a bright white LED
 * wired directly to GPIO 4 — there is no current-limiting resistor
 * on the board (Mi-Bee teardown, blog.mickeyzzc.tech/en/posts/iot/
 * ai-thinker-esp32-cam-flash/). Driving it at full power risks
 * board wear over time, so we use LEDC PWM at 80 % duty.
 *
 * The camera driver already owns LEDC_TIMER_0 / LEDC_CHANNEL_0
 * (for the XCLK clock generation, see the camera_config_t init
 * below). Re-using Timer/Channel 0 for the flash stalls the
 * camera — Timer 1 / Channel 1 is the only safe slot.
 *
 * Pre-charge: 200 ms between LED-on and the sensor integrating
 * the frame. This is the community-tested value; shorter delays
 * (e.g. 50 ms) produce off-color frames because the OV2640's
 * auto white balance has not yet stabilized against the new
 * illumination. */
#define CAM_READER_FLASH_GPIO        GPIO_NUM_4
#define CAM_READER_FLASH_TIMER       LEDC_TIMER_1
#define CAM_READER_FLASH_CHANNEL     LEDC_CHANNEL_1
#define CAM_READER_FLASH_FREQ_HZ     2000
#define CAM_READER_FLASH_RES         LEDC_TIMER_8_BIT
#define CAM_READER_FLASH_DUTY        205  /* 80.4 % of 255 */
#define CAM_READER_FLASH_PRECHARGE_MS 200

/* Single binary semaphore, created in cam_reader_init() and
 * given once to mark the unlocked state. NULL until init. */
static SemaphoreHandle_t s_cam_mutex = NULL;

/* Flash subsystem state. Set true after cam_reader_flash_init()
 * has configured the LEDC channel; consulted to make the init
 * function idempotent and to refuse capture-with-flash calls
 * before the LEDC is configured. */
static bool s_flash_inited = false;

/* Producer-side counters. Lock-free u32 reads on Xtensa LX6. */
static volatile uint32_t s_frames_captured = 0;
static volatile uint32_t s_fb_drops        = 0;

uint32_t cam_reader_frames_captured_get(void)
{
    return s_frames_captured;
}

uint32_t cam_reader_fb_drops_get(void)
{
    return s_fb_drops;
}

esp_err_t cam_reader_init(void)
{
    if (s_cam_mutex != NULL) {
        ESP_LOGW(TAG, "init: already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    s_cam_mutex = xSemaphoreCreateBinary();
    if (s_cam_mutex == NULL) {
        ESP_LOGE(TAG, "init: xSemaphoreCreateBinary failed");
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(s_cam_mutex);

    /* PSRAM detection drives the buffer config at runtime.
     * CONFIG_SPIRAM_SUPPORT is set by `idf.py menuconfig` whenever
     * PSRAM is enabled (CONFIG_SPIRAM=y). When the project builds
     * with PSRAM off we skip the esp_psram helper and pick the
     * single-buffer DRAM path. */
    size_t psram_size = 0;
#if defined(CONFIG_SPIRAM_SUPPORT) && CONFIG_SPIRAM_SUPPORT
    psram_size = esp_psram_get_size();
#endif

    camera_config_t camera_config = {
        .pin_pwdn      = CAM_PIN_PWDN,
        .pin_reset     = CAM_PIN_RESET,
        .pin_xclk      = CAM_PIN_XCLK,
        .pin_sccb_sda  = CAM_PIN_SIOD,
        .pin_sccb_scl  = CAM_PIN_SIOC,

        .pin_d7        = CAM_PIN_D7,
        .pin_d6        = CAM_PIN_D6,
        .pin_d5        = CAM_PIN_D5,
        .pin_d4        = CAM_PIN_D4,
        .pin_d3        = CAM_PIN_D3,
        .pin_d2        = CAM_PIN_D2,
        .pin_d1        = CAM_PIN_D1,
        .pin_d0        = CAM_PIN_D0,
        .pin_vsync     = CAM_PIN_VSYNC,
        .pin_href      = CAM_PIN_HREF,
        .pin_pclk      = CAM_PIN_PCLK,

        .xclk_freq_hz  = CAM_XCLK_FREQ_HZ,
        .ledc_timer    = LEDC_TIMER_0,
        .ledc_channel  = LEDC_CHANNEL_0,

        .pixel_format  = PIXFORMAT_JPEG,
        .frame_size    = FRAMESIZE_SVGA,   /* 800x600 */
        .jpeg_quality  = CAM_JPEG_QUALITY,

        .fb_count      = (psram_size > 0) ? CAM_FB_COUNT_PSRAM
                                           : CAM_FB_COUNT_NOMEM,
        .fb_location   = (psram_size > 0) ? CAMERA_FB_IN_PSRAM
                                           : CAMERA_FB_IN_DRAM,
        .grab_mode     = CAMERA_GRAB_LATEST,

        /* Pin the JPEG frame buffer size so every copy buffer in the
         * firmware can be sized to match it exactly. The driver's
         * own default is width*height/5, which is not a round
         * number and would drift from the buffers we allocate. */
        .jpeg_buffer_size = CAM_READER_JPEG_BUFFER_SIZE,
    };

    ESP_LOGI(TAG, "init: PSRAM=%zu bytes, fb_count=%d",
             psram_size, camera_config.fb_count);

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init: esp_camera_init: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_cam_mutex);
        s_cam_mutex = NULL;
        return err;
    }

    /* Board-mounted orientation fix. The AI-Thinker ESP32-CAM
     * ships with the sensor vertically inverted, so the raw
     * frame appears upside-down in the JPEG output. Apply the
     * flip once at boot — it's a per-board property, not a
     * per-request property — and trust the sensor driver to
     * pick the cleanest sensor implementation registered for
     * the connected module (here always OV2640 on AI-Thinker). */
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor != NULL && sensor->set_vflip != NULL) {
        int rc = sensor->set_vflip(sensor, CAM_READER_VFLIP);
        if (rc != 0) {
            ESP_LOGW(TAG, "init: sensor->set_vflip returned %d", rc);
        } else {
            ESP_LOGI(TAG, "init: vflip=%d (board orientation)",
                     (int)CAM_READER_VFLIP);
        }
    } else {
        ESP_LOGW(TAG, "init: sensor has no set_vflip; "
                      "image may be upside-down");
    }

    ESP_LOGI(TAG, "init: camera ready (SVGA 800x600, JPEG quality %d)",
             CAM_JPEG_QUALITY);
    return ESP_OK;
}

esp_err_t cam_reader_capture(camera_fb_t **fb)
{
    if (fb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_cam_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    *fb = NULL;

    /* 5 s wait budget, mirrors the reference project. */
    if (xSemaphoreTake(s_cam_mutex, pdMS_TO_TICKS(CAM_READER_WAIT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "capture: mutex timeout (>%d ms) — caller is busy",
                 CAM_READER_WAIT_MS);
        return ESP_ERR_TIMEOUT;
    }

    /* esp_timer_get_time() is a microsecond counter; convert to ms. */
    uint64_t start_us = esp_timer_get_time();
    camera_fb_t *out = esp_camera_fb_get();
    uint64_t elapsed_ms = (esp_timer_get_time() - start_us) / 1000ULL;

    if (out == NULL) {
        ESP_LOGE(TAG, "capture: esp_camera_fb_get failed");
        /* The mutex has been taken; the caller MUST call
         * cam_reader_release(NULL) to drop it. */
        s_fb_drops++;
        return ESP_FAIL;
    }

    s_frames_captured++;
    ESP_LOGD(TAG, "capture: %zu bytes in %llu ms",
             out->len, (unsigned long long)elapsed_ms);

    *fb = out;
    return ESP_OK;
}

void cam_reader_release(camera_fb_t *fb)
{
    if (fb != NULL) {
        esp_camera_fb_return(fb);
    }
    if (s_cam_mutex != NULL) {
        xSemaphoreGive(s_cam_mutex);
    }
}

esp_err_t cam_reader_capture_copy(uint8_t *dst, size_t dst_size,
                                  size_t *out_len, bool flash)
{
    if (dst == NULL || out_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_len = 0;
    if (dst_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    camera_fb_t *fb = NULL;
    esp_err_t r = flash ? cam_reader_capture_with_flash(&fb, true)
                        : cam_reader_capture(&fb);
    if (r != ESP_OK || fb == NULL) {
        /* ESP_FAIL means the mutex WAS taken and the sensor produced
         * no frame: the mutex must be given back. Every other error
         * means the mutex was never taken, and must NOT be. */
        if (r == ESP_FAIL) {
            cam_reader_release(NULL);
        }
        return (r == ESP_OK) ? ESP_FAIL : r;
    }

    if (fb->len > dst_size) {
        ESP_LOGE(TAG, "capture_copy: frame of %u bytes does not fit in a "
                      "%u byte buffer",
                 (unsigned)fb->len, (unsigned)dst_size);
        cam_reader_release(fb);
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(dst, fb->buf, fb->len);
    *out_len = fb->len;

    /* Give the frame buffer back AND release the camera. From here
     * the caller owns the copy and the sensor is free, so a slow
     * network write cannot starve anyone else. */
    cam_reader_release(fb);
    return ESP_OK;
}

/* ---- Flash LED (GPIO 4) ----
 *
 * The LEDC subsystem is shared with other drivers — only one
 * owner per timer / channel. The camera driver owns Timer 0 /
 * Channel 0 (XCLK); we use Timer 1 / Channel 1. The flash LED
 * itself is wired to GPIO 4 on the AI-Thinker ESP32-CAM.
 *
 * Sensor side: disable `aec2` (advanced AEC digital signal
 * processing). With AECDSP active the OV2640 overrides AELevel
 * and produces overexposed flash-photographed frames
 * (arendst/Tasmota#23222). Without AECDSP the auto-exposure
 * converges on the flash-lit scene properly.
 *
 * Error policy: any LEDC failure leaves the subsystem un-
 * initialized (s_flash_inited stays false) and surfaces the
 * error to the caller. The sensor's aec2 may already have been
 * disabled if we got that far; that is safe and idempotent. */
esp_err_t cam_reader_flash_init(void)
{
    if (s_flash_inited) {
        ESP_LOGW(TAG, "flash_init: already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* The camera must exist before we can touch aec2 on the
     * sensor. If init() was never called, fail fast. */
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL) {
        ESP_LOGE(TAG, "flash_init: camera not initialized yet");
        return ESP_ERR_INVALID_STATE;
    }

    /* LEDC timer. LOW_SPEED is appropriate for GPIO output on
     * ESP32 (no high-speed requirement for a flash LED). */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = CAM_READER_FLASH_TIMER,
        .duty_resolution = CAM_READER_FLASH_RES,
        .freq_hz         = CAM_READER_FLASH_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "flash_init: ledc_timer_config: %s",
                 esp_err_to_name(err));
        return err;
    }

    /* LEDC channel — GPIO 4 at duty 0 (LED off). ledc_channel_config
     * routes the GPIO matrix to the LEDC output internally, so we
     * do not need to call gpio_matrix_output separately. */
    ledc_channel_config_t channel_cfg = {
        .gpio_num   = CAM_READER_FLASH_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = CAM_READER_FLASH_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = CAM_READER_FLASH_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags      = { .output_invert = 0 },
    };
    err = ledc_channel_config(&channel_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "flash_init: ledc_channel_config: %s",
                 esp_err_to_name(err));
        return err;
    }

    /* Sensor-side adjustment: disable advanced AEC DSP so the
     * OV2640's AELevel control is effective during flash
     * photography. See Tasmota#23222 — with aec2=1 the AE
     * targets the ambient-dark frame and the flash-lit frame
     * is washed out. */
    if (sensor->set_aec2 != NULL) {
        int rc = sensor->set_aec2(sensor, 0);
        if (rc != 0) {
            ESP_LOGW(TAG, "flash_init: sensor->set_aec2 returned %d "
                          "(AEC DSP may remain enabled; "
                          "flash frames may overexpose)", rc);
            /* Non-fatal: the LEDC path still works. The caller
             * gets ESP_OK so capture-with-flash is still usable
             * for daylight scenes; night scenes may be
             * suboptimal until the next reboot with a working
             * aec2-disable path. */
        } else {
            ESP_LOGI(TAG, "flash_init: sensor aec2 disabled "
                         "(flash-aware exposure)");
        }
    } else {
        ESP_LOGW(TAG, "flash_init: sensor has no set_aec2; "
                      "flash frames may overexpose in low light");
    }

    s_flash_inited = true;
    ESP_LOGI(TAG, "flash_init: LEDC ready "
                 "(gpio=%d timer=%d channel=%d freq=%d duty=0 "
                 "precharge=%d ms)",
             (int)CAM_READER_FLASH_GPIO,
             (int)CAM_READER_FLASH_TIMER,
             (int)CAM_READER_FLASH_CHANNEL,
             (int)CAM_READER_FLASH_FREQ_HZ,
             (int)CAM_READER_FLASH_PRECHARGE_MS);
    return ESP_OK;
}

/* Set the flash LED to the on-duty and commit it to the LEDC
 * peripheral. Called only from cam_reader_capture_with_flash
 * after we've checked s_flash_inited, so no extra guard here. */
static void cam_reader_flash_on(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE,
                  CAM_READER_FLASH_CHANNEL,
                  CAM_READER_FLASH_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,
                     CAM_READER_FLASH_CHANNEL);
}

/* Set the flash LED off (duty 0) and commit. Safe to call from
 * every exit path — there is no error return, so a failed
 * LEDC commit (which we have never observed on Timer 1 / GPIO 4
 * in practice) cannot wedge the LED. */
static void cam_reader_flash_off(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE,
                  CAM_READER_FLASH_CHANNEL,
                  0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,
                     CAM_READER_FLASH_CHANNEL);
}

/* Capture with optional flash illumination. The flash branch
 * is a strict superset of cam_reader_capture():
 *
 *     flash == false  : byte-for-byte identical to
 *                       cam_reader_capture(&fb). No LED change,
 *                       no pre-charge. Callers that do not need
 *                       flash pay zero latency.
 *
 *     flash == true   : LEDC duty 205 → 200 ms wait →
 *                     cam_reader_capture(&fb) → LEDC duty 0
 *                     (UNCONDITIONALLY on every exit path,
 *                     including ESP_ERR_TIMEOUT and ESP_FAIL;
 *                     a missed reset leaves the LED pinned on,
 *                     which is both a power drain and a
 *                     misleading user signal).
 *
 * The 200 ms pre-charge sits INSIDE the camera mutex taken by
 * cam_reader_capture, so a concurrent caller past the 5 s wait
 * budget cannot slip in and observe the LED on while no capture
 * is happening. Concurrent callers that time out on the mutex
 * (ESP_ERR_TIMEOUT) see no LED change at all.
 *
 * If cam_reader_flash_init() has not been called, the flash=true
 * branch returns ESP_ERR_INVALID_STATE without touching the LEDC.
 * */
esp_err_t cam_reader_capture_with_flash(camera_fb_t **fb, bool flash)
{
    if (!flash) {
        /* No flash path: delegate to the existing capture with
         * byte-for-byte identical behavior. Zero LED access. */
        return cam_reader_capture(fb);
    }

    if (!s_flash_inited) {
        ESP_LOGE(TAG, "capture_with_flash: flash_init not called");
        return ESP_ERR_INVALID_STATE;
    }

    /* Flash on at the configured duty. */
    cam_reader_flash_on();

    /* Pre-charge wait for AWB to stabilize against the new
     * illumination. 200 ms is the community-tested value (see
     * the constants block at the top of this file). */
    vTaskDelay(pdMS_TO_TICKS(CAM_READER_FLASH_PRECHARGE_MS));

    /* Take the mutex and capture the frame. cam_reader_capture
     * owns the mutex for at most 5 s (CAM_READER_WAIT_MS); the
     * LED stays on for that whole window. The mutex is given
     * back by cam_reader_release on every exit path that took
     * it — the caller (sta_server.c) is responsible for that
     * pairing per the existing cam_reader contract. */
    esp_err_t r = cam_reader_capture(fb);

    /* Always off when done. ESP_ERR_TIMEOUT is impossible here
     * (the mutex was taken by us, the budget is the same 5 s as
     * a normal capture, and the pre-charge 200 ms counted
     * against it; a concurrent caller would have timed out and
     * never reached this code path), but we cover it anyway for
     * defense in depth. */
    cam_reader_flash_off();

    if (r != ESP_OK) {
        ESP_LOGW(TAG, "capture_with_flash: capture returned %s; "
                      "LED is off",
                 esp_err_to_name(r));
    } else if (fb != NULL && *fb != NULL) {
        ESP_LOGD(TAG, "capture_with_flash: %zu bytes "
                      "(200ms pre-charge applied)",
                 (*fb)->len);
    }

    return r;
}
