# WiFi Provisioning for `iot_cams`

A self-contained WiFi provisioning package for the `iot_cams`
firmware. Brings up a SoftAP, runs an httpd, and serves an
HTML form the operator can fill in a phone browser to attach
the device to their home Wi-Fi. No apps to install, no
protocomm handshake, no internet access required during
deployment.

**Scope.** SoftAP-only operator flow via a captive HTML form.
The package talks to `esp_wifi` directly; the IDF
`wifi_provisioning` / `protocomm` manager is intentionally
not linked (see [Why no protocomm](#why-no-protocomm) below).

---

## Public surface

`include/provisioning.h` exposes five top-level functions plus a
small inter-module helper that the captive portal's HTTP handler
calls when the operator submits the form:

```c
esp_err_t provisioning_init(const provisioning_config_t *cfg,
                            const provisioning_app_info_t *app_info);

bool      provisioning_is_provisioned(void);

esp_err_t provisioning_run(void);                  /* blocking */

void      provisioning_stop(void);                 /* unblock */

esp_err_t provisioning_reset_credentials(void);    /* wipe NVS */

esp_err_t provisioning_apply_captive_form(const char *ssid,
                                         const char *password); /* inter-module */
```

The header documents every error path. Application code includes
only `provisioning.h`; the captive HTML form lives at the bottom of
this README as a sibling-style flow.

---

## Operator procedure (the only flow that matters)

1. **Power on the device.** On first boot (or after a factory
   reset) no Wi-Fi credentials are in NVS, so the package brings
   up a softAP at `IoT-Cam_3091B0` (the suffix is the last 3 bytes
   of the device's MAC).
2. **From a phone, join the softAP** using the WPA2 passphrase
   from `CONFIG_PROVISIONING_SOFTAP_PASS`. The operator sees a
   captive-portal prompt from iOS/Android the moment the
   connection completes; if not (some Androids don't pop it
   without an actual probe URL), the operator navigates
   manually to `http://192.168.4.1/`.
3. **The HTML form appears.** The header carries the device
   name and a `WPA2` badge (aria-label). A short paragraph above
   the form reminds the operator that the device only supports
   **2.4 GHz** so they don't sit at an empty network list on a
   band-steering 5 GHz-only router. The network dropdown is
   auto-populated from `/scan`, sorted by RSSI (best signal
   first) and deduped across mesh APs that share an SSID.
   Below the dropdown is a "Add hidden network manually"
   disclosure (`<details>`) for hidden SSIDs. Hidden networks
   are filtered out of the scan by the scanner so this is the
   only path to a hidden AP. `Rescan` re-runs `/scan` (also
   wired to the auto-scan on page load; the `Rescan` button is
   a manual override).
4. **The operator picks the home network and enters the WPA2
   passphrase** (8–63 ASCII chars). The form has no native
   JavaScript fetches — submission is a native browser POST
   to `/provision`, matching the existing C handler's
   `application/x-www-form-urlencoded` contract. While the
   submit is in flight the primary button stays disabled and
   its label flips to `Connecting...` so the operator knows
   the form took.
5. **The device persists those credentials** via
   `esp_wifi_set_config()` and `provisioning_run()` returns. The
   softAP stays up briefly so the operator's browser gets the
   "Connected" confirmation; the application body that comes
   after `provisioning_run()` drops the AP and the device
   continues as a station.

**Open networks.** Not supported. `provisioning_apply_captive_form`
hardcodes `WIFI_AUTH_WPA2_PSK` as the station threshold and
`esp_wifi_set_config` will not connect to an open AP under that
threshold. The `password` input is therefore `required`.

**Default credentials on the workbench.** SSID = `IoT-Cam_3091B0`,
WPA2 passphrase = `abcd1234`. The `abcd1234` placeholder is in
`CONFIG_PROVISIONING_SOFTAP_PASS`; ship-time packaging must replace
both with per-fleet values.

### Page layout

```
+---------------------------------------------+
|          Witsaba Cam Setup                   |
|             iot-cam-A1B2                     |
|                  [WPA2]                      |
+---------------------------------------------+
| Pick your home network. This device only    |
| supports 2.4 GHz - if your router shows two  |
| SSIDs, choose the 2.4 GHz one.              |
+---------------------------------------------+
| Network                                      |
|   [BestNetwork  ████             v]          |
|   [OtherNetwork  ███░              ]         |
|   [Neighbor-Net  █░░░              ]         |
|   [- Hidden (type above) -           ]       |
|   + Add hidden network manually              |
|                                              |
| Password                                     |
|   [....................] [Show]              |
+---------------------------------------------+
|        [ Rescan ]    [      Connect      ]   |
+---------------------------------------------+
```

### UX changes from the previous iteration

| Change | Why |
| --- | --- |
| Submit is a native POST (no `fetch`) | One less JS round-trip; C handler unchanged |
| Submit button shows `Connecting...` until navigation | Operator no longer wonders if the form took |
| `aria-live='assertive'` error / `polite` success | Screen readers announce status changes |
| `<details>` for hidden SSID | Pure HTML disclosure, no JS toggle |
| Dropdown sorted by RSSI, mesh SSIDs deduped | Best signal at top of list; no duplicated entries |
| `2.4 GHz only` warning above the form | Stops band-steering routers from looking broken |
| WPA2 badge in the header with `aria-label` | Trust signal without a decorative SVG byte cost |
| Dark mode via `prefers-color-scheme` | Operator configures at night |
| `prefers-reduced-motion` honored | No animation regression if we add transitions later |
| `:user-invalid` styled with `--e` | Browser-native flag for invalid fields without JS |
| Device name substituted into `<h2>` (subtitle) | Operator knows which device they're configuring |
| Verbose `ESP_LOGI`/`ESP_LOGW` in this file gated with `#if 0` | Flash budget for the log format strings |
| **Round 2** | |
| Brand title `Witsaba Cam Setup` as the h1, device name as h2 subtitle | Clear brand hierarchy; both lines centered |
| Border-bottom separator on the header | Reads the title block as its own region |
| More breathing room above the lede paragraph | Visual weight separates the header from the form |
| Password show/hide toggle next to the input | Operator can verify the password before sending |
| Server-side wifi bars in `/scan` JSON (`get_wifi_bars_meter`) | One less client-side computation; dBm dropped from display |
| Drop the `dBm` suffix on dropdown entries | Operators do not read dBm at provisioning time |

---

## Bring-up sequence (firmware internals)

```
app_main
  ├── provisioning_init(cfg, info)
  │     ├── nvs_flash_init() (idempotent on repeat calls)
  │     └── store service_name + cfg
  ├── if (provisioning_is_provisioned())  → return ESP_OK
  └── provisioning_run()  ── blocking
        ├── softap_bring_up()
        │     ├── esp_netif_init  (idempotent)
        │     ├── esp_event_loop_create_default  (idempotent)
        │     ├── esp_netif_create_default_wifi_ap  (BEFORE esp_wifi_init)
        │     ├── esp_netif_set_default_netif
        │     ├── esp_wifi_init
        │     ├── esp_wifi_set_mode(WIFI_MODE_APSTA)
        │     ├── esp_wifi_start
        │     ├── mdns_init
        │     └── mdns_hostname_set
        ├── captive_portal_bring_up()  ── httpd on port 80
        │     ├── httpd_start(&server, HTTPD_DEFAULT_CONFIG)
        │     ├── httpd_register_uri_handler(/, GET)
        │     ├── httpd_register_uri_handler(/provision, POST)
        │     ├── httpd_register_uri_handler(/whoami, GET)
        │     └── httpd_register_err_handler(HTTPD_404_NOT_FOUND, ...)
        ├── xSemaphoreCreateBinary()  ── the unblock primitive
        ├── xSemaphoreTake(...)  ── blocks on this thread
        │
        │  … operator browses to 192.168.4.1/ and submits the form …
        │  captive_portal POST /provision parses form-urlencoded
        │  body and calls provisioning_apply_captive_form(ssid, password)
        │  which calls esp_wifi_set_config(WIFI_IF_STA) and signals
        │  the semaphore.
        │
        ├── xSemaphoreTake returns
        ├── captive_portal_tear_down()  ── httpd_stop
        └── return ESP_OK (or ESP_FAIL if provisioning_stop was called)
```

`esp_netif_create_default_wifi_ap` MUST run before `esp_wifi_init`
in IDF v5.5.x — IDF returns `ESP_ERR_INVALID_STATE` if the order
is reversed.

---

## Why no protocomm

The project originally wrapped IDF's `wifi_provisioning` manager
with the SoftAP scheme. That requires the operator to install the
Espressif "ESP SoftAP Provisioning" phone app on their phone.
The current deployment environment is offline / private-network
only, so the operator cannot reliably download the app at install
time.

The captive-portal HTML form runs entirely off
`httpd_register_uri_handler` + `esp_wifi_set_config`. The WPA2
passphrase on the softAP link is the operator-visible security
boundary; the security-1 PoP from the original IDF path is
accepted on the `provisioning_init()` arg for API stability but
ignored at runtime.

Removing the IDF manager also dropped the shared-httpd
`wifi_prov_scheme_softap_set_httpd_handle()` codepath, which
LoadProhibited-crashed in `httpd_find_uri_handler` during
the on-device verification pass. The simpler direct-API path
side-steps that bug.

---

## Security model

| Setting | Default | Where it lives |
| --- | --- | --- |
| Transport | SoftAP + HTTP + mDNS | inline `esp_wifi` + `esp_http_server` |
| SoftAP WPA2 passphrase | `CONFIG_PROVISIONING_SOFTAP_PASS` (8–64 bytes) | Kconfig |
| Service name | `"{prefix}_{MAC3}"` | derived in `derive_service_name()` |
| Captive-form credentials (operator-typed) | run-time, no secret persisted on the device | `esp_wifi_set_config()` writes them to NVS |

The WPA2 passphrase on the softAP is the only operator-visible
security boundary in the captive-portal flow. It must be printed
on the device label or in the install guide. The original
security-1 PoP code path was removed when the IDF manager was
dropped; the Kconfig `CONFIG_PROVISIONING_POP` is preserved for
API compatibility but ignored at runtime.

For deployments in hostile networks (or any place the operator's
phone cannot trust the softAP), the package should grow a
`wifi_prov_scheme_softap`-based flow alongside the captive path
(see [Extending](#extending)).

---

## Persistence

The package does **not** carry a parallel NVS namespace. The IDF
manager writes ssid + password to esp_wifi's NVS storage during
the `apply_config` flow; we read back through
`wifi_prov_mgr_is_provisioned()` and reset through
`wifi_prov_mgr_reset_provisioning()`.

| Key | Owner | Cleared by |
| --- | --- | --- |
| `nvs.net80211` / station ssid + password | esp_wifi (via `wifi_prov_mgr_*`) | `provisioning_reset_credentials()` |
| PoP, service_name, security_level | Kconfig at build time | none (firmware upgrade path) |

---

## Custom endpoint: `/whoami`

Registered alongside the captive portal on the same httpd server
(see [Bring-up sequence](#bring-up-sequence-firmware-internals)).

`GET /whoami` returns the device's identity as JSON. The
captive-portal flow replaces the previous (protocomm-based)
`iot-cam-info` endpoint with this httpd-served version; nothing
about the response shape changed.

Request: none (empty body).
Response:

```json
{
  "mac": "e08cfe3091b0",
  "name": "iot-cam",
  "fw": "0.1.0",
  "idf": "v5.5.3",
  "chip": "ESP32-D0WDQ6",
  "ip": "192.168.1.51",
  "rssi": -51,
  "uptime_s": 3012
}
```

`fw` is the **application** firmware version, recorded from
`provisioning_app_info_t.fw_version` at `provisioning_init()` and
served through the `prov_fw_version()` seam. It used to report
`esp_get_idf_version()` instead, which meant an operator could not
tell what firmware a device was running — precisely the thing
needed during an incident. The ESP-IDF version is still available,
under `idf`.

`ip`, `rssi`, and `uptime_s` are live state rather than identity.
They degrade gracefully when the station is down: `rssi` becomes
`-1` and `ip` becomes `0.0.0.0`.

The response is assembled with `snprintf` into a 256-byte buffer and
an explicit overflow guard that returns `500` rather than
truncating. If you add a field, grow the buffer in the same commit.

---

## Diagnostics endpoint: `/health`

Registered on the same STA httpd handle as `/whoami` and
`/capture`. Same audience: post-provisioning LAN clients, plus the
Pi monitoring agent.

```bash
curl -s http://<camera-ip>/health
```

```json
{
  "uptime_ms": 301234,
  "free_heap": 120432,
  "min_free_heap": 89200,
  "free_psram": 1966080,
  "ip": "192.168.1.51",
  "rssi": -51,
  "reconnect_count": 3,
  "last_disconn_rc": 201,
  "ws_viewer": 0
}
```

The point of this endpoint is to separate three failure shapes that
look identical from outside — a device that answers nothing at all:

| Observation | Meaning |
| --- | --- |
| `ip` = `0.0.0.0` | The station is off-network. The link dropped and did not recover. |
| `ip` present, `min_free_heap` low and falling | The httpd is alive but starved. Pool or heap exhaustion. |
| `ip` present, healthy heap | Neither. Look at the heartbeat log or the caller. |

It never returns `500`. `esp_wifi_sta_get_ap_info` and
`esp_netif_get_ip_info` failures degrade to `rssi` = `-1` and `ip` =
`0.0.0.0`, because a device that is off-network is exactly the case
where `/health` has to still answer.

`last_disconn_rc` is the `wifi_event_sta_disconnected_t.reason`
enum. `201` is `WIFI_REASON_AUTH_EXPIRE`, `202` is
`WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT`, `203` is
`WIFI_REASON_BEACON_TIMEOUT`. See `esp_wifi_types_generic.h` for
the full table — a persistent `203` means the AP stopped hearing
beacons, which is a coverage problem, not a firmware one.

---

## Network liveness

### What the device does when the link drops

Reconnection is owned by a dedicated FreeRTOS task, `sta_reconnect`,
and never by an event handler. `WIFI_EVENT_STA_DISCONNECTED` is
dispatched on the `esp_event` task, so anything that blocks there
stalls **every** event in the system, including IP and WiFi events.
The handler is therefore O(1): record the reason, clear
`s_sta_got_ip`, set the pending flag, poke the task, return.

The task blocks on `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)`, so a
healthy device spends no CPU on it. When poked it retries
**forever** at 2 / 4 / 8 / 16 / 30 s, capped at 30 s, resetting when
the station re-attaches. There is deliberately no attempt ceiling:
a device that stops retrying is the failure mode, not the fix.

In the log, `reconnect: esp_wifi_connect (backoff=Xms)` always means
"this attempt followed a sleep of X ms". The very first attempt after
a disconnect logs `attempt after disconnect` with no backoff label,
because it did not sleep.

`provisioning_supervise_sta()` is a separate 120 s backstop. It is
not part of the retry cadence — if the reconnect task is stuck, the
supervisor forces one `esp_wifi_connect()` so a wedged task cannot
become an outage.

### Heartbeat

`provisioning_supervise_sta()` also emits a periodic heartbeat
carrying the same fields as `/health`:

- station has an IP: every 10 ticks (~5 min)
- station has no IP: every tick (30 s)

The unhealthy cadence is deliberately chatty. An operator
diagnosing an outage needs to see the disconnect happen, not infer
it from silence.

### WiFi power save

`esp_wifi_set_ps(WIFI_PS_NONE)` is called after `esp_wifi_start()`
on both boot paths. The ESP-IDF default is `WIFI_PS_MIN_MODEM`,
which AP-buffers upstream ARP, ICMP, and DHCP traffic during DTIM
sleep. That is the right trade for a battery device and the wrong
one for a mains-powered camera that must always answer on the LAN.

### Task priority invariant

```
wifi_task (23) > esp_event (20) > httpd (5) > cam_stream (3)
```

The camera stream producer runs every 100 ms and does a full PSRAM
capture plus JPEG encode. If it outranks the httpd workers, `/whoami`
and `/capture` starve while the device looks alive. `cam_stream` is
deliberately below `httpd` for that reason; if you add a task that
serves a client-facing endpoint, put it at 5 or above.

### Operator recovery procedure

When a camera stops answering any endpoint:

1. `ping <ip>`. A reply means the link and IP are alive and the
   httpd is wedged — check `/health` heap numbers and whether the
   `/stream` CCTV grid was open. No reply, or an absent/stale ARP
   entry, means the station dropped off.
2. On the serial console, `reconnect: esp_wifi_connect` lines mean
   the station is actively retrying; `station connected ... ip=`
   means it recovered. Silence with no reconnect lines means the
   disconnect handler never fired — check whether the device is
   still associated (AP client list) and look for a
   `disconnected after IP was acquired` line, which would mean this
   build is not running.
3. There is no remote reboot endpoint, so recovery requires physical
   access. See "Follow-ups" in
   [`odd/tasks/iot-cams-network-liveness.md`](../../../../odd/tasks/iot-cams-network-liveness.md)
   for the deferred `/reboot` proposal and why it needs an explicit
   security decision first.

---

## Menuconfig

```
WiFi Provisioning  --->
  (abcd1234) Default proof-of-possession
  (abcd1234) SoftAP WPA2 passphrase
  (IoT-Cam) SoftAP SSID prefix
  (iot-cam) Device name exposed on the iot-cam-info endpoint
```

> The `PoP` symbol is preserved for API compatibility but is
> ignored at runtime in captive-portal mode — the WPA2
> passphrase is the operator-visible security gate.

`idf.py menuconfig` shows the menu after the package is
added to the build graph (path dependency resolves during
`idf.py reconfigure`).

---

## Factory-reset

Wipe the device by:

```c
provisioning_reset_credentials();   /* calls esp_wifi_restore() */
esp_restart();
```

Both calls live in the firmware; the package owns the reset path
only. Wiring this to a GPIO button (or another trigger) is
follow-up work — the package exposes the function so any caller
can wire it.

---

## Extending

| Need | Where to put it |
| --- | --- |
| Camera + capture + ws plane | a sibling component in `embedded/iot_cams/components/camera/`, etc. |
| BLE transport | reintroduce protocomm + wifi_provisioning as REQUIRES and write a separate scheme layer (out of scope for the offline-deployment branch). |
| Factory-reset GPIO | any caller; this package exposes the reset function, not the trigger. |
| True captive-portal OS-detection | standalone DNS server on the softAP that resolves all hostnames to 192.168.4.1; this is a meaningful follow-up, see [Out of scope](#out-of-scope-for-this-branch). |

---

## Dependencies (managed vs built-in)

| Component | Source | Why |
| --- | --- | --- |
| `provisioning`, `iot-cam-info` | this package | — |
| `wifi_provisioning`, `protocomm`, `esp_wifi`, `esp_netif`, `esp_event`, `nvs_flash` | built-in to ESP-IDF v5.5 | declared in `CMakeLists.txt` `REQUIRES` |
| `mdns` | **managed**, `espressif/mdns ^1.0.0` | moved out of the IDF tree in v5.5; declared here in `idf_component.yml::dependencies` |

The `mdns` dep is the only one that's not a built-in. Without it,
ESP-IDF v5.5.x's component manager fails with
`Failed to resolve component 'mdns' required by component 'provisioning'`
at `idf.py build` time. The lock file is frozen per consumer by
`idf.py reconfigure`; removing the dep locally requires updating
the lock in lockstep.

After the captive rewrite, `protocomm` and `wifi_provisioning`
were DROPPED from this list — the captive flow doesn't use either.

---

## Lifting to a managed component

The component is currently path-vendored from
`embedded/iot_cams/components/provisioning/`. To promote it to a
real upstream (registry, separate git repo, or private URL):

1. Move `embedded/iot_cams/components/provisioning/` to its own
   repo. Tag a release.
2. Edit `embedded/iot_cams/components/provisioning/idf_component.yml`
   `url:` to the upstream URL.
3. Edit `embedded/iot_cams/main/idf_component.yml` — replace
   `path: ../components/provisioning` with the new URL.

No header or implementation changes are needed. The repo lifts
without touching the firmware source.

---

## Out of scope for this branch

- Camera pipeline + WebSocket control plane + supervision tasks.
- BLE transport (see Extending table).
- True captive-portal OS auto-detection via a DNS server on the
  softAP. iOS/Android's "Sign in to network" prompt reads from
  `captive.apple.com` / `connectivitycheck.gstatic.com`, so
  without a DNS responder that resolves those to 192.168.4.1,
  the OS-prompt trick doesn't fire automatically. Today the
  operator must type `http://192.168.4.1/` explicitly (or follow
  the OS prompt if their device supports it). A LwIP-level DNS
  responder is the next milestone for full zero-touch UX.
- Host-side UNITY test bed inside `iot_cams/tests/`.
- TLS over the captive httpd. The WPA2 link is encrypted; for
  hostile environments the operator would need to switch to the
  protocomm path (see Extending table).
