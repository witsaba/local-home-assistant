# ESP32 cam self-recovery — incremental escalation with conditional reboot

Feature branch: `feat/iot-cams-network-liveness` (rebased onto `main` @ `ec39d6b`).
Chip-side only. Builds one firmware image for all three devices — no OTA partition exists,
so every shipped change costs one manual serial flash per device.

## Specs

**S1 — The operator's original resilience problem, verbatim:**
"the energy cuts are common here and the devices doens't have yet a battery on it, but when the energy comes, usually the WiFi takes more time to be ready, but sometimes, after 2 or 3 days the devices just disconnect and never again connect until I turn-off wait 3 seconds and turn-on again"

**S2 — The operator's proposal, verbatim:**
"is difficult to track the device due to is not connectect 24/7 to a computer, but wath if programatically the device reboot itself in any way cleaning memory and stuff?"

**S3 — The operator's chosen trigger policy, verbatim:**
"Conditional: reboot only when unhealthy"

**S4 — The operator's hard constraint on the mechanism, verbatim:**
"but this must to be incremental again, there is points where the WiFi is not ready, even, this edgecase, when the energy is very low the chips runs, but the router doens't"

**S5 — The device must recover from a power cut where the router comes back later than the chip.**
After a power cut the ESP32 may boot and run while the router is still off or still booting.
The device MUST reach the home AP on its own once the router appears, with no human action and
no reboot. The existing `sta_reconnect_task` on this branch already satisfies this: it retries
`esp_wifi_connect()` indefinitely with backoff 2→4→8→16→30 s capped at 30 s, and has **no
attempt ceiling** (`components/provisioning/src/provisioning.c:335-410`). That ceiling-free
behaviour MUST be preserved. Any change that reintroduces an attempt limit breaks S5.

**S6 — A reboot MUST NOT be used as a response to a missing router.**
A reboot cannot make a dead router come alive. When the home AP is absent the correct response
is to keep retrying quietly and indefinitely. Rebooting in that state is pure waste and, under
a long power event, produces a reboot storm that is indistinguishable from the original fault.
The device MUST distinguish "the router is not there" from "the router is there and we cannot
speak to it" and MUST only escalate to a reboot for the second class.

**S7 — The escalation MUST be incremental, cheapest action first.**
The device MUST escalate through recovery steps in increasing cost, and MUST NOT jump straight
to a reboot:
  - Step 1 — reconnect: the existing `sta_reconnect_task` (already present on this branch).
  - Step 2 — re-associate: `esp_wifi_disconnect()` followed by `esp_wifi_connect()`, clearing
    stale driver/BSSID state without losing the process.
  - Step 3 — reboot: `esp_restart()`, a full reset of CPU, radio and driver state.
Each step is given a dwell window. A step that succeeds ends the escalation and resets the
device to healthy. Steps 1 and 2 are attempted before step 3 is ever considered.

**S8 — The reboot trigger is "unhealthy", never "scheduled".**
The device MUST NOT reboot on a fixed schedule while healthy. It MUST reboot only when it
detects that it is not serving. Healthy devices never reboot, so a healthy device never pays
the downtime cost.

**S9 — The liveness signal MUST be independent of any external observer.**
The Pi stops polling a device that has been unreachable for 5 minutes
(`SURVEILLANCE_DEVICE_FRESHNESS_MINUTES=5`, `services/workers/internal/infrastructure/config/config.go:68`;
`ListFresh`, `services/workers/internal/infrastructure/devices/store.go:156-170`). A Pi-side
remediation job would therefore never see a dead device. The device MUST detect its own
illness with no network, no PC and no Pi involved.

**S10 — Two independent liveness signals, both required to be stale before escalating.**
  - (a) Camera progress: `cam_reader_frames_captured_get()` must keep advancing
    (`components/cam_reader/cam_reader.c:118`). It advances on every capture attempt
    regardless of whether a viewer is connected, so it detects a wedged camera or sensor bus.
    A camera that is failing (`ESP_FAIL` from `esp_camera_fb_get`) advances
    `cam_reader_fb_drops_get()` instead of `cam_reader_frames_captured_get()`.
  - (b) HTTP self-probe: the supervisor MUST periodically issue a loopback `GET /health`
    against `127.0.0.1` and require a response. A camera that is healthy but whose HTTP
    server is wedged still advances signal (a), so signal (a) alone cannot detect a wedged
    httpd. Both signals must be stale before the device escalates.
  Neither signal may require an external client.

**S11 — A reboot storm guard is mandatory.**
Reboots MUST be rate-limited by a persisted counter so that a device which reboots and is
still unhealthy does not reboot again immediately. The counter MUST escalate the wait between
reboots and MUST reset once the device proves healthy again, so a device that had one bad
week does not carry a long backoff forever. Rebooting is a current spike (PSRAM init, camera
XCLK, WiFi TX); under a marginal supply an unguarded timer turns the fix into the disease.

**S12 — A boot-time NVS failure MUST NOT leave the device inert.**
`provisioning_init()` currently handles only `ESP_ERR_NVS_NO_FREE_PAGES` and
`ESP_ERR_NVS_NEW_VERSION_FOUND` (`components/provisioning/src/provisioning.c:365-378`). Any
other error — for example `ESP_ERR_NVS_CORRUPT` after a brownout during an NVS write — makes
`provisioning_init()` return an error, `app_main` returns, and the device is powered but
completely dead: no camera, no SoftAP, no HTTP server. That state is flash-resident, so a
reboot does NOT fix it. The boot path MUST treat an unusable NVS as recoverable: erase and
re-initialise, then continue, so the device comes up in SoftAP provisioning mode where the
operator can re-provision it. A device that is visibly asking to be provisioned is recoverable;
a device that is silently inert is not.

**S13 — No wall-clock time source may be introduced.**
The firmware has no SNTP, no RTC and no timer infrastructure — `esp_timer_create` appears
nowhere in the codebase, and the only time base is `esp_timer_get_time()` (microseconds since
boot). All timing in this feature MUST be derived from uptime or tick counts. No NTP client
is to be added for this feature.

**S14 — `/health` MUST remain answerable in every state and MUST expose the recovery state.**
The endpoint MUST never return 500, including when the device is off-network. It MUST expose
enough state for an operator to tell, without a serial console, whether the device is healthy,
offline because the router is absent, offline because the link failed, or wedged — plus the
reboot counter, the last reset reason and the two liveness signals.

**S15 — The reset reason MUST be observable.**
On every boot the device MUST log `esp_reset_reason()` so a self-inflicted software reset
(`ESP_RST_SW`) is distinguishable in the serial log from a power-loss or watchdog reset.

**S16 — Boot and runtime logs MUST classify the outage.**
The device MUST name the outage class in its logs so an operator reading a serial console can
tell "the router is not there" from "the router is there and we cannot connect" without
reading source. The disconnect reason code from `wifi_event_sta_disconnected_t.reason` is the
available discriminator; `WIFI_REASON_NO_AP_FOUND` (201) means the AP was not found in the
scan, i.e. the router is absent.

**S17 — All work lands on the existing liveness branch, as one firmware image.**
No OTA partition exists, so each PR that changes firmware costs a manual serial flash of all
three devices. The liveness work already on this branch and this self-recovery work MUST ship
together in one branch and one image, not as two PRs requiring two flash rounds.

## Tasks

| ID | Task | Specs | Route | Commit |
|---|---|---|---|---|
| T1 | NVS boot path must never leave the device inert: handle `ESP_ERR_NVS_CORRUPT` and any other unexpected NVS error by erasing, re-initialising and continuing into SoftAP provisioning; log the recovery loudly | S12 | worker | `fix(provisioning): an unusable NVS must not brick the device` |
| T2 | Persisted reboot counter + escalating backoff in a dedicated NVS namespace, with a reset-on-healthy rule | S11, S13 | worker | `feat(provisioning): persisted reboot counter with escalating backoff` |
| T3 | Outage classification from `s_last_disconnect_reason`: `ROUTER_ABSENT` (reason 201) vs `LINK_FAILED` (any other reason) vs `WEDGED` (IP present, signals stale) | S6, S16 | worker | `feat(provisioning): classify the outage before escalating` |
| T4 | Incremental escalation ladder in the supervisor: reconnect (existing task) → re-associate (`esp_wifi_disconnect()` + `esp_wifi_connect()`) → reboot, each with its own dwell window; the `ROUTER_ABSENT` class MUST NEVER reach the reboot step | S5, S6, S7 | worker | `feat(provisioning): incremental escalation to a conditional reboot` |
| T5 | Liveness probe: camera-frame counter plus a loopback `GET /health` self-probe, both required to be stale before escalation | S8, S9, S10 | worker | `feat(provisioning): self-probing liveness signals` |
| T6 | `/health` extension: uptime, reboot counter, last reset reason, outage class, last disconnect reason, both liveness signals; never 500 | S14 | worker | `feat(sta-server): expose recovery state on /health` |
| T7 | Boot-time `esp_reset_reason()` logging and the operator-facing README section for the escalation ladder and its log lines | S15, S16 | worker | `docs(provisioning): document the escalation ladder` |
| T8 | Full clean build verification (`idf.py fullclean && idf.py build`), zero warnings, plus a differential check that `/whoami`, `/capture` and `/ws/cams` behaviour is unchanged | S17 | parent | `build: verify iot_cams after self-recovery` |

## Log

**L1** (user, verbatim): "ok, analyze the engram memories and the firmware for the esp32 cams, tell me your exploration results"

**L2** (user, verbatim): "I wanna worry about the resilent of the devices. I got a problem and is that the energy cuts are common here and the devices doens't have yet a battery on it, but when the energy comes, usually the WiFi takes more time to be ready, but sometimes, after 2 or 3 days the devices just disconnect and never again connect until I turn-off wait 3 seconds and turn-on again. I need you explore why is this posible. and, do not think yet in the solution, only try to find the current implemen tation fails points."

**L3** (user, verbatim): "is difficult to track the device due to is not connectect 24/7 to a computer, but wath if programatically the device reboot itself in any way cleaning memory and stuff?"

**L4** (user, verbatim): "the device still turned off after a while but not responding through the network." — interpreted as: the device is still powered but no longer answers over the network.

**L5** (user, verbatim, answering the trigger-policy question): "Conditional: reboot only when unhealthy"

**L6** (user, verbatim): "but this must to be incremental again, there is points where the WiFi is not ready, even, this edgecase, when the energy is very low the chips runs, but the router doens't" — this is the constraint that became S4, S5, S6 and S7: a reboot must never be the response to a router that is not up yet.

**L7** (agent, evidence): The exploration established seven failure points. Seven are recorded in Engram observation 4407 under topic `embedded/iot_cams/network-liveness/no-reconnect-defect`: (A) no STA reconnect after the first lease — the `WIFI_EVENT_STA_DISCONNECTED` handler is registered only in `softap_bring_up()` on `main`, and `s_sta_got_ip` is a one-way latch that refuses to retry; (B) httpd socket-pool exhaustion (`max_open_sockets=7`, `lru_purge_enable=false`) plus priority inversion between the priority-1 httpd worker and the priority-5 `cam_stream` task, with a binary semaphore that has no FreeRTOS priority inheritance; (C) NVS corruption bricking `app_main`; plus the port's divergence from its own reference project `rural_home_assistant/backend/iot-camera/components/wifi/wifi.c`, which reconnects on every disconnect and is armed before `esp_wifi_start()`.

**L8** (agent, evidence): `feat/iot-cams-network-liveness` was rebased onto `main` @ `ec39d6b` with no conflicts — `embedded/` had not changed since the branch's base `0d9e9ba`. Rebased commits: `7a4bab0` plan, `dcf8225` station recovery, `e4aee2b` socket pool + priorities, `9147f5c` `/health` + `/whoami`, `e0e4147` docs.

**L9** (agent, decision): The liveness work and this self-recovery work ship as ONE branch and ONE firmware image, per S17 — no OTA partition means each firmware PR costs a serial flash of three devices.

**L10** (agent, decision): The reboot is placed ABOVE the existing ceiling-free reconnect retry, never instead of it, per S5 and S6. The `ROUTER_ABSENT` class is explicitly excluded from the reboot step: rebooting cannot make a router come alive, and under a long power event it produces a storm indistinguishable from the original fault.
