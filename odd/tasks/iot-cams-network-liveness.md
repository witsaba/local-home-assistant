# Feature: Network liveness + health observability for `embedded/iot_cams/`

## Problem statement

The deployed ESP32-CAM fleet intermittently becomes **completely
unreachable**: no `/whoami`, no `/capture`, no `/ws/cams`, no
response to any endpoint. The only recovery observed so far is a
power cycle.

Two independent defects were identified by read-only code analysis
(Engram `4407`). Both produce total unresponsiveness, and they are
indistinguishable from the outside without an L2/L3 discriminator.
This branch fixes both and adds the observability needed to
diagnose the next occurrence without a serial console.

### Defect A — no STA reconnect after the first successful lease

| Evidence | Location |
| --- | --- |
| The everyday reboot path registers only `IP_EVENT_STA_GOT_IP`; the `WIFI_EVENT_STA_DISCONNECTED` handler is registered exclusively in `softap_bring_up()`, which runs only on the softAP provisioning path | `components/provisioning/src/provisioning.c:563-639` vs `:277-282` |
| The handler refuses to act once an IP was obtained: `if (s_sta_got_ip) { "disconnected after IP was acquired — NOT retrying (will rely on app_main to handle)"; return; }` | `components/provisioning/src/provisioning.c:215-221` |
| app_main handles nothing — both "supervisor loops" are `while (1) { vTaskDelay(pdMS_TO_TICKS(60000)); }` | `main/iot_cams.c:224-229`, `:288-291` |
| `s_sta_got_ip` is never cleared after boot (only `provisioning_reset_sta_state()`, called solely on the pre-provisioning path), so it is a one-way latch | `components/provisioning/src/provisioning.c:655-659` |
| IDF v5.5.3 mandates the application own reconnection: *"The station may disconnect due to many reasons, e.g., the connected AP is restarted. It is the application's responsibility to reconnect."* | `esp-idf/docs/en/api-guides/wifi.rst:1281` |

`esp_wifi_connect()` is called only at boot (`:626`, `:713`) and in
the never-taken retry branch (`:245`).

**Consequence:** any AP-side event — deauth, router reboot, band
steering, lease loss, idle timeout — leaves the camera off-network
permanently until power cycle.

### Defect B — httpd socket-pool exhaustion + worker starvation

`sta_server.c:265-269` uses `HTTPD_DEFAULT_CONFIG()` overriding only
`server_port`, `stack_size`, `max_uri_handlers`, `max_req_hdr_len`.
The socket pool therefore stays at the IDF default
`max_open_sockets = 7` with **`lru_purge_enable = false`**
(verified in `esp-idf/components/esp_http_server/include/esp_http_server.h`).

- Each `/ws/cams` viewer holds one socket for its whole session.
  Single-viewer enforcement already exists
  (`ws_cams_check_single_viewer`), so the CCTV grid of three cameras
  from PR #38 holds 3 of 7 sockets permanently.
- With `lru_purge_enable = false`, httpd does not reclaim idle sockets
  when the pool fills; new connections stall in `backlog_conn = 5`.
  A TCP connect can succeed and the response never arrive.
- The worker pool is `max_uri_handlers = 8` threads at
  `task_priority = tskIDLE_PRIORITY + 5` = **priority 1**, while
  `cam_stream` runs at **priority 5** (`components/cam_stream/cam_stream.c:265-271`)
  waking every 100 ms for a full PSRAM double-buffer
  `esp_camera_fb_get` + JPEG encode (`fb_count=2`,
  `components/cam_reader/cam_reader.c:60`). The producer outranks the
  HTTP server that discovery, surveillance, and the UI all depend on.
- `/capture` can additionally pin a thread and a socket for the full
  5 s camera-semaphore wait (`CAM_READER_WAIT_MS = 5000`,
  `components/cam_reader/cam_reader.c:76`).

## Goal

The camera rejoins the network by itself after any link loss, the
HTTP server stops starving under load, and a single `GET /health`
tells an operator — or the Pi — whether the chip is off-network, wedged,
or healthy.

## Non-goals

- **No OTA.** `partitions.csv` has no `ota_0`/`ota_1`; every fix
  costs one serial reflash of all three devices. Out of scope.
- **No remote `/reboot` endpoint.** A LAN-exposed restart is an
  unauthenticated DoS vector. Deferred pending an explicit security
  decision from the maintainer.
- **No `sdkconfig.defaults` change.** Nothing here needs a new Kconfig
  option.
- **No change to the softAP captive portal UX.**
- **No control plane on the WS socket** (deferred follow-up).

## Design decisions (locked)

| Decision | Choice | Rationale |
| --- | --- | --- |
| Reconnect ownership | A **dedicated FreeRTOS task** owns all `esp_wifi_connect()` retries | The `esp_event` task must never block. The current `vTaskDelay()` at `provisioning.c:244` stalls *all* event dispatch for 2–30 s |
| Retry ceiling | **None — retry forever**, backoff 2/4/8/16/30 s, cap 30 s | A device that stops retrying is the bug. `MAX_RETRY 5` existed only to protect the provisioning path from scan starvation; the supervisor (below) covers that concern far better |
| Disconnect handler | Fast: record reason, clear `s_sta_got_ip`, set a pending flag, return. **No sleep, no connect call** | Keeps `WIFI_EVENT_STA_DISCONNECTED` dispatch O(1) |
| Honest state | `s_sta_got_ip` is cleared on **every** disconnect | Removes the one-way latch. Also fixes the softAP-teardown decision, which reads that flag |
| Handler registration | One idempotent `ensure_sta_event_handlers()` helper called from **both** `join_ap()` and `softap_bring_up()` | Fixes the missed registration on the normal reboot path. Tolerates `ESP_ERR_INVALID_STATE` from duplicate `esp_event_handler_register` so the existing "idempotent" comments become true |
| Liveness supervisor | `provisioning_supervise_sta()` exposed in `provisioning.h`; app_main calls it from its loop | `main/iot_cams.c` forbids `esp_wifi.h` by design. The supervisor lives behind the component boundary that owns WiFi, and it makes the existing log string honest |
| Supervisor cadence | 30 s tick; forces `esp_wifi_connect()` after 60 s without an IP; emits a heartbeat log every tick | Belt and braces over the reconnect task, plus free observability |
| WiFi power save | `esp_wifi_set_ps(WIFI_IF_STA, WIFI_PS_NONE)` after `esp_wifi_start()` on **both** paths | Default is `WIFI_PS_MIN_MODEM`, which AP-buffers upstream ARP/ICMP/DHCP during DTIM sleep. An always-reachable LAN device must not sleep |
| Socket pool | `max_open_sockets = 12`, `lru_purge_enable = true` | 3 permanent WS viewers + 3 surveillance captures + discovery polling + slack. `lru_purge_enable = true` makes the pool self-reclaim under pressure — the missing safety net |
| `recv_wait_timeout` | 5 s → **3 s** | Bounds how long a half-dead client pins a thread plus socket. Request parsing needs no longer |
| `send_wait_timeout` | stays at the 5 s default | A ~30 KB JPEG on a weak link genuinely needs it; shortening this would break slow viewers |
| httpd task priority | `tskIDLE_PRIORITY+5` (=1) → **5** | httpd workers must not sit below the camera producer |
| Stream task priority | `CAM_STREAM_TASK_PRIO` 5 → **3** | Priority invariant: `wifi_task` (23) > `esp_event` (20) > `httpd` (5) > `cam_stream` (3). The producer must never outrank the HTTP server |
| Health endpoint | New `GET /health` on the **existing** STA httpd | No new port, no new component, no new dependency. Same audience as `/whoami` |
| Health payload | uptime, free internal heap, minimum-ever-free heap, free PSRAM, IP, RSSI, reconnect count, last disconnect reason, WS viewer attached | Distinguishes *off-network* (IP 0.0.0.0) from *wedged* (IP present, heap collapsing) from *healthy* — the exact question the current firmware cannot answer |
| `/whoami` contract | `"fw"` returns the **app** version (`0.1.0`), add `"idf"` for the ESP-IDF version, add `ip`/`rssi`/`uptime_s` | `"fw"` currently reports `esp_get_idf_version()` (`sta_server.c:143`) — it misreports what firmware a device runs, which is the one thing needed during an incident. Go's `encoding/json` ignores unknown fields, so the additive change is safe for the backend |
| FW version plumbing | `provisioning.c` stores `info->fw_version` at init and exposes `prov_fw_version()` | Mirrors the existing `prov_device_name()` seam already declared `extern` in `sta_server.c`. Avoids exporting a symbol from `main/` |
| Diagnostics plumbing | `provisioning_reconnect_count()`, `provisioning_last_disconnect_reason()` in `provisioning.h` | Keeps the counters private to the component that owns them |

## Public API additions

```c
/* components/provisioning/include/provisioning.h */

/* Called from app_main's supervisor loop every tick. Owns the
 * no-IP watchdog and emits the heartbeat log. */
esp_err_t provisioning_supervise_sta(void);

/* Monotonic count of reconnect attempts issued since boot. */
uint32_t provisioning_reconnect_count(void);

/* wifi_event_sta_disconnected_t.reason from the last disconnect,
 * or 0 if none since boot. */
uint8_t provisioning_last_disconnect_reason(void);

/* Application firmware version recorded at provisioning_init()
 * from provisioning_app_info_t.fw_version. Never NULL. */
const char *prov_fw_version(void);
```

## Tasks

| # | Task | Files | Commit shape |
| --- | --- | --- | --- |
| T1 | STA reconnect task + idempotent handler registration on both boot paths | `provisioning.c`, `provisioning.h` | `fix(provisioning): reconnect the station after any disconnect` |
| T2 | Liveness supervisor + heartbeat behind the component boundary | `provisioning.c`, `provisioning.h`, `main/iot_cams.c` | `feat(provisioning): STA liveness supervisor with heartbeat log` |
| T3 | Disable WiFi modem sleep on both paths | `provisioning.c` | `fix(provisioning): disable STA power save for LAN reachability` |
| T4 | Socket pool, LRU purge, recv timeout, httpd vs stream priority | `sta_server.c`, `cam_stream.c` | `fix(sta-server): size the socket pool and fix task priorities` |
| T5 | `GET /health` diagnostics endpoint | `sta_server.c` | `feat(sta-server): GET /health diagnostics endpoint` |
| T6 | `/whoami` identity correctness (`fw`, `idf`, `ip`, `rssi`, `uptime_s`) | `sta_server.c`, `provisioning.c`, `provisioning.h` | `fix(sta-server): report app fw and live state on /whoami` |
| T7 | Component README operator/diagnostic section + build evidence | `components/provisioning/README.md`, this file | `docs(provisioning): document network liveness and /health` |

Each task is one work-unit commit, Conventional Commit subject,
English artifacts.

## Acceptance criteria

1. `idf.py build` completes with exit 0 and **zero warnings** from a
   clean build directory. Build output tail recorded in this file.
2. No new managed dependency; `dependencies.lock` unchanged.
3. `git diff main...HEAD --stat` touches only the files listed above.
4. `components/provisioning/README.md` documents: what `/health`
   returns, the reconnect/backoff behaviour, the priority invariant,
   and the operator recovery procedure.
5. Manual verification on real hardware is **operator-gated** and
   recorded as pending, not claimed (Engram `4273`: never report
   firmware work complete without build evidence).

## Operator verification recipe (pending)

1. Flash each device from the worktree build.
2. Normal-path check: `curl -s http://<ip>/whoami` returns the new
   `fw`/`ip`/`rssi`/`uptime_s` fields; `curl -s http://<ip>/health`
   returns 200 with non-zero `uptime_ms`.
3. Defect A check: power-cycle the router (or force a deauth). The
   device must rejoin **without a power cycle** — the log shows
   `sta disconnected (reason=N)` followed by `reconnect:` lines and
   then `station DHCP lease acquired`.
4. Defect B check: open the `/stream` CCTV grid with all three cameras
   live, leave it up, then confirm `curl /whoami` still returns
   promptly and `curl /health` still answers.
5. Confirm `job_count` equivalent: the heartbeat line appears in
   `journalctl`/serial within 30 s of boot.

## Evidence

| Task | Commit | Build | Notes |
| --- | --- | --- | --- |
| T1+T2+T3 | `d6a3eca` | verified at `5657642` | Shipped as one commit: all three touch the same WiFi state machine in `provisioning.c` and are not separable by file. Coherent as "the station recovers and proves liveness". |
| T4 | `9c3be8c` | verified at `5657642` | Isolated as its own commit by splitting `sta_server.c` at the hunk boundary, so the capacity fix is reviewable apart from the additive observability. |
| T5+T6 | `5657642` | **exit 0, zero warnings** | Both endpoints live in `sta_server.c` and share the buffer-resize rationale. |
| T7 | (this commit) | n/a (docs) | — |

### Build verification

`idf.py fullclean && idf.py build`, ESP-IDF v5.5.3, parent-run:

```
iot_cams.bin binary size 0x100a50 bytes.
Smallest app partition is 0x180000 bytes. 0x7f5b0 bytes (33%) free.
Project build complete.
```

- Exit code **0**
- `warning:` / `error:` lines across the full build log: **0**
- Binary `0x100a50` (1 049 680 B), +4 288 B over the `main` baseline `0xff590`
- 33 % of the 1.5 MB app partition still free
- `sdkconfig*`, `partitions.csv`, and every `CMakeLists.txt` unchanged

### Review findings corrected during implementation

The first implementation pass built clean but carried two
showstopping defects, caught in review and fixed before any commit:

1. **`sta_reconnect_task` busy-spun a core in steady state.** With
   `s_sta_disconnected_pending == false` the task `continue`d to the
   top of the loop with no blocking call anywhere on that path. At
   priority 4 it starved `cam_stream` at priority 3, which would
   have killed the WebSocket stream on every device from boot —
   worse than the outage being fixed. Now blocks on
   `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)`.
2. **The first reconnect attempt was delayed ~30 minutes.** The
   inner `while (pending && elapsed < 1800000UL)` slept before doing
   anything, so the 2/4/8/16/30 s backoff was computed *after* that
   wait and never governed the cadence. Also masked by the
   supervisor calling `esp_wifi_connect()` on every tick, which had
   inverted the plan's ownership: the supervisor, not the task, was
   restoring the link.

Also corrected: `s_reconnect_count` was incremented in two places
(now owned solely by the task); the "heartbeat" logged exactly once
per boot instead of periodically, which would have left a wedged
device indistinguishable from a healthy one in the log; and the first
reconnect attempt logged a `backoff=2000ms` label while actually
sleeping 4 s.

## Acceptance status

| # | Criterion | Status |
| --- | --- | --- |
| 1 | `idf.py build` exit 0, zero warnings, clean build dir | **PASS** |
| 2 | No new managed dependency | **PASS** — `dependencies.lock` untouched |
| 3 | Diff touches only the listed files | **PASS** — 6 files, +779/-85 vs `main` |
| 4 | README documents `/health`, reconnect behaviour, priority invariant, recovery procedure | **PASS** — also corrected the `/whoami` section, which documented a softAP-era contract the code never had |
| 5 | Hardware verification | **PENDING — operator-gated.** Not claimed. |

## Follow-ups (not in this branch)

- OTA partitions, so the next firmware fix costs one flash per device
  instead of three.
- A guarded `POST /reboot` **plus** a Pi-side remediation job that
  reboots a camera silent for N ticks. Needs an explicit security
  decision from the maintainer before any implementation.
- Consider `esp_wifi_set_ps(WIFI_PS_MIN_MODEM)` behind a config
  option once the fleet has stable WiFi coverage.
