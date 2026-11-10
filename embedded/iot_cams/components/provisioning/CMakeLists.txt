# components/provisioning/CMakeLists.txt
#
# Registers the iot_cams WiFi provisioning component. Wraps the
# IDF v5.5 `wifi_provisioning` component (SoftAP scheme, security 1)
# and exposes a strict 5-function public C API to the rest of the
# firmware so app_main never sees protocomm, mDNS or esp_wifi
# directly during provisioning.
#
# REQUIRES — pulls IDF's vendored components so the build graph is
# complete:
#   - esp_wifi / esp_netif / esp_event — base radio + TCP/IP stack
#     + event loop, needed by the softAP bring-up
#   - nvs_flash  — credential persistence (esp_wifi_set_config)
#   - mdns       — captive portal's service advertising
#   - esp_http_server — captive portal's GET /, /provision, /scan
#     and sta_server's GET /whoami handlers
#   - json       — JSON /whoami response payload
#   - lwip       — raw loopback socket used by the /health self-probe
#     (the self-recovery liveness signal; deliberately not
#     esp_http_client, so the firmware gains no new managed dep)
#
# protocomm and wifi_provisioning are NO LONGER linked because
# the captive portal flow talks to esp_wifi directly without the
# manager layer (replaced after the shared-httpd crash on the
# device flash session).
idf_component_register(
    SRCS "src/provisioning.c"
         "src/captive_portal.c"
         "src/sta_server.c"
         "src/ws_cams.c"
         "src/wifi_cred.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_netif esp_event nvs_flash mdns esp_http_server json cam_reader cam_stream lwip
)
