idf_component_register(
      SRCS ota_ws_update_esp.c
      REQUIRES "driver" "esp_event" "esp_http_client" "esp_http_server" "esp_https_ota"
      "app_update" "esp_netif" "esp_wifi" "esp_https_server" "main"
      INCLUDE_DIRS include
)