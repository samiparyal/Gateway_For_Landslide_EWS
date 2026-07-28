idf_component_register(
    SRCS "wifi_conn.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_netif nvs_flash
)