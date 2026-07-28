idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES gap_scan server_comm gatt_client seedlink wifi_conn
                    PRIV_REQUIRES nvs_flash)
