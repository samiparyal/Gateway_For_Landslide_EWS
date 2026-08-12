idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "." "include"
                    REQUIRES gap_scan server_comm gatt_client seedlink wifi_conn gsm_conn
                    PRIV_REQUIRES nvs_flash)
