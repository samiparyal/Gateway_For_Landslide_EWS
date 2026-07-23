idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES gap_scan server_comm gatt_client
                    PRIV_REQUIRES nvs_flash)
