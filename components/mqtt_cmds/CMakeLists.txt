idf_component_register(SRCS "mqtt_cmds.c"
                    INCLUDE_DIRS "include"
                    REQUIRES mqtt json gatt_client)