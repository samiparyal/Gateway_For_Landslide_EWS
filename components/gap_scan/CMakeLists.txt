idf_component_register(SRCS "gap_scan.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bt json server_comm json_builder gatt_client)
