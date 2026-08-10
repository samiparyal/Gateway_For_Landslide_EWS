idf_component_register(SRCS "server_comm.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bt esp_http_client json mbedtls)
