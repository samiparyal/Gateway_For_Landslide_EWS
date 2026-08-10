idf_component_register(SRCS "gsm_conn.c"
                        INCLUDE_DIRS "."
                        REQUIRES esp_netif esp_event driver esp_modem)

 #driver for gpio.h (PWRKEY)