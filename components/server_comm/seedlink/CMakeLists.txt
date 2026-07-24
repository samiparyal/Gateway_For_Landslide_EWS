idf_component_register(
    SRC_DIRS
        "."
        "mseed"
    
    INCLUDE_DIRS
        "include"
    
    PRIV_INCLUDE_DIRS
        "."
        "mseed"
    
    REQUIRES
    
    PRIV_REQUIRES
        debug
        system
        setting
        tcp_client
        sdcard
)