set(reqs nvs_flash esp_wifi esp_event esp_timer esp_netif driver bt)

# The ieee802154 component does not exist on targets without the radio (C3, S3), so
# requiring it unconditionally would break those builds outright.
if(CONFIG_SOC_IEEE802154_SUPPORTED)
    list(APPEND reqs ieee802154)
endif()

idf_component_register(
    SRCS
        "main.c"
        "rt_stats.c"
        "rt_espnow.c"
        "rt_ble.c"
        "rt_154.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        ${reqs}
)
