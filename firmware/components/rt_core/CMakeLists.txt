# rt_core is deliberately pure C with no ESP-IDF dependencies: all of the logic worth
# testing (wire format, the observation window, the mode/radio truth table) builds and runs
# on a host, so it can be verified without hardware. See test_host/.
idf_component_register(
    SRCS
        "src/rt_chan.c"
        "src/rt_payload.c"
        "src/rt_obs.c"
        "src/rt_mode.c"
        "src/rt_report.c"
    INCLUDE_DIRS
        "include"
)
