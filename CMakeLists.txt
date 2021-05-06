idf_component_register(
    SRCS "m5core2_axp192.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES axp192 i2c_manager
)
