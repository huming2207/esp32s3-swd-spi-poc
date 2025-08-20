idf_component_register(
        SRCS
            "./source/DAP.c" "./source/dap_utility.c" "./source/DAP_vendor.c"
            "./source/JTAG_DP.c" "./source/spi_op.c" "./source/spi_switch.c"
            "./source/SW_DP.c" "./source/swd_host.c"

        INCLUDE_DIRS
            "./include" "./config"

        REQUIRES
            hal driver
)