idf_component_register(SRCS "poc_main.cpp" "swd_spi_raw.c" "swd_spi_raw.h"
                    INCLUDE_DIRS "."
        REQUIRES driver hal esp_swd_spi)
