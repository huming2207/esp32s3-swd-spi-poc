#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>
#include <hal/spi_types.h>
#include "swd_spi_raw.h"

#include <swd_host.h>

extern "C" void app_main(void)
{
//    ESP_ERROR_CHECK(swd_spi_init(GPIO_NUM_1, GPIO_NUM_2, 1000000, SPI2_HOST));
//    ESP_ERROR_CHECK(swd_spi_reset());
//    ESP_ERROR_CHECK(swd_spi_switch());
//    ESP_ERROR_CHECK(swd_spi_reset());
//    swd_spi_send_trn(2); // WTF??
//
//    vTaskDelay(10);
//
//    uint32_t idcode = 0;
//    if (swd_spi_read_idcode(&idcode) != ESP_OK) {
//        ESP_LOGE("main", "Failed to read IDCODE");
//    }
//    ESP_LOGI("main", "IDCODE = 0x%x", idcode);

    uint8_t ret = swd_init_debug();
    ESP_LOGI("main", "ret=%u", ret);

    vTaskDelay(portMAX_DELAY);
}
