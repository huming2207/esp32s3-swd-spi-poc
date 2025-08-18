#include <esp_err.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <soc/spi_struct.h>
#include <hal/spi_ll.h>
#include <string.h>
#include "swd_spi_raw.h"

#ifdef TAG
#undef TAG
#endif

#define TAG "swd_spi"

static spi_device_handle_t spi_handle = NULL;
static volatile spi_dev_t *spi_dev = (volatile spi_dev_t *)(DR_REG_SPI2_BASE);

static const uint8_t parity_table[256] = {
    #define P2(n) n, n^1, n^1, n
    #define P4(n) P2(n), P2(n^1), P2(n^1), P2(n)
    #define P6(n) P4(n), P4(n^1), P4(n^1), P4(n)

    P6(0), P6(1), P6(1), P6(0)
};

static inline size_t div_round_up(size_t a, size_t b)
{
    return (a + b - 1) / b;
}

static inline uint8_t calc_parity_u32(uint32_t v)
{
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v &= 0xf;
    return (0x6996 >> v) & 1;
}

static inline uint8_t calc_parity_u8(uint8_t v)
{
    return parity_table[v];
}

esp_err_t swd_spi_init(gpio_num_t swclk, gpio_num_t swdio, uint32_t freq_hz, spi_host_device_t host)
{
    spi_bus_config_t spi_bus_config = {};
    spi_bus_config.mosi_io_num     = swdio; // SWD I/O
    spi_bus_config.miso_io_num     = -1; // not connected
    spi_bus_config.sclk_io_num     = swclk; // SWD CLK
    spi_bus_config.quadwp_io_num   = -1;
    spi_bus_config.quadhd_io_num   = -1;
    spi_bus_config.max_transfer_sz = 0;
    // spi_bus_config.flags = SPICOMMON_BUSFLAG_IOMUX_PINS;

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << swdio) | (1ULL << swclk);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_set_drive_capability(swdio, GPIO_DRIVE_CAP_0);
    gpio_set_drive_capability(swclk, GPIO_DRIVE_CAP_0);

    esp_err_t ret = spi_bus_initialize(host, &spi_bus_config, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI Bus init fail");
        return ret;
    }

    spi_device_interface_config_t spi_dev_inf_config = {};
    spi_dev_inf_config.command_bits        = 0;
    spi_dev_inf_config.address_bits        = 0;
    spi_dev_inf_config.dummy_bits          = 0;
    spi_dev_inf_config.mode                = 0;
    spi_dev_inf_config.duty_cycle_pos      = 0;
    spi_dev_inf_config.cs_ena_pretrans     = 0;
    spi_dev_inf_config.cs_ena_posttrans    = 0;
    spi_dev_inf_config.clock_speed_hz      = (int)freq_hz;
    spi_dev_inf_config.spics_io_num        = -1;
    spi_dev_inf_config.flags               = SPI_DEVICE_3WIRE | SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_BIT_LSBFIRST;
    spi_dev_inf_config.queue_size          = 24;
    spi_dev_inf_config.pre_cb              = NULL;
    spi_dev_inf_config.post_cb             = NULL;

    ret = spi_bus_add_device(host, &spi_dev_inf_config, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device init fail");
        return ret;
    }


    ret = spi_device_acquire_bus(spi_handle, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI lock fail");
        return ret;
    }

    switch (host) {
        case SPI1_HOST: {
            spi_dev = (volatile spi_dev_t *)(DR_REG_SPI1_BASE);
            break;
        }

        case SPI2_HOST: {
            spi_dev = (volatile spi_dev_t *)(DR_REG_SPI2_BASE);
            break;
        }

        case SPI3_HOST: {
            spi_dev = (volatile spi_dev_t *)(DR_REG_SPI3_BASE);
            break;
        }

        default: {
            ESP_LOGE(TAG, "Unknown SPI peripheral selected");
            return ESP_ERR_INVALID_ARG;
        }
    }

    return ret;
}

esp_err_t swd_spi_wait_till_ready(int32_t timeout_cycles)
{
    while(timeout_cycles >= 0 && spi_dev->cmd.usr != 0) {
        timeout_cycles--;
    }

    if (timeout_cycles < 0) {
        return ESP_ERR_TIMEOUT;
    } else {
        return ESP_OK;
    }
}

void swd_spi_send_cycles(uint32_t clk_cycles, uint32_t swclk_level)
{
    spi_dev->user.usr_dummy = 0;
    spi_dev->user.usr_command = 0;
    spi_dev->user.usr_mosi = 1;
    spi_dev->user.usr_miso = 0;
    //spi_dev->misc.ck_idle_edge = 0;
//    spi_dev->user.usr_hold_pol = 1;
//    spi_dev->user.usr_dout_hold = 0;
   // spi_dev->user.ck_out_edge = 0;
    spi_dev->ms_dlen.ms_data_bitlen = clk_cycles - 1;

    for (uint32_t idx = 0; idx < 18; idx++) {
        spi_dev->data_buf[idx] = swclk_level;
    }


    spi_dev->cmd.update = 1;
    while (spi_dev->cmd.update);

    spi_dev->cmd.usr = 1; // Trigger Tx!!
}

esp_err_t swd_spi_read_idcode(uint32_t *idcode)
{
    uint8_t tmp_in[1];
    uint8_t tmp_out[4];
    tmp_in[0] = 0x00;
    swd_spi_send_bits(tmp_in, 8);

    if (swd_spi_read_dp(0, (uint32_t *)tmp_out) != 0x01) {
        return ESP_FAIL;
    }

    *idcode = (tmp_out[3] << 24) | (tmp_out[2] << 16) | (tmp_out[1] << 8) | tmp_out[0];
    return ESP_OK;
}

esp_err_t swd_spi_send_bits(uint8_t *bits, size_t bit_len)
{
    if (bits == NULL) {
        ESP_LOGE(TAG, "Bits pointer is null");
        return ESP_ERR_INVALID_ARG;
    }

    spi_dev->user.usr_dummy = 0;
    spi_dev->user.usr_command = 0;
    spi_dev->user.usr_mosi = 1;
    spi_dev->user.usr_miso = 0;
    spi_dev->user.sio = 1;
//    spi_dev->misc.ck_idle_edge = 0;
//    spi_dev->user.ck_out_edge = 0;

    switch (bit_len) {
        case 8: {
            spi_dev->data_buf[0] = (bits[0] & 0x000000ff);
            break;
        }

        case 16: {
            spi_dev->data_buf[0] = (bits[0] & 0x000000ff) | ((bits[1] << 8) & 0x0000ff00);
            break;
        }

        case 33: {
            spi_dev->data_buf[0] = (bits[0]) | (bits[1] << 8) | (bits[2] << 16) | (bits[3] << 24);
            spi_dev->data_buf[1] = (bits[4] & 0xff);
            break;
        }

        case 51: {
            spi_dev->data_buf[0] = (bits[0]) | (bits[1] << 8) | (bits[2] << 16) | (bits[3] << 24);
            spi_dev->data_buf[1] = (bits[4]) | (bits[5] << 8) | (bits[6] << 16) | 0U << 24;
            break;
        }

        default: {
            uint32_t data_buf[2];
            uint8_t *data_p = (uint8_t *)data_buf;
            size_t idx;

            for (idx = 0; idx < div_round_up(bit_len, 8); idx++) {
                data_p[idx] = data_buf[idx];
            }

            // last byte use mask:
            data_p[idx - 1] = data_p[idx - 1] & ((2U >> (bit_len % 8)) - 1U);

            spi_dev->data_buf[0] = data_buf[0];
            spi_dev->data_buf[1] = data_buf[1];
        }
    }

    spi_dev->ms_dlen.ms_data_bitlen = bit_len - 1;

    spi_dev->cmd.update = 1;
    while (spi_dev->cmd.update);

    spi_dev->cmd.usr = 1; // Trigger Tx!!

    if (swd_spi_wait_till_ready(1000) != ESP_OK) {
        ESP_LOGE(TAG, "Read bit timeout");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t swd_spi_recv_bits(uint8_t *bits, size_t bit_len)
{
    uint32_t buf[2] = {};
    uint8_t *buf_p = (uint8_t *)buf;

    if (bits == NULL) {
        ESP_LOGE(TAG, "Bits pointer is null");
        return ESP_ERR_INVALID_ARG;
    }

    spi_dev->user.usr_dummy = 0;
    spi_dev->user.usr_command = 0;
    spi_dev->user.usr_mosi = 0;
    spi_dev->user.usr_miso = 1;
//    spi_dev->misc.ck_idle_edge = 0;
//    spi_dev->user.ck_out_edge = 0;
    spi_dev->user.sio = 1;
    spi_dev->ms_dlen.ms_data_bitlen = bit_len - 1;
    spi_dev->cmd.update = 1;
    while (spi_dev->cmd.update);

    spi_dev->cmd.usr = 1; //Trigger Rx!!
    if (swd_spi_wait_till_ready(1000) != ESP_OK) {
        ESP_LOGE(TAG, "Read bit timeout");
        return ESP_ERR_TIMEOUT;
    }

    buf[0] = spi_dev->data_buf[0];
    buf[1] = spi_dev->data_buf[1];

    size_t idx = 0;
    for (idx = 0; idx < div_round_up(bit_len, 8); idx++) {
        bits[idx] = buf_p[idx];
    }

    bits[idx - 1] = bits[idx - 1] & ((2 >> (bit_len % 8)) - 1);

    return ESP_OK;
}

esp_err_t swd_spi_reset()
{
    uint8_t tmp[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    return swd_spi_send_bits(tmp, 51);
}

esp_err_t swd_spi_switch()
{
    const uint16_t swj_magic = 0xE79E;
    uint8_t tmp_in[2];
    tmp_in[0] = swj_magic & 0xff;
    tmp_in[1] = (swj_magic >> 8) & 0xff;
    return swd_spi_send_bits(tmp_in, 16);
}

static uint8_t swd_spi_transfer(uint32_t req, uint32_t *data)
{
    uint8_t req_byte = 0x81 | ((req & 0x0f) << 1) | (calc_parity_u8(req & 0x0f) << 5);
    uint8_t ack = 0;
    swd_spi_send_header(req_byte, &ack);

    if (ack == 0x01) { // OK
        if (req & SWD_REG_R) { // READ
            uint32_t data_read = 0;
            uint8_t parity_read = 0;
            swd_spi_read_data(&data_read, &parity_read);
            if (data != NULL) {
                *data = data_read;
            }
        } else { // WRITE
            uint8_t parity = calc_parity_u32(*data);
            swd_spi_write_data(*data, parity);
        }
    } else if (ack == 0x02) { // WAIT
        ESP_LOGW(TAG, "SWD WAIT received");
    } else { // FAULT
        ESP_LOGE(TAG, "SWD FAULT received: 0x%x", ack);
    }

    return ack;
}

uint8_t swd_spi_read_dp(uint8_t adr, uint32_t *val)
{
    uint32_t tmp_in;
    uint8_t tmp_out[4];
    uint8_t ack;
    uint32_t tmp;
    tmp_in = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(adr);
    ack = swd_spi_transfer(tmp_in, (uint32_t *)tmp_out);
    *val = 0;
    tmp = tmp_out[3];
    *val |= (tmp << 24);
    tmp = tmp_out[2];
    *val |= (tmp << 16);
    tmp = tmp_out[1];
    *val |= (tmp << 8);
    tmp = tmp_out[0];
    *val |= (tmp << 0);
    return (ack == 0x01);
}

uint8_t swd_spi_transfer_retry(uint32_t req, uint32_t *data)
{
    return 0;
}

void swd_spi_send_trn(uint8_t trn_cycles)
{
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    spi_dev->user.usr_dummy = 0;
    spi_dev->user.usr_command = 0;
    spi_dev->user.usr_mosi = 1;
    spi_dev->user.usr_miso = 0;
    spi_dev->ms_dlen.ms_data_bitlen = trn_cycles - 1;
    spi_dev->data_buf[0] = 0;
    spi_dev->cmd.update = 1;
    while (spi_dev->cmd.update);
    spi_dev->cmd.usr = 1;
    swd_spi_wait_till_ready(1000);
}

void swd_spi_read_trn(uint8_t trn_cycles)
{
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);
    spi_dev->user.usr_dummy = 0;
    spi_dev->user.usr_command = 0;
    spi_dev->user.usr_mosi = 0;
    spi_dev->user.usr_miso = 1;
    spi_dev->ms_dlen.ms_data_bitlen = trn_cycles - 1;
    spi_dev->cmd.update = 1;
    while (spi_dev->cmd.update);
    spi_dev->cmd.usr = 1;
    swd_spi_wait_till_ready(1000);
}

esp_err_t swd_spi_send_header(uint8_t header_data, uint8_t *ack)
{
    uint8_t data_buf[1] = {0};
    swd_spi_send_bits(&header_data, 8);
    swd_spi_read_trn(1);
    swd_spi_recv_bits(data_buf, 3);
    *ack = data_buf[0] & 0b111;
    return ESP_OK;
}

esp_err_t swd_spi_read_data(uint32_t *data_out, uint8_t *parity_out)
{
    uint8_t data_buf[5] = {0};
    swd_spi_recv_bits(data_buf, 33);
    *data_out = (data_buf[0] << 0) | (data_buf[1] << 8) | (data_buf[2] << 16) | (data_buf[3] << 24);
    *parity_out = data_buf[4] & 0x01;
    swd_spi_send_trn(1);
    return ESP_OK;
}

esp_err_t swd_spi_write_data(uint32_t data, uint8_t parity)
{
    uint8_t data_buf[5] = {0};
    data_buf[0] = (data >> 0) & 0xff;
    data_buf[1] = (data >> 8) & 0xff;
    data_buf[2] = (data >> 16) & 0xff;
    data_buf[3] = (data >> 24) & 0xff;
    data_buf[4] = parity;
    swd_spi_send_bits(data_buf, 33);
    swd_spi_send_trn(1);
    return ESP_OK;
}