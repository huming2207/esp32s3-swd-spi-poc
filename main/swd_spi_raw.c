#include <esp_err.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <soc/spi_struct.h>
#include <hal/spi_ll.h>
#include <hal/spi_hal.h>
#include <string.h>
#include <endian.h>
#include "swd_spi_raw.h"

#ifdef TAG
#undef TAG
#endif

#define TAG "swd_spi"

static spi_device_handle_t spi_handle = NULL;
static volatile spi_dev_t *spi_dev = (volatile spi_dev_t *)(DR_REG_SPI2_BASE);

static spi_hal_context_t hal_ctx = {};

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

static uint8_t bitswap8(uint8_t v)
{
    v = (v & 0xF0) >> 4 | (v & 0x0F) << 4;
    v = (v & 0xCC) >> 2 | (v & 0x33) << 2;
    v = (v & 0xAA) >> 1 | (v & 0x55) << 1;
    return v;
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

    gpio_reset_pin(swdio);
    gpio_reset_pin(swclk);

    gpio_set_pull_mode(swdio, GPIO_FLOATING);

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

    hal_ctx.hw = spi_dev;
    hal_ctx.dma_enabled = false;


    // Not using DMA
    spi_dev->user.usr_conf_nxt = 0;
    spi_dev->slave.usr_conf = 0;
    spi_dev->dma_conf.dma_rx_ena = 0;
    spi_dev->dma_conf.dma_tx_ena = 0;

    // Set to Master mode
    spi_dev->slave.slave_mode = false;

    // use all 64 bytes of the buffer
    spi_dev->user.usr_mosi_highpart = false;
    spi_dev->user.usr_miso_highpart = false;

    // Disable cs pin
    spi_dev->user.cs_setup = false;
    spi_dev->user.cs_hold = false;

    // Disable CS signal
    spi_dev->misc.cs0_dis = 1;
    spi_dev->misc.cs1_dis = 1;
    spi_dev->misc.cs2_dis = 1;
    spi_dev->misc.cs3_dis = 1;
    spi_dev->misc.cs4_dis = 1;
    spi_dev->misc.cs5_dis = 1;

    // Duplex transmit
    spi_dev->user.doutdin = false;  // half dulex

    // Set data bit order
    spi_dev->ctrl.wr_bit_order = 1;   // SWD -> LSB
    spi_dev->ctrl.rd_bit_order = 1;   // SWD -> LSB

    // Set dummy
    spi_dev->user.usr_dummy = 0; // not use

    // Set spi clk: 40Mhz 50% duty
    // CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX_CONF_U, SPI1_CLK_EQU_SYS_CLK);

    // See TRM `SPI_CLOCK_REG`
    spi_dev->clock.clk_equ_sysclk = false;
    spi_dev->clock.clkdiv_pre = 0;
    spi_dev->clock.clkcnt_n = 8 - 1;
    spi_dev->clock.clkcnt_h = 8 / 2 - 1;
    spi_dev->clock.clkcnt_l = 8 - 1;

    // MISO delay setting
    spi_dev->user.rsck_i_edge = true;
    spi_dev->din_mode.din0_mode = 0;
    spi_dev->din_mode.din1_mode = 0;
    spi_dev->din_mode.din2_mode = 0;
    spi_dev->din_mode.din3_mode = 0;
    spi_dev->din_num.din0_num = 0;
    spi_dev->din_num.din1_num = 0;
    spi_dev->din_num.din2_num = 0;
    spi_dev->din_num.din3_num = 0;

    // Set the clock polarity and phase CPOL = 1, CPHA = 0
    spi_dev->misc.ck_idle_edge = 1;  // HIGH while idle
    spi_dev->user.ck_out_edge = 0;

    // enable spi clock
    spi_dev->clk_gate.clk_en = 1;
    spi_dev->clk_gate.mst_clk_active = 1;
    spi_dev->clk_gate.mst_clk_sel = 1;

    // No command and addr for now
    spi_dev->user.usr_command = 0;
    spi_dev->user.usr_addr = 0;
    
    return ret;
}

esp_err_t swd_spi_wait_till_ready(int32_t timeout_cycles)
{
    while(timeout_cycles >= 0 && spi_ll_get_running_cmd(spi_dev) != 0) {
        timeout_cycles--;
    }

    if (timeout_cycles < 0) {
        return ESP_ERR_TIMEOUT;
    } else {
        return ESP_OK;
    }
}

esp_err_t swd_spi_read_idcode(uint32_t *idcode)
{
    uint8_t tmp_out[4] = { 0 };

    if (swd_spi_read_dp(0, (uint32_t *)tmp_out) != 0x01) {
        return ESP_FAIL;
    }

    *idcode = (tmp_out[0] << 24) | (tmp_out[1] << 16) | (tmp_out[2] << 8) | tmp_out[3];
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

    if (swd_spi_wait_till_ready(1000000) != ESP_OK) {
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
    spi_dev->user.sio = 1;
    spi_dev->ms_dlen.ms_data_bitlen = bit_len - 1;
    spi_dev->cmd.update = 1;
    while (spi_dev->cmd.update);

    spi_dev->cmd.usr = 1; //Trigger Rx!!
    if (swd_spi_wait_till_ready(1000000) != ESP_OK) {
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

    ESP_LOGI(TAG, "swd_spi_xfer: header req 0x%02x", (uint8_t)req_byte);

    if (ack == 0x01) { // OK
        if (req & SWD_REG_R) { // READ
            uint32_t data_read = 0;
            uint8_t parity_read = 0;
            swd_spi_read_data(&data_read, &parity_read);
            if (data != NULL) {
                *data = data_read;
            }
        } else { // WRITE
            if (data != NULL) {
                uint8_t parity = calc_parity_u32(*data);
                swd_spi_write_data(*data, parity);
            }
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
    spi_dev->user.usr_dummy = 0;
    spi_dev->user.usr_command = 0;
    spi_dev->user.usr_mosi = 1;
    spi_dev->user.usr_miso = 0;
    spi_dev->ms_dlen.ms_data_bitlen = trn_cycles - 1;
    spi_dev->data_buf[0] = 0;
    spi_dev->cmd.update = 1;
    while (spi_dev->cmd.update);
    spi_dev->cmd.usr = 1;
    swd_spi_wait_till_ready(1000000);
}

void swd_spi_read_trn(uint8_t trn_cycles)
{
    spi_dev->user.usr_dummy = 0;
    spi_dev->user.usr_command = 0;
    spi_dev->user.usr_mosi = 0;
    spi_dev->user.usr_miso = 1;
    spi_dev->ms_dlen.ms_data_bitlen = trn_cycles - 1;
    spi_dev->cmd.update = 1;
    while (spi_dev->cmd.update);
    spi_dev->cmd.usr = 1;
    swd_spi_wait_till_ready(1000000);
}

esp_err_t swd_spi_send_header(uint8_t header_data, uint8_t *ack)
{
    uint8_t data_buf[7] = {0};
    swd_spi_send_bits(&header_data, 8);
    swd_spi_recv_bits(data_buf, 4); // 1 turnaround + 3 bit ACK
    ESP_LOGI(TAG, "Got ACK 0x%x", data_buf[0]);
    *ack = data_buf[0] & 0b111;
    return ESP_OK;
}

esp_err_t swd_spi_read_data(uint32_t *data_out, uint8_t *parity_out)
{
    uint8_t data_buf[5] = {0};
    swd_spi_recv_bits(data_buf, 34);
    *data_out = (data_buf[3] << 0) | (data_buf[2] << 8) | (data_buf[1] << 16) | (data_buf[0] << 24);
    *parity_out = data_buf[4] & 0x01;

    ESP_LOG_BUFFER_HEX(TAG, data_buf, 5);
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