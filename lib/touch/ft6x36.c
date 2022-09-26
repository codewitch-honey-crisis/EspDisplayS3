#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include "lvgl/lvgl.h"
#include "ft6x36.h"

#define I2C_MASTER_TX_BUF_DISABLE   0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0 /*!< I2C master doesn't need buffer */
#define I2C_SLAVE_ADDRESS           (FT6236_I2C_SLAVE_ADDR)

static const int i2c_port = CONFIG_I2C_MASTER_PORT_NUM;
/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_TOUCH_I2C_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = CONFIG_TOUCH_I2C_SCK_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CONFIG_I2C_MASTER_FREQUENCY,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_port, &conf);
    if (err != ESP_OK)
    {
        return err;
    }
    return i2c_driver_install(i2c_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t i2c_read_bytes(uint8_t reg_addr, uint8_t *data, size_t data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, data_len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(i2c_port, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return result;
}

bool ft6x36_read_pos(touch_pos_t *touch_pos)
{
    uint8_t data_buf[5]; // 1 byte touch points, 2 bytes X, 2 bytes Y
    bool result = true;

    do
    {
        if (ESP_OK != i2c_read_bytes(FT6X36_TD_STAT_REG, data_buf, sizeof(data_buf)))
        {
            result = false;
            continue;
        }

        touch_pos->count = data_buf[0];
        touch_pos->last_x = ((data_buf[1] & FT6X36_MSB_MASK) << 8) | (data_buf[2] & FT6X36_LSB_MASK);
        touch_pos->last_y = ((data_buf[3] & FT6X36_MSB_MASK) << 8) | (data_buf[4] & FT6X36_LSB_MASK);

    } while (false);

    return result;
}

uint8_t ft6x36_get_gesture_id(void)
{
    uint8_t gesture_id;
    if (ESP_OK != i2c_read_bytes(FT6X36_GEST_ID_REG, &gesture_id, sizeof(gesture_id)))
    {
        gesture_id = 0u;
    }
    return gesture_id;
}

void ft6x36_init(void)
{
    i2c_master_init();
}