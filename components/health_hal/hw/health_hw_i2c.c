
#include <string.h>
#include <stdio.h>
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr.h"
#include "esp_log.h"

#include "driver/gpio.h"

#include "driver/i2c.h"

#include "health_hw_i2c.h"
#include "health_board.h"

#ifdef CONFIG_ESP_HEALTH_I2C

#define HEALTH_ADDR 0x88


static const i2c_config_t health_i2c_cfg =
{
    .mode = I2C_MODE_MASTER,
    .sda_io_num = HEALTH_IIC_DATA,
    .scl_io_num = HEALTH_IIC_CLK,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000
};

int health_i2c_init()
{
    int res;
    res = i2c_param_config(HEALTH_I2C_NUM, &health_i2c_cfg);
	
    res |= i2c_driver_install(HEALTH_I2C_NUM, health_i2c_cfg.mode, 0, 0, 0);
	
    return res;
}

int health_i2c_write_bytes(int reg, uint8_t *data, int datalen)
{
    int ret = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret |= i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, HEALTH_ADDR, 1);
    ret |= i2c_master_write_byte(cmd, reg, 1);
    ret |= i2c_master_write(cmd, data, datalen, 1);
    ret |= i2c_master_stop(cmd);
    ret |= i2c_master_cmd_begin(HEALTH_I2C_NUM, cmd, 2000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int health_i2c_write_reg(uint8_t reg_add, uint8_t data)
{
    int res = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    res |= i2c_master_start(cmd);
    res |= i2c_master_write_byte(cmd, HEALTH_ADDR, 1 /*ACK_CHECK_EN*/);
    res |= i2c_master_write_byte(cmd, reg_add, 1 /*ACK_CHECK_EN*/);
    res |= i2c_master_write_byte(cmd, data, 1 /*ACK_CHECK_EN*/);
    res |= i2c_master_stop(cmd);
    res |= i2c_master_cmd_begin(HEALTH_I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return res;
}

int health_i2c_read_reg(uint8_t reg_add, uint8_t *pData,size_t size)
{

    int res = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    res |= i2c_master_start(cmd);
    res |= i2c_master_write_byte(cmd, HEALTH_ADDR, 1 /*ACK_CHECK_EN*/);
    res |= i2c_master_write_byte(cmd, reg_add, 1 /*ACK_CHECK_EN*/);
    res |= i2c_master_stop(cmd);
    res |= i2c_master_cmd_begin(HEALTH_I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    res |= i2c_master_start(cmd);
    res |= i2c_master_write_byte(cmd, HEALTH_ADDR | 0x01, 1 /*ACK_CHECK_EN*/);
    if (size > 1)
    {
        i2c_master_read(cmd, pData, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, pData + size - 1, I2C_MASTER_NACK);
    res |= i2c_master_stop(cmd);
    res |= i2c_master_cmd_begin(HEALTH_I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);


    return res;
}


#endif
