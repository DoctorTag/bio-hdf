
#include <string.h>
#include <stdio.h>
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr.h"
#include "esp_log.h"

#include "driver/gpio.h"

#include "driver/i2c.h"

#include "health_hw_spi.h"
#include "health_board.h"

#ifdef CONFIG_ESP_HEALTH_SPI

static spi_device_handle_t g_spi = NULL;

#define HEALTH_SPI_NUM  HSPI_HOST




int health_spi_init()
{
   
    esp_err_t ret = ESP_OK;

    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_NUM_27,
        .mosi_io_num = GPIO_NUM_33,
        .sclk_io_num = GPIO_NUM_32,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,              // Clock out at 10 MHz
        .mode = 0,                              // SPI mode 0
        .spics_io_num = GPIO_NUM_0,             // CS pin
        .queue_size = 6,                        //queue 7 transactions at a time
    };
    //Initialize the SPI bus
    if (g_spi) {
        return ret;
    }
    ret = spi_bus_initialize(HEALTH_SPI_NUM, &buscfg, 0);
    assert(ret == ESP_OK);
    ret = spi_bus_add_device(HEALTH_SPI_NUM, &devcfg, &g_spi);
    assert(ret == ESP_OK);
    gpio_set_pull_mode(0, GPIO_FLOATING);
    return ret;
}



int health_spi_write_reg(uint8_t reg_add, uint8_t data)
{
    /*Note: Implement this as per your platform*/
    esp_err_t ret;
    spi_transaction_t t;
    unsigned short data = 0;
    memset(&t, 0, sizeof(t));              //Zero out the transaction
    t.length = sizeof(unsigned short) * 8; //Len is in bytes, transaction length is in bits.
#if BIGENDIAN
    data = htons(val);
    t.tx_buffer = &data;                   //Data
#else
    t.tx_buffer = &val;
#endif
    ret = spi_device_transmit(g_spi, &t); //Transmit
    assert(ret == ESP_OK);   
           
    return 0;
}

int health_spi_read_reg(uint8_t reg_add, uint8_t *pData,size_t size)
{
    /*Note: Implement this as per your platform*/
    esp_err_t ret;
    spi_transaction_t t;
    unsigned short data = 0;

    memset(&t, 0, sizeof(t));           //Zero out the transaction
    t.length = sizeof(unsigned short) * 8;
    t.rxlength = sizeof(unsigned short) * 8; //The unit of len is byte, and the unit of length is bit.
    t.rx_buffer = &data;
    ret = spi_device_transmit(g_spi, &t);   //Transmit!
#if BIGENDIAN
    *pVal = ntohs(data);
#else
    *pVal = data;
#endif
    assert(ret == ESP_OK);       

    return 0;
}


void VprocHALcleanup(void)
{   
    /*if the customer platform requires any cleanup function
    * then implement such function here.
    * Otherwise the implementation of this function is complete
    */
    int ret = 0;
    ret = spi_bus_remove_device(g_spi);
    assert(ret == ESP_OK);
    ret = spi_bus_free(HSPI_HOST);
    assert(ret == ESP_OK);
}


#endif

