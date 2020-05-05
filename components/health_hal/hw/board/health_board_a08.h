

#ifndef _HEALTH_BOARD_A08_H_
#define _HEALTH_BOARD_A08_H_


#define HEALTH_INTR_PIN     (13)

#define ECG_ADC_CHANNEL_I   (ADC1_CHANNEL_3)    //ecg_i
#define ECG_ADC_CHANNEL_II  (ADC1_CHANNEL_3)    //ecg_ii

//#define ECG_ADC_CHANNEL   (ADC1_CHANNEL_3)

#define HSENSOR_EN_PIN     (27)

#define HSENSOR_HW_FWUPGRADE_PIN     (23)


#ifdef CONFIG_ESP_HEALTH_I2C
/* I2C gpios */
#define HEALTH_IIC_CLK     (15)
#define HEALTH_IIC_DATA  (4)

#endif

#ifdef CONFIG_ESP_HEALTH_SPI
//#define HEALTH_SPI_CLK     (15)
//#define HEALTH_SPI_MOSI  (4)
//#define HEALTH_SPI_MISO  (4)
//#define HEALTH_SPI_CS  (4)

#endif

#ifdef CONFIG_ESP_HEALTH_UART
//#include "health_hw_uart.h"
#endif



#endif
