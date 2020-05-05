

#ifndef _HEALTH_BOARD_PICO_Y58_H_
#define _HEALTH_BOARD_PICO_Y58_H_


#define HEALTH_INTR_PIN     (13)
#define ECG_ADC_CHANNEL_I   (ADC1_CHANNEL_1)    //ecg_i
#define HSENSOR_AIN1_ADC_CHANNEL  (ADC1_CHANNEL_0)    //ecg_ii
#define HSENSOR_EN_PIN     (4)
#define HSENSOR_HW_FWUPGRADE_PIN     (25)


#define M_LED_IND     (39)
#define M_BAT_Q     (ADC1_CHANNEL_5)





#ifdef CONFIG_ESP_HEALTH_I2C
/* I2C gpios */
#define HEALTH_I2C_NUM  I2C_NUM_0

#define HEALTH_IIC_CLK     (27)
#define HEALTH_IIC_DATA  (32)

#endif

#ifdef CONFIG_ESP_HEALTH_SPI
#define HEALTH_SPI_CLK     (27)
#define HEALTH_SPI_MOSI  (32)
#define HEALTH_SPI_MISO  (25)
#define HEALTH_SPI_CS      (26)

#endif

#ifdef CONFIG_ESP_HEALTH_UART
#define CONFIG_HCOMM_UART UART_NUM_2
#define HEALTH_UART_TX     (25)

#define HEALTH_UART_RX     (32)
#endif



#endif
