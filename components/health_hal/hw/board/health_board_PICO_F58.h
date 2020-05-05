

#ifndef _HEALTH_BOARD_PICO_F58_H_
#define _HEALTH_BOARD_PICO_F58_H_


#define HEALTH_INTR_PIN     (13)
#define ECG_ADC_CHANNEL_I   (ADC1_CHANNEL_3)    //ecg_i
#define HSENSOR_AIN1_ADC_CHANNEL  (ADC1_CHANNEL_7)    //ecg_ii
#define HSENSOR_EN_PIN     (12)
#define HSENSOR_HW_FWUPGRADE_PIN     (13)

#define M_CHG_IND     (4)
#define M_LED_IND     (38)
#define M_BAT_Q     (ADC1_CHANNEL_6)

#define MOT_SDA     (22)
#define MOT_SCL    (21)
#define MOT_INT    (19)

#define HSENSOR_STRAPPING0    (25)
#define HSENSOR_STRAPPING1    (15)


#ifdef CONFIG_ESP_HEALTH_I2C
/* I2C gpios */
#define HEALTH_I2C_NUM  I2C_NUM_0

#define HEALTH_IIC_CLK     (27)
#define HEALTH_IIC_DATA  (14)

#endif

#ifdef CONFIG_ESP_HEALTH_SPI
#define HEALTH_SPI_CLK     (27)
#define HEALTH_SPI_MOSI  (14)
#define HEALTH_SPI_MISO  (15)
#define HEALTH_SPI_CS      (25)

#endif

#ifdef CONFIG_ESP_HEALTH_UART
#define CONFIG_HCOMM_UART UART_NUM_2
#define HEALTH_UART_TX     (15)
#define HEALTH_UART_RX     (13)
#endif



#endif
