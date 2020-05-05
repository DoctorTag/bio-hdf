

#ifndef _HEALTH_BOARD_CUSTOMER_H_
#define _HEALTH_BOARD_CUSTOMER_H_


#define HEALTH_INTR_PIN     (17)
#define ECG_ADC_CHANNEL_I   (ADC1_CHANNEL_5)    //ecg_i

#define HSENSOR_HW_FWUPGRADE_PIN     (17)

//#define HSENSOR_EN_PIN     (12)
//#define M_CHG_IND     (4)

#define M_LED_IND     (22)
#define M_BAT_Q     (ADC1_CHANNEL_0)

#define PWR_I2C_NUM  I2C_NUM_1

#define PWR_SDA     (27)
#define PWR_SCL    (14)

#define PWR_INT    (12)
#define PWR_PG    (13)


#define MOT_INT    (04)



#ifdef CONFIG_ESP_HEALTH_I2C
/* I2C gpios */
#define HEALTH_I2C_NUM  I2C_NUM_0

#define HEALTH_IIC_CLK     (18)
#define HEALTH_IIC_DATA  (05)

#endif

#ifdef CONFIG_ESP_HEALTH_SPI
#define HEALTH_SPI_CLK     (18)
#define HEALTH_SPI_MOSI  (05)
#define HEALTH_SPI_MISO  (16)
#define HEALTH_SPI_CS      (39)

#endif

#ifdef CONFIG_ESP_HEALTH_UART
#define CONFIG_HCOMM_UART UART_NUM_2
#define HEALTH_UART_TX     (16)
#define HEALTH_UART_RX     (17)
#endif



#endif
