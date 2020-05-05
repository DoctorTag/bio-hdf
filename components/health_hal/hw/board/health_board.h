

#ifndef _HEALTH_BOARD_H_
#define _HEALTH_BOARD_H_

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#if  defined(CONFIG_BOARD_CUSTOMER)
#include "health_board_customer.h"
#elif defined(CONFIG_BOARD_PIEZO_ESP32_INTERNAL)  //for BOARD_DEMO
#include "health_board_piezo_esp32_internal.h"
#elif defined(CONFIG_BOARD_DEMO_V10A)  //for BOARD_DEMO

//#define CONFIG_HLOCAL_UART UART_NUM_1


#define ECG_ADC_CHANNEL_I   (ADC1_CHANNEL_7)    //ecg_i
#define ECG_ADC_CHANNEL_II  (ADC1_CHANNEL_6)    //ecg_ii

#define H_UART_RX_PIN     (26)
#define H_UART_TX_PIN     (25)

#define HSENSOR_EN_PIN     (27)

#define HSENSOR_HW_FWUPGRADE_PIN     (19)

#ifdef CONFIG_ESP_HEALTH_I2C
/* I2C gpios */
#define HEALTH_IIC_CLK     (23)
#define HEALTH_IIC_DATA  (18)
#define HEALTH_INTR_PIN     (21)

#endif

#ifdef CONFIG_ESP_HEALTH_SPI
//#include "health_hw_spi.h"
#endif

#ifdef CONFIG_ESP_HEALTH_UART

#define CONFIG_HCOMM_UART UART_NUM_2
#define HEALTH_UART_RX    (19)
#define HEALTH_UART_TX  (18)

#endif

#elif defined(CONFIG_BOARD_DEMO_WB_F58_V00A)  //for BOARD_DEMO


#define HEALTH_INTR_PIN     (22)

#define ECG_ADC_CHANNEL_I   (ADC1_CHANNEL_6)    //ecg_i
#define ECG_ADC_CHANNEL_II  (ADC1_CHANNEL_3)    //ecg_ii


#define HSENSOR_EN_PIN     (27)

#define HSENSOR_HW_FWUPGRADE_PIN     (33)


/* I2C gpios */
#define HEALTH_I2C_NUM  I2C_NUM_0

#define HEALTH_IIC_CLK     (5)
#define HEALTH_IIC_DATA  (18)
#elif defined(CONFIG_BOARD_DEMO_PICO_F58)  //for BioSensor Module

#include "health_board_PICO_F58.h"

#elif defined(CONFIG_BOARD_DEMO_PICO_Y58)  //for BioSensor Module

#include "health_board_PICO_Y58.h"
#else
  #error please define board type!!!!
#endif


#ifdef __cplusplus
}
#endif

#endif
