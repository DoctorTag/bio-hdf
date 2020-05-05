



#ifndef _HEALTH_BOARD_PIEZO_ESP32_F58GM_20C_H_
#define _HEALTH_BOARD_PIEZO_ESP32_F58GM_20C_H_

#define SDCARD_INTR_GPIO          GPIO_NUM_16
#define BLUE_LED_GPIO             GPIO_NUM_18
#define GREEN_LED_GPIO            GPIO_NUM_5
#define RED_LED_GPIO            GPIO_NUM_17



#define HSENSOR_HW_FWUPGRADE_PIN     (26)

#define HSENSOR_EN_PIN     (21)


#define INBED_SIGNAL_ADCH     (ADC1_CHANNEL_5)
//#define INBED_SIGNAL_GPIO     (GPIO_NUM_33)
#define HR_SIGNAL_ADCH     (ADC1_CHANNEL_0)

#ifdef CONFIG_ESP_HEALTH_UART
#define CONFIG_HCOMM_UART UART_NUM_2

#define HEALTH_UART_TX     (27)
#define HEALTH_UART_RX     (26)

#else
#error the project only uart !!!
#endif



#endif



