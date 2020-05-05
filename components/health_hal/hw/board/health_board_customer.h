



#ifndef _HEALTH_BOARD_CUSTOMER_H_
#define _HEALTH_BOARD_CUSTOMER_H_

#define SDCARD_INTR_GPIO          GPIO_NUM_35
#define BLUE_LED_GPIO             GPIO_NUM_25
#define GREEN_LED_GPIO            GPIO_NUM_26
#define CHG_LED_GPIO            GPIO_NUM_16


//#define HSENSOR_HW_FWUPGRADE_PIN     (14)

#define HSENSOR_HW_FWUPGRADE_PIN     (32)

#define HSENSOR_EN_PIN     (19)
//#define M_CHG_IND     (16)

//#define M_LEDR_IND     (25)
//#define M_LEDG_IND     (26)

//#define M_BAT_Q     (ADC1_CHANNEL_5)
#define M_BAT_Q     (ADC1_CHANNEL_6)

#ifdef CONFIG_ESP_HEALTH_UART
#define CONFIG_HCOMM_UART UART_NUM_2
//#define HEALTH_UART_TX     (27)
//#define HEALTH_UART_RX     (14)

#define HEALTH_UART_TX     (33)
#define HEALTH_UART_RX     (32)

#else
#error the project only uart !!!
#endif



#endif



