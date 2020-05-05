



#ifndef _HEALTH_BOARD_AUDIO_H_
#define _HEALTH_BOARD_AUDIO_H_

#define SDCARD_INTR_GPIO          GPIO_NUM_13
#define BLUE_LED_GPIO             GPIO_NUM_25
#define GREEN_LED_GPIO            GPIO_NUM_26
#define CHG_LED_GPIO            GPIO_NUM_36


#define HEALTH_INTR_PIN     (32)

#define HSENSOR_HW_FWUPGRADE_PIN     (32)

#define HSENSOR_EN_PIN     (19)

#define M_BAT_Q     (ADC1_CHANNEL_3)

#ifdef CONFIG_ESP_HEALTH_I2C
#define HEALTH_I2C_NUM  I2C_NUM_0

#define HEALTH_IIC_CLK     (21)
#define HEALTH_IIC_DATA  (22)


#else
#error the project only i2c !!!
#endif



#endif



