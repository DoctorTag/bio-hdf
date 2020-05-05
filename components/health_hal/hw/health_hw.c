
#include "sdkconfig.h"
#include <string.h>
#include <stdio.h>
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/timer.h"
#include "driver/adc.h"
#include "driver/gpio.h"

#include "health_board.h"

#include "health_hw.h"

#ifdef CONFIG_ESP_HEALTH_I2C
#include "health_hw_i2c.h"
#endif

#ifdef CONFIG_ESP_HEALTH_SPI

#include "health_hw_spi.h"
#endif

#ifdef CONFIG_ESP_HEALTH_UART
#include "health_hw_uart.h"
#endif

#include "health_comm.h"
#define STREAM_QSIZE                              (32)

#define ECG_AD_ATTEN           ADC_ATTEN_DB_6

#define SEN_TIMER_DIVIDER         16  //  Hardware timer clock divider
#define SEN_MS_SCALE           (TIMER_BASE_CLK / (SEN_TIMER_DIVIDER*1000))  // convert counter value to ms

#define SEN_SAMPLE_CYCLE           (5)             //ms
#define SEN_TIMER_ID           (0)

QueueHandle_t hqueue = 0;
static const char *TAG = "FeelKit_HW";

#ifdef ECG_ADC_CHANNEL_I
static adc1_channel_t ecg_ch = ECG_ADC_CHANNEL_I;
#endif

#ifdef HEALTH_INTR_PIN
static void IRAM_ATTR health_intr_handler(void* arg)
{
	hdata_type_t sen_event ;
	portBASE_TYPE high_priority_task_awoken = 0;
	int dummy;
	if  (hqueue)
	{

		sen_event.type = REQ_FOR_SENSOR;
		if (xQueueIsQueueFullFromISR (hqueue))
		{
			xQueueReceiveFromISR (hqueue, &dummy, &high_priority_task_awoken);
			ESP_LOGI(TAG, "health_intr_handler queue full !!!  \n");
		}
		xQueueSendFromISR (hqueue, (void * )&sen_event, &high_priority_task_awoken);

	}



}

#endif

#ifdef INBED_SIGNAL_GPIO
static void IRAM_ATTR piezo_intr_handler(void* arg)
{
	portBASE_TYPE high_priority_task_awoken = 0;
	int dummy;
	if  (hqueue)
	{
		hdata_type_t sen_event;
		uint8_t inbed[2];
		sen_event.type = IND_STATUS;
		inbed[0]   = H_INFO_INBED;
		inbed[1] = gpio_get_level(INBED_SIGNAL_GPIO);
		sen_event.pdata = (void*)inbed;
		if (xQueueIsQueueFullFromISR (hqueue))
		{
			xQueueReceiveFromISR (hqueue, &dummy, &high_priority_task_awoken);
			ESP_LOGI(TAG, "piezo_intr_handler queue full !!!  \n");
		}
		xQueueSendFromISR (hqueue, (void * )&sen_event, &high_priority_task_awoken);

	}



}

#endif


#ifdef ECG_ADC_CHANNEL_I

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
static void IRAM_ATTR health_timer_isr(void *para)
{
	int timer_idx = (int) para;
	hdata_type_t evt ;

	/* Retrieve the interrupt status and the counter value
	   from the timer that reported the interrupt */
	uint32_t intr_status = TIMERG0.int_st_timers.val;


	/* Clear the interrupt
	   and update the alarm time for the timer with without reload */
	if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0)
	{
		TIMERG0.int_clr_timers.t0 = 1;
	}

	/* After the alarm has been triggered
	  we need enable it again, so it is triggered the next time */
	TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

	/* Now just send the event data back to the main program task */
	portBASE_TYPE high_priority_task_awoken = 0;
	int dummy;
	if  (hqueue)
	{
		evt.type = REQ_FOR_MCU_AD;

		if (xQueueIsQueueFullFromISR (hqueue))
		{
			xQueueReceiveFromISR (hqueue, &dummy, &high_priority_task_awoken);
		}
		xQueueSendFromISR (hqueue, (void * )&evt, &high_priority_task_awoken);
	}

}

static void health_setup_timer()
{
	/* Select and initialize basic parameters of the timer */
	timer_config_t config;
	config.divider = SEN_TIMER_DIVIDER;
	config.counter_dir = TIMER_COUNT_UP;
	config.counter_en = TIMER_PAUSE;
	config.alarm_en = TIMER_ALARM_EN;
	config.intr_type = TIMER_INTR_LEVEL;
	config.auto_reload = TIMER_AUTORELOAD_EN;
	timer_init(TIMER_GROUP_0, SEN_TIMER_ID, &config);

	/* Timer's counter will initially start from value below.
	   Also, if auto_reload is set, this value will be automatically reload on alarm */
	timer_set_counter_value(TIMER_GROUP_0, SEN_TIMER_ID, 0x00000000ULL);

	/* Configure the alarm value and the interrupt on alarm. */
	timer_set_alarm_value(TIMER_GROUP_0, SEN_TIMER_ID,SEN_SAMPLE_CYCLE * SEN_MS_SCALE );
	timer_enable_intr(TIMER_GROUP_0,SEN_TIMER_ID);
	timer_isr_register(TIMER_GROUP_0, SEN_TIMER_ID, health_timer_isr, (void *) SEN_TIMER_ID, ESP_INTR_FLAG_IRAM, NULL);
}

esp_err_t health_ecga_start(adc1_channel_t channel)
{
	//  adc1_config_width(ADC_WIDTH_BIT_12);

	adc1_config_channel_atten(channel, ECG_AD_ATTEN);
	return  timer_start(TIMER_GROUP_0, SEN_TIMER_ID);

}

esp_err_t health_ecga_stop(void)
{
	return    timer_pause(TIMER_GROUP_0, SEN_TIMER_ID);

}

int health_ecga_read(adc1_channel_t channel)
{
	return adc1_get_raw(ecg_ch);
}


sensor_data_t *health_read_adc(adc1_channel_t channel)
{


	sensor_data_t rdata;
	uint16_t ad_12b = health_ecga_read(channel);
	rdata.dvalue[0] = (uint8_t)ad_12b;
	rdata.dvalue[1] = (uint8_t)(ad_12b>>8);
	rdata.dtype = (SAMPLE_ECG >>4);
	return &rdata;
}

#endif

int health_write_bytes(uint8_t reg_add, uint8_t *pdata,uint8_t plen)
{
#ifdef CONFIG_ESP_HEALTH_I2C
	return health_i2c_write_bytes(reg_add, pdata, plen);
#else
#ifdef CONFIG_ESP_HEALTH_UART


	health_hw_uart_tx(UFRAME_START_WR, reg_add,pdata,plen);
	return 0;


#else
	return 0;
#endif
#endif

}
int health_write_reg(uint8_t reg_add, uint8_t data)
{
#ifdef CONFIG_ESP_HEALTH_I2C
	return health_i2c_write_reg(reg_add, data);
#else
#ifdef CONFIG_ESP_HEALTH_UART

	uint8_t tmp = data;

	health_hw_uart_tx(UFRAME_START_WR,reg_add,&tmp,1);

	return 0;


#else
	return 0;
#endif
#endif
}

int health_read_reg(uint8_t reg_add, uint8_t *pData)
{
#ifdef CONFIG_ESP_HEALTH_I2C
	return health_i2c_read_reg(reg_add,pData,1);
#else
#ifdef CONFIG_ESP_HEALTH_UART

	uint8_t rdlen;

	health_hw_uart_tx(UFRAME_START_RD, reg_add,(void*)0,0);
	health_hw_uart_wait_rsp(pData,&rdlen, 2000 / portTICK_RATE_MS,reg_add);
	return 0;


#else
	return 0;
#endif
#endif
}


//int health_hw_init(void (*health_intr_handler)(void *))
int health_hw_init(QueueHandle_t  *hsqueue)
{
	gpio_config_t io_conf;

#ifdef HSENSOR_EN_PIN

	//bit mask of the pins,
	io_conf.pin_bit_mask = BIT(HSENSOR_EN_PIN);
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	gpio_config(&io_conf);
	gpio_set_level(HSENSOR_EN_PIN, 0);
#endif
	BioSensor_HW_FwGradeDisable(1,0);
	vTaskDelay(200/portTICK_RATE_MS );

	BioSensorPowerOn(1);
	vTaskDelay(500/portTICK_RATE_MS );

#ifdef ECG_ADC_CHANNEL_I
	adc1_config_width(ADC_WIDTH_BIT_12);
	// adc1_config_channel_atten(ECG_ADC_CHANNEL_I, ECG_AD_ATTEN);

	health_setup_timer();
	//timer_start(TIMER_GROUP_0, SEN_TIMER_ID);
#endif
	hqueue = xQueueCreate(STREAM_QSIZE, sizeof(hdata_type_t));
	(*hsqueue) = hqueue ;

#ifdef INBED_SIGNAL_GPIO
	//interrupt of rising edge
	io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
	//bit mask of the pins, use GPIO4/5 here
	io_conf.pin_bit_mask = BIT64(INBED_SIGNAL_GPIO);
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

	//install gpio isr service
	gpio_install_isr_service(0);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(INBED_SIGNAL_GPIO, piezo_intr_handler, (void*)0);

#endif


#ifdef CONFIG_ESP_HEALTH_UART
	return health_hw_uart_init(hsqueue);

#else

	//interrupt of rising edge
	io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
	//bit mask of the pins, use GPIO4/5 here
	io_conf.pin_bit_mask = BIT64(HEALTH_INTR_PIN);
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

	//install gpio isr service
	gpio_install_isr_service(0);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(HEALTH_INTR_PIN, health_intr_handler, (void*)0);

	ESP_LOGI(TAG, "health_hw_init \n");



#ifdef CONFIG_ESP_HEALTH_I2C
	return health_i2c_init(); // ESP32 in master mode;
#endif

#endif


}

void  health_hw_deinit(void)
{
#ifdef ECG_ADC_CHANNEL_I
	timer_pause(TIMER_GROUP_0, SEN_TIMER_ID);
#endif
	vQueueDelete(hqueue);
	hqueue = 0;
#ifdef HEALTH_INTR_PIN

	gpio_isr_handler_remove(HEALTH_INTR_PIN);

	//uninstall gpio isr service
	gpio_uninstall_isr_service();
#endif

}

#ifdef HSENSOR_HW_FWUPGRADE_PIN

void BioSensor_HW_FwGradeDisable(uint32_t en,uint8_t strapping)
{

	gpio_config_t tio_conf;

	//bit mask of the pins,
	tio_conf.pin_bit_mask = BIT64(HSENSOR_HW_FWUPGRADE_PIN);
	//set as output mode
	tio_conf.mode = GPIO_MODE_OUTPUT;
	//enable pull-up mode
	tio_conf.pull_up_en = 1;
	tio_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	gpio_config(&tio_conf);
	gpio_set_level(HSENSOR_HW_FWUPGRADE_PIN, en);
#ifdef CONFIG_ESP_HEALTH_UART

	if((strapping >0)&& (HEALTH_UART_RX == HSENSOR_HW_FWUPGRADE_PIN))
	{
		health_hw_uart_io_reconfig();

	}
#endif
#ifdef HEALTH_INTR_PIN
	if((strapping >0)&& (HEALTH_INTR_PIN == HSENSOR_HW_FWUPGRADE_PIN))
	{
		//interrupt of rising edge
		tio_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
		//bit mask of the pins, use GPIO4/5 here
		tio_conf.pin_bit_mask = BIT(HEALTH_INTR_PIN);
		//set as input mode
		tio_conf.mode = GPIO_MODE_INPUT;
		//enable pull-up mode
		tio_conf.pull_up_en = 1;
		gpio_config(&tio_conf);
	}
#endif


}

#endif


void BioSensorPowerOn(uint32_t level)
{
#ifdef HSENSOR_EN_PIN


	gpio_set_level(HSENSOR_EN_PIN, level);

#endif

}


bio_sensor_status getBioSensorStatus(void)
{
	bio_sensor_status bio_status = BIO_LOSE;
//#ifdef CONFIG_ESP_HEALTH_I2C
	uint8_t dev_id, fwbl_ver ;
	if(health_read_reg(RegNum_Dev_Id, &dev_id) == ESP_OK)
	{
		if((dev_id&OEM_MASK) == OEM_ID)
		{
			if( health_read_reg(RegNum_BL_VERSION,&fwbl_ver) == ESP_OK)
			{
				if(dev_id&AP_MASK)
				{
					// if( health_read_reg(RegNum_AP_VER,&fwap_ver);
					if(fwbl_ver&AP_MASK)
						bio_status = BIO_NORMAL;
				}
				else if((fwbl_ver&AP_MASK) == 0)
					bio_status = BIO_BOOTL;
			}

		}
	}
//#endif
	return bio_status;
//bio_status = BIO_NORMAL;
}

int sensor_fun_setup(uint8_t fun1,uint8_t fun2)
{
	int ret = 0;
	ret |=  health_write_reg(RegNum_Fun1,  fun1);
	ret |=  health_write_reg(RegNum_Fun2,  fun2);
	return ret;
}

int sensor_fun_run(uint8_t mode,uint8_t is_analog)
{
	int ret = -1;
	if((mode &MODE_ALL_BITS) != RST_MODE)
	{
#ifdef ECG_ADC_CHANNEL_I
		if( is_analog &REG_FUN1_ATYPE )
		{
			if(mode&M_FUN_START)
			{
				switch(is_analog)
				{
				case REG_FUN1_ECG_A:
					ecg_ch = ECG_ADC_CHANNEL_I;
					break;
				case REG_FUN1_ECG_AII:
					ecg_ch = ECG_ADC_CHANNEL_I;
					break;

				case REG_FUN1_ECG_V1:
					ecg_ch = ECG_ADC_CHANNEL_I;;
					break;

				default:
					ecg_ch = ECG_ADC_CHANNEL_I;
					break;
				}
				health_ecga_start(ecg_ch);
			}
			else
			{
				health_ecga_stop();
			}
		}

#endif
		if(is_analog != REG_FUN1_ECG_AII)
		{
			ret =  health_write_reg(RegNum_RUN_Mode, mode);
#ifdef CONFIG_ESP_HEALTH_UART
			health_hw_uart_stream(mode);
#endif
		}
	}

	return ret;
}


int sensor_fun_fwupgrade(uint8_t start)
{
	if(start)
	{
#ifdef CONFIG_ESP_HEALTH_UART
		health_hw_uart_stream(M_FUN_START);
#endif
		health_write_reg(RegNum_Fun2,  BL_CMD_SENSOR_UPGRADE_REQ);


	}
#ifdef CONFIG_ESP_HEALTH_UART
	else
		health_hw_uart_stream(0);
#endif

	return 0;

}

int health_sdata_read(sensor_data_t *pdata)
{

#ifdef CONFIG_ESP_HEALTH_UART
	uint8_t *tmp = health_hw_uart_rx(portMAX_DELAY);
	if(tmp)
	{
		pdata->reg = tmp[1];
		pdata->dtype = tmp[5];
		//tmp[3] += 0x80;
		(pdata->dvalue[2]) = tmp[4];

		(pdata->dvalue[1]) =  tmp[3];

		(pdata->dvalue[0]) =  tmp[2];
	}
	else
	{
		ESP_LOGI(TAG, "Uart health_sdata_read  error !!! \n");

		return 0;
	}

#else

	uint8_t tmp[7],i;
	for( i=0; i < 3; i++)
	{
		if(health_read_reg(RegNum_Addl+i, tmp+i) != ESP_OK)
			return 0;

	}
	if(health_read_reg(RegNum_Ad_Type_Cnt,&(pdata->dtype)) != ESP_OK)
		return 0;


	//tmp[2] += 0x80;
	(pdata->dvalue[2]) = tmp[2];

	(pdata->dvalue[1]) =  tmp[1];

	(pdata->dvalue[0]) =  tmp[0];
#endif
	return 1;
}


