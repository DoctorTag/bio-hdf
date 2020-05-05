

#ifndef _HEALTH_HW_H_
#define _HEALTH_HW_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "driver/adc.h"
//#include "driver/uart.h"


#define RegNum_Addl                        0x00
#define RegNum_Addm                      0x01
#define RegNum_Addh                       0x02
#define RegNum_Ad_Type_Cnt          0x03
#define RegNum_Dev_Id                    0x04
#define RegNum_PStatus                   0x05
#define RegNum_AP_VER                   0x06
#define RegNum_BL_VERSION	          0x07

#define RegNum_Fun1                        0x08
#define RegNum_Fun2                        0x09
#define RegNum_RUN_Mode               0x0a

#define BL_REG_DATA                      0x0A

#define RegNum_AD_PGA                   0x0b
#define RegNum_OP_GAIN                  0x0c
#define RegNum_ADC_HZ                   0x0d


#define RegNum_MAX   14

//#define BL_REG_OFFSETH  RegNum_MAX    //RegNum_MAX

#define REG_ONLY_RD_LEN  8


#define SAMPLE_PIEZO   0x00
#define SAMPLE_PPG_RDC     0x10
#define SAMPLE_PPG_IRDC  0x20
#define SAMPLE_IMP              0x30

#define SAMPLE_GSR              0x40

#define SAMPLE_PPG_G     0x70
#define SAMPLE_PPG_R     0x80
#define SAMPLE_PPG_IR     0x90
#define SAMPLE_RESP             0xa0

#define SAMPLE_ECG              0xb0
#define SAMPLE_WH               0xc0

#define SAMPLE_GSENSOR       0xd0

#define FMUP_RSP         0xF0


#define REG_FUN1_ECG_AII          0x01
#define REG_FUN1_ECG_V1           0x02
#define REG_FUN1_ECG_A            0x04

#define REG_FUN1_ATYPE            (REG_FUN1_ECG_AII|REG_FUN1_ECG_V1| REG_FUN1_ECG_A )



#define M_FUN_START        0x10

#define HALT_MODE       0xe0
#define OBEY_MODE       0xc0
#define WATCH_MODE    0xa0
#define CAL_MODE          0x80
#define RST_MODE        0x60

#define MODE_ALL_BITS        0xe0

#define OEM_MASK        0x60
#define AP_MASK          0x80

#define OEM_ID        0x20
#define FWUP_PWD        0x58
#define RESET_PWD        0x68


#define BL_CMD_SENSOR_REBOOT    0X60
#define BL_CMD_SENSOR_UPGRADE_REQ    0X61
#define BL_CMD_SENSOR_ERASEAPP    0X62
#define BL_CMD_SENSOR_JMPAPP    0X63
#define BL_CMD_SENSOR_APRDY    0X64
#define BL_CMD_SENSOR_PROGRAM    0X6f

#define BL_CMD_OTA_DSEG0    0X80
#define BL_CMD_OTA_DSEG1    0X81
#define BL_CMD_OTA_DSEG2    0X82
#define BL_CMD_OTA_DSEG3    0X83
#define BL_CMD_OTA_DSEG4    0X84
#define BL_CMD_OTA_DSEG5    0X85
#define BL_CMD_OTA_DSEG6    0X86
#define BL_CMD_OTA_DSEG7    0X87

#define BL_CMD_OTA_REQ    0X88


typedef enum
{

    BIO_NORMAL =0,
    BIO_BOOTL,
    BIO_LOSE
} bio_sensor_status;

typedef enum
{
    REQ_FROM_TCP,
    REQ_FROM_BLE,     //BLUETOOTH
    
    REQ_FROM_UART,

    REQ_FOR_SENSOR,
    REQ_FOR_MCU_AD,
    REQ_UART_FOR_SENSOR,
    IND_STATUS

} __health_info_type;


typedef struct sensor_data
{
 uint8_t reg;
    uint8_t dtype;
    uint8_t dvalue[3];
} sensor_data_t;

typedef struct
{
    __health_info_type  type;
    //uint8_t type;
    void   *pdata;
} hdata_type_t;

void BioSensorPowerOn(uint32_t level);
void BioSensor_HW_FwGradeDisable(uint32_t en,uint8_t strapping);

int sensor_fun_setup(uint8_t fun1,uint8_t fun2);


int sensor_fun_run(uint8_t mode,uint8_t is_analog);



int health_write_reg(uint8_t reg_add, uint8_t data);


int health_read_reg(uint8_t reg_add, uint8_t *pData);
int health_write_bytes(uint8_t reg_add, uint8_t *pdata,uint8_t plen);

esp_err_t health_ecga_start(adc1_channel_t channel);


esp_err_t health_ecga_stop(void);

int health_hw_init(QueueHandle_t  *hsqueue);


void  health_hw_deinit(void);


int health_ecga_read(adc1_channel_t channel);


int health_sdata_read(sensor_data_t *pdata);
sensor_data_t * health_read_adc(adc1_channel_t channel);


bio_sensor_status getBioSensorStatus(void);
int sensor_fun_fwupgrade(uint8_t start);

#ifdef __cplusplus
}
#endif

#endif
