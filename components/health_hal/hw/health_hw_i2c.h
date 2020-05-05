

#ifndef _HEALTH_HW_I2C_H_
#define _HEALTH_HW_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif


int health_i2c_init();

int health_i2c_write_reg(uint8_t reg_add, uint8_t data);


int health_i2c_read_reg(uint8_t reg_add, uint8_t *pData,size_t size);
int health_i2c_write_bytes(int reg, uint8_t *data, int datalen);

#ifdef __cplusplus
}
#endif

#endif
