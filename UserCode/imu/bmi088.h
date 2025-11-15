#ifndef BMI088_H
#define BMI088_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif

// chip selection
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);
void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);

// bmi088 init
void bmi088_init(void);

// bmi088 read/write
void bmi088_write_byte(uint8_t tx_data);
void bmi088_write_reg(uint8_t reg, uint8_t data);
void bmi088_read_byte(uint8_t *rx_data, uint8_t length);

void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t data);
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *return_data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif // BMI088_H
