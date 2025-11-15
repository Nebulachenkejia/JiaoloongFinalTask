//
// Created by Nebula on 2025/11/11.
//
#include "bmi088.h"
extern SPI_HandleTypeDef hspi1;
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}


void bmi088_write_byte(uint8_t tx_data)
{
    HAL_SPI_Transmit(&hspi1, &tx_data,1,1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
}

void bmi088_read_byte(uint8_t *rx_data, uint8_t length)
{
    if (rx_data == NULL || length == 0) return;
    HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
}

void bmi088_write_reg(uint8_t reg, uint8_t data)
{
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
    uint8_t tx_buf[2] = {reg, data};
    HAL_SPI_Transmit(&hspi1, tx_buf,2,100);
}


void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data)
{
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();

    bmi088_write_byte(reg & 0x7F); //与运算保证最高位置0
    bmi088_write_byte(data);

    BMI088_ACCEL_NS_H();
}
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) // 加速度计读取，注意需要忽略第一位数据dummy byte
{
    if (rx_data == NULL || length == 0) return;
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);

    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();

    uint8_t dummy;
    bmi088_write_byte(reg | 0x80); //或与运算保证最高位置1
    bmi088_read_byte(&dummy,1);
    bmi088_read_byte(rx_data,length);

    BMI088_ACCEL_NS_H();
}
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length){
    if (rx_data == NULL || length == 0) return;
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);

    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();

    bmi088_write_byte(reg | 0x80); //或与运算保证最高位置1
    bmi088_read_byte(rx_data,length);

    BMI088_GYRO_NS_H();
}
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t data)
{
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY);
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();

    bmi088_write_byte(reg & 0x7F); //与运算保证最高位置零
    bmi088_write_byte(data);

    BMI088_GYRO_NS_H();

}

void bmi088_init(void) {
    // Soft Reset ACCEL
    BMI088_ACCEL_NS_L();
    bmi088_write_reg(0x7E, 0xB6); // Write 0xB6 to ACC_SOFTRESET(0x7E)
    HAL_Delay(1); //1ms用于初始化
    BMI088_ACCEL_NS_H();
// Soft Reset GYRO
    BMI088_GYRO_NS_L();
    bmi088_write_reg(0x14, 0xB6); // Write 0xB6 to GYRO_SOFTRESET(0x14)
    bmi088_write_reg(0x0F, 0x01);
    HAL_Delay(30);  //30ms用于初始化
    BMI088_GYRO_NS_H();
    // Switch ACCEL to Normal Mode
    BMI088_ACCEL_NS_L();
    HAL_Delay(1);
    bmi088_write_reg(0x7D, 0x04); // Write 0x04 to ACC_PWR_CTRL(0x7D)
    //bmi088_write_reg(0x41, 0x03);
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();
}